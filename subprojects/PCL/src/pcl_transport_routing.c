/// \file pcl_transport_routing.c
/// \brief Manifest-driven per-endpoint transport routing (see header).
#include "pcl/pcl_transport_routing.h"

#include "pcl/pcl_capabilities.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_plugin_loader.h"
#include "pcl/pcl_transport.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PCL_ROUTING_MAX_PEERS 8u
#define PCL_ROUTING_MAX_GROUP_SIDE_MEMBERS 16u

typedef struct {
  pcl_plugin_handle_t*   handle;
  const pcl_transport_t* vtable;
  char                   peer_id[64];
} pcl_routing_transport_t;

typedef struct {
  char                endpoint_name[64];
  pcl_endpoint_kind_t endpoint_kind;
} pcl_routing_route_t;

/* A declared `exclusive` group: two named sides (rpc-shaped vs. pub/sub-
   shaped realizations of one logical leg, in this codebase's usage, but the
   grammar itself is contract-agnostic -- just two named endpoint sets). */
typedef struct {
  char     name[64];
  char     side_a[PCL_ROUTING_MAX_GROUP_SIDE_MEMBERS][64];
  uint32_t side_a_count;
  char     side_b[PCL_ROUTING_MAX_GROUP_SIDE_MEMBERS][64];
  uint32_t side_b_count;
} pcl_routing_exclusive_group_t;

struct pcl_transport_routing_t {
  pcl_executor_t*          executor;  /* borrowed; used to unregister on destroy */
  pcl_routing_transport_t* transports;
  size_t                   count;
  size_t                   capacity;
  pcl_routing_route_t*     routes;    /* endpoint routes installed on the executor */
  size_t                   route_count;
  size_t                   route_capacity;
  pcl_routing_exclusive_group_t* groups;  /* declared `exclusive` stanzas */
  size_t                          group_count;
  size_t                          group_capacity;
};

static void set_diag(char* diag, size_t diag_size, const char* fmt, ...) {
  va_list ap;
  if (!diag || diag_size == 0u) return;
  va_start(ap, fmt);
  vsnprintf(diag, diag_size, fmt, ap);
  va_end(ap);
}

/* Implements: REQ_PCL_428. */
static int kind_from_str(const char* s, pcl_endpoint_kind_t* out) {
  if (strcmp(s, "publisher") == 0)            { *out = PCL_ENDPOINT_PUBLISHER; return 1; }
  if (strcmp(s, "subscriber") == 0)           { *out = PCL_ENDPOINT_SUBSCRIBER; return 1; }
  if (strcmp(s, "provided") == 0)             { *out = PCL_ENDPOINT_PROVIDED; return 1; }
  if (strcmp(s, "consumed") == 0)             { *out = PCL_ENDPOINT_CONSUMED; return 1; }
  if (strcmp(s, "stream_provided") == 0)      { *out = PCL_ENDPOINT_STREAM_PROVIDED; return 1; }
  if (strcmp(s, "stream_consumed") == 0)      { *out = PCL_ENDPOINT_STREAM_CONSUMED; return 1; }
  return 0;
}

/* Implements: REQ_PCL_275. */
static int reliability_from_str(const char* s, pcl_qos_reliability_t* out) {
  if (strcmp(s, "best_effort") == 0) { *out = PCL_QOS_RELIABILITY_BEST_EFFORT; return 1; }
  if (strcmp(s, "reliable") == 0)    { *out = PCL_QOS_RELIABILITY_RELIABLE; return 1; }
  return 0;
}

static pcl_status_t routing_push(pcl_transport_routing_t* r,
                                 pcl_plugin_handle_t*     handle,
                                 const pcl_transport_t*   vtable,
                                 const char*              peer_id) {
  if (r->count == r->capacity) {
    size_t new_cap = r->capacity ? r->capacity * 2u : 4u;
    pcl_routing_transport_t* grown = (pcl_routing_transport_t*)realloc(
        r->transports, new_cap * sizeof(*grown));
    if (!grown) return PCL_ERR_NOMEM;
    r->transports = grown;
    r->capacity   = new_cap;
  }
  r->transports[r->count].handle = handle;
  r->transports[r->count].vtable = vtable;
  snprintf(r->transports[r->count].peer_id,
           sizeof(r->transports[r->count].peer_id), "%s", peer_id);
  r->count++;
  return PCL_OK;
}

/* Record an endpoint route installed on the executor so it can be rolled back if
   a later manifest line fails. */
static pcl_status_t routing_push_route(pcl_transport_routing_t* r,
                                       const char*              endpoint_name,
                                       pcl_endpoint_kind_t      endpoint_kind) {
  if (r->route_count == r->route_capacity) {
    size_t new_cap = r->route_capacity ? r->route_capacity * 2u : 4u;
    pcl_routing_route_t* grown = (pcl_routing_route_t*)realloc(
        r->routes, new_cap * sizeof(*grown));
    if (!grown) return PCL_ERR_NOMEM;
    r->routes        = grown;
    r->route_capacity = new_cap;
  }
  snprintf(r->routes[r->route_count].endpoint_name,
           sizeof(r->routes[r->route_count].endpoint_name), "%s", endpoint_name);
  r->routes[r->route_count].endpoint_kind = endpoint_kind;
  r->route_count++;
  return PCL_OK;
}

/* Record a declared `exclusive` group (copied by value; the caller's stack
   copy is transient). */
static pcl_status_t routing_push_group(pcl_transport_routing_t*             r,
                                       const pcl_routing_exclusive_group_t* group) {
  if (r->group_count == r->group_capacity) {
    size_t new_cap = r->group_capacity ? r->group_capacity * 2u : 4u;
    pcl_routing_exclusive_group_t* grown = (pcl_routing_exclusive_group_t*)realloc(
        r->groups, new_cap * sizeof(*grown));
    if (!grown) return PCL_ERR_NOMEM;
    r->groups        = grown;
    r->group_capacity = new_cap;
  }
  r->groups[r->group_count] = *group;
  r->group_count++;
  return PCL_OK;
}

/* Trim leading/trailing ASCII whitespace in place; returns the start pointer. */
static char* trim(char* s) {
  char* end;
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') ++s;
  end = s + strlen(s);
  while (end > s &&
         (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
    *--end = '\0';
  }
  return s;
}

/* Pop the next whitespace-delimited token from *cursor, advancing it. Returns
   NULL when no token remains. */
static char* next_token(char** cursor) {
  char* s = *cursor;
  char* tok;
  while (*s == ' ' || *s == '\t') ++s;
  if (*s == '\0') { *cursor = s; return NULL; }
  tok = s;
  while (*s != '\0' && *s != ' ' && *s != '\t') ++s;
  if (*s != '\0') { *s = '\0'; ++s; }
  *cursor = s;
  return tok;
}

/* Inject routing-owned context into a transport's JSON config, so a static
   manifest can wire transports that bind to the executor and identify inbound
   traffic by its declared peer. Preserves any author-supplied keys. */
static const char* inject_routing_context(const pcl_executor_t* e,
                                          const char* peer_id,
                                          const char* config,
                                          char* buf, size_t buf_size) {
  const char* body = NULL;  /* everything after the opening '{', incl. '}' */
  unsigned long long ptr = (unsigned long long)(uintptr_t)e;

  if (config) {
    const char* open = strchr(config, '{');
    if (open) {
      const char* p = open + 1;
      while (*p == ' ' || *p == '\t') ++p;
      if (*p != '}') body = open + 1;  /* non-empty object: keep its contents */
    }
  }
  if (body) {
    snprintf(buf, buf_size, "{\"executor\":%llu,\"peer_id\":\"%s\",%s",
             ptr, peer_id, body);
  } else {
    snprintf(buf, buf_size, "{\"executor\":%llu,\"peer_id\":\"%s\"}",
             ptr, peer_id);
  }
  return buf;
}

/* Implements: REQ_PCL_275, REQ_PCL_418, REQ_PCL_423, REQ_PCL_318,
   REQ_PCL_419. */
static pcl_status_t handle_transport_line(pcl_executor_t*          e,
                                          pcl_transport_routing_t* r,
                                          char*                    cursor,
                                          char*                    diag,
                                          size_t                   diag_size) {
  char* peer   = next_token(&cursor);
  char* plugin = next_token(&cursor);
  char* raw_config;
  char  config_buf[2304];
  const char* config;
  pcl_plugin_handle_t* handle = NULL;
  const pcl_transport_t* vtable = NULL;
  pcl_transport_caps_t caps = PCL_CAP_NONE;
  pcl_qos_t qos = {PCL_QOS_RELIABILITY_UNSPECIFIED};
  pcl_status_t rc;

  if (!peer || !plugin) {
    set_diag(diag, diag_size, "transport line needs <peer_id> <plugin_path>");
    return PCL_ERR_INVALID;
  }
  /* Fail closed on a peer already registered on the executor. Otherwise the
     register call below would overwrite a transport this manifest did not
     create, and teardown/rollback (which unregisters every manifest-owned peer)
     would then delete that pre-existing transport. */
  if (pcl_executor_get_transport_for_peer(e, peer) != NULL) {
    set_diag(diag, diag_size, "transport '%s': peer already registered", peer);
    return PCL_ERR_INVALID;
  }
  raw_config = trim(cursor);          /* remainder of line = opaque plugin config */
  config = inject_routing_context(e, peer, raw_config[0] ? raw_config : NULL,
                                  config_buf, sizeof(config_buf));

  rc = pcl_plugin_load_transport(plugin, config, &handle, &vtable);
  if (rc != PCL_OK) {
    set_diag(diag, diag_size, "transport '%s': failed to load %s (rc=%d)",
             peer, plugin, (int)rc);
    return rc;
  }
  /* Record before registering so a later failure still tears it down. */
  rc = routing_push(r, handle, vtable, peer);
  if (rc != PCL_OK) {
    // GCOVR_EXCL_START: routing_push fails only on heap exhaustion, which is
    // not injectable through this path.
    pcl_plugin_unload_transport(handle, vtable);
    return rc;
    // GCOVR_EXCL_STOP
  }
  /* Authoritative declared caps + offered QoS (coupled plugins carry
     fail-closed vtable stubs, so we must not derive from the vtable). */
  pcl_plugin_transport_caps(handle, config, vtable, &caps);
  pcl_plugin_transport_qos(handle, config, &qos);
  rc = pcl_executor_register_transport_caps(e, peer, vtable, caps);
  if (rc != PCL_OK) {
    set_diag(diag, diag_size, "transport '%s': register failed (rc=%d)", peer, (int)rc);
    return rc;
  }
  pcl_executor_register_transport_qos(e, peer, qos);
  return PCL_OK;
}

/* Split a comma-separated endpoint-name list in place into `out`/`out_count`,
   bounded by `max_count`. Fails closed on an empty member (leading/trailing/
   doubled comma) or an oversized list -- both are malformed input, not a
   silently-truncated one. */
/* Implements: REQ_PCL_467. */
static pcl_status_t split_endpoint_list(char*                list_str,
                                        char                 out[][64],
                                        uint32_t*             out_count,
                                        uint32_t              max_count,
                                        const char*           group_name,
                                        const char*           which_side,
                                        char*                 diag,
                                        size_t                diag_size) {
  char* p = list_str;
  *out_count = 0u;
  while (p && *p) {
    char* comma = strchr(p, ',');
    if (comma) *comma = '\0';
    if (*p == '\0') {
      set_diag(diag, diag_size, "exclusive '%s': empty endpoint name in %s",
               group_name, which_side);
      return PCL_ERR_INVALID;
    }
    if (*out_count >= max_count) {
      set_diag(diag, diag_size, "exclusive '%s': too many endpoints in %s",
               group_name, which_side);
      return PCL_ERR_INVALID;
    }
    snprintf(out[*out_count], 64, "%s", p);
    (*out_count)++;
    p = comma ? comma + 1 : NULL;
  }
  if (*out_count == 0u) {
    // GCOVR_EXCL_START: the caller accepts only non-empty tokens before
    // invoking the list splitter.
    set_diag(diag, diag_size, "exclusive '%s': %s has no endpoints",
             group_name, which_side);
    return PCL_ERR_INVALID;
    // GCOVR_EXCL_STOP
  }
  return PCL_OK;
}

/* Implements: REQ_PCL_466, REQ_PCL_467, REQ_PCL_464. */
static pcl_status_t handle_exclusive_line(pcl_transport_routing_t* r,
                                          char*                    cursor,
                                          char*                    diag,
                                          size_t                   diag_size) {
  char* group_name = next_token(&cursor);
  char* side_a_s   = next_token(&cursor);
  char* side_b_s   = next_token(&cursor);
  pcl_routing_exclusive_group_t group;
  pcl_status_t rc;
  size_t i;

  if (!group_name || !side_a_s || !side_b_s) {
    set_diag(diag, diag_size,
             "exclusive line needs <group_name> <side_a_endpoints> <side_b_endpoints>");
    return PCL_ERR_INVALID;
  }

  /* Declare-before-use requires group identity to be unambiguous: reject a
     redeclared group name rather than silently shadowing the first. */
  for (i = 0; i < r->group_count; ++i) {
    if (strcmp(r->groups[i].name, group_name) == 0) {
      set_diag(diag, diag_size, "exclusive '%s': group already declared", group_name);
      return PCL_ERR_INVALID;
    }
  }

  memset(&group, 0, sizeof(group));
  snprintf(group.name, sizeof(group.name), "%s", group_name);

  rc = split_endpoint_list(side_a_s, group.side_a, &group.side_a_count,
                           PCL_ROUTING_MAX_GROUP_SIDE_MEMBERS, group_name, "side A",
                           diag, diag_size);
  if (rc != PCL_OK) return rc;
  rc = split_endpoint_list(side_b_s, group.side_b, &group.side_b_count,
                           PCL_ROUTING_MAX_GROUP_SIDE_MEMBERS, group_name, "side B",
                           diag, diag_size);
  if (rc != PCL_OK) return rc;

  return routing_push_group(r, &group);
}

/* First endpoint name in [list, list+count) that is routed on the executor
   under any kind, or NULL if none. Queries the live executor rather than
   just `r->routes` (the routes this manifest load itself installed): by
   the time this runs, the executor already reflects both this load's own
   routes AND anything routed before this load started (a previous
   manifest load into the same executor, or a direct programmatic
   pcl_executor_set_endpoint_route call) -- the D5 fail-closed guarantee
   must hold against that pre-existing state too, not just against routes
   this one manifest happens to declare together. */
/* Implements: REQ_PCL_463, REQ_PCL_474, REQ_PCL_475. */
static const char* route_matching_list(const pcl_transport_routing_t* r,
                                       const char                    list[][64],
                                       uint32_t                       count) {
  uint32_t i;
  for (i = 0; i < count; ++i) {
    if (pcl_executor_endpoint_route_exists_any_kind(r->executor, list[i])) {
      return list[i];
    }
  }
  return NULL;
}

/* D5 compose-time exclusivity: checked once, after the whole manifest has
   been parsed (not incrementally per `route` line), so the result is
   independent of whether a manifest's `exclusive` declaration appears
   before or after the `route` lines for its members -- a route line
   processed while its group was not yet declared must not silently bypass
   the check. For every declared group, fail closed if *both* sides have at
   least one routed member; any number of same-side endpoints routed
   together, and endpoints in no group, are unaffected. */
/* Implements: REQ_PCL_463, REQ_PCL_465, REQ_PCL_469, REQ_PCL_474,
   REQ_PCL_475. */
static pcl_status_t validate_exclusivity(const pcl_transport_routing_t* r,
                                         char*                          diag,
                                         size_t                          diag_size) {
  size_t i;
  for (i = 0; i < r->group_count; ++i) {
    const pcl_routing_exclusive_group_t* g = &r->groups[i];
    const char* a_hit = route_matching_list(r, g->side_a, g->side_a_count);
    const char* b_hit = route_matching_list(r, g->side_b, g->side_b_count);

    if (a_hit && b_hit) {
      set_diag(diag, diag_size,
               "exclusive '%s': '%s' and '%s' are routed from opposite sides",
               g->name, a_hit, b_hit);
      return PCL_ERR_STATE;
    }
  }
  return PCL_OK;
}

/* Implements: REQ_PCL_275, REQ_PCL_428, REQ_PCL_422, REQ_PCL_420,
   REQ_PCL_426, REQ_PCL_427, REQ_PCL_472. */
static pcl_status_t handle_route_line(pcl_executor_t*          e,
                                      pcl_transport_routing_t* r,
                                      char*                    cursor,
                                      char*                    diag,
                                      size_t                   diag_size) {
  char* endpoint = next_token(&cursor);
  char* kind_s   = next_token(&cursor);
  char* peers_s  = next_token(&cursor);
  char* rel_s    = next_token(&cursor);
  pcl_endpoint_kind_t kind;
  const char* peer_ids[PCL_ROUTING_MAX_PEERS];
  uint32_t peer_count = 0u;
  pcl_endpoint_route_t route;
  pcl_status_t rc;
  char* p;

  if (!endpoint || !kind_s || !peers_s) {
    set_diag(diag, diag_size, "route line needs <endpoint> <kind> <peers>");
    return PCL_ERR_INVALID;
  }
  if (!kind_from_str(kind_s, &kind)) {
    set_diag(diag, diag_size, "route '%s': unknown kind '%s'", endpoint, kind_s);
    return PCL_ERR_INVALID;
  }

  /* Split the comma-separated peer list in place. */
  p = peers_s;
  while (p && *p) {
    char* comma = strchr(p, ',');
    if (comma) *comma = '\0';
    if (peer_count >= PCL_ROUTING_MAX_PEERS) {
      set_diag(diag, diag_size, "route '%s': too many peers", endpoint);
      return PCL_ERR_INVALID;
    }
    peer_ids[peer_count++] = p;
    p = comma ? comma + 1 : NULL;
  }

  memset(&route, 0, sizeof(route));
  route.endpoint_name = endpoint;
  route.endpoint_kind = kind;
  route.route_mode    = PCL_ROUTE_REMOTE;
  route.peer_ids      = peer_ids;
  route.peer_count    = peer_count;
  if (rel_s) {
    if (!reliability_from_str(rel_s, &route.qos_floor.reliability)) {
      set_diag(diag, diag_size, "route '%s': unknown reliability '%s'",
               endpoint, rel_s);
      return PCL_ERR_INVALID;
    }
  }

  /* Fail closed on a duplicate route rather than overwriting an existing one:
     rollback can only clear routes this load installed, so silently clobbering a
     pre-existing route (e.g. a programmatic default) would leave the executor
     mutated even when the load fails. */
  if (pcl_executor_endpoint_route_exists(e, endpoint, kind)) {
    set_diag(diag, diag_size, "route '%s': already routed", endpoint);
    return PCL_ERR_INVALID;
  }

  rc = pcl_executor_set_endpoint_route(e, &route);
  if (rc != PCL_OK) {
    set_diag(diag, diag_size, "route '%s': set failed (rc=%d)", endpoint, (int)rc);
    return rc;
  }
  /* Record before validating so a validation failure (or any later line's
     failure) still rolls this route back -- the load must leave nothing
     installed on error. */
  rc = routing_push_route(r, endpoint, kind);
  if (rc != PCL_OK) {
    // GCOVR_EXCL_START: routing_push_route fails only on heap exhaustion,
    // which is not injectable through this path.
    pcl_executor_clear_endpoint_route(e, endpoint, kind);
    return rc;
    // GCOVR_EXCL_STOP
  }
  /* D5 compose-time exclusivity is validated once, after the whole manifest
     is parsed (validate_exclusivity, called from pcl_transport_routing_load)
     -- not here per-line, so it does not depend on `exclusive` declaration
     order relative to this route's line. */
  /* Compose-time validation: caps + QoS floor, fail closed. */
  return pcl_executor_validate_endpoint_route(e, &route, diag, diag_size);
}

/* Implements: REQ_PCL_210, REQ_PCL_416, REQ_PCL_417, REQ_PCL_421,
   REQ_PCL_468, REQ_PCL_469, REQ_PCL_474, REQ_PCL_319, REQ_PCL_224. */
pcl_status_t pcl_transport_routing_load(pcl_executor_t*           e,
                                        const char*               manifest_path,
                                        pcl_transport_routing_t** out_routing,
                                        char*                     diag,
                                        size_t                    diag_size) {
  FILE* file;
  char  line[2048];
  char  local_diag[512] = {0};
  char* active_diag;
  size_t active_diag_size;
  pcl_transport_routing_t* r;
  pcl_status_t rc = PCL_OK;

  if (diag && diag_size) diag[0] = '\0';
  if (!e || !manifest_path || manifest_path[0] == '\0' || !out_routing) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport routing configuration failed: executor, manifest path, "
            "and output pointer are required");
    return PCL_ERR_INVALID;
  }
  active_diag = (diag && diag_size) ? diag : local_diag;
  active_diag_size = (diag && diag_size) ? diag_size : sizeof(local_diag);
  *out_routing = NULL;

  file = fopen(manifest_path, "r");
  if (!file) {
    set_diag(active_diag, active_diag_size,
             "manifest not found: %s", manifest_path);
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport routing configuration failed: %s", active_diag);
    return PCL_ERR_NOT_FOUND;
  }

  r = (pcl_transport_routing_t*)calloc(1u, sizeof(*r));
  if (!r) { fclose(file); return PCL_ERR_NOMEM; }
  r->executor = e;

  while (fgets(line, (int)sizeof(line), file)) {
    char* start = trim(line);
    char* cursor;
    char* directive;

    if (start[0] == '\0' || start[0] == '#') continue;

    cursor = start;
    directive = next_token(&cursor);
    if (!directive) continue;

    if (strcmp(directive, "transport") == 0) {
      rc = handle_transport_line(e, r, cursor, active_diag, active_diag_size);
    } else if (strcmp(directive, "exclusive") == 0) {
      rc = handle_exclusive_line(r, cursor, active_diag, active_diag_size);
    } else if (strcmp(directive, "route") == 0) {
      rc = handle_route_line(e, r, cursor, active_diag, active_diag_size);
    } else {
      set_diag(active_diag, active_diag_size,
               "unknown manifest directive '%s'", directive);
      rc = PCL_ERR_INVALID;
    }
    if (rc != PCL_OK) break;
  }

  fclose(file);

  if (rc == PCL_OK) {
    rc = validate_exclusivity(r, active_diag, active_diag_size);
  }

  if (rc != PCL_OK) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport routing configuration failed for manifest '%s': %s (rc=%d)",
            manifest_path,
            active_diag[0] ? active_diag : "no diagnostic was provided",
            (int)rc);
    pcl_transport_routing_destroy(r);
    return rc;
  }
  *out_routing = r;
  return PCL_OK;
}

/* Implements: REQ_PCL_209, REQ_PCL_425, REQ_PCL_424, REQ_PCL_421. */
void pcl_transport_routing_destroy(pcl_transport_routing_t* routing) {
  size_t i;
  if (!routing) return;
  /* Roll back endpoint routes before tearing down transports, so the executor is
     not left routing endpoints to peers whose transports are about to be
     unloaded. */
  if (routing->executor) {
    for (i = 0; i < routing->route_count; ++i) {
      pcl_executor_clear_endpoint_route(routing->executor,
                                        routing->routes[i].endpoint_name,
                                        routing->routes[i].endpoint_kind);
    }
  }
  free(routing->routes);
  /* Declared groups are pure manifest-level bookkeeping -- never registered
     on the executor, so nothing to unregister, just free the array. */
  free(routing->groups);
  for (i = 0; i < routing->count; ++i) {
    /* Unregister from the executor BEFORE unloading the library, or the executor
       is left holding vtable function pointers into a dlclose'd .so. */
    if (routing->executor) {
      pcl_executor_register_transport_caps(
          routing->executor, routing->transports[i].peer_id, NULL, PCL_CAP_NONE);
    }
    pcl_plugin_unload_transport(routing->transports[i].handle,
                                routing->transports[i].vtable);
  }
  free(routing->transports);
  free(routing);
}

/* Implements: REQ_PCL_317. */
size_t pcl_transport_routing_transport_count(const pcl_transport_routing_t* routing) {
  return routing ? routing->count : 0u;
}

/* Known optional plugin-exported gateway accessors (see pcl_plugin.h's
   PCL_TRANSPORT_PLUGIN_CAPS_SYMBOL for the established precedent of an
   optional, dlsym-resolved extra a transport plugin may or may not export).
   Not every transport type has a gateway symbol here -- only those with a
   real gateway-container pattern; unlisted/no-match transports correctly
   fall through to "no gateway", not an error. */
static const char* const kRoutingGatewaySymbols[] = {
  "pcl_shm_transport_plugin_gateway",
  "pcl_socket_transport_plugin_gateway",
};

typedef pcl_container_t* (*pcl_routing_gateway_fn)(const pcl_transport_t*);

/* Implements: REQ_PCL_471. */
pcl_status_t pcl_transport_routing_get_gateway(
    const pcl_transport_routing_t* routing,
    const char*                    peer_id,
    pcl_container_t**              out_gateway) {
  size_t i;
  if (!out_gateway) return PCL_ERR_INVALID;
  *out_gateway = NULL;
  if (!routing || !peer_id) return PCL_ERR_INVALID;

  for (i = 0; i < routing->count; ++i) {
    if (strcmp(routing->transports[i].peer_id, peer_id) == 0) {
      size_t si;
      for (si = 0; si < sizeof(kRoutingGatewaySymbols) / sizeof(kRoutingGatewaySymbols[0]);
           ++si) {
        void* sym = pcl_plugin_symbol(routing->transports[i].handle,
                                      kRoutingGatewaySymbols[si]);
        if (sym) {
          pcl_routing_gateway_fn fn = (pcl_routing_gateway_fn)sym;
          *out_gateway = fn(routing->transports[i].vtable);
          return PCL_OK;
        }
      }
      return PCL_OK;  /* peer found; its transport just has no gateway */
    }
  }
  return PCL_ERR_NOT_FOUND;
}
