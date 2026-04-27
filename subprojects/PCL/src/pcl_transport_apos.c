/// \file pcl_transport_apos.c
/// \brief APOS Local Virtual Channel transport built on the template adapter.
#include "pcl/pcl_transport_apos.h"

#include "apos.h"
#include "pcl/pcl_transport_template.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define PCL_APOS_MAX_FRAME 65536u

struct pcl_apos_transport_t {
  unsigned int              process_id;
  int                       send_lvc;
  int                       recv_lvc;
  pcl_transport_template_t* template_transport;
};

static uint16_t pcl_apos_read_u16_be(const uint8_t* p) {
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t pcl_apos_read_u32_be(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) |
         ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8) |
         (uint32_t)p[3];
}

static void pcl_apos_write_u16_be(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFFu);
}

static void pcl_apos_write_u32_be(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)((v >> 16) & 0xFFu);
  p[2] = (uint8_t)((v >> 8) & 0xFFu);
  p[3] = (uint8_t)(v & 0xFFu);
}

static uint32_t pcl_apos_strlen_u32(const char* s) {
  return s ? (uint32_t)strlen(s) : 0u;
}

static pcl_status_t pcl_apos_alloc_string(const uint8_t** p,
                                          const uint8_t*  end,
                                          uint16_t        len,
                                          const char**    out) {
  char* value;
  if ((size_t)(end - *p) < len) return PCL_ERR_INVALID;
  value = (char*)malloc((size_t)len + 1u);
  if (!value) return PCL_ERR_NOMEM;
  if (len > 0u) memcpy(value, *p, len);
  value[len] = '\0';
  *p += len;
  *out = value;
  return PCL_OK;
}

static pcl_status_t pcl_apos_encode_frame(const pcl_template_frame_t* frame,
                                          uint8_t**                   out_data,
                                          uint32_t*                   out_size) {
  uint32_t topic_len = pcl_apos_strlen_u32(frame->topic);
  uint32_t service_len = pcl_apos_strlen_u32(frame->service_name);
  uint32_t type_len = pcl_apos_strlen_u32(frame->type_name);
  uint32_t size;
  uint8_t* data;
  uint8_t* p;

  if (!frame || !out_data || !out_size) return PCL_ERR_INVALID;
  if (topic_len > 65535u || service_len > 65535u || type_len > 65535u) {
    return PCL_ERR_INVALID;
  }
  if (frame->payload_size > 0u && !frame->payload) return PCL_ERR_INVALID;

  switch (frame->kind) {
    case PCL_TEMPLATE_FRAME_PUBLISH:
      size = 1u + 2u + topic_len + 2u + type_len + 4u + frame->payload_size;
      break;
    case PCL_TEMPLATE_FRAME_SVC_REQ:
      size = 1u + 4u + 2u + service_len + 2u + type_len + 4u +
             frame->payload_size;
      break;
    case PCL_TEMPLATE_FRAME_SVC_RESP:
      size = 1u + 4u + 2u + type_len + 4u + frame->payload_size;
      break;
    default:
      return PCL_ERR_INVALID;
  }
  if (size > PCL_APOS_MAX_FRAME) return PCL_ERR_NOMEM;

  data = (uint8_t*)malloc(size);
  if (!data) return PCL_ERR_NOMEM;

  p = data;
  *p++ = (uint8_t)frame->kind;
  if (frame->kind == PCL_TEMPLATE_FRAME_PUBLISH) {
    pcl_apos_write_u16_be(p, (uint16_t)topic_len); p += 2u;
    if (topic_len > 0u) memcpy(p, frame->topic, topic_len);
    p += topic_len;
    pcl_apos_write_u16_be(p, (uint16_t)type_len); p += 2u;
    if (type_len > 0u) memcpy(p, frame->type_name, type_len);
    p += type_len;
  } else if (frame->kind == PCL_TEMPLATE_FRAME_SVC_REQ) {
    pcl_apos_write_u32_be(p, frame->seq_id); p += 4u;
    pcl_apos_write_u16_be(p, (uint16_t)service_len); p += 2u;
    if (service_len > 0u) memcpy(p, frame->service_name, service_len);
    p += service_len;
    pcl_apos_write_u16_be(p, (uint16_t)type_len); p += 2u;
    if (type_len > 0u) memcpy(p, frame->type_name, type_len);
    p += type_len;
  } else {
    pcl_apos_write_u32_be(p, frame->seq_id); p += 4u;
    pcl_apos_write_u16_be(p, (uint16_t)type_len); p += 2u;
    if (type_len > 0u) memcpy(p, frame->type_name, type_len);
    p += type_len;
  }

  pcl_apos_write_u32_be(p, frame->payload_size); p += 4u;
  if (frame->payload_size > 0u) {
    memcpy(p, frame->payload, frame->payload_size);
  }

  *out_data = data;
  *out_size = size;
  return PCL_OK;
}

static pcl_status_t pcl_apos_decode_frame(const uint8_t*           data,
                                          uint32_t                 size,
                                          pcl_template_frame_t*    out) {
  const uint8_t* p = data;
  const uint8_t* end = data + size;
  uint16_t len;
  uint32_t payload_size;
  pcl_status_t rc;

  if (!data || !out || size < 1u) return PCL_ERR_INVALID;
  memset(out, 0, sizeof(*out));
  out->kind = (pcl_template_frame_kind_t)*p++;

  if (out->kind == PCL_TEMPLATE_FRAME_PUBLISH) {
    if ((size_t)(end - p) < 2u) return PCL_ERR_INVALID;
    len = pcl_apos_read_u16_be(p); p += 2u;
    rc = pcl_apos_alloc_string(&p, end, len, &out->topic);
    if (rc != PCL_OK) return rc;
    if ((size_t)(end - p) < 2u) return PCL_ERR_INVALID;
    len = pcl_apos_read_u16_be(p); p += 2u;
    rc = pcl_apos_alloc_string(&p, end, len, &out->type_name);
    if (rc != PCL_OK) return rc;
  } else if (out->kind == PCL_TEMPLATE_FRAME_SVC_REQ) {
    if ((size_t)(end - p) < 6u) return PCL_ERR_INVALID;
    out->seq_id = pcl_apos_read_u32_be(p); p += 4u;
    len = pcl_apos_read_u16_be(p); p += 2u;
    rc = pcl_apos_alloc_string(&p, end, len, &out->service_name);
    if (rc != PCL_OK) return rc;
    if ((size_t)(end - p) < 2u) return PCL_ERR_INVALID;
    len = pcl_apos_read_u16_be(p); p += 2u;
    rc = pcl_apos_alloc_string(&p, end, len, &out->type_name);
    if (rc != PCL_OK) return rc;
  } else if (out->kind == PCL_TEMPLATE_FRAME_SVC_RESP) {
    if ((size_t)(end - p) < 6u) return PCL_ERR_INVALID;
    out->seq_id = pcl_apos_read_u32_be(p); p += 4u;
    len = pcl_apos_read_u16_be(p); p += 2u;
    rc = pcl_apos_alloc_string(&p, end, len, &out->type_name);
    if (rc != PCL_OK) return rc;
  } else {
    return PCL_ERR_INVALID;
  }

  if ((size_t)(end - p) < 4u) return PCL_ERR_INVALID;
  payload_size = pcl_apos_read_u32_be(p); p += 4u;
  if ((size_t)(end - p) != payload_size) return PCL_ERR_INVALID;
  if (payload_size > 0u) {
    void* payload = malloc(payload_size);
    if (!payload) return PCL_ERR_NOMEM;
    memcpy(payload, p, payload_size);
    out->payload = payload;
    out->payload_size = payload_size;
  }
  return PCL_OK;
}

static void pcl_apos_free_decoded_frame(pcl_template_frame_t* frame) {
  if (!frame) return;
  free((void*)frame->topic);
  free((void*)frame->service_name);
  free((void*)frame->type_name);
  free((void*)frame->payload);
  memset(frame, 0, sizeof(*frame));
}

static pcl_status_t pcl_apos_hook_open(void* user_data) {
  pcl_apos_transport_t* ctx = (pcl_apos_transport_t*)user_data;
  if (!ctx) return PCL_ERR_INVALID;
  setupAPOS(ctx->process_id);
  return PCL_OK;
}

static pcl_status_t pcl_apos_hook_send(void*                       user_data,
                                       const pcl_template_frame_t* frame) {
  pcl_apos_transport_t* ctx = (pcl_apos_transport_t*)user_data;
  uint8_t* data = 0;
  uint32_t size = 0u;
  RS_Status status = RS_ERROR;
  pcl_status_t rc;

  if (!ctx || !frame) return PCL_ERR_INVALID;
  rc = pcl_apos_encode_frame(frame, &data, &size);
  if (rc != PCL_OK) return rc;

  sendMessageNonBlocking(ctx->send_lvc, data, (int)size, &status);
  free(data);
  return status == RS_SUCCESS ? PCL_OK : PCL_ERR_STATE;
}

static pcl_status_t pcl_apos_hook_recv(void*                 user_data,
                                       pcl_template_frame_t* out_frame,
                                       uint32_t              timeout_ms) {
  pcl_apos_transport_t* ctx = (pcl_apos_transport_t*)user_data;
  CTimeout timeout;
  TM_Status tm_status = TM_ERROR;
  RS_Status rs_status = RS_ERROR;
  int channels[1];
  int ready[1];
  int data_size = 0;
  uint8_t* data;
  pcl_status_t rc;

  if (!ctx || !out_frame) return PCL_ERR_INVALID;

  timeout.uiSeconds = timeout_ms / 1000u;
  timeout.uiNanoSeconds = (timeout_ms % 1000u) * 1000000u;
  channels[0] = ctx->recv_lvc;
  ready[0] = -1;
  waitOnMultiChannel(channels, 1, ready, &timeout, &tm_status);
  if (tm_status == TM_TIMEOUT) return PCL_ERR_TIMEOUT;
  if (tm_status != TM_SUCCESS || ready[0] != ctx->recv_lvc) {
    return PCL_ERR_STATE;
  }

  data = (uint8_t*)malloc(PCL_APOS_MAX_FRAME);
  if (!data) return PCL_ERR_NOMEM;
  receiveMessageNonBlocking(ctx->recv_lvc,
                            data,
                            (int)PCL_APOS_MAX_FRAME,
                            &data_size,
                            &rs_status);
  if (rs_status == RS_RESOURCE) {
    free(data);
    return PCL_ERR_TIMEOUT;
  }
  if (rs_status != RS_SUCCESS || data_size <= 0) {
    free(data);
    return PCL_ERR_STATE;
  }

  rc = pcl_apos_decode_frame(data, (uint32_t)data_size, out_frame);
  free(data);
  if (rc != PCL_OK) {
    pcl_apos_free_decoded_frame(out_frame);
  }
  return rc;
}

static void pcl_apos_hook_wake(void* user_data) {
  (void)user_data;
}

pcl_apos_transport_t* pcl_apos_transport_create(unsigned int    process_id,
                                                int             send_lvc,
                                                int             recv_lvc,
                                                pcl_executor_t* executor) {
  pcl_apos_transport_t* ctx;
  pcl_template_io_hooks_t hooks;

  if (!executor) return 0;

  ctx = (pcl_apos_transport_t*)calloc(1u, sizeof(*ctx));
  if (!ctx) return 0;
  ctx->process_id = process_id;
  ctx->send_lvc = send_lvc;
  ctx->recv_lvc = recv_lvc;

  memset(&hooks, 0, sizeof(hooks));
  hooks.user_data = ctx;
  hooks.open = pcl_apos_hook_open;
  hooks.send_blocking = pcl_apos_hook_send;
  hooks.recv_blocking = pcl_apos_hook_recv;
  hooks.wake = pcl_apos_hook_wake;

  ctx->template_transport = pcl_transport_template_create(&hooks, executor);
  if (!ctx->template_transport) {
    free(ctx);
    return 0;
  }
  return ctx;
}

pcl_status_t pcl_apos_transport_set_peer_id(pcl_apos_transport_t* ctx,
                                            const char*           peer_id) {
  if (!ctx) return PCL_ERR_INVALID;
  return pcl_transport_template_set_peer_id(ctx->template_transport, peer_id);
}

const pcl_transport_t* pcl_apos_transport_get_transport(
    pcl_apos_transport_t* ctx) {
  if (!ctx) return 0;
  return pcl_transport_template_get_transport(ctx->template_transport);
}

void pcl_apos_transport_destroy(pcl_apos_transport_t* ctx) {
  if (!ctx) return;
  pcl_transport_template_destroy(ctx->template_transport);
  free(ctx);
}
