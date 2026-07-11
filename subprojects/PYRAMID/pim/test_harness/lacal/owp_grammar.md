# `owp` frame grammar — pinned reference (LA-CAL rung 1, Phase 0)

**Status:** authoritative for this repo's LA-CAL work.
**Date pinned:** 2026-07-11.
**Sources (cross-checked, both agree):**

1. **OMSC-SPC-013 RevB** — Language-Agnostic CAL Specification, OMS Standard
   v2.5, `open-arsenal/oms`,
   `docs_markdown_unofficial/20_OMSC-SPC-013_RevB_LanguageAgnostic_CAL_Specification_DandD_v2_5.md`
   (tables 5.1-1 … 5.1-7, §6.1). Read via the unofficial Markdown
   conversion; **re-verify quote-level normative claims against the official
   `.docx` in `docs_official/` before any compliance-facing use.**
2. **Sleet** reference server (Rust, Apache-2.0),
   `open-arsenal/ams-gra-hello-world-sk-infra-sleet`,
   `sleet-types/src/owp.rs` (frame parse/serialize) — behaviour matches the
   spec on every point below.

This is the frozen contract the Phase-1 `owp` client and its mock server
implement. Anything not stated here is out of scope for rung 1.

---

## Transport and framing

- **WebSocket** (RFC 6455). The client's opening handshake **must** carry
  `Sec-WebSocket-Protocol: owp`; a server requires it. Subprotocol name is
  the literal token `owp` (Table 5.1-1).
- **UTF-8 text frames.** One protocol operation per WebSocket message
  ("Only one protocol operation is allowed per WebSocket message"). No
  explicit line terminator — the WebSocket frame boundary delimits the
  operation.
- **Field delimiter:** space (`0x20`) **or** horizontal tab (`0x09`).
  **Consecutive delimiters collapse to one** (Sleet: `split_first_token`
  skips leading delimiters, then splits at the next). CR (`0x0D`) and LF
  (`0x0A`) are **not** delimiters and may appear inside a field value
  (relevant: pretty-printed JSON payloads survive intact).
- **Operation token** is the first whitespace-delimited token of the frame;
  the remainder is parsed per operation. Message/JSON bodies are the
  **verbatim remainder** after the fixed leading tokens (internal
  whitespace/newlines preserved — do not re-tokenize the body).
- **Identifier regex** (applies to `service_id`, `subscription_id`, `topic`,
  `group`, `server_id`): `^[A-Za-z0-9_\-.]+$`. `message_name` is **not**
  identifier-validated (it may carry a `{namespace}` prefix — see SUB).

---

## Client → server operations

### `INIT <json>`  — first op, exactly once
Must be the first operation; sending twice, or not first, is `Illegal-State`.
JSON object (Table 5.1-4):

| field | type | required | notes |
|---|---|---|---|
| `versions` | array<string> | yes | desired subprotocol versions, non-empty (e.g. `["1.0"]`) |
| `schema` | string | yes | desired schema version for message validation (Sleet: e.g. `"002.5.0"`) |
| `service_id` | string | yes | Service Identifier; identifier-validated |
| `verbose` | bool | no | spec default `true`; Sleet struct has no serde default → **always send it explicitly** |

Wire: `INIT ` then the JSON text. Example:
`INIT {"versions":["1.0"],"schema":"002.5.0","service_id":"my-svc","verbose":true}`

INIT-time error replies (as `-ERR`): `Illegal-Argument` (bad JSON),
`Illegal-State` (twice/not-first), `Unsupported-Version`,
`Unsupported-Schema`, `Unsupported-Service`.

### `SUB <subscription_id> <message_name> <topic> [group]`
Field order is exactly this (confirmed in `owp.rs`, tokenize max 4).
- `subscription_id` — client-generated, unique per connection,
  identifier-validated. Reusing an active id ⇒ `Illegal-State`.
- `message_name` — the UCI global-element name. `{name}` when the target
  namespace is `https://www.vdl.afrl.af.mil/programs/oam`; otherwise
  `{target_namespace}name` with **no space** between the brace-wrapped
  namespace and the name. Not identifier-validated.
- `topic` — identifier-validated.
- `group` (optional) — identifier-validated; members of a named group form a
  load-balanced set: a matching published message is delivered to **exactly
  one** group member.
- Errors: `Illegal-Argument` (any field fails validation), `Illegal-State`
  (id in use / topic not allowed).

### `UNSUB <subscription_id>`
Terminates the subscription; no further `MSG` for that id. `Illegal-Argument`
(bad id), `Illegal-State` (id not active).

### `PUB <topic> <message>`
`topic` identifier-validated; `message` is the verbatim remainder — an OMS
JSON CAL message (§6.1). Delivered to all matching subscribers.
Errors: `Illegal-Argument` (bad topic / unparseable), `Illegal-State` (topic
not allowed), `Invalid-Message` (fails schema validation).

---

## Server → client operations

### `INFO <json>` — after INIT; may repeat on topology change
JSON object (Table 5.1-5):

| field | type | required | notes |
|---|---|---|---|
| `version` | string | yes | negotiated owp version, e.g. `"1.0"` |
| `server_id` | string | yes | identifier-validated |
| `uuids` | Identifiers | yes | see below |
| `system_label` | string | yes | label for the initializing service |
| `connect_urls` | array<string> | no | alternative server URLs |

Identifiers object (Table 5.1-6): `system` (string RFC-4122, yes),
`service` (string RFC-4122, yes), `subsystem` (string, no),
`capabilities` (object name→uuid, no), `components` (object name→uuid, no).

### `MSG <subscription_id> <message>`
`subscription_id` references the originating SUB; `message` is the verbatim
OMS JSON remainder. No error replies (server-generated).

### `+OK`  — verbose only
No arguments. Sent after each successful client operation **only when
`verbose=true`**. The **first `+OK` is sent before the `INFO`** on a verbose
connection — i.e. the post-INIT sequence a verbose client sees is
`+OK` then `INFO`.

### `-ERR <error_name> [details]`
Sent on verbose **and** non-verbose connections for violations. `error_name`
is one of (Table 5.1-7): `Unsupported-Version`, `Unsupported-Schema`,
`Unsupported-Service`, `Illegal-Operation`, `Illegal-Argument`,
`Illegal-State`, `Internal-Error`, `Invalid-Message`. `details` is an
optional implementation-specific remainder.

---

## Handshake sequence (what the client drives)

```
client → INIT {json, verbose:true}
server → +OK            (verbose only; precedes INFO)
server → INFO {json}    ← identity (system/service UUIDs, server_id, version)
... then per routed topic:
client → SUB <sid> <message_name> <topic> [group]
server → +OK            (verbose)
server → MSG <sid> <message>   (asynchronously, on match)
client → PUB <topic> <message>
server → +OK            (verbose)
client → UNSUB <sid>  /  close
```

**Fail-closed rule (plan D3):** the client treats "INIT sent, but `INFO` not
received within the bounded timeout" as a hard failure (Sleet silently drops
unregistered services — surface it loudly, naming url + service_id). Identity
is taken from `INFO`, never invented (plan D6).

---

## Decisions this pins

- **D2 (QoS):** OMSC-SPC-013 does **not** address QoS/reliability/buffering
  at all ("Not explicitly addressed"). `owp` rides a reliable WebSocket, but
  the CAL delivery/buffering/shelf-life contract is the server's, and the
  protocol earns no delivery guarantee on its own. → the plugin declares
  **`BEST_EFFORT`** by default; a deploy-time `"declare_reliability":"reliable"`
  config override is the only way to claim RELIABLE (mirrors the UDP
  downgrade convention). Resolved: default BEST_EFFORT.
- **Vocabulary:** Sleet **validates** every PUB/MSG OMS-JSON body against the
  UCI XSD (`schema_version="002.5.0"`,
  `UCI_MessageDefinitions_v2_5_0.xsd`) and `message_name` must be a UCI
  **global element**. → the real-Sleet E2E path (Phases 3–4) must use
  **kit-native UCI messages**, not the hand-authored `agra_example`
  (`MA_*`) vocabulary, which is a UCI-2.3-based A-GRA extension not in the
  OMS-2.5 UCI set. **Phases 1–2 are unaffected**: they run against an
  in-process mock server we control, which does not schema-validate.
