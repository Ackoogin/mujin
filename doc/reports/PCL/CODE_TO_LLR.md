# PCL Code-to-LLR Traceability Matrix

> **Generated file.** Regenerate with `python3 subprojects/PCL/scripts/gen_code_trace.py`.
> Do not hand-edit; fix the source annotations or `LLR.md` and regenerate.

Generated: 2026-07-18

Direct trace from production source code to low-level requirements (GAP-C-08). Every non-static function in `subprojects/PCL/src/*.c` carries either an `Implements: REQ_PCL_NNN` comment or a reviewed `No LLR:` justification naming the requirements that cover it indirectly. Static helpers are tagged when they carry requirement behaviour of their own; otherwise they inherit their callers' trace. Tags in the C++ wrapper headers (`include/pcl/*.hpp`) are collected for the reverse index, but the strict function-inventory check applies to the C production sources only, which form the certified boundary (see `DO178C_GAP_ANALYSIS.md`, section 4).

## Summary

| Metric | Count |
|--------|-------|
| C production functions (incl. static) | 437 |
| Functions with Implements tags | 251 |
| Non-static functions with a reviewed No-LLR justification | 19 |
| Non-static functions with neither tag nor justification | 0 |
| LLRs | 444 |
| LLRs implemented by at least one tagged function | 440 |
| LLRs exempt (test-only / documentation-only) | 4 |
| LLRs with no implementation tag and no exemption | 0 |
| Tags naming unknown LLRs | 0 |

## Function to LLR (by source file)

### `pcl_alloc.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `pcl_alloc` | external | 213 |
| `pcl_calloc` | external | 214 |
| `pcl_realloc` | external | 105, 106, 215 |
| `pcl_free` | external | 104, 216 |

### `pcl_bridge.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `bridge_sub_cb` | static | 079, 080, 081, 083, 328 |
| `bridge_on_configure` | static | 082 |
| `pcl_bridge_create` | external | 075, 076, 089 |
| `pcl_bridge_container` | external | 077 |
| `pcl_bridge_destroy` | external | 078, 096 |

### `pcl_capabilities.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `pcl_transport_caps_from_vtable` | external | 347, 348, 349, 350, 351, 352 |
| `pcl_endpoint_required_caps` | external | 363, 375 |
| `pcl_transport_caps_supports` | external | 364 |
| `pcl_qos_satisfies` | external | 370 |
| `pcl_qos_reliability_name` | external | 376 |

### `pcl_codec_registry.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `find_codec_at` | static | 343 |
| `contains_codec` | static | 342 |
| `reserve_codec_slots` | static | 339 |
| `pcl_codec_registry_create` | external | *No LLR (justified in source)* |
| `pcl_codec_registry_destroy` | external | *No LLR (justified in source)* |
| `pcl_codec_registry_default` | external | 345 |
| `pcl_codec_registry_clear` | external | 220 |
| `pcl_codec_registry_register` | external | 339, 340, 341, 342 |
| `pcl_codec_registry_get` | external | 219, 337, 338, 341 |
| `pcl_codec_registry_get_at` | external | 219, 343 |
| `pcl_codec_registry_count` | external | 344 |

### `pcl_container.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `find_or_add_param` | static | 016 |
| `validate_port_definition` | static | 020, 021, 022 |
| `copy_route_config` | static | 235 |
| `pcl_container_create` | external | 001, 002, 028, 114 |
| `pcl_container_destroy` | external | 003, 111 |
| `pcl_container_configure` | external | 004, 005, 007, 008 |
| `pcl_container_activate` | external | 004, 005, 008, 011 |
| `pcl_container_deactivate` | external | 004, 005, 008, 009 |
| `pcl_container_cleanup` | external | 004, 005, 008, 010, 012 |
| `pcl_container_shutdown` | external | 004, 006 |
| `pcl_container_state` | external | 111 |
| `pcl_container_name` | external | 111 |
| `pcl_container_set_tick_rate_hz` | external | 029, 030 |
| `pcl_container_get_tick_rate_hz` | external | 029, 111 |
| `pcl_container_set_param_str` | external | 013, 015, 018 |
| `pcl_container_set_param_f64` | external | 014, 015, 018 |
| `pcl_container_set_param_i64` | external | 014, 015, 018 |
| `pcl_container_set_param_bool` | external | 014, 015, 018 |
| `pcl_container_get_param_str` | external | 013, 017, 018 |
| `pcl_container_get_param_f64` | external | 014, 017, 018 |
| `pcl_container_get_param_i64` | external | 014, 017, 018 |
| `pcl_container_get_param_bool` | external | 014, 017, 018 |
| `pcl_container_add_publisher` | external | 019, 020, 021, 022 |
| `pcl_container_add_subscriber` | external | 019, 020, 021, 022 |
| `pcl_container_add_service` | external | 019, 020, 021, 022 |
| `pcl_port_publish` | external | 023, 024, 025, 026, 027, 174 |
| `pcl_container_invoke_async` | external | *No LLR (justified in source)* |
| `pcl_service_respond` | external | 459, 460, 461 |
| `pcl_service_context_free` | external | 223 |
| `pcl_container_add_stream_service` | external | 172 |
| `pcl_port_set_route` | external | 235 |
| `pcl_stream_send` | external | 098, 167, 171 |
| `pcl_stream_end` | external | 097, 171 |
| `pcl_stream_abort` | external | 169, 171 |
| `pcl_stream_is_cancelled` | external | 168, 171 |
| `pcl_stream_cancel` | external | 168, 171 |

### `pcl_executor.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `free_pending_msg` | static | 052 |
| `port_route_mode` | static | 315, 316, 326, 327 |
| `subscribe_port_with_transport` | static | 495, 497 |
| `subscribe_container_ports` | static | 236, 495 |
| `subscribe_existing_ports` | static | 229, 496 |
| `route_accepts` | static | 173 |
| `peer_is_allowed` | static | 173 |
| `pcl_executor_create` | external | 031 |
| `pcl_executor_containers_lock` | external | *No LLR (justified in source)* |
| `pcl_executor_containers_unlock` | external | *No LLR (justified in source)* |
| `pcl_executor_destroy` | external | 052, 061, 094, 185 |
| `pcl_executor_add` | external | 037, 038, 230, 236, 495 |
| `pcl_executor_remove` | external | 039 |
| `dispatch_incoming_now` | static | 044, 045, 046, 173 |
| `drain_resp_cb_queue` | static | 057, 058, 059 |
| `drain_incoming_queue` | static | 054 |
| `drain_svc_req_queue` | static | 181, 182, 183, 459 |
| `tick_container` | static | 032, 033, 034 |
| `pcl_executor_spin` | external | 035, 055 |
| `pcl_executor_spin_once` | external | 032, 043, 091, 092, 093 |
| `pcl_executor_request_shutdown` | external | 035, 113 |
| `pcl_executor_shutdown_graceful` | external | 036, 041, 042 |
| `pcl_executor_set_transport_caps` | external | 062, 063, 229, 230, 384 |
| `pcl_executor_set_transport` | external | 062, 063, 384 |
| `pcl_executor_get_transport` | external | *No LLR (justified in source)* |
| `pcl_executor_get_transport_for_peer` | external | *No LLR (justified in source)* |
| `pcl_executor_register_transport_caps` | external | 178, 230, 425, 496 |
| `pcl_executor_register_transport` | external | 178, 230, 425, 496 |
| `pcl_executor_set_transport_qos` | external | 377 |
| `pcl_executor_register_transport_qos` | external | 378 |
| `validate_one_transport` | static | 365, 366, 367, 371, 372, 373, 374 |
| `pcl_executor_validate_endpoint_route` | external | 365, 366, 367, 368, 369, 371, 372, 373, 374, 379, 380 |
| `pcl_executor_set_endpoint_route` | external | *No LLR (justified in source)* |
| `pcl_executor_endpoint_route_exists` | external | *No LLR (justified in source)* |
| `pcl_executor_endpoint_route_exists_any_kind` | external | 474, 475 |
| `pcl_executor_clear_endpoint_route` | external | *No LLR (justified in source)* |
| `pcl_executor_dispatch_incoming` | external | 044, 046 |
| `find_service` | external | 040, 173 |
| `find_stream_service` | external | 170 |
| `pcl_executor_invoke_service` | external | 040 |
| `pcl_executor_invoke_service_remote` | external | 163, 177 |
| `pcl_executor_publish_port` | external | 027, 174 |
| `pcl_executor_publish` | external | 047, 048 |
| `pcl_executor_invoke_async` | external | 164, 165, 166, 175, 473 |
| `pcl_executor_invoke_stream` | external | 170, 470, 473 |
| `pcl_executor_post_response_cb` | external | 057, 058 |
| `pcl_executor_post_response_msg` | external | 057, 058, 060, 090 |
| `enqueue_incoming_message` | static | 049, 050, 051, 085, 086, 087, 088, 231, 232 |
| `pcl_executor_set_incoming_queue_limit` | external | 498 |
| `pcl_executor_get_incoming_queue_depth` | external | 499 |
| `pcl_executor_post_incoming` | external | 049, 050, 051, 053, 056, 095 |
| `pcl_executor_post_remote_incoming` | external | 173, 201, 327 |
| `enqueue_svc_req` | static | 180, 184 |
| `pcl_executor_post_service_request` | external | 180, 183, 184 |
| `pcl_executor_post_service_request_remote` | external | 438 |

### `pcl_log.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `default_handler` | static | 073, 074 |
| `pcl_log_set_handler` | external | 068, 072 |
| `pcl_log_set_level` | external | 066, 070 |
| `pcl_log` | external | 064, 065, 066, 067, 069, 070, 071 |

### `pcl_plugin_loader.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `retain_codec_plugin` | static | 405 |
| `pcl_plugin_load_codec` | external | 385, 386, 393, 394, 395 |
| `pcl_codec_registry_load_plugins_from_paths` | external | 221, 399, 405 |
| `pcl_codec_registry_load_plugins_from_env` | external | 400 |
| `pcl_codec_registry_load_plugins_from_manifest` | external | 391, 392, 401 |
| `pcl_plugin_load_transport` | external | 383, 396, 397, 398 |
| `pcl_plugin_transport_caps` | external | 353, 354, 355 |
| `pcl_plugin_transport_qos` | external | 362 |
| `pcl_plugin_unload_transport` | external | 381, 382, 407 |
| `pcl_plugin_open` | external | 402 |
| `pcl_plugin_unload` | external | 402 |
| `pcl_plugin_symbol` | external | 383, 402 |

### `pcl_process_runtime.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `runtime_signal_handler` | static | 487 |
| `kind_name` | static | 483 |
| `make_temporary_path` | static | 485 |
| `activate_gateways` | static | 479 |
| `pcl_process_runtime_create` | external | 225, 476, 494 |
| `pcl_process_runtime_executor` | external | 225, 476 |
| `pcl_process_runtime_load_codec` | external | 225, 226, 477, 492 |
| `pcl_process_runtime_load_ports_file` | external | 478, 480, 481, 482, 483, 484, 485, 490, 493 |
| `pcl_process_runtime_run` | external | 228, 486, 487, 488, 489, 491 |
| `pcl_process_runtime_request_shutdown` | external | 225 |
| `pcl_process_runtime_error` | external | 225 |
| `pcl_process_runtime_destroy` | external | 225, 227 |

### `pcl_transport_apos.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `pcl_apos_alloc_string` | static | 325 |
| `pcl_apos_encode_frame` | static | 300 |
| `pcl_apos_decode_frame` | static | 325 |
| `pcl_apos_transport_create` | external | 298, 446, 447 |
| `pcl_apos_transport_set_peer_id` | external | *No LLR (justified in source)* |
| `pcl_apos_transport_get_transport` | external | 445 |
| `pcl_apos_transport_destroy` | external | 110 |

### `pcl_transport_routing.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `kind_from_str` | static | 428 |
| `reliability_from_str` | static | 275 |
| `handle_transport_line` | static | 275, 318, 418, 419, 423 |
| `split_endpoint_list` | static | 467 |
| `handle_exclusive_line` | static | 464, 466, 467 |
| `route_matching_list` | static | 463, 474, 475 |
| `validate_exclusivity` | static | 463, 465, 469, 474, 475 |
| `handle_route_line` | static | 275, 420, 422, 426, 427, 428, 472 |
| `pcl_transport_routing_load` | external | 210, 224, 319, 416, 417, 421, 468, 469, 474 |
| `pcl_transport_routing_destroy` | external | 209, 421, 424, 425 |
| `pcl_transport_routing_transport_count` | external | 317 |
| `pcl_transport_routing_get_gateway` | external | 471 |

### `pcl_transport_shared_memory.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `pcl_shm_find_remote_service` | static | 306, 314 |
| `pcl_shm_find_remote_stream_service` | static | 311, 314 |
| `pcl_shm_collect_local_services` | static | 208, 313 |
| `pcl_shm_pending_clear` | static | 309 |
| `pcl_shm_pending_stream_clear` | static | 207 |
| `pcl_shm_publish_once_locked` | static | 211, 233 |
| `pcl_shm_publish_with_worker_backpressure` | static | 457 |
| `pcl_shm_find_provider_slot_locked` | static | 308 |
| `pcl_shm_worker_svc_req` | static | 188, 189, 501 |
| `pcl_shm_worker_stream_open` | static | 308 |
| `pcl_shm_publish` | static | 186, 212 |
| `pcl_shm_subscribe` | static | 500 |
| `pcl_shm_invoke_async` | static | 453 |
| `pcl_shm_shutdown` | static | 305 |
| `pcl_shm_gateway_sub_cb` | static | 306, 307 |
| `pcl_shm_gateway_stream_sub_cb` | static | 311, 312 |
| `pcl_shm_handle_frame` | static | 187, 449 |
| `pcl_shared_memory_transport_create` | external | 310 |
| `pcl_shared_memory_transport_get_transport` | external | *No LLR (justified in source)* |
| `pcl_shared_memory_transport_gateway_container` | external | *No LLR (justified in source)* |
| `pcl_shared_memory_transport_set_topic_backpressure` | external | 101, 102, 103, 212 |
| `pcl_shared_memory_transport_destroy` | external | 207, 309 |

### `pcl_transport_shared_memory_plugin.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `pcl_transport_abi_version` | external | *No LLR (justified in source)* |
| `pcl_transport_plugin_caps` | external | 358 |
| `pcl_transport_plugin_qos` | external | 361 |
| `pcl_transport_plugin_entry` | external | 387, 388, 406 |
| `pcl_shm_transport_plugin_gateway` | external | 387 |
| `pcl_shm_transport_plugin_destroy` | external | 383 |
| `pcl_transport_plugin_teardown` | external | 407 |

### `pcl_transport_socket.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `enable_keepalive` | static | 197 |
| `connect_with_timeout` | static | 205 |
| `try_connect_addrinfo` | static | 196, 205 |
| `fire_state_cb` | static | 194 |
| `enqueue_outbound_frame` | static | 125, 502 |
| `send_thread_main` | static | 125, 161, 503 |
| `socket_publish` | static | 125 |
| `socket_invoke_async` | static | 164, 454 |
| `gateway_sub_cb` | static | 163, 177 |
| `socket_subscribe` | static | 158 |
| `socket_shutdown` | static | 159 |
| `recv_thread_main` | static | 120, 121, 126, 163, 176, 195, 204, 412, 413, 415, 451 |
| `gateway_on_configure` | static | 122, 409 |
| `socket_transport_create_common` | static | 115, 117 |
| `pcl_socket_transport_create_server_ex` | external | 115, 116, 197 |
| `pcl_socket_transport_create_server` | external | 115, 116 |
| `pcl_socket_transport_get_port` | external | *No LLR (justified in source)* |
| `pcl_socket_transport_create_client_ex` | external | 117, 118, 119, 190, 191, 192, 194, 205, 414 |
| `pcl_socket_transport_create_client` | external | 117, 118, 119 |
| `pcl_socket_transport_get_transport` | external | 129 |
| `pcl_socket_transport_set_peer_id` | external | 179 |
| `pcl_socket_transport_gateway_container` | external | 122, 123, 124 |
| `pcl_socket_transport_get_state` | external | 193 |
| `pcl_socket_transport_dropped_publishes` | external | 234, 502 |
| `pcl_socket_transport_invoke_remote_async` | external | 126, 127, 128, 162, 410, 411 |
| `pcl_socket_transport_destroy` | external | 130, 160, 161, 503 |

### `pcl_transport_socket_plugin.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `pcl_transport_abi_version` | external | *No LLR (justified in source)* |
| `pcl_transport_plugin_caps` | external | 357 |
| `pcl_transport_plugin_qos` | external | 360 |
| `pcl_transport_plugin_entry` | external | 403, 404 |
| `pcl_socket_transport_plugin_gateway` | external | 403 |
| `pcl_socket_transport_plugin_destroy` | external | 383 |
| `pcl_transport_plugin_teardown` | external | 403 |

### `pcl_transport_template.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `tpl_outbound_clone` | static | 455, 456 |
| `tpl_send_enqueue` | static | 320, 455, 456 |
| `tpl_pending_drain` | static | 321 |
| `tpl_alias_remember_locked` | static | 286, 439 |
| `tpl_alias_remember` | static | 286, 439 |
| `tpl_svc_response_cb` | static | 438, 442 |
| `tpl_send_thread_main` | static | 290, 292, 458 |
| `tpl_recv_thread_main` | static | 293, 294, 436, 438, 448 |
| `tpl_publish` | static | 320, 455 |
| `tpl_invoke_async` | static | 320, 437, 452, 456 |
| `tpl_subscribe` | static | 295 |
| `tpl_shutdown` | static | 320 |
| `pcl_transport_template_create` | external | 429, 430, 433, 441, 442, 443 |
| `pcl_transport_template_set_peer_id` | external | 286, 432 |
| `pcl_transport_template_get_transport` | external | 433 |
| `pcl_transport_template_destroy` | external | 288, 290, 321, 431, 434, 435, 439, 440, 458 |

### `pcl_transport_udp.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `udp_record_sequence` | static | 505 |
| `udp_send_thread_main` | static | 200 |
| `udp_publish` | static | 200, 202 |
| `udp_subscribe` | static | 301 |
| `udp_shutdown` | static | 206 |
| `udp_recv_thread_main` | static | 200, 201, 303, 450, 504 |
| `udp_resolve_remote` | static | 302 |
| `pcl_udp_transport_create` | external | 099, 198, 203, 302 |
| `pcl_udp_transport_get_local_port` | external | *No LLR (justified in source)* |
| `pcl_udp_transport_received_datagrams` | external | 504 |
| `pcl_udp_transport_dropped_datagrams` | external | 505 |
| `pcl_udp_transport_set_peer_id` | external | 100, 199 |
| `pcl_udp_transport_get_transport` | external | *No LLR (justified in source)* |
| `pcl_udp_transport_destroy` | external | 099, 304 |

### `pcl_transport_udp_plugin.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `pcl_transport_abi_version` | external | *No LLR (justified in source)* |
| `pcl_transport_plugin_caps` | external | 356 |
| `pcl_transport_plugin_qos` | external | 359 |
| `pcl_transport_plugin_entry` | external | 389, 390, 408 |
| `pcl_udp_transport_plugin_destroy` | external | 383 |
| `pcl_transport_plugin_teardown` | external | 389 |

### `apos.c`

| Function | Linkage | LLR(s) |
|----------|---------|--------|
| `setupAPOS` | external | 107 |
| `setupApos` | external | 107 |
| `setDelayFunction` | external | 109 |
| `sendMessage` | external | 297 |
| `sendMessageNonBlocking` | external | 109, 444 |
| `receiveMessage` | external | 297, 324 |
| `receiveMessageNonBlocking` | external | 322, 444 |
| `waitOnMultiChannel` | external | 218, 222, 323 |
| `logEvent` | external | 108 |

### C++ wrapper headers (`include/pcl/*.hpp`)

| File | Function | LLR(s) |
|------|----------|--------|
| `component.hpp` | `Component` | 131 |
| `component.hpp` | `Component` | 132 |
| `component.hpp` | `Component` | 143 |
| `component.hpp` | `(file-level)` | 144 |
| `component.hpp` | `setTickRateHz` | 141 |
| `component.hpp` | `tickRateHz` | 141 |
| `component.hpp` | `setParam` | 137 |
| `component.hpp` | `setParam` | 138 |
| `component.hpp` | `setParam` | 139 |
| `component.hpp` | `setParam` | 140 |
| `component.hpp` | `paramStr` | 137 |
| `component.hpp` | `paramF64` | 138 |
| `component.hpp` | `paramI64` | 139 |
| `component.hpp` | `paramBool` | 140 |
| `component.hpp` | `addPublisher` | 134 |
| `component.hpp` | `addSubscriber` | 135 |
| `component.hpp` | `addService` | 136 |
| `component.hpp` | `logDebug` | 142 |
| `component.hpp` | `logInfo` | 142 |
| `component.hpp` | `logWarn` | 142 |
| `component.hpp` | `logError` | 142 |
| `component.hpp` | `on_configure` | 217 |
| `component.hpp` | `on_activate` | 217 |
| `component.hpp` | `on_deactivate` | 217 |
| `component.hpp` | `on_cleanup` | 217 |
| `component.hpp` | `on_shutdown` | 217 |
| `component.hpp` | `trampoline_configure` | 133 |
| `component.hpp` | `trampoline_activate` | 133 |
| `component.hpp` | `trampoline_deactivate` | 133 |
| `component.hpp` | `trampoline_cleanup` | 133 |
| `component.hpp` | `trampoline_shutdown` | 133 |
| `executor.hpp` | `Executor` | 145 |
| `executor.hpp` | `Executor` | 146 |
| `executor.hpp` | `Executor` | 155 |
| `executor.hpp` | `(file-level)` | 156 |
| `executor.hpp` | `add` | 147, 157 |
| `executor.hpp` | `add` | 148 |
| `executor.hpp` | `spinOnce` | 149 |
| `executor.hpp` | `requestShutdown` | 150 |
| `executor.hpp` | `shutdownGraceful` | 151 |
| `executor.hpp` | `setTransport` | 152 |
| `executor.hpp` | `dispatchIncoming` | 153 |
| `executor.hpp` | `postIncoming` | 154 |

## LLR to Function (reverse index)

| LLR | Implemented by |
|-----|----------------|
| 001 | `pcl_container_create` (pcl_container.c) |
| 002 | `pcl_container_create` (pcl_container.c) |
| 003 | `pcl_container_destroy` (pcl_container.c) |
| 004 | `pcl_container_configure` (pcl_container.c); `pcl_container_activate` (pcl_container.c); `pcl_container_deactivate` (pcl_container.c); `pcl_container_cleanup` (pcl_container.c); `pcl_container_shutdown` (pcl_container.c) |
| 005 | `pcl_container_configure` (pcl_container.c); `pcl_container_activate` (pcl_container.c); `pcl_container_deactivate` (pcl_container.c); `pcl_container_cleanup` (pcl_container.c) |
| 006 | `pcl_container_shutdown` (pcl_container.c) |
| 007 | `pcl_container_configure` (pcl_container.c) |
| 008 | `pcl_container_configure` (pcl_container.c); `pcl_container_activate` (pcl_container.c); `pcl_container_deactivate` (pcl_container.c); `pcl_container_cleanup` (pcl_container.c) |
| 009 | `pcl_container_deactivate` (pcl_container.c) |
| 010 | `pcl_container_cleanup` (pcl_container.c) |
| 011 | `pcl_container_activate` (pcl_container.c) |
| 012 | `pcl_container_cleanup` (pcl_container.c) |
| 013 | `pcl_container_set_param_str` (pcl_container.c); `pcl_container_get_param_str` (pcl_container.c) |
| 014 | `pcl_container_set_param_f64` (pcl_container.c); `pcl_container_set_param_i64` (pcl_container.c); `pcl_container_set_param_bool` (pcl_container.c); `pcl_container_get_param_f64` (pcl_container.c); `pcl_container_get_param_i64` (pcl_container.c); `pcl_container_get_param_bool` (pcl_container.c) |
| 015 | `pcl_container_set_param_str` (pcl_container.c); `pcl_container_set_param_f64` (pcl_container.c); `pcl_container_set_param_i64` (pcl_container.c); `pcl_container_set_param_bool` (pcl_container.c) |
| 016 | `find_or_add_param` (pcl_container.c) |
| 017 | `pcl_container_get_param_str` (pcl_container.c); `pcl_container_get_param_f64` (pcl_container.c); `pcl_container_get_param_i64` (pcl_container.c); `pcl_container_get_param_bool` (pcl_container.c) |
| 018 | `pcl_container_set_param_str` (pcl_container.c); `pcl_container_set_param_f64` (pcl_container.c); `pcl_container_set_param_i64` (pcl_container.c); `pcl_container_set_param_bool` (pcl_container.c); `pcl_container_get_param_str` (pcl_container.c); `pcl_container_get_param_f64` (pcl_container.c); `pcl_container_get_param_i64` (pcl_container.c); `pcl_container_get_param_bool` (pcl_container.c) |
| 019 | `pcl_container_add_publisher` (pcl_container.c); `pcl_container_add_subscriber` (pcl_container.c); `pcl_container_add_service` (pcl_container.c) |
| 020 | `validate_port_definition` (pcl_container.c); `pcl_container_add_publisher` (pcl_container.c); `pcl_container_add_subscriber` (pcl_container.c); `pcl_container_add_service` (pcl_container.c) |
| 021 | `validate_port_definition` (pcl_container.c); `pcl_container_add_publisher` (pcl_container.c); `pcl_container_add_subscriber` (pcl_container.c); `pcl_container_add_service` (pcl_container.c) |
| 022 | `validate_port_definition` (pcl_container.c); `pcl_container_add_publisher` (pcl_container.c); `pcl_container_add_subscriber` (pcl_container.c); `pcl_container_add_service` (pcl_container.c) |
| 023 | `pcl_port_publish` (pcl_container.c) |
| 024 | `pcl_port_publish` (pcl_container.c) |
| 025 | `pcl_port_publish` (pcl_container.c) |
| 026 | `pcl_port_publish` (pcl_container.c) |
| 027 | `pcl_port_publish` (pcl_container.c); `pcl_executor_publish_port` (pcl_executor.c) |
| 028 | `pcl_container_create` (pcl_container.c) |
| 029 | `pcl_container_set_tick_rate_hz` (pcl_container.c); `pcl_container_get_tick_rate_hz` (pcl_container.c) |
| 030 | `pcl_container_set_tick_rate_hz` (pcl_container.c) |
| 031 | `pcl_executor_create` (pcl_executor.c) |
| 032 | `tick_container` (pcl_executor.c); `pcl_executor_spin_once` (pcl_executor.c) |
| 033 | `tick_container` (pcl_executor.c) |
| 034 | `tick_container` (pcl_executor.c) |
| 035 | `pcl_executor_spin` (pcl_executor.c); `pcl_executor_request_shutdown` (pcl_executor.c) |
| 036 | `pcl_executor_shutdown_graceful` (pcl_executor.c) |
| 037 | `pcl_executor_add` (pcl_executor.c) |
| 038 | `pcl_executor_add` (pcl_executor.c) |
| 039 | `pcl_executor_remove` (pcl_executor.c) |
| 040 | `find_service` (pcl_executor.c); `pcl_executor_invoke_service` (pcl_executor.c) |
| 041 | `pcl_executor_shutdown_graceful` (pcl_executor.c) |
| 042 | `pcl_executor_shutdown_graceful` (pcl_executor.c) |
| 043 | `pcl_executor_spin_once` (pcl_executor.c) |
| 044 | `dispatch_incoming_now` (pcl_executor.c); `pcl_executor_dispatch_incoming` (pcl_executor.c) |
| 045 | `dispatch_incoming_now` (pcl_executor.c) |
| 046 | `dispatch_incoming_now` (pcl_executor.c); `pcl_executor_dispatch_incoming` (pcl_executor.c) |
| 047 | `pcl_executor_publish` (pcl_executor.c) |
| 048 | `pcl_executor_publish` (pcl_executor.c) |
| 049 | `enqueue_incoming_message` (pcl_executor.c); `pcl_executor_post_incoming` (pcl_executor.c) |
| 050 | `enqueue_incoming_message` (pcl_executor.c); `pcl_executor_post_incoming` (pcl_executor.c) |
| 051 | `enqueue_incoming_message` (pcl_executor.c); `pcl_executor_post_incoming` (pcl_executor.c) |
| 052 | `free_pending_msg` (pcl_executor.c); `pcl_executor_destroy` (pcl_executor.c) |
| 053 | `pcl_executor_post_incoming` (pcl_executor.c) |
| 054 | `drain_incoming_queue` (pcl_executor.c) |
| 055 | `pcl_executor_spin` (pcl_executor.c) |
| 056 | `pcl_executor_post_incoming` (pcl_executor.c) |
| 057 | `drain_resp_cb_queue` (pcl_executor.c); `pcl_executor_post_response_cb` (pcl_executor.c); `pcl_executor_post_response_msg` (pcl_executor.c) |
| 058 | `drain_resp_cb_queue` (pcl_executor.c); `pcl_executor_post_response_cb` (pcl_executor.c); `pcl_executor_post_response_msg` (pcl_executor.c) |
| 059 | `drain_resp_cb_queue` (pcl_executor.c) |
| 060 | `pcl_executor_post_response_msg` (pcl_executor.c) |
| 061 | `pcl_executor_destroy` (pcl_executor.c) |
| 062 | `pcl_executor_set_transport_caps` (pcl_executor.c); `pcl_executor_set_transport` (pcl_executor.c) |
| 063 | `pcl_executor_set_transport_caps` (pcl_executor.c); `pcl_executor_set_transport` (pcl_executor.c) |
| 064 | `pcl_log` (pcl_log.c) |
| 065 | `pcl_log` (pcl_log.c) |
| 066 | `pcl_log_set_level` (pcl_log.c); `pcl_log` (pcl_log.c) |
| 067 | `pcl_log` (pcl_log.c) |
| 068 | `pcl_log_set_handler` (pcl_log.c) |
| 069 | `pcl_log` (pcl_log.c) |
| 070 | `pcl_log_set_level` (pcl_log.c); `pcl_log` (pcl_log.c) |
| 071 | `pcl_log` (pcl_log.c) |
| 072 | `pcl_log_set_handler` (pcl_log.c) |
| 073 | `default_handler` (pcl_log.c) |
| 074 | `default_handler` (pcl_log.c) |
| 075 | `pcl_bridge_create` (pcl_bridge.c) |
| 076 | `pcl_bridge_create` (pcl_bridge.c) |
| 077 | `pcl_bridge_container` (pcl_bridge.c) |
| 078 | `pcl_bridge_destroy` (pcl_bridge.c) |
| 079 | `bridge_sub_cb` (pcl_bridge.c) |
| 080 | `bridge_sub_cb` (pcl_bridge.c) |
| 081 | `bridge_sub_cb` (pcl_bridge.c) |
| 082 | `bridge_on_configure` (pcl_bridge.c) |
| 083 | `bridge_sub_cb` (pcl_bridge.c) |
| 084 | *test-only* |
| 085 | `enqueue_incoming_message` (pcl_executor.c) |
| 086 | `enqueue_incoming_message` (pcl_executor.c) |
| 087 | `enqueue_incoming_message` (pcl_executor.c) |
| 088 | `enqueue_incoming_message` (pcl_executor.c) |
| 089 | `pcl_bridge_create` (pcl_bridge.c) |
| 090 | `pcl_executor_post_response_msg` (pcl_executor.c) |
| 091 | `pcl_executor_spin_once` (pcl_executor.c) |
| 092 | `pcl_executor_spin_once` (pcl_executor.c) |
| 093 | `pcl_executor_spin_once` (pcl_executor.c) |
| 094 | `pcl_executor_destroy` (pcl_executor.c) |
| 095 | `pcl_executor_post_incoming` (pcl_executor.c) |
| 096 | `pcl_bridge_destroy` (pcl_bridge.c) |
| 097 | `pcl_stream_end` (pcl_container.c) |
| 098 | `pcl_stream_send` (pcl_container.c) |
| 099 | `pcl_udp_transport_create` (pcl_transport_udp.c); `pcl_udp_transport_destroy` (pcl_transport_udp.c) |
| 100 | `pcl_udp_transport_set_peer_id` (pcl_transport_udp.c) |
| 101 | `pcl_shared_memory_transport_set_topic_backpressure` (pcl_transport_shared_memory.c) |
| 102 | `pcl_shared_memory_transport_set_topic_backpressure` (pcl_transport_shared_memory.c) |
| 103 | `pcl_shared_memory_transport_set_topic_backpressure` (pcl_transport_shared_memory.c) |
| 104 | `pcl_free` (pcl_alloc.c) |
| 105 | `pcl_realloc` (pcl_alloc.c) |
| 106 | `pcl_realloc` (pcl_alloc.c) |
| 107 | `setupAPOS` (apos.c); `setupApos` (apos.c) |
| 108 | `logEvent` (apos.c) |
| 109 | `setDelayFunction` (apos.c); `sendMessageNonBlocking` (apos.c) |
| 110 | `pcl_apos_transport_destroy` (pcl_transport_apos.c) |
| 111 | `pcl_container_destroy` (pcl_container.c); `pcl_container_state` (pcl_container.c); `pcl_container_name` (pcl_container.c); `pcl_container_get_tick_rate_hz` (pcl_container.c) |
| 112 | *test-only* |
| 113 | `pcl_executor_request_shutdown` (pcl_executor.c) |
| 114 | `pcl_container_create` (pcl_container.c) |
| 115 | `socket_transport_create_common` (pcl_transport_socket.c); `pcl_socket_transport_create_server_ex` (pcl_transport_socket.c); `pcl_socket_transport_create_server` (pcl_transport_socket.c) |
| 116 | `pcl_socket_transport_create_server_ex` (pcl_transport_socket.c); `pcl_socket_transport_create_server` (pcl_transport_socket.c) |
| 117 | `socket_transport_create_common` (pcl_transport_socket.c); `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c); `pcl_socket_transport_create_client` (pcl_transport_socket.c) |
| 118 | `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c); `pcl_socket_transport_create_client` (pcl_transport_socket.c) |
| 119 | `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c); `pcl_socket_transport_create_client` (pcl_transport_socket.c) |
| 120 | `recv_thread_main` (pcl_transport_socket.c) |
| 121 | `recv_thread_main` (pcl_transport_socket.c) |
| 122 | `gateway_on_configure` (pcl_transport_socket.c); `pcl_socket_transport_gateway_container` (pcl_transport_socket.c) |
| 123 | `pcl_socket_transport_gateway_container` (pcl_transport_socket.c) |
| 124 | `pcl_socket_transport_gateway_container` (pcl_transport_socket.c) |
| 125 | `enqueue_outbound_frame` (pcl_transport_socket.c); `send_thread_main` (pcl_transport_socket.c); `socket_publish` (pcl_transport_socket.c) |
| 126 | `recv_thread_main` (pcl_transport_socket.c); `pcl_socket_transport_invoke_remote_async` (pcl_transport_socket.c) |
| 127 | `pcl_socket_transport_invoke_remote_async` (pcl_transport_socket.c) |
| 128 | `pcl_socket_transport_invoke_remote_async` (pcl_transport_socket.c) |
| 129 | `pcl_socket_transport_get_transport` (pcl_transport_socket.c) |
| 130 | `pcl_socket_transport_destroy` (pcl_transport_socket.c) |
| 131 | `Component` (component.hpp) |
| 132 | `Component` (component.hpp) |
| 133 | `trampoline_configure` (component.hpp); `trampoline_activate` (component.hpp); `trampoline_deactivate` (component.hpp); `trampoline_cleanup` (component.hpp); `trampoline_shutdown` (component.hpp) |
| 134 | `addPublisher` (component.hpp) |
| 135 | `addSubscriber` (component.hpp) |
| 136 | `addService` (component.hpp) |
| 137 | `setParam` (component.hpp); `paramStr` (component.hpp) |
| 138 | `setParam` (component.hpp); `paramF64` (component.hpp) |
| 139 | `setParam` (component.hpp); `paramI64` (component.hpp) |
| 140 | `setParam` (component.hpp); `paramBool` (component.hpp) |
| 141 | `setTickRateHz` (component.hpp); `tickRateHz` (component.hpp) |
| 142 | `logDebug` (component.hpp); `logInfo` (component.hpp); `logWarn` (component.hpp); `logError` (component.hpp) |
| 143 | `Component` (component.hpp) |
| 144 | `(file-level)` (component.hpp) |
| 145 | `Executor` (executor.hpp) |
| 146 | `Executor` (executor.hpp) |
| 147 | `add` (executor.hpp) |
| 148 | `add` (executor.hpp) |
| 149 | `spinOnce` (executor.hpp) |
| 150 | `requestShutdown` (executor.hpp) |
| 151 | `shutdownGraceful` (executor.hpp) |
| 152 | `setTransport` (executor.hpp) |
| 153 | `dispatchIncoming` (executor.hpp) |
| 154 | `postIncoming` (executor.hpp) |
| 155 | `Executor` (executor.hpp) |
| 156 | `(file-level)` (executor.hpp) |
| 157 | `add` (executor.hpp) |
| 158 | `socket_subscribe` (pcl_transport_socket.c) |
| 159 | `socket_shutdown` (pcl_transport_socket.c) |
| 160 | `pcl_socket_transport_destroy` (pcl_transport_socket.c) |
| 161 | `send_thread_main` (pcl_transport_socket.c); `pcl_socket_transport_destroy` (pcl_transport_socket.c) |
| 162 | `pcl_socket_transport_invoke_remote_async` (pcl_transport_socket.c) |
| 163 | `pcl_executor_invoke_service_remote` (pcl_executor.c); `gateway_sub_cb` (pcl_transport_socket.c); `recv_thread_main` (pcl_transport_socket.c) |
| 164 | `pcl_executor_invoke_async` (pcl_executor.c); `socket_invoke_async` (pcl_transport_socket.c) |
| 165 | `pcl_executor_invoke_async` (pcl_executor.c) |
| 166 | `pcl_executor_invoke_async` (pcl_executor.c) |
| 167 | `pcl_stream_send` (pcl_container.c) |
| 168 | `pcl_stream_is_cancelled` (pcl_container.c); `pcl_stream_cancel` (pcl_container.c) |
| 169 | `pcl_stream_abort` (pcl_container.c) |
| 170 | `find_stream_service` (pcl_executor.c); `pcl_executor_invoke_stream` (pcl_executor.c) |
| 171 | `pcl_stream_send` (pcl_container.c); `pcl_stream_end` (pcl_container.c); `pcl_stream_abort` (pcl_container.c); `pcl_stream_is_cancelled` (pcl_container.c); `pcl_stream_cancel` (pcl_container.c) |
| 172 | `pcl_container_add_stream_service` (pcl_container.c) |
| 173 | `route_accepts` (pcl_executor.c); `peer_is_allowed` (pcl_executor.c); `dispatch_incoming_now` (pcl_executor.c); `find_service` (pcl_executor.c); `pcl_executor_post_remote_incoming` (pcl_executor.c) |
| 174 | `pcl_port_publish` (pcl_container.c); `pcl_executor_publish_port` (pcl_executor.c) |
| 175 | `pcl_executor_invoke_async` (pcl_executor.c) |
| 176 | `recv_thread_main` (pcl_transport_socket.c) |
| 177 | `pcl_executor_invoke_service_remote` (pcl_executor.c); `gateway_sub_cb` (pcl_transport_socket.c) |
| 178 | `pcl_executor_register_transport_caps` (pcl_executor.c); `pcl_executor_register_transport` (pcl_executor.c) |
| 179 | `pcl_socket_transport_set_peer_id` (pcl_transport_socket.c) |
| 180 | `enqueue_svc_req` (pcl_executor.c); `pcl_executor_post_service_request` (pcl_executor.c) |
| 181 | `drain_svc_req_queue` (pcl_executor.c) |
| 182 | `drain_svc_req_queue` (pcl_executor.c) |
| 183 | `drain_svc_req_queue` (pcl_executor.c); `pcl_executor_post_service_request` (pcl_executor.c) |
| 184 | `enqueue_svc_req` (pcl_executor.c); `pcl_executor_post_service_request` (pcl_executor.c) |
| 185 | `pcl_executor_destroy` (pcl_executor.c) |
| 186 | `pcl_shm_publish` (pcl_transport_shared_memory.c) |
| 187 | `pcl_shm_handle_frame` (pcl_transport_shared_memory.c) |
| 188 | `pcl_shm_worker_svc_req` (pcl_transport_shared_memory.c) |
| 189 | `pcl_shm_worker_svc_req` (pcl_transport_shared_memory.c) |
| 190 | `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c) |
| 191 | `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c) |
| 192 | `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c) |
| 193 | `pcl_socket_transport_get_state` (pcl_transport_socket.c) |
| 194 | `fire_state_cb` (pcl_transport_socket.c); `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c) |
| 195 | `recv_thread_main` (pcl_transport_socket.c) |
| 196 | `try_connect_addrinfo` (pcl_transport_socket.c) |
| 197 | `enable_keepalive` (pcl_transport_socket.c); `pcl_socket_transport_create_server_ex` (pcl_transport_socket.c) |
| 198 | `pcl_udp_transport_create` (pcl_transport_udp.c) |
| 199 | `pcl_udp_transport_set_peer_id` (pcl_transport_udp.c) |
| 200 | `udp_send_thread_main` (pcl_transport_udp.c); `udp_publish` (pcl_transport_udp.c); `udp_recv_thread_main` (pcl_transport_udp.c) |
| 201 | `pcl_executor_post_remote_incoming` (pcl_executor.c); `udp_recv_thread_main` (pcl_transport_udp.c) |
| 202 | `udp_publish` (pcl_transport_udp.c) |
| 203 | `pcl_udp_transport_create` (pcl_transport_udp.c) |
| 204 | `recv_thread_main` (pcl_transport_socket.c) |
| 205 | `connect_with_timeout` (pcl_transport_socket.c); `try_connect_addrinfo` (pcl_transport_socket.c); `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c) |
| 206 | `udp_shutdown` (pcl_transport_udp.c) |
| 207 | `pcl_shm_pending_stream_clear` (pcl_transport_shared_memory.c); `pcl_shared_memory_transport_destroy` (pcl_transport_shared_memory.c) |
| 208 | `pcl_shm_collect_local_services` (pcl_transport_shared_memory.c) |
| 209 | `pcl_transport_routing_destroy` (pcl_transport_routing.c) |
| 210 | `pcl_transport_routing_load` (pcl_transport_routing.c) |
| 211 | `pcl_shm_publish_once_locked` (pcl_transport_shared_memory.c) |
| 212 | `pcl_shm_publish` (pcl_transport_shared_memory.c); `pcl_shared_memory_transport_set_topic_backpressure` (pcl_transport_shared_memory.c) |
| 213 | `pcl_alloc` (pcl_alloc.c) |
| 214 | `pcl_calloc` (pcl_alloc.c) |
| 215 | `pcl_realloc` (pcl_alloc.c) |
| 216 | `pcl_free` (pcl_alloc.c) |
| 217 | `on_configure` (component.hpp); `on_activate` (component.hpp); `on_deactivate` (component.hpp); `on_cleanup` (component.hpp); `on_shutdown` (component.hpp) |
| 218 | `waitOnMultiChannel` (apos.c) |
| 219 | `pcl_codec_registry_get` (pcl_codec_registry.c); `pcl_codec_registry_get_at` (pcl_codec_registry.c) |
| 220 | `pcl_codec_registry_clear` (pcl_codec_registry.c) |
| 221 | `pcl_codec_registry_load_plugins_from_paths` (pcl_plugin_loader.c) |
| 222 | `waitOnMultiChannel` (apos.c) |
| 223 | `pcl_service_context_free` (pcl_container.c) |
| 224 | `pcl_transport_routing_load` (pcl_transport_routing.c) |
| 225 | `pcl_process_runtime_create` (pcl_process_runtime.c); `pcl_process_runtime_executor` (pcl_process_runtime.c); `pcl_process_runtime_load_codec` (pcl_process_runtime.c); `pcl_process_runtime_request_shutdown` (pcl_process_runtime.c); `pcl_process_runtime_error` (pcl_process_runtime.c); `pcl_process_runtime_destroy` (pcl_process_runtime.c) |
| 226 | `pcl_process_runtime_load_codec` (pcl_process_runtime.c) |
| 227 | `pcl_process_runtime_destroy` (pcl_process_runtime.c) |
| 228 | `pcl_process_runtime_run` (pcl_process_runtime.c) |
| 229 | `subscribe_existing_ports` (pcl_executor.c); `pcl_executor_set_transport_caps` (pcl_executor.c) |
| 230 | `pcl_executor_add` (pcl_executor.c); `pcl_executor_set_transport_caps` (pcl_executor.c); `pcl_executor_register_transport_caps` (pcl_executor.c); `pcl_executor_register_transport` (pcl_executor.c) |
| 231 | `enqueue_incoming_message` (pcl_executor.c) |
| 232 | `enqueue_incoming_message` (pcl_executor.c) |
| 233 | `pcl_shm_publish_once_locked` (pcl_transport_shared_memory.c) |
| 234 | `pcl_socket_transport_dropped_publishes` (pcl_transport_socket.c) |
| 235 | `copy_route_config` (pcl_container.c); `pcl_port_set_route` (pcl_container.c) |
| 236 | `subscribe_container_ports` (pcl_executor.c); `pcl_executor_add` (pcl_executor.c) |
| 275 | `reliability_from_str` (pcl_transport_routing.c); `handle_transport_line` (pcl_transport_routing.c); `handle_route_line` (pcl_transport_routing.c) |
| 286 | `tpl_alias_remember_locked` (pcl_transport_template.c); `tpl_alias_remember` (pcl_transport_template.c); `pcl_transport_template_set_peer_id` (pcl_transport_template.c) |
| 288 | `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 290 | `tpl_send_thread_main` (pcl_transport_template.c); `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 292 | `tpl_send_thread_main` (pcl_transport_template.c) |
| 293 | `tpl_recv_thread_main` (pcl_transport_template.c) |
| 294 | `tpl_recv_thread_main` (pcl_transport_template.c) |
| 295 | `tpl_subscribe` (pcl_transport_template.c) |
| 297 | `sendMessage` (apos.c); `receiveMessage` (apos.c) |
| 298 | `pcl_apos_transport_create` (pcl_transport_apos.c) |
| 300 | `pcl_apos_encode_frame` (pcl_transport_apos.c) |
| 301 | `udp_subscribe` (pcl_transport_udp.c) |
| 302 | `udp_resolve_remote` (pcl_transport_udp.c); `pcl_udp_transport_create` (pcl_transport_udp.c) |
| 303 | `udp_recv_thread_main` (pcl_transport_udp.c) |
| 304 | `pcl_udp_transport_destroy` (pcl_transport_udp.c) |
| 305 | `pcl_shm_shutdown` (pcl_transport_shared_memory.c) |
| 306 | `pcl_shm_find_remote_service` (pcl_transport_shared_memory.c); `pcl_shm_gateway_sub_cb` (pcl_transport_shared_memory.c) |
| 307 | `pcl_shm_gateway_sub_cb` (pcl_transport_shared_memory.c) |
| 308 | `pcl_shm_find_provider_slot_locked` (pcl_transport_shared_memory.c); `pcl_shm_worker_stream_open` (pcl_transport_shared_memory.c) |
| 309 | `pcl_shm_pending_clear` (pcl_transport_shared_memory.c); `pcl_shared_memory_transport_destroy` (pcl_transport_shared_memory.c) |
| 310 | `pcl_shared_memory_transport_create` (pcl_transport_shared_memory.c) |
| 311 | `pcl_shm_find_remote_stream_service` (pcl_transport_shared_memory.c); `pcl_shm_gateway_stream_sub_cb` (pcl_transport_shared_memory.c) |
| 312 | `pcl_shm_gateway_stream_sub_cb` (pcl_transport_shared_memory.c) |
| 313 | `pcl_shm_collect_local_services` (pcl_transport_shared_memory.c) |
| 314 | `pcl_shm_find_remote_service` (pcl_transport_shared_memory.c); `pcl_shm_find_remote_stream_service` (pcl_transport_shared_memory.c) |
| 315 | `port_route_mode` (pcl_executor.c) |
| 316 | `port_route_mode` (pcl_executor.c) |
| 317 | `pcl_transport_routing_transport_count` (pcl_transport_routing.c) |
| 318 | `handle_transport_line` (pcl_transport_routing.c) |
| 319 | `pcl_transport_routing_load` (pcl_transport_routing.c) |
| 320 | `tpl_send_enqueue` (pcl_transport_template.c); `tpl_publish` (pcl_transport_template.c); `tpl_invoke_async` (pcl_transport_template.c); `tpl_shutdown` (pcl_transport_template.c) |
| 321 | `tpl_pending_drain` (pcl_transport_template.c); `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 322 | `receiveMessageNonBlocking` (apos.c) |
| 323 | `waitOnMultiChannel` (apos.c) |
| 324 | `receiveMessage` (apos.c) |
| 325 | `pcl_apos_alloc_string` (pcl_transport_apos.c); `pcl_apos_decode_frame` (pcl_transport_apos.c) |
| 326 | `port_route_mode` (pcl_executor.c) |
| 327 | `port_route_mode` (pcl_executor.c); `pcl_executor_post_remote_incoming` (pcl_executor.c) |
| 328 | `bridge_sub_cb` (pcl_bridge.c) |
| 337 | `pcl_codec_registry_get` (pcl_codec_registry.c) |
| 338 | `pcl_codec_registry_get` (pcl_codec_registry.c) |
| 339 | `reserve_codec_slots` (pcl_codec_registry.c); `pcl_codec_registry_register` (pcl_codec_registry.c) |
| 340 | `pcl_codec_registry_register` (pcl_codec_registry.c) |
| 341 | `pcl_codec_registry_register` (pcl_codec_registry.c); `pcl_codec_registry_get` (pcl_codec_registry.c) |
| 342 | `contains_codec` (pcl_codec_registry.c); `pcl_codec_registry_register` (pcl_codec_registry.c) |
| 343 | `find_codec_at` (pcl_codec_registry.c); `pcl_codec_registry_get_at` (pcl_codec_registry.c) |
| 344 | `pcl_codec_registry_count` (pcl_codec_registry.c) |
| 345 | `pcl_codec_registry_default` (pcl_codec_registry.c) |
| 346 | *test-only* |
| 347 | `pcl_transport_caps_from_vtable` (pcl_capabilities.c) |
| 348 | `pcl_transport_caps_from_vtable` (pcl_capabilities.c) |
| 349 | `pcl_transport_caps_from_vtable` (pcl_capabilities.c) |
| 350 | `pcl_transport_caps_from_vtable` (pcl_capabilities.c) |
| 351 | `pcl_transport_caps_from_vtable` (pcl_capabilities.c) |
| 352 | `pcl_transport_caps_from_vtable` (pcl_capabilities.c) |
| 353 | `pcl_plugin_transport_caps` (pcl_plugin_loader.c) |
| 354 | `pcl_plugin_transport_caps` (pcl_plugin_loader.c) |
| 355 | `pcl_plugin_transport_caps` (pcl_plugin_loader.c) |
| 356 | `pcl_transport_plugin_caps` (pcl_transport_udp_plugin.c) |
| 357 | `pcl_transport_plugin_caps` (pcl_transport_socket_plugin.c) |
| 358 | `pcl_transport_plugin_caps` (pcl_transport_shared_memory_plugin.c) |
| 359 | `pcl_transport_plugin_qos` (pcl_transport_udp_plugin.c) |
| 360 | `pcl_transport_plugin_qos` (pcl_transport_socket_plugin.c) |
| 361 | `pcl_transport_plugin_qos` (pcl_transport_shared_memory_plugin.c) |
| 362 | `pcl_plugin_transport_qos` (pcl_plugin_loader.c) |
| 363 | `pcl_endpoint_required_caps` (pcl_capabilities.c) |
| 364 | `pcl_transport_caps_supports` (pcl_capabilities.c) |
| 365 | `validate_one_transport` (pcl_executor.c); `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 366 | `validate_one_transport` (pcl_executor.c); `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 367 | `validate_one_transport` (pcl_executor.c); `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 368 | `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 369 | `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 370 | `pcl_qos_satisfies` (pcl_capabilities.c) |
| 371 | `validate_one_transport` (pcl_executor.c); `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 372 | `validate_one_transport` (pcl_executor.c); `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 373 | `validate_one_transport` (pcl_executor.c); `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 374 | `validate_one_transport` (pcl_executor.c); `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 375 | `pcl_endpoint_required_caps` (pcl_capabilities.c) |
| 376 | `pcl_qos_reliability_name` (pcl_capabilities.c) |
| 377 | `pcl_executor_set_transport_qos` (pcl_executor.c) |
| 378 | `pcl_executor_register_transport_qos` (pcl_executor.c) |
| 379 | `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 380 | `pcl_executor_validate_endpoint_route` (pcl_executor.c) |
| 381 | `pcl_plugin_unload_transport` (pcl_plugin_loader.c) |
| 382 | `pcl_plugin_unload_transport` (pcl_plugin_loader.c) |
| 383 | `pcl_plugin_load_transport` (pcl_plugin_loader.c); `pcl_plugin_symbol` (pcl_plugin_loader.c); `pcl_shm_transport_plugin_destroy` (pcl_transport_shared_memory_plugin.c); `pcl_socket_transport_plugin_destroy` (pcl_transport_socket_plugin.c); `pcl_udp_transport_plugin_destroy` (pcl_transport_udp_plugin.c) |
| 384 | `pcl_executor_set_transport_caps` (pcl_executor.c); `pcl_executor_set_transport` (pcl_executor.c) |
| 385 | `pcl_plugin_load_codec` (pcl_plugin_loader.c) |
| 386 | `pcl_plugin_load_codec` (pcl_plugin_loader.c) |
| 387 | `pcl_transport_plugin_entry` (pcl_transport_shared_memory_plugin.c); `pcl_shm_transport_plugin_gateway` (pcl_transport_shared_memory_plugin.c) |
| 388 | `pcl_transport_plugin_entry` (pcl_transport_shared_memory_plugin.c) |
| 389 | `pcl_transport_plugin_entry` (pcl_transport_udp_plugin.c); `pcl_transport_plugin_teardown` (pcl_transport_udp_plugin.c) |
| 390 | `pcl_transport_plugin_entry` (pcl_transport_udp_plugin.c) |
| 391 | `pcl_codec_registry_load_plugins_from_manifest` (pcl_plugin_loader.c) |
| 392 | `pcl_codec_registry_load_plugins_from_manifest` (pcl_plugin_loader.c) |
| 393 | `pcl_plugin_load_codec` (pcl_plugin_loader.c) |
| 394 | `pcl_plugin_load_codec` (pcl_plugin_loader.c) |
| 395 | `pcl_plugin_load_codec` (pcl_plugin_loader.c) |
| 396 | `pcl_plugin_load_transport` (pcl_plugin_loader.c) |
| 397 | `pcl_plugin_load_transport` (pcl_plugin_loader.c) |
| 398 | `pcl_plugin_load_transport` (pcl_plugin_loader.c) |
| 399 | `pcl_codec_registry_load_plugins_from_paths` (pcl_plugin_loader.c) |
| 400 | `pcl_codec_registry_load_plugins_from_env` (pcl_plugin_loader.c) |
| 401 | `pcl_codec_registry_load_plugins_from_manifest` (pcl_plugin_loader.c) |
| 402 | `pcl_plugin_open` (pcl_plugin_loader.c); `pcl_plugin_unload` (pcl_plugin_loader.c); `pcl_plugin_symbol` (pcl_plugin_loader.c) |
| 403 | `pcl_transport_plugin_entry` (pcl_transport_socket_plugin.c); `pcl_socket_transport_plugin_gateway` (pcl_transport_socket_plugin.c); `pcl_transport_plugin_teardown` (pcl_transport_socket_plugin.c) |
| 404 | `pcl_transport_plugin_entry` (pcl_transport_socket_plugin.c) |
| 405 | `retain_codec_plugin` (pcl_plugin_loader.c); `pcl_codec_registry_load_plugins_from_paths` (pcl_plugin_loader.c) |
| 406 | `pcl_transport_plugin_entry` (pcl_transport_shared_memory_plugin.c) |
| 407 | `pcl_plugin_unload_transport` (pcl_plugin_loader.c); `pcl_transport_plugin_teardown` (pcl_transport_shared_memory_plugin.c) |
| 408 | `pcl_transport_plugin_entry` (pcl_transport_udp_plugin.c) |
| 409 | `gateway_on_configure` (pcl_transport_socket.c) |
| 410 | `pcl_socket_transport_invoke_remote_async` (pcl_transport_socket.c) |
| 411 | `pcl_socket_transport_invoke_remote_async` (pcl_transport_socket.c) |
| 412 | `recv_thread_main` (pcl_transport_socket.c) |
| 413 | `recv_thread_main` (pcl_transport_socket.c) |
| 414 | `pcl_socket_transport_create_client_ex` (pcl_transport_socket.c) |
| 415 | `recv_thread_main` (pcl_transport_socket.c) |
| 416 | `pcl_transport_routing_load` (pcl_transport_routing.c) |
| 417 | `pcl_transport_routing_load` (pcl_transport_routing.c) |
| 418 | `handle_transport_line` (pcl_transport_routing.c) |
| 419 | `handle_transport_line` (pcl_transport_routing.c) |
| 420 | `handle_route_line` (pcl_transport_routing.c) |
| 421 | `pcl_transport_routing_load` (pcl_transport_routing.c); `pcl_transport_routing_destroy` (pcl_transport_routing.c) |
| 422 | `handle_route_line` (pcl_transport_routing.c) |
| 423 | `handle_transport_line` (pcl_transport_routing.c) |
| 424 | `pcl_transport_routing_destroy` (pcl_transport_routing.c) |
| 425 | `pcl_executor_register_transport_caps` (pcl_executor.c); `pcl_executor_register_transport` (pcl_executor.c); `pcl_transport_routing_destroy` (pcl_transport_routing.c) |
| 426 | `handle_route_line` (pcl_transport_routing.c) |
| 427 | `handle_route_line` (pcl_transport_routing.c) |
| 428 | `kind_from_str` (pcl_transport_routing.c); `handle_route_line` (pcl_transport_routing.c) |
| 429 | `pcl_transport_template_create` (pcl_transport_template.c) |
| 430 | `pcl_transport_template_create` (pcl_transport_template.c) |
| 431 | `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 432 | `pcl_transport_template_set_peer_id` (pcl_transport_template.c) |
| 433 | `pcl_transport_template_create` (pcl_transport_template.c); `pcl_transport_template_get_transport` (pcl_transport_template.c) |
| 434 | `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 435 | `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 436 | `tpl_recv_thread_main` (pcl_transport_template.c) |
| 437 | `tpl_invoke_async` (pcl_transport_template.c) |
| 438 | `pcl_executor_post_service_request_remote` (pcl_executor.c); `tpl_svc_response_cb` (pcl_transport_template.c); `tpl_recv_thread_main` (pcl_transport_template.c) |
| 439 | `tpl_alias_remember_locked` (pcl_transport_template.c); `tpl_alias_remember` (pcl_transport_template.c); `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 440 | `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 441 | `pcl_transport_template_create` (pcl_transport_template.c) |
| 442 | `tpl_svc_response_cb` (pcl_transport_template.c); `pcl_transport_template_create` (pcl_transport_template.c) |
| 443 | `pcl_transport_template_create` (pcl_transport_template.c) |
| 444 | `sendMessageNonBlocking` (apos.c); `receiveMessageNonBlocking` (apos.c) |
| 445 | `pcl_apos_transport_get_transport` (pcl_transport_apos.c) |
| 446 | `pcl_apos_transport_create` (pcl_transport_apos.c) |
| 447 | `pcl_apos_transport_create` (pcl_transport_apos.c) |
| 448 | `tpl_recv_thread_main` (pcl_transport_template.c) |
| 449 | `pcl_shm_handle_frame` (pcl_transport_shared_memory.c) |
| 450 | `udp_recv_thread_main` (pcl_transport_udp.c) |
| 451 | `recv_thread_main` (pcl_transport_socket.c) |
| 452 | `tpl_invoke_async` (pcl_transport_template.c) |
| 453 | `pcl_shm_invoke_async` (pcl_transport_shared_memory.c) |
| 454 | `socket_invoke_async` (pcl_transport_socket.c) |
| 455 | `tpl_outbound_clone` (pcl_transport_template.c); `tpl_send_enqueue` (pcl_transport_template.c); `tpl_publish` (pcl_transport_template.c) |
| 456 | `tpl_outbound_clone` (pcl_transport_template.c); `tpl_send_enqueue` (pcl_transport_template.c); `tpl_invoke_async` (pcl_transport_template.c) |
| 457 | `pcl_shm_publish_with_worker_backpressure` (pcl_transport_shared_memory.c) |
| 458 | `tpl_send_thread_main` (pcl_transport_template.c); `pcl_transport_template_destroy` (pcl_transport_template.c) |
| 459 | `pcl_service_respond` (pcl_container.c); `drain_svc_req_queue` (pcl_executor.c) |
| 460 | `pcl_service_respond` (pcl_container.c) |
| 461 | `pcl_service_respond` (pcl_container.c) |
| 462 | *test-only* |
| 463 | `route_matching_list` (pcl_transport_routing.c); `validate_exclusivity` (pcl_transport_routing.c) |
| 464 | `handle_exclusive_line` (pcl_transport_routing.c) |
| 465 | `validate_exclusivity` (pcl_transport_routing.c) |
| 466 | `handle_exclusive_line` (pcl_transport_routing.c) |
| 467 | `split_endpoint_list` (pcl_transport_routing.c); `handle_exclusive_line` (pcl_transport_routing.c) |
| 468 | `pcl_transport_routing_load` (pcl_transport_routing.c) |
| 469 | `validate_exclusivity` (pcl_transport_routing.c); `pcl_transport_routing_load` (pcl_transport_routing.c) |
| 470 | `pcl_executor_invoke_stream` (pcl_executor.c) |
| 471 | `pcl_transport_routing_get_gateway` (pcl_transport_routing.c) |
| 472 | `handle_route_line` (pcl_transport_routing.c) |
| 473 | `pcl_executor_invoke_async` (pcl_executor.c); `pcl_executor_invoke_stream` (pcl_executor.c) |
| 474 | `pcl_executor_endpoint_route_exists_any_kind` (pcl_executor.c); `route_matching_list` (pcl_transport_routing.c); `validate_exclusivity` (pcl_transport_routing.c); `pcl_transport_routing_load` (pcl_transport_routing.c) |
| 475 | `pcl_executor_endpoint_route_exists_any_kind` (pcl_executor.c); `route_matching_list` (pcl_transport_routing.c); `validate_exclusivity` (pcl_transport_routing.c) |
| 476 | `pcl_process_runtime_create` (pcl_process_runtime.c); `pcl_process_runtime_executor` (pcl_process_runtime.c) |
| 477 | `pcl_process_runtime_load_codec` (pcl_process_runtime.c) |
| 478 | `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 479 | `activate_gateways` (pcl_process_runtime.c) |
| 480 | `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 481 | `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 482 | `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 483 | `kind_name` (pcl_process_runtime.c); `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 484 | `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 485 | `make_temporary_path` (pcl_process_runtime.c); `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 486 | `pcl_process_runtime_run` (pcl_process_runtime.c) |
| 487 | `runtime_signal_handler` (pcl_process_runtime.c); `pcl_process_runtime_run` (pcl_process_runtime.c) |
| 488 | `pcl_process_runtime_run` (pcl_process_runtime.c) |
| 489 | `pcl_process_runtime_run` (pcl_process_runtime.c) |
| 490 | `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 491 | `pcl_process_runtime_run` (pcl_process_runtime.c) |
| 492 | `pcl_process_runtime_load_codec` (pcl_process_runtime.c) |
| 493 | `pcl_process_runtime_load_ports_file` (pcl_process_runtime.c) |
| 494 | `pcl_process_runtime_create` (pcl_process_runtime.c) |
| 495 | `subscribe_port_with_transport` (pcl_executor.c); `subscribe_container_ports` (pcl_executor.c); `pcl_executor_add` (pcl_executor.c) |
| 496 | `subscribe_existing_ports` (pcl_executor.c); `pcl_executor_register_transport_caps` (pcl_executor.c); `pcl_executor_register_transport` (pcl_executor.c) |
| 497 | `subscribe_port_with_transport` (pcl_executor.c) |
| 498 | `pcl_executor_set_incoming_queue_limit` (pcl_executor.c) |
| 499 | `pcl_executor_get_incoming_queue_depth` (pcl_executor.c) |
| 500 | `pcl_shm_subscribe` (pcl_transport_shared_memory.c) |
| 501 | `pcl_shm_worker_svc_req` (pcl_transport_shared_memory.c) |
| 502 | `enqueue_outbound_frame` (pcl_transport_socket.c); `pcl_socket_transport_dropped_publishes` (pcl_transport_socket.c) |
| 503 | `send_thread_main` (pcl_transport_socket.c); `pcl_socket_transport_destroy` (pcl_transport_socket.c) |
| 504 | `udp_recv_thread_main` (pcl_transport_udp.c); `pcl_udp_transport_received_datagrams` (pcl_transport_udp.c) |
| 505 | `udp_record_sequence` (pcl_transport_udp.c); `pcl_udp_transport_dropped_datagrams` (pcl_transport_udp.c) |

## Gaps

### Tags naming unknown LLRs

None.

### Non-static functions with neither tag nor justification

None.

### LLRs with no implementation tag and no exemption

None.

### Exempt LLRs (marked in LLR.md)

- REQ_PCL_084 (test-only)
- REQ_PCL_112 (test-only)
- REQ_PCL_346 (test-only)
- REQ_PCL_462 (test-only)

