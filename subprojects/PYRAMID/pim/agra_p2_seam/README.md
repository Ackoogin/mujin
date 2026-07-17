# A-GRA P2 interaction overlay

This directory adds PYRAMID interaction services to the generated A-GRA 5.0a
P2 data model. It contains four Request/Entity command services and twelve
single-variant Information services. Every operation uses reliable, volatile
delivery with queue depth 10. Topic names are the A-GRA global element names.

## Correlation fields

The command and status messages carry the same typed identifier. The paths
below name the fields in the converted proto. Each identifier ultimately
contains `ID_Type.uuid`, represented by exactly 16 bytes and written on the
A-GRA wire as 32 hexadecimal characters without hyphens.

| Command and status pair | Correlation field in each message | UUID value path |
|---|---|---|
| `MA_MissionPlanCommandMT` / `MA_MissionPlanCommandStatusMT` | `message_data.command_id` (`MissionPlanCommandID_Type`) | `message_data.command_id.base.uuid` |
| `MA_MissionPlanActivationCommandMT` / `MA_MissionPlanActivationCommandStatusMT` | `message_data.command_id` (`MissionPlanActivationCommandID_Type`) | `message_data.command_id.base.uuid` |
| `MA_PlanningFunctionSettingsCommandMT` / `MA_PlanningFunctionSettingsCommandStatusMT` | `message_data.base.command_id` (`CommandID_Type`) | `message_data.base.command_id.base.uuid` |
| `MA_ApprovalRequestMT` / `MA_ApprovalRequestStatusMT` | `message_data.base.request_id` (`RequestID_Type`) | `message_data.base.request_id.base.uuid` |

The `base` members model XSD extension composition. The OMS-JSON codec flattens
them, so the corresponding wire elements remain `CommandID` or `RequestID`.

## Windows-safe generated inputs

The binding generator scans only the input directory supplied on its command
line. Its OMS-JSON emitter searches the data-model proto's parent directories
for the first `wire_names.json`, and accepts it only when its `package` matches
the proto package. Therefore this overlay keeps byte-for-byte copies of the P2
generated proto and `wire_names.json` inside the overlay instead of using the
P1 overlay's symlinks. This works in native Windows checkouts where Git may
materialize a symlink as a text file. The tests compare both copies with
`pim/uci_generated/agra_5_0a/`; the sidecar comparison protects the otherwise
silent wire-name lookup failure.

`pyramid.options.proto` is also a byte-for-byte copy of the shared PIM options
contract. Port helpers use the separate
`pyramid.data_model.agra_port_grammar` package so they cannot replace the
generated `pyramid.data_model.agra` types header.

## Schema and drop identity

`binding_metadata.json` uses the generator's generic optional manifest metadata
channel. The generated `binding_manifest.json` contains:

```json
{
  "metadata": {
    "drop": "agra_5_0a",
    "owp_init_schema": "005.0a.ASK",
    "schema_version": "005.0a.ASK-20260423-f1380e7"
  }
}
```

`owp_init_schema` is the literal accepted by the pinned Sleet instance during
OWP `INIT`; `schema_version` retains the full pinned drop version.

## Generate bindings

From the repository root:

```bat
python subprojects\PYRAMID\pim\generate_bindings.py ^
  subprojects\PYRAMID\pim\agra_p2_seam output\agra_p2 ^
  --languages cpp --backends oms_json
```
