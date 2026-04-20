# PYRAMID Scripts

This directory contains PYRAMID helper scripts for binding generation, Tactical Objects coverage, GNAT/Ada support, and socket/interop smoke tests.

PCL coverage helpers live under `subprojects/PCL/scripts/`.

## Binding Generation

After changing `.proto` contracts, regenerate generated C++/Ada/codecs from the workspace root:

```bat
subprojects\PYRAMID\scripts\generate_bindings.bat
```

```bash
subprojects/PYRAMID/scripts/generate_bindings.sh
```

Generated binding architecture and current coverage status are documented in:

- [`../doc/architecture/generated_bindings.md`](../doc/architecture/generated_bindings.md)
- [`../../../doc/reports/PYRAMID/generated_bindings_status.md`](../../../doc/reports/PYRAMID/generated_bindings_status.md)

## Windows (GCC/gcov)

Uses GCC with gcov and gcovr (same toolchain as Linux). Requires:

- GCC (MinGW, msys2, winlibs, etc.)
- Make (Unix Makefiles)
- gcovr: `pip install gcovr`

| Script | Module | Output |
|--------|--------|--------|
| `subprojects\PCL\scripts\coverage_pcl.bat` | PCL (pcl_container, pcl_executor, pcl_log, pcl_bridge) | `coverage_pcl\index.html`, `coverage_pcl\summary.txt` |
| `coverage_tactical_objects.bat` | tactical_objects runtime + component | `coverage_tactical_objects\index.html`, `coverage_tactical_objects\summary.txt`, `requirement_traceability.md` |

Run from repo root:

```bat
subprojects\PCL\scripts\coverage_pcl.bat
subprojects\PYRAMID\scripts\coverage_tactical_objects.bat
```

## Linux (GCC/Clang)

Uses gcov + gcovr. Install:

```bash
pip install gcovr
```

| Script | Module | Output |
|--------|--------|--------|
| `subprojects/PCL/scripts/coverage_pcl.sh` | PCL | `coverage_pcl/index.html`, `coverage_pcl/summary.txt` |
| `coverage_tactical_objects.sh` | tactical_objects | `coverage_tactical_objects/index.html`, `coverage_tactical_objects/summary.txt`, `requirement_traceability.md` |

Run from repo root:

```bash
chmod +x subprojects/PCL/scripts/*.sh subprojects/PYRAMID/scripts/*.sh
./subprojects/PCL/scripts/coverage_pcl.sh
./subprojects/PYRAMID/scripts/coverage_tactical_objects.sh
```

## Requirement Traceability (tactical_objects)

The tactical_objects coverage script also runs `gen_requirement_trace.py`, which produces `coverage_tactical_objects/requirement_traceability.md`:

- **HLR → Tests**: From `doc/reports/PYRAMID/tactical_objects/HLR_COVERAGE.md` (TOBJ.001–TOBJ.053, RESP.001–RESP.017)
- **LLR → HLR → Test**: From `subprojects/PYRAMID/doc/requirements/tactical_objects/LLR.md` (REQ_TACTICAL_OBJECTS_001–…) with trace to HLR and verification test
- **Test → Requirements**: From `///< REQ_` / `///< TOBJ.` / `///< RESP.` tags in test files

Use this report to verify requirement-to-test trace completeness.

## Notes

- **Debug build**: Coverage tools require Debug builds for accurate line mapping (optimisation can distort results).
- **PCL**: C sources in `subprojects/PCL/src/`; tests include lifecycle, executor, logging, robustness, transports, streaming, bridge, and C++ wrappers.
- **tactical_objects**: C++ sources in `subprojects/PYRAMID/tactical_objects/src/`; all `test_tobj_*` binaries are exercised.
