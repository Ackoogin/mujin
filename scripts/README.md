# Coverage Scripts

Scripts to generate source statement coverage reports for PCL and tactical_objects modules, with HLR→LLR→test traceability for tactical_objects.

## Windows (GCC/gcov)

Uses GCC with gcov and gcovr (same toolchain as Linux). Requires:

- GCC (MinGW, msys2, winlibs, etc.)
- Make (Unix Makefiles)
- gcovr: `pip install gcovr`

| Script | Module | Output |
|--------|--------|--------|
| `coverage_pcl.bat` | PCL (pcl_container, pcl_executor, pcl_log, pcl_bridge) | `coverage_pcl\index.html`, `coverage_pcl\summary.txt` |
| `coverage_tactical_objects.bat` | tactical_objects runtime + component | `coverage_tactical_objects\index.html`, `coverage_tactical_objects\summary.txt`, `requirement_traceability.md` |

Run from repo root:

```bat
scripts\coverage_pcl.bat
scripts\coverage_tactical_objects.bat
```

## Linux (GCC/Clang)

Uses gcov + gcovr. Install:

```bash
pip install gcovr
```

| Script | Module | Output |
|--------|--------|--------|
| `coverage_pcl.sh` | PCL | `coverage_pcl/index.html`, `coverage_pcl/summary.txt` |
| `coverage_tactical_objects.sh` | tactical_objects | `coverage_tactical_objects/index.html`, `coverage_tactical_objects/summary.txt`, `requirement_traceability.md` |

Run from repo root:

```bash
chmod +x scripts/*.sh
./scripts/coverage_pcl.sh
./scripts/coverage_tactical_objects.sh
```

## Requirement Traceability (tactical_objects)

The tactical_objects coverage script also runs `gen_requirement_trace.py`, which produces `coverage_tactical_objects/requirement_traceability.md`:

- **HLR → Tests**: From `HLR_COVERAGE.md` (TOBJ.001–TOBJ.053, RESP.001–RESP.017)
- **LLR → HLR → Test**: From `LLR.md` (REQ_TACTICAL_OBJECTS_001–…) with trace to HLR and verification test
- **Test → Requirements**: From `///< REQ_` / `///< TOBJ.` / `///< RESP.` tags in test files

Use this report to verify requirement-to-test trace completeness.

## Notes

- **Debug build**: Coverage tools require Debug builds for accurate line mapping (optimisation can distort results).
- **PCL**: C sources in `src/pcl/`; tests: test_pcl_lifecycle, test_pcl_executor, test_pcl_log, test_pcl_robustness, test_pcl_dining, test_pcl_oom.
- **tactical_objects**: C++ sources in `pyramid/tactical_objects/src/`; all test_tobj_* binaries are exercised.
