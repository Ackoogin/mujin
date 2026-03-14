# Coverage Scripts

Scripts to generate source statement coverage reports for PCL and tactical_objects modules.

## Windows (GCC/gcov)

Uses GCC with gcov and gcovr (same toolchain as Linux). Requires:

- GCC (MinGW, msys2, winlibs, etc.)
- Ninja (`choco install ninja` or via msys2)
- gcovr: `pip install gcovr`

| Script | Module | Output |
|--------|--------|--------|
| `coverage_pcl.bat` | PCL (pcl_container, pcl_executor, pcl_log, pcl_bridge) | `coverage_pcl\index.html`, `coverage_pcl\summary.txt` |
| `coverage_tactical_objects.bat` | tactical_objects runtime + component | `coverage_tactical_objects\index.html`, `coverage_tactical_objects\summary.txt` |

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
| `coverage_tactical_objects.sh` | tactical_objects | `coverage_tactical_objects/index.html`, `coverage_tactical_objects/summary.txt` |

Run from repo root:

```bash
chmod +x scripts/*.sh
./scripts/coverage_pcl.sh
./scripts/coverage_tactical_objects.sh
```

## Notes

- **Debug build**: Coverage tools require Debug builds for accurate line mapping (optimisation can distort results).
- **PCL**: C sources in `src/pcl/`; tests: test_pcl_lifecycle, test_pcl_executor, test_pcl_log, test_pcl_robustness, test_pcl_dining.
- **tactical_objects**: C++ sources in `pyramid/tactical_objects/src/`; all test_tobj_* binaries are exercised.
