# PYRAMID — Defect Remediation Plan

**Written 2026-08-04.** This plan orders the known PYRAMID defects by how much
damage they do and proposes a fix for each. It deliberately puts defects ahead
of features: every open feature item in
[`doc/todo/PYRAMID/TODO.md`](../../todo/PYRAMID/TODO.md) (the Ada facade
follow-ons, the Tactical Objects conversion, the A-GRA evidence steps) is
listed at the end as deferred, because none of them can be validated while the
test suite cannot be trusted to say whether a change broke something.

Everything below was reproduced on 2026-08-04 against commit `cee1fc4` with
submodules `PCL` at `7284846` and `PYRAMID` at `8971055`, on Windows 11 with
Visual Studio 2022, using the documented build commands. Where a finding
differs from what the tracker records, the difference is stated rather than
smoothed over.

## Progress as of 2026-08-04

| Defect | State |
|--------|-------|
| D-1 CTest discovery | **Fixed** for discovery (PYRAMID `de9dfde`). Two Ada tests still need the same treatment — see the note under D-1 |
| D-2 Bridge plugin path | **Partly fixed** (PYRAMID `de9dfde`). The path defect is gone; a second failure behind it is not |
| D-3 Ada access violation | **Open, and needs re-scoping** — see the revised note under D-3 |
| D-4 Codec name collision | Open |
| D-5 JSON codec fail-closed | Open |
| D-6 Stale baselines | **Fixed** (PYRAMID `e78e0e1`) |
| D-7 Linux-only tests | **Fixed** (PYRAMID `e78e0e1`) |

**Correction to the measurements below.** The "six failures" this plan
originally recorded were measured with the generated-codec directory added to
`PATH` by hand — the D-1 workaround. That masked two further failures. The
honest clean-shell figures are: before any fix, CTest enumerated **nothing**;
after D-1 and D-2, the suite runs and reports **8 failures out of 828**. With
the codec directory on `PATH`, the count is 6 both before and after the fixes,
which is the like-for-like check confirming no regression.

## Why the suite comes first

The tracker's standing regression bar says CTest is green at 927 of 927 and
that "any CTest failure now is breakage from current work". On this machine,
following the documented build and test commands, CTest does not run at all:
it fails during test discovery. With a workaround applied it runs and reports
six failures. Both numbers cannot be right at once, and the difference matters
more than either number: a bar that reads "green" on one platform and "cannot
start" on another does not tell the next person whether their change was safe.

So the first three items below exist to make one sentence true again — "a red
suite means I broke something" — and nothing else should be scheduled until
they are done.

---

## P0. The test suite cannot be trusted

### D-1. CTest cannot discover tests on a clean Windows build

**What happens.** After `cmake --preset default` and
`cmake --build --preset release`, running the documented
`ctest --test-dir build --output-on-failure -C Release` fails before any test
executes:

```
Error running test executable.
  Path: '.../proofs/tests/Release/test_pcl_generated_component_stream_handle.exe'
  Result: Exit code 0xc0000135
```

`0xc0000135` is the Windows status code for "a required DLL was not found".
`gtest_discover_tests` runs each test binary at build time to enumerate its
cases, so one binary that cannot start aborts discovery for the whole project,
and no test runs.

**Cause.** `pyramid_generated_codecs.dll` is written to
`build/subprojects/PYRAMID/Release/`, while the test binaries that link against
it are written to `build/subprojects/PYRAMID/proofs/tests/Release/`. The
Windows loader searches the directory of the executable, not the directory of
the library, so the import cannot be resolved. This is a consequence of the
generated codec libraries becoming shared libraries; it does not arise on Linux,
where `RPATH` covers the same case.

**Confirmed by** putting the library directory on `PATH` and re-running:
discovery then succeeds and finds 828 tests, where before it found none.

**Fix.** Make the test binaries able to find the libraries they link, by one of:

1. Set `RUNTIME_OUTPUT_DIRECTORY` on the generated codec libraries so they land
   beside the test binaries. Simplest, but it duplicates the library if more
   than one directory needs it.
2. Add the library directory to each test's `ENVIRONMENT` property via
   `set_tests_properties(... ENVIRONMENT "PATH=...")`. This fixes execution but
   **not** discovery, because discovery runs the binary outside CTest's
   environment — so it must be paired with `DISCOVERY_MODE PRE_TEST`, which
   defers enumeration to test time.
3. Set `gtest_discover_tests(... PROPERTIES ENVIRONMENT ...)` together with
   CMake's `TARGET_RUNTIME_DLLS` generator expression and a post-build copy
   step. This is the idiomatic modern answer and handles transitive
   dependencies as well.

Option 3 is recommended; option 1 is acceptable as an immediate unblock.

**Accept.** `ctest --test-dir build -C Release -N` lists the full test set on a
clean Windows checkout with no environment variables set by hand, and a fresh
`ctest` run executes tests rather than erroring during discovery.

**Done 2026-08-04 (PYRAMID `de9dfde`), with one part outstanding.** Option 3 was
taken: a Windows post-build step copies the linked runtime DLLs beside the
executable, and the discovered tests carry the codec directory in their runtime
`PATH`. The same copy was added beside `tactical_objects_app`. Verified in a
clean shell: all 828 tests enumerate where none did before.

What it does not cover: `ada_generated_bindings_roundtrip` and
`ada_cpp_codec_roundtrip` still fail in a clean shell, reporting
`codec plugin load failed: cannot open .../pyramid_codec_json_*.dll`. Both pass
once the codec directory is on `PATH`, so this is the same defect reaching
executables that CTest runs from a source-tree `bin/` directory
(`${ADA_TEST_SRC_DIR}/bin/`) rather than from the build tree — the post-build
copy never reaches them. The remedy is the same shape as the rest of this fix:
give those two tests the codec directory in their `ENVIRONMENT` property, beside
the `PYRAMID_CODEC_PLUGINS` they already carry. Note that CMake here is 3.21, so
`ENVIRONMENT_MODIFICATION` (3.22+) is not available and a bare
`ENVIRONMENT "PATH=..."` replaces rather than prepends — append the
configure-time `PATH` explicitly.

---

### D-2. `pyramid_bridge_e2e` looks for codec plugins in the wrong directory

**What happens.** With D-1 worked around, the test fails after 26 seconds. The
driver reports an empty plugin list and the bridge then refuses to start:

```
[driver] PYRAMID_CODEC_PLUGINS=
[pyramid_bridge] FAIL: fail-closed: no codec plugin registered for content type application/json
[evidence_client] publish standard.object_evidence failed rc=-6   (repeated)
```

The repeated publish failures and the stub that receives nothing are exactly the
symptoms the tracker recorded against this test on 2026-07-21, so this is the
same defect, still present, not a new one.

**Cause.** `proofs/scripts/test_pyramid_bridge_e2e.bat` derives the plugin
directory from the application binary's own path (line 107):

```bat
for %%D in ("!_APP_DIR!..\..") do set "_PYRAMID_BUILD_DIR=%%~fD"
set "PLUGIN_DIR=!_PYRAMID_BUILD_DIR!\!_APP_CONFIG!"
```

Walking two directories up was correct when `tactical_objects_app.exe` lived at
`subprojects/PYRAMID/tactical_objects/<Config>/`. The 2026-07-22 split moved it
to `subprojects/PYRAMID/proofs/tactical_objects/<Config>/`, one level deeper, so
the walk now lands on `subprojects/PYRAMID/proofs` instead of
`subprojects/PYRAMID`. Measured on this build: the computed directory
`.../PYRAMID/proofs/Release` holds one DLL and none of the codec plugins, while
the real location `.../PYRAMID/Release` holds all eleven.

**Fix.** Stop deriving the directory by counting parent directories, which is
what made the script silently wrong when the tree moved. Pass the plugin paths
in from CMake instead, using `$<TARGET_FILE:...>` generator expressions, exactly
as `proofs/tests/CMakeLists.txt` already does at lines 1745 and 1748 for the
Ada tests. The script keeps its "unless the caller already set it" branch, so
the CMake-supplied value simply wins.

**Also check the `.sh` counterpart.** `test_pyramid_bridge_e2e.sh` has the same
shape at line 126. It may be masked on Linux by a different working directory,
but it should be converted at the same time rather than left as a second copy
of the same assumption.

**Accept.** `pyramid_bridge_e2e` passes on Windows from a clean build, and the
driver line reports a non-empty `PYRAMID_CODEC_PLUGINS`. Add an explicit check
in the script that fails with a clear message when the list comes out empty, so
this class of breakage reports its cause instead of surfacing as a fail-closed
error much later.

**Partly done 2026-08-04 (PYRAMID `de9dfde`).** CMake now supplies the plugin
paths through `PYRAMID_CODEC_PLUGINS` using `$<TARGET_FILE:...>`; both drivers
stopped counting parent directories, kept their caller-supplied branch, and gained
the empty-list check. The driver now prints six absolute plugin paths and the
fail-closed codec error is gone.

**The test still fails, on a second defect the first was hiding.** It now times
out with the evidence client returning `rc=-6` for a missing publish route and
the backend receiving zero facts. That is a shared-memory routing failure, not a
plugin-discovery one. It needs its own diagnosis; do not treat it as unfinished
D-2 work, because the path defect is genuinely fixed and the remaining fault
reproduces with the plugins correctly loaded.

---

### D-3. Five Ada Tactical Objects tests crash with an access violation

**What happens.** `tobj_ada_socket_e2e`, `tobj_ada_active_find_e2e`,
`tobj_ada_active_find_flatbuffers_e2e`, `tobj_ada_active_find_app_e2e`, and
`tobj_ada_active_find_app_flatbuffers_e2e` all fail. The Ada client starts,
configures, activates, sends its request, and then dies:

```
[ada_tobj_client] Spinning to receive service response and entity matches...
[driver] FAIL: Ada client exited with code -1073741819
```

`-1073741819` is `0xC0000005`, an access violation.

**This is not the "binaries absent" case.** The tracker warns that Ada tests
report success when their binaries are missing. Here the opposite is true: the
binaries are present and running (GNAT 2021 is installed at `C:\GNAT\2021`), and
they crash after doing real work. A skip would have been silent; this is a
genuine memory fault.

**Cause: not yet diagnosed, and the premise has changed since this was
written.** The access violation above was observed with the codec directory on
`PATH`. After D-1 and D-2 landed, these five were reported as failing *earlier*
— in their own drivers, unable to load their codec plugin dependencies and
failing closed — without reaching the access violation at all. That has not been
independently confirmed, and it matters: if true, the visible fault is another
instance of the D-1 DLL-resolution problem reaching yet another set of drivers,
and the access violation is a second layer underneath that may or may not
survive fixing the first.

So the first step is no longer "get the faulting frame". It is to establish
which of the two failures is actually in front, by running one of these tests in
a clean shell and again with the codec directory on `PATH`, and comparing. Only
once the plugin loading succeeds does chasing the access violation make sense.

The original crash signature, still the likely second layer, is recorded below.

**Fix.** Investigate in this order:

1. Separate the two failures, as described above. Give these five drivers the
   same codec-directory treatment the two Ada roundtrip tests need under D-1,
   then re-run. If the access violation returns, it is real and independent; if
   the tests pass, this item collapses into D-1.
2. If the crash is real, build the Ada side with `-gnata` (assertions on) and
   run under a debugger to get the faulting frame.
3. Suspect the C-to-Ada boundary in the generated marshalling code — an
   unbounded string or slice whose length is read from the wire.

**Accept.** All five pass on Windows from a clean build, with the cause recorded
in the tracker. If the cause turns out to be Windows-only and expensive, an
explicit, reasoned skip on Windows is an acceptable outcome — but it must be a
skip that says why, not a failure that everyone learns to ignore.

---

## P1. Codecs accept input they should refuse

These two are grouped because they are the same underlying mistake — a codec
saying "yes" to data it does not actually understand — and because the third
candidate fix for D-4 also fixes D-5. They are the most dangerous defects in the
repository: neither produces an error, and both corrupt data quietly.

### D-4. Codec dispatch picks a codec by unqualified type name

**Status.** Recorded in the tracker's WS-D table on 2026-08-04; repeated here
because it is the highest-severity open defect and belongs in the schedule
rather than in a deferred list.

**What happens.** The codec registry lets several codecs share one content type
and expects dispatch to tell them apart by schema id — but the schema id is the
*unqualified* type name (`Ack`, `Query`, `Identifier`).
`pyramid_try_registry_encode` and `pyramid_try_registry_decode` walk every codec
registered for the content type in registration order and take the first that
recognises the name. Two contracts that each define a type called `Ack` are
therefore indistinguishable, and whichever plugin loaded first answers for both.
Nothing reports a conflict: the value is encoded against the wrong definition and
any field the winning definition lacks is dropped.

**Why it matters more than it looks.** This is not a test-only problem. Any
deployment that loads codec plugins generated from two different contracts hits
it, and the symptom is missing field values, not a failure.

**How it surfaced.** `test_pcl_generated_interaction_facade` was loading the main
contract tree's plugins alongside the three A-GRA example plugins it asks for by
name. The main tree's `Ack` has only `success`; the A-GRA example's `Ack` also
has `identifier`. The provider's acknowledgement arrived with an empty
identifier, and the transition query keyed on it matched nothing.

**What was already done, and what it did not do.** The test was excluded from
the blanket plugin-loading loop in `proofs/tests/CMakeLists.txt` so it now loads
only its own three plugins. That restored a green test. It did not touch the
defect.

**Fix.** Three candidates, from the tracker, with a recommendation:

1. Key the registry on a contract-qualified schema id. The proto package is
   already known at generation time, so this is available without new inputs.
2. Give each contract its own registry handle, passed to the generated
   encode/decode helpers instead of the process-wide default.
3. Have dispatch verify that the value it is handed matches the schema it knows,
   and fail closed on a mismatch.

**Recommended: (1) as the primary fix, with (3) as a safety net.** Qualifying
the schema id removes the ambiguity at its source, and it is a generation-time
change rather than an API change, so it does not move the burden onto every
caller the way (2) would. Adding (3) as well is worth the cost because it turns
any remaining mismatch into a loud failure and simultaneously closes D-5.

**Accept.** A process that loads codec plugins from two contracts defining the
same unqualified type name encodes and decodes each against its own definition.
Add a test that does exactly that and asserts both round trips keep their fields
— the `Ack`/`identifier` case above is a ready-made fixture. Re-including
`test_pcl_generated_interaction_facade` in the blanket plugin loop should then
pass, which makes the earlier workaround removable and is the cleanest proof the
defect is gone.

---

### D-5. The generated JSON codec accepts input that is not JSON

**What happens.** Handed a FlatBuffers payload, the generated JSON codec returns
success and fills the target with default values instead of rejecting it. A peer
configured with the wrong codec therefore produces silently empty data where the
fail-closed design intends a startup error.

**Cause (traced to one line).** `_write_from_json` in
`subprojects/PYRAMID/pim/cpp/json_codec_gen.py:290` emits:

```cpp
auto j = nlohmann::json::parse(s);
```

and then guards every field read with `if (j.contains("<field>"))`. A
FlatBuffers payload's leading byte parses as a bare JSON number, so `j` is a
number rather than an object; `contains()` returns false for every field on a
non-object; the struct is left default-constructed; and the codec returns
`PCL_OK`.

**This is the only hole of its kind.** The two neighbouring paths already do the
right thing, which is what makes the fix narrow and safe:

- the array path (`codec_plugin_gen.py:531`) checks `arr.is_array()` and returns
  `PCL_ERR_INVALID` otherwise;
- the scalar alias path uses `nlohmann::json::parse(payload).get<T>()`, which
  throws on a type mismatch and is caught into `PCL_ERR_CALLBACK`.

**Fix.** Emit an object check immediately after the parse in `_write_from_json`,
so the struct path fails closed the way the array path already does. This is one
line in the generator.

**Note on the standing regression bar.** This changes generated output, so
recorded baselines need refreshing — coordinate it with D-6, which is refreshing
baselines anyway.

**Accept.** The generated JSON codec refuses a FlatBuffers payload with an error
rather than reporting success. Add the negative test that was originally written
against this behaviour and removed when it failed: the tracker records that an
early version of `test_ports_file_codec_selection_e2e.cpp` asserted "the JSON
codec must refuse these bytes". That assertion should come back. The current
test compares the decoded value rather than the decode status, so it keeps
passing either way and needs no change.

---

## P2. The regression bar does not mean what it says

### D-6. Recorded baselines predate the A-GRA MA grounding contract

**Status.** Recorded in the tracker on 2026-08-04 and confirmed here.
`test_proto_parser` asserts the legacy tree holds 13 proto files when it now
holds 14 — reproduced exactly:

```
AssertionError: 14 != 13
```

The pytest failures are the "keeps legacy names" regression tests
(`test_generic_grpc_ros2.py`, `test_ros2_codec_plugin_generation.py`,
`test_generic_binding_contract.py`) plus the manifest tests, all comparing
against recorded file and symbol lists that predate that contract.

**Fix.** Refresh the recorded lists. The tracker is right that this is the whole
of the work; the value of doing it now is that it clears the way for D-5, which
also moves baselines.

**Accept.** `python -m pytest subprojects/PYRAMID/tests -q` and
`python -m unittest ...test_proto_parser` are both green on Linux, and the
tracker's item 2 can state that plainly instead of carrying an exception list.

**Done 2026-08-04 (PYRAMID `e78e0e1`).** The recorded sets gained the
`agra_ma_grounding` entries and the parser counts moved 13/16 to 14/17. One
change was more than a refreshed list and is worth knowing about: the manifest
test asserted that `marshal_modules` equals the total count of C-ABI marshal
sources, which held only because no *component* marshal source existed until the
grounding contract added one. It now filters to data-model marshal sources,
which is the invariant it was always checking — at the cost of no longer
covering component marshal sources in that assertion.

---

### D-7. Six further pytest failures on Windows that the tracker does not record

**What happens.** The tracker records six pytest failures. On Windows there are
twelve:

```
12 failed, 53 passed
```

The six the tracker does not mention are all in `test_sdk_packaging_contract.py`
(`test_linux_packager_rejects_missing_base_transport`, the three
`test_linux_packager_rejects_missing_generated_shared_library` cases,
`test_linux_packager_copies_generated_shared_libraries`, and
`test_sdk_release_scripts_identify_grpc_as_unsupported`). Each invokes a Linux
`.sh` packager through `subprocess` and dies with:

```
OSError: [WinError 193] %1 is not a valid Win32 application
```

**Why it matters.** These tests are testing the *Linux* packager, so they cannot
pass on Windows and are not meant to. But they fail rather than skip, so a
Windows developer following the standing regression bar sees twelve failures,
six of which are expected and six of which are real, with nothing distinguishing
them. That is how a real regression gets waved through.

**Fix.** Mark them to skip on Windows —
`@pytest.mark.skipif(sys.platform == "win32", reason="exercises the Linux
packager shell scripts")` — so the platform limitation is stated in the test
rather than in someone's memory.

**Accept.** On Windows the suite reports six failures before D-6 and zero after,
with six skips whose reason names the Linux packager.

**Done 2026-08-04 (PYRAMID `e78e0e1`).** Verified on Windows: the suite went
from 12 failed / 53 passed to **59 passed, 6 skipped**, and `test_proto_parser`
runs 4 tests with no failures.

---

## P3. Documents that contradict the tree

These are cheap, and each one currently misleads someone reading it.

### D-8. The C-ABI determinism defect is fixed but still listed as open

The tracker's WS-D table and the roadmap's `DEFECT-3` both describe
`cabi_codegen.py` emitting generic container typedefs in a hash-seed-dependent
order, deferred because fixing it would require refreshing baselines.

**That work is done.** PYRAMID commit `0506f11` (2026-07-22, "Make C-ABI header
generation deterministic (sort toposort dependencies)") sorted the dependency
iteration in both generators — `pim/cabi_codegen.py:139` and
`pim/ada_cabi_codegen.py:183` now read `sorted(deps.get(name, set()))` — and
added `pim/test_cabi_determinism.py`, which passes:

```
2 passed in 0.09s
```

**Fix.** Delete the WS-D row and move it to Delivered; drop `DEFECT-3` from the
roadmap. Note in passing that the commit found the same defect in the *Ada*
C-ABI generator too, which the original analysis had not identified.

---

### D-9. The A-GRA compatibility status contradicts WS-G

`subprojects/PYRAMID/proofs/doc/architecture/oms_agra_compatibility.md` is named
in both `CLAUDE.md` and the tracker as the live support statement — the document
an outsider reads to find out what is actually supported. Its one-page status
table (lines 32–34) says of the formal A-GRA P2 column:

| Row | What it says |
|-----|--------------|
| OMS-JSON codec (C++) | "Not generated — the P2 tree exists only as an offline conversion artifact" |
| OMS-JSON codec (Ada) | "Not attempted" |
| Interaction facade | "Not attempted" |

WS-G steps 4 and 5 record all three as built and proven: the codec target
`pyramid_codec_oms_json_agra` builds and is packaged by `--gra`, all 20 roots
round-trip, the Ada encoder object-compiles over the full 1,192-message closure
and matches C++ wire bytes, and the interaction overlay exists at
`proofs/contracts/agra_p2_seam/`.

**Direction of the error matters.** The document understates what exists, so it
does not overclaim compliance — the conservative direction, and the reason this
is P3 rather than P1. But it is still wrong, and someone planning from it would
conclude the P2 codec work has not started.

**Fix.** Update the three rows to state what is proven offline while keeping the
live-interop claim unchanged, since that genuinely has not happened (WS-G steps 6
and 7 remain open). The distinction the document already draws between offline
fidelity and independent-peer evidence is the right one; it simply has not been
applied to these rows.

---

### D-10. The roadmap dashboard is a fortnight behind the tracker

`doc/roadmap/pyramid.yaml` is stamped 2026-08-01 and was not touched by the
2026-08-04 tracker update, which added 262 lines. The roadmap therefore still
carries:

- `DEFECT-1` as "six pre-existing CTest failures ... 804 of 810 tests passing",
  superseded twice over;
- `DEFECT-3`, fixed since 2026-07-22 (D-8);
- `AGRA-G2` as an untracked-work placeholder, when the P3 seam's own README now
  documents its scope and evidence boundary;
- no entry at all for WS-J, the codec-name collision, or the native in-process
  binding work.

**Fix.** Regenerate it from the refreshed tracker and rerun
`python tools/roadmap/generate_roadmap.py`, committing both the data file and
the generated pages as `tools/roadmap/README.md` requires.

---

## Suggested order of work

The first three restore the ability to tell whether anything else worked, so
they come first regardless of size. D-2 is placed before D-3 because it may
resolve it.

| Order | Item | Size | Why here |
|-------|------|------|----------|
| 1 | D-1 CTest discovery | S | Nothing else is verifiable until this is done |
| 2 | D-2 Bridge plugin path | S | Mechanical; may also fix D-3 |
| 3 | D-3 Ada access violation | M–L | Unknown until diagnosed; step 1 is cheap |
| 4 | D-6 Refresh baselines | S | Clears the way for D-5 |
| 5 | D-7 Skip Linux-only tests | XS | Makes the Windows bar readable |
| 6 | D-4 Codec name collision | M | Highest severity; needs a trustworthy suite first |
| 7 | D-5 JSON codec fail-closed | S | One generator line plus a baseline refresh |
| 8 | D-8, D-9, D-10 Document fixes | S | Cheap; each currently misleads a reader |

D-4 is the most serious defect but is deliberately sixth. It is a generation-time
change that moves recorded output across the whole contract set, and doing that
while the suite cannot report reliably would mean landing the repository's most
dangerous fix blind.

## Deliberately deferred until the above is done

These are features and evidence steps, not defects, and all of them are already
described in the tracker. None is blocked by this plan except in the sense that
its acceptance cannot currently be demonstrated.

| Item | Where it is described |
|------|----------------------|
| F1a Ada cross-process transport proof | TODO WS-F |
| F1b Ada re-send to an open read stream | TODO WS-F |
| F1c Ada facade proof into CI | TODO WS-F |
| F2b Tactical Objects showcase conversion | TODO WS-F |
| E5 `StandardBridge` raw PCL wiring | TODO WS-E |
| G1 steps 5–7 (packaged smoke, Sleet interop, evidence publication) | TODO WS-G |
| G2 P3 tracker entry | TODO WS-G; the P3 seam README now supplies the content |
| WS-J J2(2), J2(3), J3, J5, J6 | TODO WS-J |
| N4 generator native services codegen | `high_efficiency_process_bindings_plan.md` |

Two of these are cheaper than the tracker implies and could be picked up
opportunistically. **G2** no longer needs someone with the original context:
`proofs/contracts/agra_p3_seam/README.md` states the scope (343 roots, 2,856
messages, 951 services from Table 3-1) and, importantly, its own evidence
boundary — "unlike the P1/P2 seams it has no offline fidelity or live interop
evidence". That is the missing tracker entry, and it also resolves the
contradiction with WS-G's "full P3 coverage" non-goal, since the seam makes no
compliance claim. **The `high_efficiency_process_bindings_plan.md` status line**
still reads "first cut scheduled", while its own table records N1, N2, N2b, N3a,
N5, N6, and the port abstraction as done, with only N4 codegen deferred; the
tracker's companion-document table still calls the plan "proposed".

## One more thing worth recording

A constraint documented in `subprojects/PYRAMID/doc/architecture/sdk_packaging.md`
does not appear in the tracker or the roadmap: **FlatBuffers codecs cannot be
built at full A-GRA P3 seam scope on MSVC.** A Windows import library holds at
most 65,535 symbols, and the aggregate of the four services' FlatBuffers wrapper
codecs exceeds it (`LNK1189`). Splitting them into their own shared library was
tried and does not help. The JSON path builds cleanly, and the
`agra/p3_three_process` example already selects JSON by design, so nothing is
broken today — but it is a real limit on a supported toolchain, and it should be
a WS-D row with a trigger rather than a paragraph in an architecture document
that a planner is unlikely to read.
