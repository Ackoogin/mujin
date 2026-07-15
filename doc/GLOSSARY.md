# Glossary of recurring terms

Documentation in this repository is read by software engineers of varying
experience levels, many of whom are new to the project and some of whom are
not native English speakers. This page defines the short-hand terms that
repository documents use repeatedly. Documents should link here (or define
the term inline in one sentence) the first time they use one of these
words with its repository-specific meaning.

| Term | Meaning here |
|------|--------------|
| **check in / checked in** | To commit a file to git so it is part of the repository. "The generated tree is checked in" means the generator's output is committed, not produced on every build. |
| **closure** (of a profile) | Everything a set of selected messages pulls in: starting from the chosen root messages, follow every type they reference, then every type those reference, and so on until nothing new appears. The result — all the messages and enums that must be generated — is the profile's closure. Not related to the programming-language meaning of "closure". |
| **contract** | The machine-readable description of what two components exchange: the `.proto` message definitions plus service/port declarations. Code generators consume the contract to produce bindings and codecs. |
| **codec** | The encode/decode layer that converts between in-memory message structs and the bytes/JSON actually sent on a transport. |
| **drop** (schema drop) | One published release of a schema set, for example "UCI 2.5" or "A-GRA 5.0a". Two drops are treated as unrelated inputs even when they look similar: each gets its own generated output, and nothing generated is shared between drops. |
| **facade** (interaction facade) | The generated, typed API layer a component calls instead of using raw publish/subscribe primitives: typed request/response and subscription callbacks, generated from the contract. |
| **fail closed** | On any doubt — wrong schema, missing capability, malformed input — refuse and report an error rather than continue in a degraded or guessing mode. |
| **fetch-not-vendor** | The upstream file is downloaded by a script when needed and is never committed to this repository; only its cryptographic hash (see *pin*) is committed. Used where redistribution rights are unresolved. Opposite of *vendor*. |
| **fixture** | A small, fixed input file used by tests: a hand-written schema, a captured message, a miniature contract. Tests compare tool output against fixtures so behaviour changes are caught as diffs. |
| **frozen** | Kept permanently unchanged, on purpose. A frozen file (for example a retired hand-written codec) exists only as a comparison reference; new functionality must never be added to it. |
| **golden** (golden file, golden output) | A committed copy of a tool's known-correct output. Tests regenerate the output and require it to match the golden copy byte for byte, so any behaviour change shows up as a reviewable diff. |
| **live-proven / live leg** | Verified against a real running peer (for example the Sleet server), not only against files on disk. The "offline" counterpart means verified against the schema and test files only, with no server involved. |
| **object-compile** | To compile generated source files just far enough to prove they are valid code (producing object files), without linking or running them. Used as a cheap correctness gate for large generated trees. |
| **overlay** | A small set of hand-written contract files layered on top of a large generated tree, adding what the generator cannot know (port declarations, topic names) without modifying the generated files themselves. |
| **pin / pinned** | Recording the exact version and cryptographic hash (sha256) of an external input so every future run uses byte-identical input. "The XSD is pinned" means: the hash is committed, and a fetched file that does not match it is rejected. |
| **port** | A declared interaction endpoint of a component in the PYRAMID port grammar: a Request port (sends commands, receives correlated status), a Requirement port (the providing side of that), or an Information port (publishes/consumes a data topic). |
| **profile** | A named selection of top-level messages to convert from a schema drop, recorded as a manifest file (for example `p1_kitty_hawk.json`). The converter generates the profile's *closure*. |
| **root** (root message) | One of the top-level messages a profile selects by name. Everything else in the generated output is there because a root (directly or indirectly) references it. |
| **seam** | The proven interaction boundary used as a worked example: a correlated command/status exchange between two processes over the real transport. "The seam test" exercises exactly that boundary. |
| **sidecar** | A companion file that carries metadata about a main file, living next to it. Here: `wire_names.json` carries the exact on-the-wire element names for a generated `.proto` tree. |
| **SKIP-gated** | A test that checks for its external prerequisites (a schema file, a running server, a toolchain) and reports itself as skipped, with a printed reason, when they are absent — instead of failing. The default build and test run must pass with everything optional absent. |
| **strict-clean** | The converter ran with no relaxations enabled and encountered nothing it could not handle: no construct was skipped or approximated. |
| **vendor / vendored** | To copy an external file into this repository and commit it, so the build never needs to download it. Only done when the licence clearly permits redistribution. Opposite of *fetch-not-vendor*. |
| **wire name / wire form** | The exact element name or byte/JSON layout as it appears in transmitted messages — as opposed to the (possibly renamed) identifiers used in generated code. |
| **byte-stable / byte-identical** | Re-running the generator on the same input produces output identical byte for byte. This is test-enforced so generated output can be committed and any change is a deliberate, reviewable diff. |
