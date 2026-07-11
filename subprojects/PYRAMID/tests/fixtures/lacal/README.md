# LA-CAL OMS JSON fixtures

These fixtures freeze the two UCI 2.5 message shapes used by the initial
PYRAMID OMS JSON codec subset.

- `signal_report.json` follows `make_signal_report()` in the independently
  authored AMS GRA starter-kit RF skill, tag `v2026.06.01`, commit
  `af13bd2926b15253e795920c91320733a29927ea`.
- `position_report.json` is the RF skill's PositionReport test fixture at the
  same commit. The skill subscribes to this message and copies its position
  into enriched SignalReports.

Both shapes target UCI schema `002.5.0`, pinned by the LA-CAL harness at UCI
commit `73a286fc4b881d9f328dbd64148525606a92c33a`. The upstream projects are
Apache-2.0; these small JSON instances contain no upstream implementation code.
