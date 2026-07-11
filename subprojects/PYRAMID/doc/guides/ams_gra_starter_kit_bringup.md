# Recreating the AMS GRA Hello-World Starter Kit Bring-Up

**Purpose:** step-by-step recreation of the local, persistent AMS GRA
("Kitty Hawk") demo environment used for
[LA-CAL integration plan](../../../../doc/plans/PYRAMID/la_cal_integration_plan.md)
Phase 6 (MS-leg demo). Phases 0–5 only needed the `sleet` server by itself
(pulled as a single container, run ad hoc); Phase 6 needs the **full
simulation stack** — Supercell (DIS truth + JSBSim), Squall (RF/IR sensor
sim), the `rf-fm-demod`/`ir-search-and-track` Skills, and (optionally)
Graupel/Worldview for visual truth-vs-perceived — so this guide documents
managing it as a standing, git-ignored local checkout instead of ad hoc.

**Companion:**
[`ams_gra_starter_kit_review.md`](../../../../doc/research/AME/ams_gra_starter_kit_review.md)
(what each component is and why), `pim/test_harness/lacal/README.md` (the
PCL-side Phase 3–5 harnesses that talk to whichever Sleet is running).

---

## 1. Layout

Everything lives under `external/ams-gra/` at the repo root, which is
git-ignored (`/external/` in `.gitignore`) — a persistent local working copy,
not something committed. Each upstream project is its own git clone at tag
`v2026.06.01` (the version pinned in `getting-started/manifest.yaml`):

```
external/ams-gra/
  getting-started/        # meta-repo: top-level compose.yaml, install.sh, docs/
  supercell/               <- symlinked into getting-started/supercell
  squall/                   <- symlinked into getting-started/squall
  sleet/                    <- symlinked into getting-started/sleet
  graupel/                  <- symlinked into getting-started/graupel
  worldview/                <- symlinked into getting-started/worldview
  rf-fm-demod/               <- symlinked into getting-started/rf-fm-demod
  ir-search-and-track/       <- symlinked into getting-started/ir-search-and-track
  la-cal-harness/          # Phase 4 foreign-peer harness (not part of compose)
```

**Why the symlinks:** `getting-started/compose.yaml` wires the stack together
with `include: [supercell/compose.yaml, squall/compose.yaml, ...]`, expecting
each project checked out as a subdirectory of `getting-started/` (that's how
the official release bundle ships them). Cloning each project as an
independent sibling repo instead — so each has its own `.git`, can be
`git pull`-ed independently, and matches how the projects are actually
versioned upstream — means `getting-started/` needs symlinks pointing at the
siblings for `include:` to resolve.

## 2. Clone the repos

All primary development is on GitLab (`gitlab.com/open-arsenal/ams-gra/hello-world-sk/...`);
GitHub mirrors exist but lag (per `CONTRIBUTING.md` in each repo). Clone at
the pinned tag:

```bash
mkdir -p external/ams-gra && cd external/ams-gra
BASE=https://gitlab.com/open-arsenal/ams-gra/hello-world-sk
for repo in getting-started sim/supercell sensors/squall viz/graupel \
            viz/worldview infra/sleet skills/rf-fm-demod \
            skills/ir-search-and-track test/la-cal-harness; do
  name=$(basename "$repo")
  git clone --depth 1 --branch v2026.06.01 "$BASE/$repo.git" "$name"
done

cd getting-started
for d in supercell squall graupel worldview sleet ir-search-and-track rf-fm-demod; do
  ln -s "../$d" "$d"
done
cp .env.example .env
```

## 3. Install podman + podman-compose

The kit's `install.sh` hard-requires `podman`/`podman-compose` (not Docker).
If `podman` isn't installed system-wide, it needs `sudo apt install podman`
(interactive password — do this yourself if scripting/automation can't sudo).
`podman-compose` does **not** need a system package or sudo — it installs
cleanly as a user package:

```bash
sudo apt install podman            # one-time, needs a password
pip3 install --user podman-compose  # no sudo needed
```

Verify: `podman --version` (this guide was run against 3.4.4) and
`podman-compose --version` (1.6.0). Note podman 3.4.4 has two known gaps that
show up during bring-up (see §6) — both benign for this use case.

## 4. Pull images

Every component publishes a `:latest` tag in its GitLab container registry
**except `viz/worldview`**, which only has `v2026.06.01`/`main`/a commit-hash
tag. Pull everything, then locally tag worldview so the stock
`compose.yaml` (which references `:latest`) resolves:

```bash
IMAGES=(
  infra/sleet sim/supercell sim/supercell/jsbsim
  sensors/squall/rf sensors/squall/ir sensors/squall/couloir
  sensors/squall/oms-rf-adapter sensors/squall/oms-ir-adapter
  skills/rf-fm-demod skills/ir-search-and-track viz/graupel
)
for img in "${IMAGES[@]}"; do
  podman pull "registry.gitlab.com/open-arsenal/ams-gra/hello-world-sk/${img}:latest"
done
podman pull registry.gitlab.com/open-arsenal/ams-gra/hello-world-sk/viz/worldview:v2026.06.01
podman tag registry.gitlab.com/open-arsenal/ams-gra/hello-world-sk/viz/worldview:v2026.06.01 \
           registry.gitlab.com/open-arsenal/ams-gra/hello-world-sk/viz/worldview:latest
podman pull docker.io/protomaps/go-pmtiles:v1.30.1
podman pull quay.io/prometheus/prometheus:latest
podman pull docker.io/grafana/grafana:latest
```

**Skip the PMTiles basemap.** `worldview/README.md` points at a ~135GB
full-planet `v4.pmtiles` download for the CesiumJS background map. That's
irrelevant to the AME data pipeline (it only affects Worldview's basemap
tiles, not the DIS/UCI truth-vs-perceived data), so this bring-up does not
fetch it — the `tileserver` container comes up but has no useful tiles to
serve. Fetch it yourself only if you want the actual 3D-globe basemap
rendered in a browser.

## 5. Registering AME's own services with Sleet

Sleet requires every connecting service to be pre-registered
(`service_id`/`allowed_topics`/`topic_bindings` in a `.toml`) or it silently
rejects the connection. The published `infra/sleet` image bakes in
registrations **only for the kit's own components**
(`sleet/config/services.d/*.toml`: `supercell`, `squall-rf-oms-adapter`,
`squall-ir-oms-adapter`, `rf-fm-demod`, `ir-search-and-track`, `graupel`) —
there is no volume mount for `services.d` in the stock `sleet/compose.yaml`,
so any additional service (a scratch OWP sniffer, or later the real AME
MS-leg bridge) needs its own registration layered on top locally:

```bash
mkdir -p external/ams-gra/sleet/services.d.local
cp external/ams-gra/sleet/config/services.d/*.toml \
   external/ams-gra/sleet/services.d.local/
# then add your own, e.g. services.d.local/ame-sniffer.toml:
cat > external/ams-gra/sleet/services.d.local/ame-sniffer.toml <<'EOF'
service_id = "ame-sniffer"
service_uuid = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"
allowed_topics = [
  "mission.position-report",
  "mission.signal-report",
  "mission.observation-measurement-report",
  "mission.service-status"
]
EOF
```

Then add a volume mount to `external/ams-gra/sleet/compose.yaml` (edited
directly — this is a local clone, not upstream, so it's ours to change):

```yaml
services:
  sleet:
    volumes:
      - /ABSOLUTE/PATH/TO/external/ams-gra/sleet/services.d.local:/opt/sleet/config/services.d:ro,Z
```

**Use an absolute host path, not `./services.d.local`.** `podman-compose`
resolves relative volume paths in an `include`-d compose file against the
**top-level project directory** (`getting-started/`), not the file's own
directory — a relative path here silently creates (and mounts) an empty
directory under `getting-started/`, and Sleet then fails to start with
`configuration error: services_dir './config/services.d' contains no
service TOML files`.

## 6. Bring the stack up

```bash
cd external/ams-gra/getting-started
podman-compose up -d
podman ps   # expect ~14 containers Up; prometheus/grafana may be absent, see below
```

**Known benign issues on podman 3.4.4** (harmless, don't block the demo):

- `Error validating CNI config file ...: plugin firewall does not support
  config version "1.0.0"` — a warning on every `podman`/`podman-compose`
  invocation. Cosmetic; the containers still start.
- `Error: invalid IP address in add-host: "host-gateway"` — `prometheus`
  (and therefore `grafana`, which depends on it) fails to start. Podman
  3.4.4 predates `host-gateway` support for `extra_hosts`. This only affects
  the Prometheus/Grafana observability side-stack, not Supercell / Squall /
  Sleet / the Skills / Graupel / Worldview.
- `WARNING:podman_compose: Ignored ... condition check due to podman 3.4.4
  doesn't support healthy!` — `depends_on: condition: service_healthy` is
  downgraded to unordered start. Everything that depends on ordering here
  (Couloir before the Squall OMS adapters, JSBSim before Supercell) retries
  with backoff, so it self-heals within a few seconds regardless.

**If you edit `sleet/compose.yaml` or its mounted `services.d.local` and
need to restart just Sleet** (`podman-compose up -d --force-recreate
sleet`), the Squall OMS adapters (`squall-rf-oms-adapter`,
`squall-ir-oms-adapter`) do not reconnect cleanly on their own afterwards —
they get stuck emitting `owp publish error: Operation canceled` instead of
retrying INIT. `podman restart squall-rf-oms-adapter
squall-ir-oms-adapter` clears it. `rf-fm-demod-skill` and
`ir-search-and-track-skill` do not have this problem; their own retry loop
recovers unattended.

## 7. Verify it's actually producing traffic

`podman logs <service_id>` per container is authoritative (the `podman logs
sleet` output shows `accepted`/`initialized`/`disconnected` per connection).
For the actual UCI content, use the read-only sniffer registered in §5:

```bash
python3 -m venv .venv && .venv/bin/pip install websocket-client
.venv/bin/python subprojects/PYRAMID/pim/test_harness/lacal/kittyhawk_owp_sniff.py --duration 30
```

Expect `PositionReport` (Supercell's Eagle-1 ownship kinematics, ~1 Hz),
`ObservationMeasurementReport` (`ir-search-and-track`'s LOS Az/El detections,
several Hz whenever the IR sensor has a target in frame) and `ServiceStatus`
(`NORMAL` from both Skills) within the first 30 seconds. `SignalReport`
(`rf-fm-demod`) is intermittent by design — the Kitty Hawk pentagon flight
pattern is built so the ownship genuinely gains and loses RF lock on the FM
tower, so an empty run isn't a failure.

## 8. Tear down / resume

```bash
cd external/ams-gra/getting-started
podman-compose down          # stop everything
podman-compose up -d         # resume later; images and clones persist locally
```

Nothing here is pushed anywhere or shared — `external/` is git-ignored, so
this is purely a local, restartable environment for validating Phase 6 work
against.
