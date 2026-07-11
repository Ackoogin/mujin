#!/usr/bin/env bash
# Build and, when a reachable Sleet is explicitly supplied, run the generated
# UCI ActionCommand interaction-facade LA-CAL seam witness.  It is deliberately
# script-driven: facade and OMS JSON sources are generated into scratch space.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated_lacal_generated_seam"
scratch="$here/lacal_generated_seam_scratch"
bin="$here/lacal_generated_seam_test"
codec="$here/liblacal_generated_seam_oms_json_codec.so"

cleanup() {
  if [[ -n "${provider_pid:-}" ]]; then kill "$provider_pid" 2>/dev/null || true; fi
  rm -rf "$scratch"
}
trap cleanup EXIT

echo "== generating UCI bindings =="
rm -rf "$gen" "$scratch"
mkdir -p "$gen" "$scratch"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/uci_seam_example" "$gen" \
  --languages cpp --backends oms_json >/dev/null
# The interaction facade's generic streaming helper requires these query
# fields; UCI's request contract declares an empty Query.  They are local
# client-side subscription controls and never serialized by the OMS codec.
sed -i 's/struct Query {$/struct Query {\n    std::vector<std::string> id = {};\n    tl::optional<bool> one_shot;/' \
  "$gen/pyramid_data_model_uci_types.hpp"

build_root="${PYRAMID_BUILD_DIR:-$root}"
[[ -d "$build_root" ]] || {
  echo "PYRAMID_BUILD_DIR is not a directory: $build_root"
  exit 2
}

libpcl="$build_root/subprojects/PCL/src/libpcl_core.a"
if [[ ! -f "$libpcl" ]]; then
  libpcl=""
fi
for candidate in $(find "$build_root" -path '*/libpcl_core.a' -type f | sort); do
  [[ -n "$libpcl" ]] && break
  # Capture nm output first: `nm | grep -q` under `set -o pipefail` reports
  # failure when grep short-circuits and nm dies of SIGPIPE, a false negative.
  nm_out="$(nm -g "$candidate" 2>/dev/null || true)"
  if grep -q ' T pcl_transport_routing_load$' <<<"$nm_out"; then
    libpcl="$candidate"
    break
  fi
done
[[ -n "$libpcl" ]] || {
  echo "libpcl_core.a with pcl_transport_routing_load not found under $build_root"
  exit 2
}

lacal_plugin="$(find "$build_root" -name 'libpyramid_lacal_transport_plugin.so' -type f | sort | head -1)"
[[ -n "$lacal_plugin" ]] || {
  echo "libpyramid_lacal_transport_plugin.so not found under $build_root"
  exit 2
}

inc=(-I"$gen" -I"$pyramid/../PCL/include" -I"$pyramid/core/external")
pcl_alloc_obj="$gen/pcl_alloc.o"
cc -std=c11 -fPIC -I"$pyramid/../PCL/include" -c "$pyramid/../PCL/src/pcl_alloc.c" -o "$pcl_alloc_obj"

codec_sources=("$gen/oms_json/cpp/pyramid_data_model_uci_oms_json_codec_plugin.cpp"
               "$gen/pyramid_data_model_uci_codec.cpp"
               "$gen/pyramid_data_model_uci_cabi_marshal.cpp"
               "$pcl_alloc_obj")
echo "== building generated OMS JSON codec .so =="
g++ -std=c++17 -fPIC -shared "${inc[@]}" "${codec_sources[@]}" -o "$codec"

main_pkgs=(pyramid_data_model_uci pyramid_components_uci_mission_autonomy_services_provided
           pyramid_components_uci_c2_station_services_consumed)
srcs=("$here/lacal_generated_seam_test.cpp"
      "$gen/pyramid_services_uci_mission_autonomy_provided.cpp"
      "$gen/pyramid_services_uci_c2_station_consumed.cpp")
for pkg in "${main_pkgs[@]}"; do
  for suffix in _codec.cpp _cabi_marshal.cpp; do
    [[ -f "$gen/$pkg$suffix" ]] && srcs+=("$gen/$pkg$suffix")
  done
done

echo "== compiling generated facade seam binary =="
g++ -std=c++17 "${inc[@]}" \
  -include "$gen/pyramid_components_uci_mission_autonomy_services_provided_types.hpp" \
  -include "$gen/pyramid_components_uci_c2_station_services_consumed_types.hpp" \
  -DPYRAMID_LACAL_PLUGIN_PATH=\"$lacal_plugin\" \
  -DPYRAMID_OMS_JSON_CODEC_PATH=\"$codec\" \
  "${srcs[@]}" "$libpcl" -ldl -lpthread -o "$bin"

if [[ -z "${SLEET_URL:-}" ]]; then
  echo "SKIP: set SLEET_URL to a running Sleet with lacal/services loaded"
  exit 0
fi

host_port="${SLEET_URL#ws://}"
host_port="${host_port%%/*}"
host="${host_port%:*}"
port="${host_port##*:}"
if [[ "$host" == "$host_port" || -z "$port" ]]; then
  echo "FAIL: SLEET_URL must be ws://host:port[/path]"
  exit 2
fi

reachable=false
for _ in $(seq 1 50); do
  if python3 - "$host" "$port" <<'PY'
import socket
import sys
try:
    with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=0.2):
        pass
except OSError:
    raise SystemExit(1)
PY
  then reachable=true; break; fi
  sleep 0.1
done
if [[ "$reachable" != true ]]; then
  echo "SKIP: Sleet is unreachable at ${SLEET_URL}"
  exit 0
fi

ready="$scratch/provider.ready"
output="$scratch/consumer.out"
"$bin" provider "$SLEET_URL" "$ready" "$scratch" 12 &
provider_pid=$!
for _ in $(seq 1 100); do
  [[ -f "$ready" ]] && break
  if ! kill -0 "$provider_pid" 2>/dev/null; then
    wait "$provider_pid" || true
    echo "FAIL: provider exited before becoming ready"
    exit 2
  fi
  sleep 0.1
done
[[ -f "$ready" ]] || { echo "FAIL: provider did not become ready"; exit 2; }
"$bin" consumer "$SLEET_URL" "$output" "$scratch" 12
if ! wait "$provider_pid"; then
  provider_pid=""
  echo "FAIL: provider did not observe the request"
  exit 2
fi
provider_pid=""
[[ -f "$output" ]] && grep -q '^seam-ok$' "$output" || {
  echo "FAIL: consumer did not receive correlated RECEIVED and ACCEPTED transitions"
  exit 2
}
echo "PASS: generated UCI facade LA-CAL seam over Sleet"
