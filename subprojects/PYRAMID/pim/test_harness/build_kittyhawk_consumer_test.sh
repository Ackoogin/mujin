#!/usr/bin/env bash
# Build/run the generated-P1 Kitty Hawk consumer. This is not CTest-registered:
# it requires the user-managed multi-container Kitty Hawk stack.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated_kittyhawk_consumer"
bin="$here/kittyhawk_consumer_test"
codec="$here/libkittyhawk_consumer_oms_json_codec.so"
scratch="$here/kittyhawk_consumer_scratch"

cleanup() { rm -rf "$scratch"; }
trap cleanup EXIT

if [[ -z "${KITTYHAWK_SLEET_URL:-}" ]]; then
  echo "SKIP: set KITTYHAWK_SLEET_URL to the running Kitty Hawk Sleet"
  exit 0
fi
host_port="${KITTYHAWK_SLEET_URL#ws://}"; host_port="${host_port%%/*}"
host="${host_port%:*}"; port="${host_port##*:}"
if [[ "$host" == "$host_port" || -z "$port" ]]; then
  echo "FAIL: KITTYHAWK_SLEET_URL must be ws://host:port[/path]"
  exit 2
fi
if ! python3 - "$host" "$port" <<'PY'
import socket, sys
try:
    socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=1).close()
except OSError:
    raise SystemExit(1)
PY
then
  echo "SKIP: Sleet is unreachable at ${KITTYHAWK_SLEET_URL}"
  exit 0
fi

rm -rf "$gen"
mkdir -p "$scratch"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/uci_p1_seam" "$gen" \
  --languages cpp --backends oms_json >/dev/null
build_root="${PYRAMID_BUILD_DIR:-$root}"
libpcl="$(find "$build_root" -path '*/libpcl_core.a' -type f | sort | head -1)"
lacal_plugin="$(find "$build_root" -name 'libpyramid_lacal_transport_plugin.so' -type f | sort | head -1)"
[[ -n "$libpcl" && -n "$lacal_plugin" ]] || { echo "FAIL: required PCL/LA-CAL build artifacts not found"; exit 2; }
inc=(-I"$gen" -I"$pyramid/../PCL/include" -I"$pyramid/core/external")
cc -std=c11 -fPIC -I"$pyramid/../PCL/include" -c "$pyramid/../PCL/src/pcl_alloc.c" -o "$gen/pcl_alloc.o"
g++ -std=c++17 -fPIC -shared "${inc[@]}" \
  "$gen/oms_json/cpp/pyramid_data_model_uci_oms_json_codec_plugin.cpp" \
  "$gen/pyramid_data_model_uci_codec.cpp" "$gen/pyramid_data_model_uci_cabi_marshal.cpp" \
  "$gen/pcl_alloc.o" -o "$codec"
g++ -std=c++17 "${inc[@]}" \
  -DPYRAMID_LACAL_PLUGIN_PATH=\"$lacal_plugin\" -DPYRAMID_OMS_JSON_CODEC_PATH=\"$codec\" \
  "$here/lacal/kittyhawk_consumer_test.cpp" \
  "$gen/pyramid_services_uci_information_provided.cpp" \
  "$gen/pyramid_components_uci_information_services_provided_codec.cpp" \
  "$gen/pyramid_components_uci_information_services_provided_cabi_marshal.cpp" \
  "$gen/pyramid_data_model_uci_codec.cpp" \
  "$gen/pyramid_data_model_uci_cabi_marshal.cpp" \
  "$libpcl" -ldl -lpthread -o "$bin"
"$bin" "$KITTYHAWK_SLEET_URL" "$scratch"
