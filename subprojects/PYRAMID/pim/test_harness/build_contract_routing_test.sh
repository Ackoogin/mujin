#!/usr/bin/env bash
# Build + run contract-derived endpoint routing validation.
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated_routing"
plugin="$here/libcontract_transport_plugin.so"
route_manifest="$gen/contract_routes.pcl"

echo "== generating bindings =="
rm -rf "$gen"; mkdir -p "$gen"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/test" "$gen" \
    --languages cpp --backends json >/dev/null

libpcl=""
for candidate in $(find "$root" -path '*/libpcl_core.a' -type f | sort); do
  if nm -g "$candidate" 2>/dev/null | grep -q ' T pcl_transport_routing_load$'; then
    libpcl="$candidate"
    break
  fi
done
[ -n "$libpcl" ] || { echo "libpcl_core.a with routing API not found under build*/"; exit 2; }

inc=(-I"$pyramid/../PCL/include")

echo "== building contract transport plugin =="
cc -std=c11 -fPIC -shared "${inc[@]}" \
  "$here/contract_transport_plugin.c" -o "$plugin"

echo "== deriving routing manifest from binding manifest =="
python3 "$here/contract_routing_manifest.py" \
  "$gen/binding_manifest.json" "$plugin" "$route_manifest"

echo "== compiling validation harness =="
g++ -std=c++17 "${inc[@]}" "$here/contract_routing_validation.cpp" \
  "$libpcl" -ldl -lpthread -o "$here/contract_routing_validation"

echo "== running =="
"$here/contract_routing_validation" "$route_manifest"
