#!/usr/bin/env bash
# Build + run the plugin-system comms test for the new PIM proto.
# Generates C++/JSON bindings, compiles the generated sources + the osprey
# sensor_products JSON codec plugin against libpcl_core, and runs the test.
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated"

echo "== generating bindings =="
rm -rf "$gen"; mkdir -p "$gen"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/test" "$gen" \
    --languages cpp --backends json >/dev/null

libpcl="$(find "$root/build" -name libpcl_core.a | head -1)"
[ -n "$libpcl" ] || { echo "libpcl_core.a not found under build/"; exit 2; }

inc=(-I"$gen" -I"$pyramid/../PCL/include" -I"$pyramid/core/external")

# Generated sources in the osprey sensor_products dependency closure (the
# packages its RPC payloads + oneof wrappers reference). Other components are
# omitted to keep the link tight.
pkgs=(
  pyramid_data_model_base
  pyramid_data_model_common
  pyramid_data_model_generic_pim_generic
  pyramid_data_model_common_pim_components_authorisation
  pyramid_data_model_common_pim_components_sensor_products
  pyramid_data_model_pim_osprey_sensor_products
  pyramid_components_pim_osprey_sensor_products_services_provided
)
srcs=("$here/components_comms_test.cpp"
      "$gen/pyramid_services_pim_osprey_sensor_products_provided.cpp"
      "$gen/pyramid_services_pim_osprey_sensor_products_json_codec_plugin.cpp")
for p in "${pkgs[@]}"; do
  for suffix in _codec.cpp _cabi_marshal.cpp; do
    [ -f "$gen/$p$suffix" ] && srcs+=("$gen/$p$suffix")
  done
done

echo "== compiling ($(echo "${srcs[@]}" | wc -w) sources) =="
g++ -std=c++17 "${inc[@]}" "${srcs[@]}" "$libpcl" -lpthread \
    -o "$here/comms_test"

echo "== running =="
"$here/comms_test"
