#!/usr/bin/env bash
# Build + run the plugin-system comms test for the new PIM proto.
# Generates C++/JSON bindings, compiles the generated sources + the osprey
# sensor_products JSON codec plugin against libpcl_core, and runs the test.
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated"
flat_plugin="$here/libosprey_sensor_products_flatbuffers_codec.so"

echo "== generating bindings =="
rm -rf "$gen"; mkdir -p "$gen"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/test" "$gen" \
    --languages cpp --backends json,flatbuffers >/dev/null

libpcl=""
for candidate in $(find "$root" -path '*/libpcl_core.a' -type f | sort); do
  if nm -g "$candidate" 2>/dev/null | grep -q ' T pcl_alloc$'; then
    libpcl="$candidate"
    break
  fi
done
[ -n "$libpcl" ] || { echo "libpcl_core.a with pcl_alloc not found under build*/"; exit 2; }
flatc="$(find "$root" -path '*/_deps/flatbuffers-build/flatc' -type f -perm -u+x | head -1)"
[ -n "$flatc" ] || { echo "flatc not found under build*/_deps/flatbuffers-build/"; exit 2; }
flat_inc="$(find "$root" -path '*/_deps/flatbuffers-src/include/flatbuffers/flatbuffers.h' -type f | head -1)"
[ -n "$flat_inc" ] || { echo "FlatBuffers headers not found under build*/_deps/flatbuffers-src/include/"; exit 2; }
flat_inc_dir="$(dirname "$(dirname "$flat_inc")")"

inc=(-I"$gen" -I"$pyramid/../PCL/include" -I"$pyramid/core/external" -I"$flat_inc_dir")

echo "== generating FlatBuffers headers =="
"$flatc" --cpp --gen-object-api -o "$gen/flatbuffers/cpp" \
  "$gen/flatbuffers/cpp/pyramid_services_pim_osprey_sensor_products.fbs"

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
  pyramid_components_pim_osprey_sensor_products_services_consumed
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

plug=("$gen/pyramid_services_pim_osprey_sensor_products_flatbuffers_codec_plugin.cpp"
      "$gen/flatbuffers/cpp/pyramid_services_pim_osprey_sensor_products_flatbuffers_codec.cpp")
for p in "${pkgs[@]}"; do
  for suffix in _codec.cpp _cabi_marshal.cpp; do
    [ -f "$gen/$p$suffix" ] && plug+=("$gen/$p$suffix")
  done
done
pcl_alloc_obj="$gen/pcl_alloc.o"
cc -std=c11 -fPIC -I"$pyramid/../PCL/include" \
  -c "$pyramid/../PCL/src/pcl_alloc.c" -o "$pcl_alloc_obj"
plug+=("$pcl_alloc_obj")
echo "== building FlatBuffers codec plugin .so ($(echo "${plug[@]}" | wc -w) sources) =="
g++ -std=c++17 -fPIC -shared "${inc[@]}" "${plug[@]}" -o "$flat_plugin"

echo "== compiling ($(echo "${srcs[@]}" | wc -w) sources) =="
g++ -std=c++17 "${inc[@]}" "${srcs[@]}" "$libpcl" -ldl -lpthread \
    -o "$here/comms_test"

echo "== running =="
"$here/comms_test" "$flat_plugin"
