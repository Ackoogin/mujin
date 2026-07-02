#!/usr/bin/env bash
# build_plugin_load_test.sh -- no-relink plugin demonstration (Linux).
#
# Proves a component built from the new PIM proto can use a *prebuilt* codec
# plugin with no relink, and that configuration is threaded through the loader
# to the plugin:
#
#   1. Generate C++/JSON bindings from pim/test.
#   2. Build the osprey sensor_products JSON codec plugin as a STANDALONE .so
#      (g++ -shared) -- the plugin, complete with its wire codec.
#   3. Build plugin_load_test from the component-side sources ONLY (native
#      types, C-ABI marshal, service facade, libpcl_core.a). No codec/plugin
#      source is compiled into the test binary.
#   4. Run the test: it loads the .so at runtime via pcl_plugin_load_codec(),
#      passing a config_json string, and drives a full provider/client
#      round-trip through the loaded codec.
#
# Windows counterpart: build_plugin_load_test.bat.
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated"
cfg="${1:-}"                       # optional config_json override
plugin_so="$here/libosprey_sensor_products_json_codec.so"

echo "== generating bindings =="
rm -rf "$gen"; mkdir -p "$gen"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/test" "$gen" \
    --languages cpp --backends json >/dev/null

libpcl=""
for candidate in $(find "$root" -path '*/libpcl_core.a' -type f | sort); do
  if nm -g "$candidate" 2>/dev/null | grep -q ' T pcl_alloc$'; then
    libpcl="$candidate"
    break
  fi
done
[ -n "$libpcl" ] || { echo "libpcl_core.a with pcl_alloc not found under build*/"; exit 2; }

inc=(-I"$gen" -I"$pyramid/../PCL/include" -I"$pyramid/core/external")

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

# 1) codec plugin as a standalone .so (wire codec + marshal, no PCL runtime).
plug=("$gen/pyramid_services_pim_osprey_sensor_products_json_codec_plugin.cpp")
for p in "${pkgs[@]}"; do
  for suffix in _codec.cpp _cabi_marshal.cpp; do
    [ -f "$gen/$p$suffix" ] && plug+=("$gen/$p$suffix")
  done
done
pcl_alloc_obj="$gen/pcl_alloc.o"
cc -std=c11 -fPIC -I"$pyramid/../PCL/include" \
  -c "$pyramid/../PCL/src/pcl_alloc.c" -o "$pcl_alloc_obj"
plug+=("$pcl_alloc_obj")
echo "== building codec plugin .so ($(echo "${plug[@]}" | wc -w) sources) =="
g++ -std=c++17 -fPIC -shared "${inc[@]}" "${plug[@]}" -o "$plugin_so"

# 2) client/provider test with NO codec compiled in (component-side only).
test_srcs=("$here/plugin_load_test.cpp"
           "$gen/pyramid_services_pim_osprey_sensor_products_provided.cpp")
for p in "${pkgs[@]}"; do
  [ -f "$gen/${p}_cabi_marshal.cpp" ] && test_srcs+=("$gen/${p}_cabi_marshal.cpp")
done
echo "== building plugin_load_test (no codec compiled in) =="
g++ -std=c++17 "${inc[@]}" "${test_srcs[@]}" "$libpcl" -ldl -lpthread \
    -o "$here/plugin_load_test"

echo "== running (loads the .so at runtime, no relink) =="
"$here/plugin_load_test" "$plugin_so" ${cfg:+"$cfg"}
