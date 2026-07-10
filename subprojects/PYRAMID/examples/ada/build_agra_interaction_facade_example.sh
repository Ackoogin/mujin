#!/usr/bin/env bash
# Build + run the Ada interaction-facade showcase (WS-F/F2c):
# agra_interaction_facade_example.adb demonstrates the generated
# MA_Action_Client_Bind/Submit_*/Transitions and
# MA_Action_Provider_Bind/Send_Transition surface end to end, with the
# realization picked at the command line (same manifest-style Config_Json
# string on both sides -- see examples/cpp/agra_interaction_facade_example.cpp
# for the C++ analogue this mirrors).
#
# Usage: build_agra_interaction_facade_example.sh [--binding=rpc|pubsub]
#
# Requires GNAT/gprbuild (gnatmake) on PATH.
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated_agra_interaction_facade_example"

command -v gnatmake >/dev/null 2>&1 || {
  echo "gnatmake not found on PATH -- install GNAT to run this example."
  exit 2
}

echo "== generating bindings (ada + cpp, for the shared C-ABI marshal layer) =="
rm -rf "$gen"; mkdir -p "$gen"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/agra_example" "$gen" \
    --languages cpp,ada --backends json >/dev/null

libpcl=""
for candidate in $(find -L "$root" -path '*/libpcl_core.a' -type f | sort); do
  if nm -g "$candidate" 2>/dev/null | grep -q ' T pcl_executor_create$'; then
    libpcl="$candidate"
    break
  fi
done
[ -n "$libpcl" ] || { echo "libpcl_core.a not found under build*/ -- build PCL first"; exit 2; }

pcl_include="$pyramid/../PCL/include"
pcl_ada="$pyramid/../PCL/bindings/ada"
tl_optional_include="$pyramid/core/external"

echo "== building per-role JSON codec plugins (.so, dlopen'd at runtime) =="
common_pkgs=(pyramid_data_model_base pyramid_data_model_common pyramid_data_model_agra)

pcl_alloc_obj="$gen/pcl_alloc.o"
cc -std=c11 -fPIC -I"$pcl_include" -c "$pyramid/../PCL/src/pcl_alloc.c" -o "$pcl_alloc_obj"

build_codec_plugin() {
  local role_label="$1"      # ma | c2
  local components_pkg="$2"  # e.g. pyramid_components_agra_mission_autonomy_services_provided
  local plugin_pkg="$3"      # e.g. pyramid_services_agra_mission_autonomy

  local pkgs=("${common_pkgs[@]}" "$components_pkg")
  local srcs=("$gen/${plugin_pkg}_json_codec_plugin.cpp")
  for p in "${pkgs[@]}"; do
    for suffix in _codec.cpp _cabi_marshal.cpp; do
      [ -f "$gen/$p$suffix" ] && srcs+=("$gen/$p$suffix")
    done
  done
  srcs+=("$pcl_alloc_obj")
  g++ -std=c++17 -fPIC -shared -I"$gen" -I"$pcl_include" -I"$tl_optional_include" \
      "${srcs[@]}" -o "$here/lib${role_label}_agra_json_codec.so"
}

build_codec_plugin ma pyramid_components_agra_mission_autonomy_services_provided \
    pyramid_services_agra_mission_autonomy
build_codec_plugin c2 pyramid_components_agra_c2_station_services_consumed \
    pyramid_services_agra_c2_station

echo "== compiling the shared C-ABI marshal layer the Ada bindings call into =="
marshal_names=(
  pyramid_data_model_base_cabi_marshal
  pyramid_data_model_common_cabi_marshal
  pyramid_data_model_agra_cabi_marshal
  pyramid_components_agra_mission_autonomy_services_provided_cabi_marshal
  pyramid_components_agra_c2_station_services_consumed_cabi_marshal
)
marshal_objs=()
for name in "${marshal_names[@]}"; do
  g++ -std=c++17 -c -I"$gen" -I"$tl_optional_include" -I"$pcl_include" \
      "$gen/$name.cpp" -o "$gen/$name.o"
  marshal_objs+=("$gen/$name.o")
done

echo "== compiling + linking the example =="
gnatmake -q -gnat2020 -I"$gen" -I"$pcl_ada" \
    -o "$here/agra_interaction_facade_example" \
    "$here/agra_interaction_facade_example.adb" \
    -largs "${marshal_objs[@]}" "$libpcl" -ldl -lpthread -lstdc++

echo "== running =="
"$here/agra_interaction_facade_example" \
    "$here/libma_agra_json_codec.so" "$here/libc2_agra_json_codec.so" "$@"
