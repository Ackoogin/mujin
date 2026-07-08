#!/usr/bin/env bash
# Build + run Phase A of doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md:
# the routed-egress SHM proof. Generates C++/JSON bindings for the existing
# pim_osprey sensor_products contract, compiles them + the JSON codec plugin
# against libpcl_core, and drives the request/requirement/information topics
# through a real libpcl_transport_shared_memory_plugin.so loaded via
# pcl_transport_routing_load -- first with two executors in one process, then
# split across two OS processes.
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated_routed_egress"
scratch="$here/routed_egress_scratch"

echo "== generating bindings =="
rm -rf "$gen"; mkdir -p "$gen"
rm -rf "$scratch"; mkdir -p "$scratch"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/test" "$gen" \
    --languages cpp --backends json >/dev/null

libpcl=""
for candidate in $(find "$root" -path '*/libpcl_core.a' -type f | sort); do
  if nm -g "$candidate" 2>/dev/null | grep -q ' T pcl_transport_routing_load$'; then
    libpcl="$candidate"
    break
  fi
done
[ -n "$libpcl" ] || { echo "libpcl_core.a with pcl_transport_routing_load not found under build*/"; exit 2; }

shm_plugin=""
for candidate in $(find "$root" -name 'libpcl_transport_shared_memory_plugin.so' -type f | sort); do
  shm_plugin="$candidate"
  break
done
[ -n "$shm_plugin" ] || { echo "libpcl_transport_shared_memory_plugin.so not found under build*/"; exit 2; }

inc=(-I"$gen" -I"$pyramid/../PCL/include" -I"$pyramid/core/external")

# Generated sources in the osprey sensor_products dependency closure (same
# closure as build_comms_test.sh -- the packages its RPC payloads + oneof
# wrappers reference). Other components are omitted to keep the link tight.
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
srcs=("$here/routed_egress_shm_test.cpp"
      "$gen/pyramid_services_pim_osprey_sensor_products_provided.cpp"
      "$gen/pyramid_services_pim_osprey_sensor_products_json_codec_plugin.cpp")
for p in "${pkgs[@]}"; do
  for suffix in _codec.cpp _cabi_marshal.cpp; do
    [ -f "$gen/$p$suffix" ] && srcs+=("$gen/$p$suffix")
  done
done

echo "== compiling ($(echo "${srcs[@]}" | wc -w) sources) =="
g++ -std=c++17 "${inc[@]}" "${srcs[@]}" "$libpcl" -ldl -lpthread \
    -o "$here/routed_egress_shm_test"

echo "== running =="
"$here/routed_egress_shm_test" "$shm_plugin" "$scratch"
