#!/usr/bin/env bash
# Viability check for the new PIM proto set (subprojects/PYRAMID/pim/test/).
#
# 1. Generates C++ bindings from the new proto tree.
# 2. Syntax-checks a representative component facade against the PCL headers.
#
# Exits non-zero (with the compiler diagnostics) while the generator cannot
# consume the new proto shape. See FINDINGS.md for the two blocking defects.
set -u

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"          # subprojects/PYRAMID
root="$(cd "$pyramid/../.." && pwd)"          # repo root
proto_dir="$pyramid/pim/test"
out="${1:-$here/generated}"

echo "== generating bindings from $proto_dir =="
rm -rf "$out"; mkdir -p "$out"
python3 "$pyramid/pim/generate_bindings.py" "$proto_dir" "$out" --languages cpp \
    | grep -E 'Found|Done' || { echo "generation failed"; exit 2; }

inc=(-I"$out" -I"$out/"
     -I"$pyramid/../PCL/include"
     -I"$pyramid/core/external")

echo
echo "== syntax-checking osprey.sensor_products provided facade =="
cat > "$out/_probe.cpp" <<'EOF'
#include "pyramid_services_pim_osprey_sensor_products_provided.hpp"
int main() { return 0; }
EOF

if g++ -std=c++17 "${inc[@]}" -fsyntax-only "$out/_probe.cpp"; then
    echo "PASS: bindings compile -- run components_comms_test.cpp next"
    exit 0
else
    echo
    echo "FAIL: new proto is not yet viable for the C++ plugin bindings."
    echo "      See FINDINGS.md."
    exit 1
fi
