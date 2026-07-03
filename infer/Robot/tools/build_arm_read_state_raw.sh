#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot_dir="$(cd "${script_dir}/.." && pwd)"
output="${1:-/tmp/arm_read_state_raw}"

g++ -std=c++17 -O2 \
    -I"${robot_dir}/include" \
    "${robot_dir}/src/arm_read_state_raw.cpp" \
    -o "${output}"

echo "Built ${output}"
