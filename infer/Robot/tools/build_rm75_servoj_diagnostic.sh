#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot_dir="$(cd "${script_dir}/.." && pwd)"
output="${1:-${robot_dir}/build/rm75_servoj_diagnostic}"
mkdir -p "$(dirname "${output}")"

g++ -std=c++17 -O2 -pthread \
    -I"${robot_dir}/include" \
    -I/usr/include/eigen3 \
    "${robot_dir}/src/arm_servoj_line_test.cpp" \
    "${robot_dir}/src/realman_command.cpp" \
    "${robot_dir}/src/realman_kinematics.cpp" \
    -o "${output}"

echo "Built ${output}"
