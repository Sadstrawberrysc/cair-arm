#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "--execute" ]]; then
  echo "This script performs 10 real RM75 motions." >&2
  echo "Usage: $0 --execute" >&2
  exit 2
fi
shift

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="$ROOT_DIR/infer/Robot/build"
BIN="$BUILD_DIR/rm75_servoj_diagnostic"

if [[ ! -x "$BIN" ]]; then
  "$ROOT_DIR/infer/Robot/tools/build_rm75_servoj_diagnostic.sh"
fi

RUN_DIR="$BUILD_DIR/logs/precision_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"

SUMMARY="$RUN_DIR/summary.csv"
echo "case,final_position_error_cm,final_rotation_error_deg,max_lateral_error_cm,missed_periods,csv,svg" > "$SUMMARY"

run_case() {
  local name="$1"
  shift

  local csv="$RUN_DIR/${name}.csv"
  local out="$RUN_DIR/${name}.txt"
  echo
  echo "=== $name ==="
  "$BIN" "$@" --trajectory-log "$csv" --summary-only --execute | tee "$out"

  local final_position_error_cm final_rotation_error_deg max_lateral_error_cm missed_periods
  final_position_error_cm="$(awk -F, '$1=="# summary" && $2=="final_position_error_cm"{print $3}' "$csv")"
  final_rotation_error_deg="$(awk -F, '$1=="# summary" && $2=="final_rotation_error_deg"{print $3}' "$csv")"
  max_lateral_error_cm="$(awk -F, '$1=="# summary" && $2=="max_lateral_error_cm"{print $3}' "$csv")"
  missed_periods="$(awk -F, '$1=="# summary" && $2=="missed_periods"{print $3}' "$csv")"

  echo "${name},${final_position_error_cm},${final_rotation_error_deg},${max_lateral_error_cm},${missed_periods},${csv},${csv%.csv}.svg" >> "$SUMMARY"
}

cd "$BUILD_DIR"

run_case "single_x_neg_5cm" --delta-cm "-5,0,0"
run_case "single_x_return_5cm" --delta-cm "5,0,0"
run_case "single_y_neg_5cm" --delta-cm "0,-5,0"
run_case "single_y_return_5cm" --delta-cm "0,5,0"
run_case "single_z_neg_5cm" --delta-cm "0,0,-5"
run_case "single_z_return_5cm" --delta-cm "0,0,5"

run_case "combo_xyz_neg_norm_5cm" --delta-cm "-2.88675,-2.88675,-2.88675"
run_case "combo_xyz_return_5cm" --delta-cm "2.88675,2.88675,2.88675"

run_case "pure_rz_neg_5deg" --delta-rotation-deg "0,0,-5"
run_case "pure_rz_return_5deg" --delta-rotation-deg "0,0,5"

echo
echo "precision_summary: $SUMMARY"
