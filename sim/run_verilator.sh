#!/usr/bin/env bash
# Build and run the NVMe RAM disk testbench with Verilator (5.x, --timing).
#   ./run_verilator.sh             build + batch run, log in verilator.log
#   ./run_verilator.sh waves       also dump waves.vcd (open with: gtkwave waves.vcd)
#   ./run_verilator.sh seeds N     run N random seeds (build once), report pass/fail per seed
#   SEED=123 ./run_verilator.sh    run with a specific random seed
set -euo pipefail
cd "$(dirname "$0")"
command -v verilator >/dev/null || { echo "verilator not found" >&2; exit 1; }

verilator --binary --timing -sv --trace -j "$(nproc)" \
  -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD \
  -f files.f --top-module tb_top --Mdir obj_vl -o Vtb_top > verilator_build.log 2>&1 \
  || { tail -30 verilator_build.log; exit 1; }

seed="${SEED:-1}"
case "${1:-}" in
  waves)
    ./obj_vl/Vtb_top +verilator+seed+"$seed" +vcd 2>&1 | tee verilator.log ;;
  seeds)
    n="${2:-20}"; fails=0
    for s in $(seq 1 "$n"); do
      if ./obj_vl/Vtb_top +verilator+seed+"$s" > "verilator_seed$s.log" 2>&1 \
         && grep -q "TEST PASSED" "verilator_seed$s.log"; then
        echo "seed $s: PASS"; rm -f "verilator_seed$s.log"
      else
        echo "seed $s: FAIL (see verilator_seed$s.log)"; fails=$((fails + 1))
      fi
    done
    echo "$((n - fails))/$n seeds passed"
    [ "$fails" -eq 0 ] ;;
  *)
    ./obj_vl/Vtb_top +verilator+seed+"$seed" 2>&1 | tee verilator.log ;;
esac
grep -E "TEST (PASSED|FAILED)" verilator.log 2>/dev/null || true
