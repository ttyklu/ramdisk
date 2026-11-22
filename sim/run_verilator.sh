#!/usr/bin/env bash
# Build and run the NVMe RAM disk testbench with Verilator (5.x, --timing).
#   ./run_verilator.sh             build + batch run, log in verilator.log
#   ./run_verilator.sh waves       also dump waves.vcd (open with: gtkwave waves.vcd)
#   ./run_verilator.sh seeds N     run N random seeds (build once), report pass/fail per seed
#   SEED=123 ./run_verilator.sh    run with a specific random seed
#   REC=20 ./run_verilator.sh      build with 1 MB compression records (default REC=17: 128 KB)
set -euo pipefail
cd "$(dirname "$0")"
command -v verilator >/dev/null || { echo "verilator not found" >&2; exit 1; }

rec="${REC:-17}"
big=""; [ "$rec" -ge 20 ] && big="--big"
python3 ../scripts/gen_zvec.py zvec.hex $big > /dev/null
verilator --binary --timing -sv --trace -j "$(nproc)" \
  -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD -I../tb -GREC_LOG2="$rec" \
  -f files.f --top-module tb_top --Mdir "obj_vl_$rec" -o Vtb_top > verilator_build.log 2>&1 \
  || { tail -30 verilator_build.log; exit 1; }
exe="./obj_vl_$rec/Vtb_top"

seed="${SEED:-1}"
case "${1:-}" in
  waves)
    $exe +verilator+seed+"$seed" +zvec=zvec.hex +vcd 2>&1 | tee verilator.log ;;
  seeds)
    n="${2:-20}"; fails=0
    for s in $(seq 1 "$n"); do
      if $exe +verilator+seed+"$s" +zvec=zvec.hex > "verilator_seed$s.log" 2>&1 \
         && grep -q "TEST PASSED" "verilator_seed$s.log"; then
        echo "seed $s: PASS"; rm -f "verilator_seed$s.log"
      else
        echo "seed $s: FAIL (see verilator_seed$s.log)"; fails=$((fails + 1))
      fi
    done
    echo "$((n - fails))/$n seeds passed"
    [ "$fails" -eq 0 ] ;;
  *)
    $exe +verilator+seed+"$seed" +zvec=zvec.hex 2>&1 | tee verilator.log ;;
esac
grep -E "TEST (PASSED|FAILED)" verilator.log 2>/dev/null || true
