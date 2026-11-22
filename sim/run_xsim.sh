#!/usr/bin/env bash
# Compile and run the NVMe RAM disk testbench with Vivado's simulator (xsim).
#   ./run_xsim.sh          batch run, log in xsim.log
#   ./run_xsim.sh waves    batch run and record all signals to waves.wdb
#   ./run_xsim.sh gui      open the xsim GUI
set -euo pipefail
cd "$(dirname "$0")"
if ! command -v xvlog >/dev/null; then
  echo "xvlog not found: run  source <Vivado install>/settings64.sh  first" >&2
  exit 1
fi
rec="${REC:-17}"
big=""; [ "$rec" -ge 20 ] && big="--big"
python3 ../scripts/gen_zvec.py zvec.hex $big > /dev/null
xvlog -sv -i ../tb -f files.f 2>&1 | tee xvlog.log
xelab -debug typical -generic_top "REC_LOG2=$rec" tb_top -s tb_top_sim 2>&1 | tee xelab.log
case "${1:-}" in
  gui)   xsim tb_top_sim -testplusarg zvec=zvec.hex -gui ;;
  waves) xsim tb_top_sim -testplusarg zvec=zvec.hex -tclbatch waves.tcl -wdb waves.wdb 2>&1 | tee xsim.log ;;
  *)     xsim tb_top_sim -testplusarg zvec=zvec.hex -R 2>&1 | tee xsim.log ;;
esac
grep -E "TEST (PASSED|FAILED)" xsim.log || true
