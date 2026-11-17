#!/bin/sh
# =============================================================================
# File      sim/xcelium/run.sh                                     Tracker: F63
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   The Xcelium flow (xrun; irun with IRUN=1, for Incisive and
#           NCLaunch): build and run any test by its ID, as run.tcl does for
#           xsim. Run from the repository root; each build goes to its own
#           folder sim/xcelium/work/<testbench>[__<defines>].
# Usage     sh sim/xcelium/run.sh -test CT02 [-define TB_MESH_N=8]... [-plus +QUICK]...
#                                 [-seed n] [-bench tb_mesh_d] [-gui]
#           sh sim/xcelium/run.sh -list b7           (SIM=xrun sh sim/vcs/regress.sh b7)
#   -test   a test ID (U01 ... CT05, PERF, S01 ...) or a testbench name
#   -bench  the testbench, when the test runs on several (PERF, N01, S01 ...)
#   -gui    SimVision (-access +rwc -gui) instead of a batch run
# Environment: LOG (the log's name in the build folder, default
# xrun_<test>.log), WSUF (added to the build folder's name; sweep.sh parts)
# NCLaunch: point a run at sim/files_rtl.f and sim/files_tb.f (in that order),
# the include folders tb/common, tb/mesh and tb/checkers, the top module
# this script prints, and the plusargs +SEED=1 +TEST=<id>.
# =============================================================================
cd "$(dirname "$0")/../.." || exit 1
TEST=U01; BENCH=""; SEED=1; DEFS=""; PLUS=""; GUI=0
while [ $# -gt 0 ]; do
  case "$1" in
    -test)   TEST=$2; shift 2 ;;
    -bench)  BENCH=$2; shift 2 ;;
    -seed)   SEED=$2; shift 2 ;;
    -define) DEFS="$DEFS $2"; shift 2 ;;
    -plus)   PLUS="$PLUS $2"; shift 2 ;;
    -gui)    GUI=1; shift ;;
    -list)   SIM=xrun exec sh sim/vcs/regress.sh "$2" "$SEED" ;;
    *) echo "run.sh: unknown option $1 (options: -test -bench -seed -define -plus -gui -list)"; exit 1 ;;
  esac
done
TBS=$(grep -v '^[[:space:]]*//' sim/files_tb.f | grep -v '^[[:space:]]*$' | grep -v '^+')
if [ -n "$BENCH" ]; then TOP=$BENCH
elif echo "$TEST" | grep -q '^tb_'; then TOP=$TEST
else
  TOP=""
  for f in $TBS; do
    if grep -qE "//[[:space:]]*@tests[[:space:]].*(^|[[:space:]])$TEST([[:space:]]|\$)" "$f"; then TOP=$(basename "$f" .sv); break; fi
  done
fi
[ -n "$TOP" ] || { echo "run.sh: no testbench runs '$TEST' (give -bench)"; exit 1; }
SUF=""; for d in $DEFS; do SUF="${SUF}__$(echo "$d" | sed 's/[^A-Za-z0-9]/_/g')"; done
WORK=sim/xcelium/work/$TOP$SUF${WSUF:-}
LOG=${LOG:-xrun_$TEST.log}
mkdir -p "$WORK"
TESTARG=""; echo "$TEST" | grep -q '^tb_' || TESTARG="+TEST=$TEST"
DARGS=""; for d in $DEFS; do DARGS="$DARGS +define+$d"; done
TOOL=xrun; LIBOPT=-xmlibdirname; LIBD=xcelium.d
[ "${IRUN:-0}" = "1" ] && { TOOL=irun; LIBOPT=-nclibdirname; LIBD=INCA_libs; }
GARGS=""; [ "$GUI" = "1" ] && GARGS="-access +rwc -gui"
echo "run.sh: $TOOL $TOP $DEFS, seed $SEED $TESTARG $PLUS"
$TOOL -64bit -sv -timescale 1ns/1ps -f sim/files_rtl.f -f sim/files_tb.f $DARGS -top "$TOP" \
      $LIBOPT "$WORK/$LIBD" -l "$WORK/$LOG" $GARGS +SEED=$SEED $TESTARG $PLUS > /dev/null 2>&1
r=$(grep -a -m1 '^RESULT' "$WORK/$LOG")
c=$(grep -a 'cycles simulated' "$WORK/$LOG" | tail -1 | awk '{print $NF}')
if [ -n "$r" ]; then echo "$TOP $TEST seed $SEED: $r ${c:+($c cycles)}"; else echo "$TOP $TEST: NO RESULT LINE, see $WORK/$LOG"; head -5 "$WORK/$LOG"; exit 1; fi
