#!/bin/sh
# =============================================================================
# File      sim/vcs/regress.sh                                     Tracker: F62
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   Runs a regression list (sim/lists/<name>.list, the format of
#           regress.tcl) with VCS (default) or Xcelium and checks every
#           RESULT line against the count the list expects at seed 1.
# Usage     sh sim/vcs/regress.sh <list> [seed]            (from the repository root)
#           SIM=xrun sh sim/vcs/regress.sh <list> [seed]   (Xcelium, through sim/xcelium/run.sh)
#           make -f sim/vcs/Makefile regress LIST=<list>
# Output    one line per entry (PASS, COUNT, FAIL, NONE) and a summary, also in
#           sim/vcs/work/regress_<list>_seed<n>.txt (sim/xcelium/work/... for xrun)
# =============================================================================
LIST=${1:?usage: regress.sh <list> [seed]}
SEED=${2:-1}
SIM=${SIM:-vcs}
cd "$(dirname "$0")/../.." || exit 1
F=sim/lists/$LIST.list
[ -f "$F" ] || { echo "regress.sh: no list $F"; exit 1; }
if [ "$SIM" = "xrun" ]; then OUTD=sim/xcelium/work; else OUTD=sim/vcs/work; fi
mkdir -p $OUTD
OUT=$OUTD/regress_${LIST}_seed$SEED.txt
: > $OUT
np=0; nc=0; nf=0; nn=0
sed 's/#.*//' "$F" | while read -r label test exp rest; do
  [ -z "$label" ] && continue
  defs=""; plus=""
  set -- $rest
  while [ $# -gt 0 ]; do
    case "$1" in
      -define) defs="$defs $2"; shift 2 ;;
      -plus)   plus="$plus $2"; shift 2 ;;
      *) echo "regress.sh: $LIST.list: $label: unknown option $1"; shift ;;
    esac
  done
  if [ "$SIM" = "xrun" ]; then
    args=""; for d in $defs; do args="$args -define $d"; done; for p in $plus; do args="$args -plus $p"; done
    line=$(sh sim/xcelium/run.sh -test "$test" -seed "$SEED" $args < /dev/null 2>&1 | grep -a 'RESULT\|NO RESULT' | tail -1)
  else
    line=$(make -s -f sim/vcs/Makefile run TEST="$test" DEFINES="$defs" PLUS="$plus" SEED="$SEED" < /dev/null 2>&1 | grep -a 'RESULT\|NO RESULT\|failed\|no testbench' | tail -1)
  fi
  chk=$(echo "$line" | sed -n 's/.*checks=\([0-9]*\).*/\1/p')
  if echo "$line" | grep -q ' PASS '; then
    v=PASS
    if [ "$SEED" = "1" ] && [ "$exp" != "-" ] && [ "$chk" != "$exp" ]; then v=COUNT; fi
  elif echo "$line" | grep -q ' FAIL '; then v=FAIL
  else v=NONE
  fi
  e=""; [ "$SEED" = "1" ] && [ "$exp" != "-" ] && e=" (expected $exp)"
  printf '%-14s %-12s %-6s checks=%s%s\n' "$label" "$test" "$v" "${chk:--}" "$e" | tee -a $OUT
done
echo "summary: $(grep -c ' PASS ' $OUT) PASS, $(grep -c ' COUNT ' $OUT) COUNT, $(grep -c ' FAIL ' $OUT) FAIL, $(grep -c ' NONE ' $OUT) NONE" | tee -a $OUT
echo "Summary file: $OUT"
