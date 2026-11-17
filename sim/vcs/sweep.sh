#!/bin/bash
# =============================================================================
# File      sim/vcs/sweep.sh                                       Tracker: F62
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   sweep.tcl's job on VCS (default) or Xcelium (SIM=xrun): runs an
#           experiment's PERF points (sim/sweeps/<name>.sweep) unattended and
#           collects their CSV rows, for the runs too large for xsim (E2's
#           16x16 mesh) or to share a long sweep between machines. The same
#           files as sweep.tcl, so its CSV, compare_csv.tcl and the chart
#           workbook work alike; a point already in the CSV is skipped, so a
#           sweep stopped at any time continues when started again.
# Usage     bash sim/vcs/sweep.sh <name> [k/n] [glob]   (from the repository root)
#           SIM=xrun bash sim/vcs/sweep.sh <name> [k/n] [glob]
#           make -f sim/vcs/Makefile sweep SWEEP=<name> [PART=k/n] [ONLY=glob]
#   k/n     part k of n (default 1/1): the points whose place in the sweep,
#           counted from 0, leaves k-1 when divided by n. Start n of them
#           side by side to use n cores; each builds in its own folder
#           (__part<k>) and writes <name>.part<k>.csv and sweep.part<k>.txt,
#           and merges every part's rows into <name>.csv when it ends.
#   glob    run only the points whose run_id matches (quote it: '*_D_*');
#           give every part the same glob (parts split the matching points)
# Output    results/<name>/<name>.csv, logs/<run_id>.log, sweep.txt and
#           trace/<run_id>.csv (PERF +TRACE), as sweep.tcl
# =============================================================================
NAME=${1:?usage: sweep.sh <name> [k/n]   (sweeps in sim/sweeps)}
PART=${2:-1/1}
ONLY=${3:-*}
SIM=${SIM:-vcs}
cd "$(dirname "$0")/../.." || exit 1
F=sim/sweeps/$NAME.sweep
[ -f "$F" ] || { echo "sweep.sh: no sweep $F"; exit 1; }
case "$PART" in */*) K=${PART%/*}; N=${PART#*/} ;; *) K=0; N=0 ;; esac
if ! [ "$K" -ge 1 ] 2>/dev/null || ! [ "$N" -ge "$K" ] 2>/dev/null; then echo "sweep.sh: the part is k/n with 1 <= k <= n, e.g. 2/4"; exit 1; fi
OUT=results/$NAME
mkdir -p "$OUT/logs"
CSV=$OUT/$NAME.csv
MINE=$CSV; TXT=$OUT/sweep.txt; WSUF=""
if [ "$N" -gt 1 ]; then MINE=$OUT/$NAME.part$K.csv; TXT=$OUT/sweep.part$K.txt; WSUF=__part$K; fi

# the run_ids already in the CSV or any part's
done_ids() { for f in "$CSV" "$OUT/$NAME".part*.csv; do [ -f "$f" ] && tr -d '\r' < "$f" | awk -F, 'FNR > 1 && $1 != "" { print $1 }'; done; }
DONE=$(done_ids | sort -u)

# <name>.csv from every file's rows, in the sweep's order; through a temporary file
merge() {
  local tmp=$CSV.tmp$$
  sed 's/#.*//' "$F" | awk 'NF { print $1 }' > "$tmp.order"
  { for f in "$CSV" "$OUT/$NAME".part*.csv; do [ -f "$f" ] && tr -d '\r' < "$f"; done; } |
    awk -F, -v ord="$tmp.order" '
      BEGIN { while ((getline l < ord) > 0) { n++; pos[l] = n } }
      NR == 1 || $1 == "run_id" { if (hdr == "") hdr = $0; next }
      $1 != "" && !($1 in row) { row[$1] = $0; ids[++m] = $1 }
      END {
        print hdr
        for (i = 1; i <= n; i++) for (j = 1; j <= m; j++) if (ids[j] != "" && pos[ids[j]] == i) { print row[ids[j]]; ids[j] = "" }
        for (j = 1; j <= m; j++) if (ids[j] != "") print row[ids[j]]
      }' > "$tmp"
  mv -f "$tmp" "$CSV"; rm -f "$tmp.order"
  echo "sweep.sh: $(( $(wc -l < "$CSV") - 1 )) rows merged into $CSV"
}

total=0; todo=0; ok=0; bad=0; t0=$(date +%s)
idx=-1
while read -r rid test rest; do
  [ -z "$rid" ] && continue
  case "$rid" in $ONLY) ;; *) continue ;; esac
  idx=$((idx + 1)); total=$((total + 1))
  [ $((idx % N)) -ne $((K - 1)) ] && continue
  echo "$DONE" | grep -qx "$rid" && continue
  todo=$((todo + 1))
  seed=1; defs=""; plus=""
  set -- $rest
  while [ $# -gt 0 ]; do
    case "$1" in
      -seed)   seed=$2; shift 2 ;;
      -define) defs="$defs $2"; shift 2 ;;
      -plus)   plus="$plus $2"; shift 2 ;;
      *) echo "sweep.sh: $NAME.sweep: $rid: unknown option $1"; shift ;;
    esac
  done
  log=sweep_$rid.log
  s0=$(date +%s)
  if [ "$SIM" = "xrun" ]; then
    args=""; for d in $defs; do args="$args -define $d"; done; for p in $plus +RUNID=$rid; do args="$args -plus $p"; done
    suf=""; for d in $defs; do suf="${suf}__$(echo "$d" | sed 's/[^A-Za-z0-9]/_/g')"; done
    work=sim/xcelium/work/$test$suf$WSUF
    LOG=$log WSUF=$WSUF sh sim/xcelium/run.sh -test "$test" -seed "$seed" $args < /dev/null > /dev/null 2>&1
  else
    suf=""; for d in $defs; do suf="${suf}__${d//=/_}"; done
    work=sim/vcs/work/$test$suf$WSUF
    make -s -f sim/vcs/Makefile run TEST="$test" DEFINES="$defs" PLUS="$plus +RUNID=$rid" SEED="$seed" LOG="$log" WSUF="$WSUF" < /dev/null > /dev/null 2>&1
  fi
  secs=$(( $(date +%s) - s0 ))
  L=$work/$log
  [ -f "$L" ] && cp -f "$L" "$OUT/logs/$rid.log"
  res=$(grep -a -m1 '^RESULT' "$L" 2>/dev/null)
  v=NONE; chk=-; err=-
  case "$res" in *" PASS "*) v=PASS ;; *" FAIL "*) v=FAIL ;; esac
  [ -n "$res" ] && chk=$(echo "$res" | sed -n 's/.*checks=\([0-9]*\).*/\1/p') && err=$(echo "$res" | sed -n 's/.*errors=\([0-9]*\).*/\1/p')
  cyc=$(grep -a 'cycles simulated' "$L" 2>/dev/null | tail -1 | awk '{print $NF}')
  csvl=$(grep -a '^CSV,' "$L" 2>/dev/null | tr -d '\r' | sed 's/^CSV,//')
  if [ "$v" = "PASS" ] && [ "$(echo "$csvl" | grep -c .)" -ge 2 ]; then
    hdr=$(echo "$csvl" | tail -2 | head -1); row=$(echo "$csvl" | tail -1)
    if [ ! -s "$MINE" ]; then echo "$hdr" > "$MINE"
    elif [ "$(head -1 "$MINE" | tr -d '\r')" != "$hdr" ]; then echo "sweep.sh: $rid: WARNING: its CSV header differs from the file's; the row is written anyway"; fi
    echo "$row" >> "$MINE"
    ok=$((ok + 1))
    if grep -aq '^TRACE,' "$L"; then mkdir -p "$OUT/trace"; grep -a '^TRACE,' "$L" | tr -d '\r' | sed 's/^TRACE,//' > "$OUT/trace/$rid.csv"; fi
  else
    [ "$v" = "PASS" ] && v=NOCSV
    bad=$((bad + 1))
  fi
  line="$(date '+%Y-%m-%d %H:%M:%S') $rid $v checks=$chk errors=$err cycles=${cyc:--} $secs s"
  echo "$line" >> "$TXT"
  echo "sweep.sh: [$todo] $line"
done < <(sed 's/#.*//' "$F")
# twice, a few seconds apart: a part that ended at the same moment may have
# replaced the file with a merge that missed this part's last row
if [ "$N" -gt 1 ] || ls "$OUT/$NAME".part*.csv > /dev/null 2>&1; then merge > /dev/null; sleep 3; merge; fi
echo "==================== sweep.sh $NAME part $K/$N ($SIM): $ok points written, $bad not, $(( ($(date +%s) - t0 + 30) / 60 )) min ===================="
echo "CSV: $CSV"
echo "Log of every point: $TXT"
