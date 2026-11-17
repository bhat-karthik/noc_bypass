# =============================================================================
# File      sim/vivado/sweep.tcl                                   Tracker: F60
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   Runs an experiment's list of performance points (the PERF test)
#           in the Vivado simulator, unattended, and collects their CSV rows
#           in one file per experiment. Each testbench is built once per set
#           of defines; a point already in the CSV is skipped, so a sweep
#           stopped at any time continues where it stopped when started
#           again.
# -----------------------------------------------------------------------------
# Vivado Tcl console (from the repository):
#     cd D:/noc_bypass
#     set argv {-sweep E1_4x4}; source -notrace sim/vivado/sweep.tcl
# Windows command prompt (for overnight runs), after settings64.bat:
#     vivado -mode batch -notrace -nojournal -nolog -source sim/vivado/sweep.tcl -tclargs -sweep E1_4x4
# Options
#   -sweep <name>   sim/sweeps/<name>.sweep (generate them with
#                   sim/sweeps/make_sweeps.tcl)
#   -out <dir>      where the CSV and the logs go (default results/<name>)
#   -max <n>        run at most n points this time (then stop)
#   -dry            list what would run, run nothing
#   -rebuild        build every testbench again even if up to date
#   -part <k>/<n>   run only part k of n: the points whose place in the
#                   sweep, counted from 0, leaves k-1 when divided by n.
#                   Start n Vivado processes with -part 1/n ... n/n to use n
#                   cores (xsim uses one). Each part has its own builds
#                   (work/<testbench>...__part<k>), its own <name>.part<k>.csv
#                   and sweep.part<k>.txt; each merges every part's rows into
#                   <name>.csv when it ends. Memory: about 1-2 GB per 8x8 run.
#   -merge          only merge: write <name>.csv from <name>.csv and every
#                   <name>.part<k>.csv, in the sweep's order; run nothing
#   -only <glob>    run only the points whose run_id matches, e.g.
#                   -only *_uniform_* or -only *_D_*; with -part, give every
#                   part the same -only (parts split the matching points)
# Sweep format, one point per line ('#' starts a comment):
#   <run_id> <test ID or testbench> [-seed n] [-define N=V]... [-plus +ARG]...
# e.g.  E1_D_N4_uniform_r0.30_s1  PERF  -seed 1  -define TB_MESH_N=4  -plus +PATTERN=uniform -plus +RATE=0.30
# The run_id goes to the PERF test as +RUNID, so it is the CSV's first
# column. A point is done when a row with its run_id is in the CSV.
# Output
#   results/<name>/<name>.csv         header once, then one row per point
#   results/<name>/logs/<run_id>.log  the simulation log of each point
#   results/<name>/sweep.txt          one line per point: verdict, checks,
#                                     cycles, time (appended each run)
#   results/<name>/trace/<run_id>.csv the point's time series, when the
#                                     sweep gives PERF +TRACE=<n> (E6)
#   with -part k/n: results/<name>/<name>.part<k>.csv and sweep.part<k>.txt,
#   merged into <name>.csv at the end of each part (or with -merge)
# A point whose test FAILs or prints no RESULT line writes no CSV row; it is
# listed in sweep.txt and tried again on the next start.
# =============================================================================

set sw_lib [file join [file dirname [file normalize [info script]]] xsim_lib.tcl]
if {[llength [info commands get_property]]} { source -notrace $sw_lib } else { source $sw_lib }

set sw_name  ""
set sw_out   ""
set sw_max   -1
set sw_dry   0
set sw_force 0
set sw_pk    1
set sw_pn    1
set sw_merge 0
set sw_only  *
if {![info exists argv]} { set argv {} }
for {set i 0} {$i < [llength $argv]} {incr i} {
  set a [lindex $argv $i]
  switch -- $a {
    -sweep   { incr i; set sw_name [lindex $argv $i] }
    -out     { incr i; set sw_out [lindex $argv $i] }
    -max     { incr i; set sw_max [lindex $argv $i] }
    -dry     { set sw_dry 1 }
    -rebuild { set sw_force 1 }
    -merge   { set sw_merge 1 }
    -only    { incr i; set sw_only [lindex $argv $i] }
    -part    {
      incr i
      if {![regexp {^(\d+)/(\d+)$} [lindex $argv $i] -> sw_pk sw_pn] || $sw_pn < 1 || $sw_pk < 1 || $sw_pk > $sw_pn} {
        error "sweep.tcl: -part takes k/n with 1 <= k <= n, e.g. -part 2/4"
      }
      set sw_pk [scan $sw_pk %d]
      set sw_pn [scan $sw_pn %d]
    }
    default  { error "sweep.tcl: unknown option '$a' (options: -sweep -out -max -dry -rebuild -part -merge -only)" }
  }
}
if {$sw_name eq ""} { error "sweep.tcl: name a sweep, e.g. set argv {-sweep E1_4x4} (sweeps in sim/sweeps)" }
set sw_file [file join $xl_root sim sweeps "$sw_name.sweep"]
if {![file exists $sw_file]} { error "sweep.tcl: no sweep $sw_file" }
if {$sw_out eq ""} { set sw_out [file join $xl_root results $sw_name] }
file mkdir [file join $sw_out logs]
set sw_csv [file join $sw_out "$sw_name.csv"]
set sw_txt [file join $sw_out sweep.txt]
# this run's own files: the main ones, or part k's
set sw_mine $sw_csv
if {$sw_pn > 1} {
  set sw_mine [file join $sw_out "$sw_name.part$sw_pk.csv"]
  set sw_txt  [file join $sw_out "sweep.part$sw_pk.txt"]
  set xl_wsuffix "__part$sw_pk"
}

# ---------------------------------------------------------------- points
set sw_points {}
set ln 0
foreach line [split [xl_slurp $sw_file] "\n"] {
  incr ln
  regsub {#.*$} $line {} line
  set line [string trim $line]
  if {$line eq ""} continue
  set w [regexp -all -inline {\S+} $line]
  if {[llength $w] < 2} { error "sweep.tcl: $sw_name.sweep line $ln: expected <run_id> <test>" }
  lassign $w rid test
  set seed 1
  set defs {}
  set plus {}
  for {set k 2} {$k < [llength $w]} {incr k} {
    switch -- [lindex $w $k] {
      -seed   { incr k; set seed [lindex $w $k] }
      -define { incr k; lappend defs [lindex $w $k] }
      -plus   { incr k; lappend plus [string trimleft [lindex $w $k] "+"] }
      default { error "sweep.tcl: $sw_name.sweep line $ln: unknown option '[lindex $w $k]'" }
    }
  }
  if {![regexp {^[A-Za-z0-9_.\-]+$} $rid]} { error "sweep.tcl: $sw_name.sweep line $ln: run_id '$rid' may hold only letters, digits, '_', '.', '-'" }
  set top [xl_top_of $test]
  if {$top eq ""} { error "sweep.tcl: $sw_name.sweep line $ln: no testbench runs '$test'" }
  lappend sw_points [list $rid $test $top $seed $defs $plus]
}

# Every row of the main CSV and of the part CSVs: {header {run_id row ...}}
proc sw_rows {} {
  global sw_out sw_name
  set hdr ""
  set rows {}
  set files [list [file join $sw_out "$sw_name.csv"]]
  foreach f [lsort [glob -nocomplain -directory $sw_out "$sw_name.part*.csv"]] { lappend files $f }
  foreach f $files {
    if {![file exists $f]} continue
    set lines [split [string trim [string map {"\r" ""} [xl_slurp $f]]] "\n"]
    if {[llength $lines] == 0} continue
    if {$hdr eq ""} { set hdr [lindex $lines 0] }
    foreach r [lrange $lines 1 end] {
      set id [lindex [split $r ","] 0]
      if {$id ne "" && ![dict exists $rows $id]} { dict set rows $id $r }
    }
  }
  return [list $hdr $rows]
}

# <name>.csv from every file's rows, in the sweep's order (rows of points no
# longer in the sweep last); written to a temporary file and renamed, so a
# part that reads it meanwhile never sees half a file
proc sw_merge {} {
  global sw_points sw_csv
  lassign [sw_rows] hdr rows
  if {$hdr eq ""} { return 0 }
  set out "$hdr\n"
  set n 0
  foreach p $sw_points {
    set id [lindex $p 0]
    if {[dict exists $rows $id]} { append out "[dict get $rows $id]\n"; dict unset rows $id; incr n }
  }
  dict for {id r} $rows { append out "$r\n"; incr n }
  set tmp "$sw_csv.tmp[pid]"
  xl_spit $tmp $out
  for {set t 0} {$t < 10} {incr t} {
    if {![catch {file rename -force $tmp $sw_csv}]} { return $n }
    after 1000
  }
  puts "sweep.tcl: WARNING: could not replace $sw_csv (open in Excel?); the merged rows are in $tmp"
  return $n
}

if {$sw_merge} {
  puts "sweep.tcl: $sw_name: [sw_merge] rows merged into $sw_csv"
  return
}

# run_ids already done, in any part; this part's points
lassign [sw_rows] - sw_done
set sw_todo {}
set idx 0
foreach p $sw_points {
  if {![string match $sw_only [lindex $p 0]]} continue
  if {$idx % $sw_pn == $sw_pk - 1 && ![dict exists $sw_done [lindex $p 0]]} { lappend sw_todo $p }
  incr idx
}
set sw_part [expr {$sw_pn > 1 ? " (part $sw_pk of $sw_pn)" : ""}]
if {$sw_only ne "*"} { append sw_part " (run_ids matching $sw_only)" }
puts "sweep.tcl: $sw_name$sw_part: [llength $sw_points] points in the sweep, [dict size $sw_done] already done, [llength $sw_todo] to run here"
# the header of the file this run appends to ("" until it exists)
set sw_header ""
if {[file exists $sw_mine]} { set sw_header [lindex [split [string map {"\r" ""} [xl_slurp $sw_mine]] "\n"] 0] }
if {$sw_max >= 0 && [llength $sw_todo] > $sw_max} { set sw_todo [lrange $sw_todo 0 [expr {$sw_max - 1}]] }
if {$sw_dry} {
  foreach p $sw_todo {
    lassign $p rid test top seed defs plus
    set t [expr {[regexp {^[A-Z][A-Z0-9]*$} $test] ? " TEST=$test" : ""}]
    puts "  $rid: $top [join $defs { }] SEED=$seed$t [join $plus { }]"
  }
  return
}

# ---------------------------------------------------------------- run
set sw_t0 [clock seconds]
set sw_built {}
set sw_ok 0
set sw_bad 0
set k 0
foreach p $sw_todo {
  lassign $p rid test top seed defs plus
  incr k
  set key "$top|$defs"
  if {![dict exists $sw_built $key]} {
    puts "sweep.tcl: building $top [join $defs { }]"
    dict set sw_built $key [xl_build $top $defs $sw_force]
    puts "sweep.tcl: $top: [lindex [dict get $sw_built $key] 3]"
  }
  lassign [dict get $sw_built $key] ok work w msg
  set stamp [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]
  if {!$ok} {
    set line "$stamp $rid BUILD $msg"
    incr sw_bad
  } else {
    set pa [list "SEED=$seed" "RUNID=$rid"]
    if {[regexp {^[A-Z][A-Z0-9]*$} $test]} { lappend pa "TEST=$test" }
    set pa [concat $pa $plus]
    set r [xl_sim $work $top $pa "sweep_point.log"]
    file copy -force [dict get $r log] [file join $sw_out logs "$rid.log"]
    set v [dict get $r verdict]
    set csv [dict get $r csv]
    if {$v eq "PASS" && [llength $csv] >= 2} {
      set hdr [lindex $csv end-1]
      set row [lindex $csv end]
      if {$sw_header eq ""} {
        set sw_header $hdr
        xl_spit $sw_mine "$hdr\n"
      } elseif {$hdr ne $sw_header} {
        puts "sweep.tcl: $rid: WARNING: its CSV header differs from the file's; the row is written anyway"
      }
      xl_spit $sw_mine "$row\n" a
      incr sw_ok
      if {[llength [dict get $r trace]]} {
        file mkdir [file join $sw_out trace]
        xl_spit [file join $sw_out trace "$rid.csv"] "[join [dict get $r trace] \n]\n"
      }
    } else {
      incr sw_bad
      if {$v eq "PASS"} { set v "NOCSV" }
    }
    set line [format "%s %s %s checks=%s errors=%s cycles=%s %.0f s" $stamp $rid $v [dict get $r checks] [dict get $r errors] [dict get $r cycles] [dict get $r secs]]
  }
  xl_spit $sw_txt "$line\n" a
  set el [expr {[clock seconds] - $sw_t0}]
  set eta [expr {$k ? int(double($el) / $k * ([llength $sw_todo] - $k) / 60.0) : 0}]
  puts "sweep.tcl: \[$k/[llength $sw_todo]\] $line (about $eta min left)"
}
if {$sw_pn > 1 || [llength [glob -nocomplain -directory $sw_out "$sw_name.part*.csv"]]} {
  # twice, a few seconds apart: a part that ended at the same moment may
  # have replaced the file with a merge that missed this part's last row
  sw_merge
  after 3000
  puts "sweep.tcl: [sw_merge] rows of every part merged into $sw_csv"
}
puts "==================== sweep.tcl $sw_name$sw_part: $sw_ok points written, $sw_bad not, [expr {([clock seconds] - $sw_t0 + 30) / 60}] min ===================="
puts "CSV: $sw_csv"
puts "Log of every point: $sw_txt"
