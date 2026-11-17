# =============================================================================
# File      sim/vivado/xsim_lib.tcl                                Tracker: F61
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   Procedures that regress.tcl (F61) and sweep.tcl (F60) share: find
#           the testbench of a test ID, build it once per set of defines in
#           xsim (xvlog + xelab) and reuse the build while no source changed,
#           run one simulation with given plusargs and read its RESULT and
#           CSV lines. run.tcl (F11) stays as it is: these procedures repeat
#           its Windows-safe way of passing arguments (an options file read
#           with --file whenever an argument holds '=', ',' or ';').
# Usage     source -notrace sim/vivado/xsim_lib.tcl   (the scripts do this)
# =============================================================================

set xl_here [file dirname [file normalize [info script]]]
set xl_root [file normalize [file join $xl_here .. ..]]

proc xl_slurp {path} {
  set fh [open $path r]
  set txt [read $fh]
  close $fh
  return $txt
}

proc xl_spit {path txt {mode w}} {
  set fh [open $path $mode]
  fconfigure $fh -translation lf
  puts -nonewline $fh $txt
  close $fh
}

# A file list: {sources include-dirs defines}, paths from the repo root
proc xl_read_flist {path} {
  global xl_root
  set srcs {}
  set incs {}
  set defs {}
  foreach line [split [xl_slurp $path] "\n"] {
    regsub {//.*$} $line {} line
    set line [string trim $line]
    if {$line eq ""} continue
    if {[string match {+incdir+*} $line]} {
      lappend incs [file join $xl_root [string range $line 8 end]]
    } elseif {[string match {+define+*} $line]} {
      lappend defs [string range $line 8 end]
    } else {
      lappend srcs [file join $xl_root $line]
    }
  }
  return [list $srcs $incs $defs]
}

proc xl_tests_of {path} {
  if {[regexp {//\s*@tests\s+([^\n]*)} [xl_slurp $path] -> ids]} { return [string trim $ids] }
  return {}
}

proc xl_tool {name} {
  set exe [auto_execok $name]
  if {$exe eq ""} {
    error "xsim_lib: '$name' not found. Run from Vivado or the Vivado Tcl Shell, so that Vivado's bin directory is on the PATH."
  }
  return $exe
}

proc xl_exec {log cmd} {
  if {[catch {exec {*}$cmd >& $log}]} { return 0 }
  return 1
}

# A Vivado tool with options that may hold '=', ',' or ';': they go on one
# line of <log>.args, read by the tool itself (--file), as in run.tcl
proc xl_run_tool {log exe fixed opts} {
  set risky 0
  foreach a $opts { if {[regexp {[=,;]} $a]} { set risky 1 } }
  if {!$risky} { return [xl_exec $log [concat $exe $fixed $opts]] }
  set optfile "[file rootname $log].args"
  xl_spit $optfile "[join $opts { }]\n"
  return [xl_exec $log [concat $exe $fixed [list --file $optfile]]]
}

proc xl_count {log pattern} {
  if {![file exists $log]} { return 0 }
  set n 0
  foreach line [split [xl_slurp $log] "\n"] { if {[regexp $pattern $line]} { incr n } }
  return $n
}

proc xl_head {log n} {
  if {![file exists $log]} { return "" }
  return [join [lrange [split [xl_slurp $log] "\n"] 0 [expr {$n - 1}]] "\n"]
}

# ---------------------------------------------------------------- the repository
lassign [xl_read_flist [file join $xl_root sim files_rtl.f]] xl_rtl_srcs xl_rtl_incs xl_rtl_defs
lassign [xl_read_flist [file join $xl_root sim files_tb.f]]  xl_tb_srcs  xl_tb_incs  xl_tb_defs

# The testbench module that runs <id> (a test ID or a module name), or ""
proc xl_top_of {id} {
  global xl_tb_srcs
  foreach f $xl_tb_srcs {
    set ids [xl_tests_of $f]
    if {[llength $ids] == 0} continue
    set mod [file rootname [file tail $f]]
    if {$id eq $mod || [lsearch -exact $ids $id] >= 0} { return $mod }
  }
  return ""
}

# Every file a build reads: the lists, the RTL and testbench trees (with the
# included .svh files) and this library
proc xl_inputs {} {
  global xl_root
  set out [list [file join $xl_root sim files_rtl.f] [file join $xl_root sim files_tb.f]]
  foreach dir {rtl tb} {
    set stack [list [file join $xl_root $dir]]
    while {[llength $stack]} {
      set d [lindex $stack 0]
      set stack [lrange $stack 1 end]
      foreach p [glob -nocomplain -directory $d *] {
        if {[file isdirectory $p]} { lappend stack $p } elseif {[regexp {\.(sv|svh|v|vh)$} $p]} { lappend out $p }
      }
    }
  }
  return $out
}

proc xl_newest {files} {
  set t 0
  foreach f $files { if {[file exists $f] && [file mtime $f] > $t} { set t [file mtime $f] } }
  return $t
}

# Work directory of a build: run.tcl's naming, so both share the builds.
# xl_wsuffix is added to it (sweep.tcl -part k/n: "__part<k>", so that
# sweeps running side by side never share a build or its run files)
set xl_wsuffix ""
proc xl_workdir {top defs} {
  global xl_here xl_wsuffix
  set suf ""
  foreach d $defs { append suf "__[regsub -all {[^A-Za-z0-9]+} $d {_}]" }
  return [file join $xl_here work "$top$suf$xl_wsuffix"]
}

# Build <top> with <defs> unless a build newer than every input exists.
# Returns {ok work warnings message}
proc xl_build {top defs {force 0}} {
  global xl_rtl_srcs xl_rtl_incs xl_rtl_defs xl_tb_srcs xl_tb_incs xl_tb_defs
  set work [xl_workdir $top $defs]
  file mkdir $work
  set stamp [file join $work build.ok]
  if {!$force && [file exists $stamp] && [file mtime $stamp] >= [xl_newest [xl_inputs]]} {
    return [list 1 $work [string trim [xl_slurp $stamp]] "up to date"]
  }
  catch {file delete $stamp}
  set vargs [list --sv]
  foreach d [concat $xl_rtl_incs $xl_tb_incs] { lappend vargs --include $d }
  set vargs [concat $vargs $xl_rtl_srcs $xl_tb_srcs]
  set vdefs {}
  foreach d [concat $xl_rtl_defs $xl_tb_defs $defs] { lappend vdefs --define $d }
  set old [pwd]
  cd $work
  set ok [xl_run_tool compile.log [xl_tool xvlog] $vargs $vdefs]
  if {$ok} {
    set ok [xl_run_tool elaborate.log [xl_tool xelab] {} [list --debug off --timescale 1ns/1ps --snapshot ${top}_snap work.$top]]
  }
  set w [expr {[xl_count compile.log {^WARNING}] + [xl_count elaborate.log {^WARNING}]}]
  cd $old
  if {!$ok} { return [list 0 $work $w "build failed: see [file join $work compile.log] and elaborate.log"] }
  xl_spit $stamp "$w\n"
  return [list 1 $work $w "built"]
}

# One simulation of a built snapshot. plus: plusargs without the '+'.
# Returns a dict: verdict (PASS, FAIL or NONE), checks, errors, cycles,
# secs, log, csv (the run's CSV lines, header first), trace (its TRACE
# lines, PERF's +TRACE time series, header first)
proc xl_sim {work top plus logname} {
  set old [pwd]
  cd $work
  set sargs {}
  foreach p $plus { lappend sargs --testplusarg $p }
  lappend sargs --runall
  set t0 [clock milliseconds]
  xl_run_tool $logname [xl_tool xsim] [list ${top}_snap] $sargs
  set secs [expr {([clock milliseconds] - $t0) / 1000.0}]
  set txt ""
  if {[file exists $logname]} { set txt [xl_slurp $logname] }
  cd $old
  set r [dict create verdict NONE checks - errors - cycles - secs $secs log [file join $work $logname] csv {} trace {}]
  if {[regexp {RESULT\s+\S+\s+(PASS|FAIL)\s+checks=(\d+)\s+errors=(\d+)} $txt -> v c e]} {
    dict set r verdict $v
    dict set r checks $c
    dict set r errors $e
  }
  if {[regexp {INFO\s+cycles simulated\s+(\d+)} $txt -> n]} { dict set r cycles $n }
  set csv {}
  set trc {}
  foreach line [split $txt "\n"] {
    set line [string trimright $line "\r"]
    if {[string match "CSV,*" $line]}   { lappend csv [string range $line 4 end] }
    if {[string match "TRACE,*" $line]} { lappend trc [string range $line 6 end] }
  }
  dict set r csv $csv
  dict set r trace $trc
  return $r
}

# "1234567" -> "1,234,567"
proc xl_commas {n} {
  if {![string is integer -strict $n]} { return $n }
  set s [string reverse $n]
  regsub -all {(\d{3})(?=\d)} $s {\1,} s
  return [string reverse $s]
}
