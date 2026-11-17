# =============================================================================
# File      sim/compare_csv.tcl                                    Tracker: F61
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   S06: do two simulators agree? Compares the CSV rows of the same
#           points from two runs (e.g. xsim and Riviera-PRO, or xsim and VCS)
#           column by column, matched by run_id.
# Usage     tclsh sim/compare_csv.tcl <a> <b>
#           Vivado:  set argv {results/S06/S06.csv R-1114-S06-riviera.log}; source sim/compare_csv.tcl
#           <a> and <b> may each be a CSV file (header, then rows), a log
#           file (its lines that start with "CSV," are taken), or a folder
#           (every .csv and .log file in it).
# Output    one line per run_id: SAME, DIFF (with the columns that differ and
#           both values) or ONLY A / ONLY B; then a summary. Every column
#           must match: the same seed gives the same run on every simulator.
# =============================================================================

proc cc_read {path} {
  set fh [open $path r]
  set t [read $fh]
  close $fh
  return $t
}

# rows of one source: a dict run_id -> list of values, and the header
proc cc_rows {src} {
  set files {}
  if {[file isdirectory $src]} {
    set files [lsort [concat [glob -nocomplain -directory $src *.csv] [glob -nocomplain -directory $src *.log]]]
  } else {
    set files [list $src]
  }
  set hdr {}
  set rows [dict create]
  foreach f $files {
    set lines {}
    foreach l [split [cc_read $f] "\n"] {
      set l [string trimright $l "\r"]
      if {[string match "CSV,*" $l]} { set l [string range $l 4 end] } elseif {![string match "*.csv" $f]} continue
      if {$l eq ""} continue
      lappend lines $l
    }
    foreach l $lines {
      set v [split $l ","]
      if {[lindex $v 0] eq "run_id"} { set hdr $v; continue }
      dict set rows [lindex $v 0] $v
    }
  }
  return [list $hdr $rows]
}

if {![info exists argv] || [llength $argv] != 2} { error "compare_csv.tcl: give two sources: set argv {<a> <b>}" }
lassign [cc_rows [lindex $argv 0]] ha ra
lassign [cc_rows [lindex $argv 1]] hb rb
if {$ha ne $hb && [llength $ha] && [llength $hb]} { puts "compare_csv: WARNING: the two headers differ" }
set hdr [expr {[llength $ha] ? $ha : $hb}]
set n_same 0; set n_diff 0; set n_only 0
foreach id [lsort [lsort -unique [concat [dict keys $ra] [dict keys $rb]]]] {
  if {![dict exists $rb $id]} { puts [format "%-40s ONLY A" $id]; incr n_only; continue }
  if {![dict exists $ra $id]} { puts [format "%-40s ONLY B" $id]; incr n_only; continue }
  set a [dict get $ra $id]
  set b [dict get $rb $id]
  set d {}
  for {set i 0} {$i < max([llength $a], [llength $b])} {incr i} {
    if {[lindex $a $i] ne [lindex $b $i]} {
      set col [expr {$i < [llength $hdr] ? [lindex $hdr $i] : "column [expr {$i + 1}]"}]
      lappend d "$col [lindex $a $i] / [lindex $b $i]"
    }
  }
  if {[llength $d]} {
    puts [format "%-40s DIFF  %s" $id [join $d {; }]]
    incr n_diff
  } else {
    puts [format "%-40s SAME" $id]
    incr n_same
  }
}
puts "compare_csv: $n_same SAME, $n_diff DIFF, $n_only in one source only"
if {$n_diff == 0 && $n_only == 0 && $n_same > 0} { puts "S06 PASS: every row is identical" } else { puts "S06 FAIL" }
