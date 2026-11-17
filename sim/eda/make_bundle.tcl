# =============================================================================
# File      sim/eda/make_bundle.tcl                    Tracker: F10 (generator)
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   Writes the files you paste into EDA Playground:
#             sim/eda/design.sv     every RTL file of sim/files_rtl.f, in order
#             sim/eda/testbench.sv  the testbench support files and every
#                                   testbench of sim/files_tb.f, each inside
#                                   `ifdef TEST_<ID>, so one test runs at a time
#           EDA Playground takes at most 100,000 characters per file, so a
#           bundle larger than LIMIT continues in part files (design_part2.sv,
#           tb_part2.sv, ...), each holding whole source files. The first file
#           `includes its parts at its end: EDA Playground compiles an added
#           SystemVerilog tab only through `include. The script prints the
#           files to paste; add each part as a new tab with exactly that name.
#           Run it again after changing any source file.
# Usage     tclsh sim/eda/make_bundle.tcl                  (from any directory)
#           Vivado Tcl console: source <repo>/sim/eda/make_bundle.tcl
# =============================================================================

set here [file dirname [file normalize [info script]]]
set root [file normalize [file join $here .. ..]]

# Read a file list: returns {sources include-dirs}
proc read_flist {path root} {
  set srcs {}
  set incs {}
  set fh [open $path r]
  while {[gets $fh line] >= 0} {
    regsub {//.*$} $line {} line
    set line [string trim $line]
    if {$line eq ""} continue
    if {[string match {+incdir+*} $line]} {
      lappend incs [file join $root [string range $line 8 end]]
    } elseif {![string match {+*} $line]} {
      lappend srcs [file join $root $line]
    }
  }
  close $fh
  return [list $srcs $incs]
}

proc slurp {path} {
  set fh [open $path r]
  set txt [read $fh]
  close $fh
  return $txt
}

proc relpath {path root} {
  return [string range $path [expr {[string length $root] + 1}] end]
}

# Test IDs a testbench declares in its "// @tests" line (empty for support files)
proc tests_of {path} {
  if {[regexp {//\s*@tests\s+([^\n]*)} [slurp $path] -> ids]} {
    return [string trim $ids]
  }
  return {}
}

# Absolute path of an `include target, searched next to the file, then in incs
proc find_inc {name from incs} {
  foreach d [concat [list [file dirname $from]] $incs] {
    set p [file join $d $name]
    if {[file exists $p]} { return [file normalize $p] }
  }
  error "make_bundle: cannot find `include \"$name\" (from $from)"
}

# Text of a file with each `include replaced by the included text, once per
# bundle; later includes of the same file become a comment. Comment lines
# are left out (pragmas kept): the mesh bench with its four test files is
# one chunk, over EDA Playground's limit with its comments (B5).
proc inline {path incs done_var root} {
  upvar 1 $done_var done
  set out ""
  foreach line [split [string trimright [slurp $path] "\n"] "\n"] {
    if {[regexp {^\s*`include\s+"([^"]+)"} $line -> name]} {
      set inc [find_inc $name $path $incs]
      if {[lsearch -exact $done $inc] >= 0} {
        append out "// `include \"$name\": inlined earlier in this bundle\n"
      } else {
        lappend done $inc
        append out "// ---- begin [relpath $inc $root]\n"
        append out [inline $inc $incs done $root]
        append out "// ---- end [relpath $inc $root]\n"
      }
    } elseif {[regexp {^\s*//} $line] && ![regexp {translate_|synopsys|pragma|verilator|lint_|@tests} $line]} {
      # comment lines stay in the repository; the bundle drops them so that
      # the mesh bench with its test files fits in one EDA file (B5)
      continue
    } else {
      append out $line "\n"
    }
  }
  return $out
}

proc write_file {path txt} {
  set fh [open $path w]
  fconfigure $fh -translation lf
  puts -nonewline $fh $txt
  close $fh
}

set bar "// ============================================================================="

# Most characters a bundle file may hold: EDA Playground takes fewer than
# 100,000. Sizes are counted the way a browser may send them, with CRLF line
# ends (one extra character a line), so a file that fits here fits there
set LIMIT 99000

# Bytes of the UTF-8 text (never fewer than its characters) plus one per line
proc size_of {txt} {
  return [expr {[string length [encoding convertto utf-8 $txt]] + [regexp -all {\n} $txt]}]
}

# Pack chunks, in order, into files of at most LIMIT characters. The first
# file is head + chunks + an `include of each later part + tail; a part holds
# whole chunks after a short header. Returns {name text name text ...}.
proc pack {base head chunks tail what} {
  global LIMIT bar
  set parts {}
  set cur {}
  set cur_size [expr {[size_of $head] + [size_of $tail] + 400}]
  foreach c $chunks {
    set n [size_of $c]
    if {$n + 400 > $LIMIT} { error "make_bundle: one source file is $n characters, more than $LIMIT" }
    if {$cur_size + $n > $LIMIT && [llength $cur] > 0} {
      lappend parts $cur
      set cur {}
      set cur_size 400
    }
    lappend cur $c
    incr cur_size $n
  }
  lappend parts $cur
  set np [llength $parts]
  set out {}
  for {set k 0} {$k < $np} {incr k} {
    set body [join [lindex $parts $k] ""]
    if {$k == 0} {
      set name "$base.sv"
      set inc ""
      if {$np > 1} {
        append inc "\n$bar\n// ==== The rest of this bundle: EDA Playground tabs with exactly these names\n$bar\n"
        for {set j 2} {$j <= $np} {incr j} { append inc "`include \"[part_name $base $j].sv\"\n" }
      }
      lappend out $name "$head$body$inc$tail"
    } else {
      set name "[part_name $base [expr {$k + 1}]].sv"
      set h "$bar\n// File      sim/eda/$name   (part [expr {$k + 1}] of $np of $base.sv)\n"
      append h "// GENERATED by sim/eda/make_bundle.tcl. Do not edit. In EDA Playground add\n"
      append h "// a tab named exactly $name and paste this into it; $base.sv includes it.\n$bar\n"
      lappend out $name "$h$body"
    }
  }
  return $out
}

# Split text into pieces of at most max characters, only at lines where an
# inlined file begins ("// ---- begin"), joining neighbours while they fit.
# The mesh bench with its five test files is over EDA Playground's limit
# (B6), so its module body continues from one bundle file into the next:
# each piece sits under the same `ifdef, and the file holding the first
# piece is `included just before the file holding the second.
proc split_text {txt max} {
  set segs {}
  set cur ""
  foreach line [split [string trimright $txt "\n"] "\n"] {
    if {[string match "// ---- begin *" $line] && $cur ne ""} {
      lappend segs $cur
      set cur ""
    }
    append cur $line "\n"
  }
  if {$cur ne ""} { lappend segs $cur }
  set pieces {}
  set p ""
  foreach sg $segs {
    if {$p ne "" && [size_of $p] + [size_of $sg] > $max} {
      lappend pieces $p
      set p ""
    }
    append p $sg
  }
  if {$p ne ""} { lappend pieces $p }
  return $pieces
}

proc part_name {base k} {
  if {$base eq "testbench"} { return "tb_part$k" }
  return "${base}_part$k"
}

proc write_parts {here files} {
  set names {}
  foreach {name txt} $files {
    write_file [file join $here $name] $txt
    lappend names "$name ([size_of $txt] characters)"
  }
  return $names
}

# ---------------------------------------------------------------- design.sv
lassign [read_flist [file join $root sim files_rtl.f] $root] rtl_srcs rtl_incs
set done {}
set d ""
append d "$bar\n"
append d "// File      sim/eda/design.sv                                      Tracker: F10\n"
append d "// GENERATED by sim/eda/make_bundle.tcl from sim/files_rtl.f. Do not edit:\n"
append d "// change the source files and run the script again.\n"
append d "// EDA Playground: paste this into the design.sv pane.\n"
append d "$bar\n"
append d "`ifndef NOC_BYPASS_DESIGN_SV\n`define NOC_BYPASS_DESIGN_SV\n"
set dchunks {}
foreach f $rtl_srcs {
  lappend dchunks "\n$bar\n// ==== [relpath $f $root]\n$bar\n[inline $f $rtl_incs done $root]"
}
set dfiles [write_parts $here [pack design $d $dchunks "\n`endif // NOC_BYPASS_DESIGN_SV\n" design]]

# ---------------------------------------------------------------- testbench.sv
lassign [read_flist [file join $root sim files_tb.f] $root] tb_srcs tb_incs
set incs [concat $rtl_incs $tb_incs]
set tests {}
foreach f $tb_srcs {
  foreach id [tests_of $f] { lappend tests $id }
}
if {[llength $tests] == 0} { error "make_bundle: no // @tests line in any testbench" }
set first [lindex $tests 0]

# RUN_<TESTBENCH> is defined when one of that testbench's tests is chosen
proc run_macro {mod} { return "RUN_[string toupper $mod]" }

# Testbenches a support module serves ("// @support tb_a tb_b"); empty when
# the file is needed by every test (packages, or no tag)
proc support_of {path} {
  if {[regexp {//\s*@support\s+([^\n]*)} [slurp $path] -> mods]} { return [string trim $mods] }
  return {}
}

set t ""
append t "$bar\n"
append t "// File      sim/eda/testbench.sv                                   Tracker: F10\n"
append t "// GENERATED by sim/eda/make_bundle.tcl from sim/files_tb.f. Do not edit:\n"
append t "// change the source files and run the script again.\n"
append t "// EDA Playground: paste this into the testbench.sv pane and design.sv into\n"
append t "// the design pane. Simulator: Aldec Riviera-PRO. If this file ends with\n"
append t "// `include \"tb_part2.sv\" (and so on), add each such file as a new tab in\n"
append t "// the testbench pane, named exactly like that, and paste it in. The same\n"
append t "// goes for design_part2.sv and so on, at the end of design.sv.\n"
append t "// Pick ONE test with a compile option, for example +define+TEST_$first\n"
append t "// Tests in this bundle: [join $tests {, }]. Without a define, TEST_$first runs.\n"
append t "// A testbench that holds several tests runs only the one you define.\n"
append t "// Only the chosen test's testbench and the modules it uses are compiled, so\n"
append t "// the simulator finds exactly one top module.\n"
append t "// Run options: +SEED=<n> to change the seed; +VCD for EPWave waveforms.\n"
append t "$bar\n"
foreach id $tests { append t "`ifndef TEST_$id\n" }
append t "`define TEST_$first\n"
foreach id $tests { append t "`endif\n" }
append t "\n// The chosen test, for tb_pkg::start (same effect as +TEST=<id>)\n"
foreach id $tests { append t "`ifdef TEST_$id\n  `define TB_TEST \"$id\"\n`endif\n" }
append t "\n// The testbench that holds the chosen test\n"
foreach f $tb_srcs {
  set ids [tests_of $f]
  if {[llength $ids] == 0} continue
  set run [run_macro [file rootname [file tail $f]]]
  foreach id $ids { append t "`ifdef TEST_$id\n  `define $run\n`endif\n" }
}
append t "\n// design.sv is compiled only once, whichever order EDA Playground uses\n"
append t "`include \"design.sv\"\n"

# Headers a testbench includes at file scope (before its first module or
# package) come first, outside any `ifdef, so each test finds them. An
# `include inside a module body stays where it is and is inlined there.
set done {}
foreach f $tb_srcs {
  foreach line [split [slurp $f] "\n"] {
    if {[regexp {^\s*(module|package|interface|program)\s} $line]} break
    if {[regexp {^\s*`include\s+"([^"]+)"} $line -> name]} {
      set inc [find_inc $name $f $incs]
      if {[lsearch -exact $done $inc] < 0} {
        lappend done $inc
        append t "\n$bar\n// ==== [relpath $inc $root]\n$bar\n"
        append t [inline $inc $incs done $root]
      }
    }
  }
}
set nsup 0
set tchunks {}
foreach f $tb_srcs {
  set ids  [tests_of $f]
  set sups [support_of $f]
  set c "\n$bar\n// ==== [relpath $f $root]"
  if {[llength $ids]}  { append c "   (tests: $ids)" }
  if {[llength $sups]} { append c "   (used by: $sups)" }
  append c "\n$bar\n"
  if {[llength $ids]} {
    set run [run_macro [file rootname [file tail $f]]]
    append c "`ifdef $run\n"
    append c [inline $f $incs done $root]
    append c "`endif // $run\n"
  } elseif {[llength $sups]} {
    # a module only some testbenches use: compiled when one of them runs;
    # a module too large for one bundle file is split (split_text)
    incr nsup
    set need "NEED_SUPPORT_$nsup"
    foreach m $sups { append c "`ifdef [run_macro $m]\n  `define $need\n`endif\n" }
    set pieces [split_text [inline $f $incs done $root] [expr {$LIMIT - 2000}]]
    set np [llength $pieces]
    for {set k 0} {$k < $np} {incr k} {
      if {$k > 0} {
        lappend tchunks $c
        set c "\n$bar\n// ==== [relpath $f $root], continued ([expr {$k + 1}] of $np pieces of one module)\n$bar\n"
      }
      append c "`ifdef $need\n"
      append c [lindex $pieces $k]
      append c "`endif // $need\n"
    }
  } else {
    append c [inline $f $incs done $root]
  }
  lappend tchunks $c
}
set tfiles [write_parts $here [pack testbench $t $tchunks "" testbench]]

puts "make_bundle: tests [join $tests {, }]"
puts "make_bundle: design pane    [join $dfiles {, }]"
puts "make_bundle: testbench pane [join $tfiles {, }]"
puts "make_bundle: sizes counted with CRLF line ends; at most $LIMIT per file (EDA Playground: under 100,000)"
