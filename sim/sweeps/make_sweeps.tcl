# =============================================================================
# File      sim/sweeps/make_sweeps.tcl                             Tracker: F60
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   Writes the point lists (.sweep) of the experiments E0-E10 and of
#           S06 from the experiment plan (tracker Section 9), for sweep.tcl.
#           Each line is one PERF point: run_id, testbench, seed, defines and
#           plusargs. Edit the settings below and run it again to change a
#           sweep; sweep.tcl skips the points already in an experiment's CSV.
# Usage     tclsh sim/sweeps/make_sweeps.tcl     (or source it in Vivado)
# Rates     Each point's +RATE is a fraction of its configuration's knee (the
#           table below; D61): the offered load, in packets/node/cycle, at
#           which design B's accepted throughput falls below 95 % of the
#           offered (the CSV's sat), measured in B8 for every pattern, mesh,
#           VC count, packet mix and sink stall chance the sweeps use. Run ids
#           carry the fraction as k<f>. E0 uses absolute rates, as [1]; S06
#           keeps its B8 shares of sat_rate() so its rows stay comparable.
# =============================================================================

set ms_here [file dirname [file normalize [info script]]]

# the testbench of each design, and its defines on 8x8 and larger
proc ms_bench {d} { return [dict get {A tb_mesh B tb_mesh_b C tb_mesh_c D tb_mesh_d} $d] }

set ms_count {}
proc ms_write {name lines} {
  global ms_here ms_count
  set fh [open [file join $ms_here "$name.sweep"] w]
  fconfigure $fh -translation lf
  puts $fh "# $name: [llength $lines] points (written by make_sweeps.tcl; run with sweep.tcl -sweep $name)"
  foreach l $lines { puts $fh $l }
  close $fh
  dict set ms_count $name [llength $lines]
}

# one point: run_id, design, seed, defines, plusargs
proc ms_pt {rid d seed defs plus} {
  set l [format "%-44s %-10s -seed %d" $rid [ms_bench $d] $seed]
  foreach x $defs { append l " -define $x" }
  append l " -plus +TEST=PERF"
  foreach x $plus { append l " -plus +$x" }
  return $l
}

proc ms_n {n} { if {$n == 4} { return {} } ; return [list "TB_MESH_N=$n"] }
# ... and for designs C and D on 8x8 and larger D19's epoch (2^15 on 8x8, 2^18 on
# 16x16; D60): the testbenches' default there is 4,096, short for the tests.
# On 4x4 their default is D19's epoch already (sched_pkg::epoch_len, 2^11).
proc ms_nd {n d} {
  set l [ms_n $n]
  if {$n >= 8 && ($d eq "C" || $d eq "D")} { lappend l [expr {$n == 8 ? "TB_EPOCH=32768" : "TB_EPOCH=262144"}] }
  return $l
}
proc ms_f {x} { return [format "%.2f" $x] }

# The knees (D61), packets/node/cycle, keyed N/V/p_data/pattern[/p<PLOCAL>][/s<STALL>].
# Measured in B8 on design B (minimal adaptive routing, B_MAX 10, TIGHT),
# seed 1, 2,000-cycle warm-up and 10,000-cycle window: a grid of rates, then
# four points inside the bracket; the knee is where accepted/offered crosses
# 0.95, interpolated (results/precheck/B8_knees.csv; about +/-3 %, and +/-5 %
# for 1 VC on 4x4). The rows are the same on every simulator (S06), so these
# are xsim's knees too. 16x16 does not fit Verilator here: its three knees
# are 8x8's halved (uniform saturation scales as 1/N) and the first 16x16
# points check them.
# On 8x8 the designs with minimal adaptive routing collapse just past the
# knee (uniform: accepted 0.305 -> 0.056 flits/node/cycle; the bypass
# timetable breaks cyclic waits only when the waiting pair's turn comes):
# see ms_top below.
set ms_knees {
  4/1/0.333/transpose          0.0867
  4/1/0.333/uniform            0.0602
  4/1/0.333/uniform/s0.05      0.0579
  4/1/0.333/uniform/s0.10      0.0636
  4/1/0.333/uniform/s0.20      0.0619
  4/2/0.333/transpose          0.1908
  4/2/0.333/uniform            0.1610
  4/4/0.333/bitcomp            0.1353
  4/4/0.333/bitrot             0.3300
  4/4/0.333/hotspot            0.1059
  4/4/0.333/region/p0.50       0.3185
  4/4/0.333/shuffle            0.3082
  4/4/0.333/transpose          0.3126
  4/4/0.333/uniform            0.2763
  4/4/0.333/uniform/s0.05      0.2756
  4/4/0.333/uniform/s0.10      0.2728
  4/4/0.333/uniform/s0.20      0.2632
  8/1/0.333/transpose          0.0340
  8/1/0.333/uniform            0.0272
  8/2/0.333/transpose          0.0799
  8/2/0.333/uniform            0.0714
  8/4/0.000/uniform            0.2242
  8/4/0.333/bitcomp            0.0639
  8/4/0.333/bitrot             0.1641
  8/4/0.333/hotspot            0.0303
  8/4/0.333/region/p0.00       0.1084
  8/4/0.333/region/p0.25       0.1293
  8/4/0.333/region/p0.50       0.1650
  8/4/0.333/region/p0.75       0.2178
  8/4/0.333/region/p0.90       0.2582
  8/4/0.333/shuffle            0.1448
  8/4/0.333/transpose          0.1356
  8/4/0.333/uniform            0.1332
  8/4/0.500/uniform            0.1078
  16/4/0.333/transpose         0.0678
  16/4/0.333/uniform           0.0666
  16/4/0.333/region/p0.50      0.0825
}
proc ms_knee {n v pd pat {pl ""} {st ""}} {
  global ms_knees
  set k "$n/$v/$pd/$pat"
  if {$pl ne ""} { append k "/p$pl" }
  if {$st ne "" && $st ne "0.00"} { append k "/s$st" }
  if {![dict exists $ms_knees $k]} { error "make_sweeps: no knee for $k" }
  return [dict get $ms_knees $k]
}
# +RATE for fraction f of a knee
proc ms_rate {knee f} { return [format "RATE=%.5f" [expr {$knee * $f}]] }
# The fractions of a sweep for design d on an n x n mesh. Past the knee only
# where such a point still drains within +DRAIN (200,000 cycles): A plateaus,
# B and C collapse but warm up for 2,000 cycles only. Design D warms up for
# five epochs (D60), so on 8x8 and 16x16 a point past its collapse would hold
# 160,000 cycles of backlog and fail: its fractions above 0.95 are left out
# and 0.95, then 0.85, take their places. (B8 pilot, 8x8 uniform: D holds at
# 0.93 of B's knee and collapses at 0.97, still draining within the limit; C
# collapses at 1.0.)
proc ms_top {fracs n d} {
  if {$n < 8 || $d ne "D"} { return $fracs }
  set out {}
  set cut 0
  foreach f $fracs { if {$f <= 0.95} { lappend out $f } else { incr cut } }
  foreach x {0.95 0.85} {
    if {$cut > 0 && [lsearch -exact $out $x] < 0} { lappend out $x; incr cut -1 }
  }
  return [lsort -real $out]
}

# ---------------------------------------------------------------- E0: does our BuffeRS reproduce [1]?
# design B (and A for reference), 4x4: bit rotation, transpose, shuffle;
# V = 1, 4; p_data 0, 1/3; B_MAX 5, 10, 15; TIGHT; 14 rates 0.02-0.52
# (one VC: rates up to 0.22, past its saturation of about 0.16)
set L {}
set rates {0.02 0.06 0.10 0.14 0.18 0.22 0.26 0.30 0.34 0.38 0.42 0.46 0.50 0.52}
foreach pat {bitrot transpose shuffle} {
  foreach v {1 4} {
    foreach pd {0.000 0.333} {
      foreach r $rates {
        if {$v == 1 && $r > 0.22} continue
        lappend L [ms_pt "E0_A_${pat}_V${v}_p${pd}_r$r" A 1 [list "TB_NUM_VC=$v"] [list "PATTERN=$pat" "RATE=$r" "PDATA=$pd"]]
        foreach b {5 10 15} {
          lappend L [ms_pt "E0_B_${pat}_V${v}_p${pd}_b${b}_r$r" B 1 [list "TB_NUM_VC=$v" "TB_B_MAX=$b"] [list "PATTERN=$pat" "RATE=$r" "PDATA=$pd"]]
        }
      }
    }
  }
}
ms_write E0_4x4 $L

# ---------------------------------------------------------------- E1: does two-level scheduling beat BuffeRS?
# A-D; uniform, transpose, bit complement, bit rotation, shuffle, hotspot;
# V = 4; 10 fractions of each pattern's knee, denser near it; 2 seeds on
# 4x4, 1 on 8x8
set fr {0.10 0.30 0.50 0.60 0.70 0.80 0.90 1.00 1.10 1.20}
foreach n {4 8} {
  set L {}
  set seeds [expr {$n == 4 ? {1 2} : {1}}]
  foreach s $seeds {
    foreach d {A B C D} {
      foreach pat {uniform transpose bitcomp bitrot shuffle hotspot} {
        set kn [ms_knee $n 4 0.333 $pat]
        foreach f [ms_top $fr $n $d] {
          lappend L [ms_pt "E1_${n}x${n}_${d}_${pat}_k${f}_s$s" $d $s [ms_nd $n $d] [list "PATTERN=$pat" [ms_rate $kn $f]]]
        }
      }
    }
  }
  ms_write "E1_${n}x${n}" $L
}

# ---------------------------------------------------------------- E2: does the gain grow with mesh size?
# B-D; uniform, transpose, region-local 0.5; 5 fractions of the knee; 16x16
# on VCS
foreach grp {{4 8} {16}} {
  set L {}
  foreach n $grp {
    foreach d {B C D} {
      foreach pat {uniform transpose region} {
        set pl {}; if {$pat eq "region"} { set pl 0.50 }
        set kn [ms_knee $n 4 0.333 $pat $pl]
        foreach f [ms_top {0.20 0.50 0.80 1.00 1.20} $n $d] {
          set plus [list "PATTERN=$pat" [ms_rate $kn $f]]
          if {$pat eq "region"} { lappend plus "PLOCAL=0.5" }
          lappend L [ms_pt "E2_${n}x${n}_${d}_${pat}_k$f" $d 1 [ms_nd $n $d] $plus]
        }
      }
    }
  }
  ms_write [expr {[llength $grp] == 1 ? "E2_16x16" : "E2_4x4_8x8"}] $L
}

# ---------------------------------------------------------------- E3: how do results depend on the number of VCs?
# A, B, D; V = 1, 2, 4 on 4x4 and 8x8; uniform, transpose; 4 fractions of the
# knee at that V, the last past it (saturation against V)
set L {}
foreach n {4 8} {
  foreach d {A B D} {
    foreach v {1 2 4} {
      foreach pat {uniform transpose} {
        set kn [ms_knee $n $v 0.333 $pat]
        foreach f [ms_top {0.30 0.60 0.90 1.10} $n $d] {
          lappend L [ms_pt "E3_${n}x${n}_${d}_V${v}_${pat}_k$f" $d 1 [concat [ms_nd $n $d] [list "TB_NUM_VC=$v"]] [list "PATTERN=$pat" [ms_rate $kn $f]]]
        }
      }
    }
  }
}
ms_write E3 $L

# ---------------------------------------------------------------- E4: does the packet mix change the picture?
# B, D; p_data 0, 1/3, 1/2; uniform; 8x8; E1's fractions of each mix's knee
set L {}
foreach d {B D} {
  foreach pd {0.000 0.333 0.500} {
    set kn [ms_knee 8 4 $pd uniform]
    foreach f [ms_top $fr 8 $d] {
      lappend L [ms_pt "E4_${d}_p${pd}_k$f" $d 1 [ms_nd 8 $d] [list "PATTERN=uniform" [ms_rate $kn $f] "PDATA=$pd"]]
    }
  }
}
ms_write E4 $L

# ---------------------------------------------------------------- E5: where do regions pay off?
# B-D; 8x8; region-local share p = 0, 0.25, 0.5, 0.75, 0.9; 0.5 and 0.9 of
# each share's knee
set L {}
foreach d {B C D} {
  foreach p {0.00 0.25 0.50 0.75 0.90} {
    set kn [ms_knee 8 4 0.333 region $p]
    foreach f {0.50 0.90} {
      lappend L [ms_pt "E5_${d}_p${p}_k$f" $d 1 [ms_nd 8 $d] [list "PATTERN=region" "PLOCAL=$p" [ms_rate $kn $f]]]
    }
  }
}
ms_write E5 $L

# ---------------------------------------------------------------- E6: does the controller follow changing traffic?
# B-D; 8x8 with D19's epoch (2^15); uniform and region-local 0.75
# alternating every 100,000 and every 300,000 cycles, four phases each, at
# 0.5 and 0.8 of the lower of the two phases' knees (uniform's); a TRACE line
# every 4,096 cycles (12 long runs)
set kn [expr {min([ms_knee 8 4 0.333 uniform], [ms_knee 8 4 0.333 region 0.75])}]
set L {}
foreach d {B C D} {
  foreach per {100000 300000} {
    foreach f {0.50 0.80} {
      set defs [ms_nd 8 $d]
      lappend L [ms_pt "E6_${d}_per${per}_k$f" $d 1 $defs \
        [list "PATTERN=phase" "PHASE_A=uniform" "PHASE_B=region" "PLOCAL=0.75" "PHASE_LEN=$per" [ms_rate $kn $f] \
              "WARMUP=4096" "MEASURE=[expr {4 * $per}]" "TRACE=4096"]]
    }
  }
}
ms_write E6 $L

# ---------------------------------------------------------------- E7: what do regions and adaptivity each contribute?
# C, D-PROP, D-SQRT, D without damping; 8x8; uniform, hotspot, region-local
# 0.5; 5 fractions of the knee. No K_MIN = 0 variant (D62): below
# saturation it gives K_MIN = 1's plans, since even traffic only across
# regions makes share bin 7 (a packet ends in its destination's region),
# whose table entry is 1 (B8 pre-check)
set L {}
foreach var {C D_PROP D_SQRT D_nodamp} {
  set d [string index $var 0]
  set defs [ms_nd 8 $d]
  set extra {}
  switch -- $var {
    D_SQRT   { lappend extra "POLICY=SQRT" }
    D_nodamp { lappend extra "DAMP=0" }
  }
  foreach pat {uniform hotspot region} {
    set pl {}; if {$pat eq "region"} { set pl 0.50 }
    set kn [ms_knee 8 4 0.333 $pat $pl]
    foreach f [ms_top {0.20 0.50 0.80 1.00 1.20} 8 $d] {
      set plus [concat [list "PATTERN=$pat" [ms_rate $kn $f]] $extra]
      if {$pat eq "region"} { lappend plus "PLOCAL=0.5" }
      lappend L [ms_pt "E7_${var}_${pat}_k$f" $d 1 $defs $plus]
    }
  }
}
ms_write E7 $L

# ---------------------------------------------------------------- E8: how sensitive are the chosen parameters?
# D on 8x8, one factor at a time around epoch 2^15, K_MAX 15, 4x4 regions,
# B_MAX 10: epoch 2^13-2^17; K_MAX 7, 15, 31; regions 2x2, 4x4; B_MAX 5, 10, 15;
# uniform and region-local 0.5 at 0.4, 0.7 and 0.95 of the knee (the base
# configuration's knee for every factor)
set cfgs {}
foreach e {8192 16384 32768 65536 131072} { lappend cfgs [list "ep$e" [list "TB_EPOCH=$e"]] }
foreach k {7 31} { lappend cfgs [list "kmax$k" [list "TB_EPOCH=32768" "TB_K_MAX=$k"]] }
lappend cfgs [list "reg2" [list "TB_EPOCH=32768" "TB_REGION_N=2"]]
foreach b {5 15} { lappend cfgs [list "bmax$b" [list "TB_EPOCH=32768" "TB_B_MAX=$b"]] }
set L {}
foreach c $cfgs {
  lassign $c tag defs
  foreach pat {uniform region} {
    set pl {}; if {$pat eq "region"} { set pl 0.50 }
    set kn [ms_knee 8 4 0.333 $pat $pl]
    foreach f {0.40 0.70 0.95} {
      set plus [list "PATTERN=$pat" [ms_rate $kn $f]]
      if {$pat eq "region"} { lappend plus "PLOCAL=0.5" }
      lappend L [ms_pt "E8_D_${tag}_${pat}_k$f" D 1 [concat [ms_n 8] $defs] $plus]
    }
  }
}
ms_write E8 $L

# ---------------------------------------------------------------- E9: do the guarantees hold in practice?
# B-D; 8x8; hotspot; long windows (200,000 cycles); 0.5 and 0.9 of hotspot's
# knee (one node takes 20 % of the packets: its ejection port saturates at
# about a quarter of uniform's knee), 2 seeds
set kn [ms_knee 8 4 0.333 hotspot]
set L {}
foreach d {B C D} {
  foreach f {0.50 0.90} {
    foreach s {1 2} {
      lappend L [ms_pt "E9_${d}_hotspot_k${f}_s$s" $d $s [ms_nd 8 $d] [list "PATTERN=hotspot" [ms_rate $kn $f] "MEASURE=200000"]]
    }
  }
}
ms_write E9 $L

# ---------------------------------------------------------------- E10: how robust is the reservation?
# B, D; 4x4; sink stall chance 0, 5%, 10%, 20%; 1 and 4 VCs; 0.5 and 0.9 of
# the knee at that VC count and stall chance
set L {}
foreach d {B D} {
  foreach v {1 4} {
    foreach st {0.00 0.05 0.10 0.20} {
      set kn [ms_knee 4 $v 0.333 uniform "" $st]
      foreach f {0.50 0.90} {
        lappend L [ms_pt "E10_${d}_V${v}_st${st}_k$f" $d 1 [list "TB_NUM_VC=$v"] [list "PATTERN=uniform" "STALL=$st" [ms_rate $kn $f]]]
      }
    }
  }
}
ms_write E10 $L

# ---------------------------------------------------------------- S06: do the simulators agree?
# short points on every design (4x4): the same CSV rows on every simulator
set L {}
foreach d {A B C D} {
  foreach pat {uniform transpose} {
    foreach f {0.30 0.70} {
      lappend L [ms_pt "S06_${d}_${pat}_f$f" $d 1 {} [list "PATTERN=$pat" "RATE_FRAC=$f" "WARMUP=500" "MEASURE=2000"]]
    }
  }
}
ms_write S06 $L

foreach k [dict keys $ms_count] { puts [format "make_sweeps: %-12s %4d points" $k [dict get $ms_count $k]] }
