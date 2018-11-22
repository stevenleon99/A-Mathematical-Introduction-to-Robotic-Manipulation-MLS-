# lyapunov.g - gnuplot commands to generate lyapunov.fig
# RMM, 4 Aug 93
#
# This file uses data created by lyapunov.m and lyapunov.cmd
# and creates fig plots [lyapunov-1.fig, lypapunov-2.fig] which
# can be combined to create ch4-lyapunov.fig
#

set terminal bfig
set size 0.721, 1
set nokey

set output 'ch4-lyapunov-1.fig'
plot \
  'harmonic.dat' using 2:3 with lines, \
  'round.dat' using 2:3 with lines 2, \
  'round.dat' using 4:5 with lines 2

set output 'ch4-lyapunov-2.fig'
plot \
  'harmonic.dat' using 2:3 with lines, \
  'skew.dat' using 2:3 with lines 2, \
  'skew.dat' using 4:5 with lines 2
