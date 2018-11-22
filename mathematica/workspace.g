# workspace.g - gnuplot code to generate joint vs work space comparisoins
# RMM, 4 Aug 93
#
# Reads data created by workspace.m and generates the subplots used
# to create ch4-jointct.fig and ch4-workct.fig

# define some variables to keep things straight
time=1
th1 = 2
th2 = 3
xe = 8
ye = 9

set terminal bfig

#
# Joint space plots
#

set size 0.721, 1
set nokey
set title "Trajectory in joint coordinates"
set output "ch4-jointct-1a.fig"
plot [-0.5:2] [-0.5:2] 'joint.dat' using th1:th2 with lines

set size 0.721, 0.5
set key
set title "Joint space trajectory versus time"
set output "ch4-jointct-2a.fig"
plot [0:2] [-0.5:2] \
  'joint.dat' using time:th1 title "$\theta_1$" with lines, \
  'joint.dat' using time:th2 title "$\theta_2$" with lines

set size 0.721, 1
set nokey
set title "Trajectory in Cartesian coordinates"
set output "ch4-jointct-1b.fig"
plot [-0.5:2] [-0.5:2] 'joint.dat' using xe:ye with lines

set size 0.721, 0.5
set key
set title "Work space trajectory versus time"
set output "ch4-jointct-2b.fig"
plot [0:2] [-.5:2] \
  'joint.dat' using time:xe title "$x$" with lines, \
  'joint.dat' using time:ye title "$y$" with lines

#
# Work space plots
#

set size 0.721, 1
set nokey
set title "Trajectory in joint coordinates"
set output "ch4-workct-1a.fig"
plot [-0.5:2] [-0.5:2] 'work.dat' using th1:th2 with lines

set size 0.721, 0.5
set key
set title "Joint space trajectory versus time"
set output "ch4-workct-2a.fig"
plot [0:2] [-0.5:2] \
  'work.dat' using time:th1 title "$\theta_1$" with lines, \
  'work.dat' using time:th2 title "$\theta_2$" with lines

set size 0.721, 1
set nokey
set title "Trajectory in Cartesian coordinates"
set output "ch4-workct-1b.fig"
plot [-0.5:2] [-0.5:2] 'work.dat' using xe:ye with lines

set size 0.721, 0.5
set key
set title "Work space trajectory versus time"
set output "ch4-workct-2b.fig"
plot [0:2] [-.5:2] \
  'work.dat' using time:xe title "$x$" with lines, \
  'work.dat' using time:ye title "$y$" with lines

quit
