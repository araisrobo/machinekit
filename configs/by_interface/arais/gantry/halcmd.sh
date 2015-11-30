#!/bin/bash

#  tclsh ${HOME}/proj/emc2-dev/tcl/bin/halshow.tcl

echo "bp-tick: " `halcmd getp wosi.bp-tick`

# for a in 0 1 2 3 4 5; do
for a in 4; do
echo "enc_pos[$a]:   " `halcmd getp wosi.stepgen.$a.enc_pos`
echo "cmd_pos[$a]:   " `halcmd getp wosi.stepgen.$a.cmd-pos`
echo "rawcount32[$a]:   " `halcmd getp wosi.stepgen.$a.rawcount32`
# echo "ferror[$a]:    " `halcmd getp wosi.stepgen.$a.ferror-flag`
# echo "axis.$a.joint-pos-cmd:   " `halcmd getp axis.$a.joint-pos-cmd`
# echo "axis.$a.joint-pos-fb:    " `halcmd getp axis.$a.joint-pos-fb`
# echo "axis.$a.motor-pos-cmd:   " `halcmd getp axis.$a.motor-pos-cmd`
# echo "axis.$a.motor-pos-fb:    " `halcmd getp axis.$a.motor-pos-fb`
# # echo "axis.$a.probed-pos:      " `halcmd getp axis.$a.probed-pos`
# echo "wosi.stepgen.$a.position-cmd:   " `halcmd getp wosi.stepgen.$a.position-cmd`
# echo "wosi.stepgen.$a.position-fb:    " `halcmd getp wosi.stepgen.$a.position-fb`
# echo "wosi.stepgen.$a.rawcount32:     " `halcmd getp wosi.stepgen.$a.rawcount32`
done

# echo "axis.0.home-state:         ....." `halcmd getp axis.0.home-state`
# echo "motion.rcmd-state:          ....." `halcmd getp motion.rcmd-state`
# 
# # for a in 0 1 2 3 4 5 6 7; do
# # echo "debug-$a:      " `halcmd getp wosi.debug.value-$a`
# # done
# 
echo "debug-00,  pid[4].error     ......" `halcmd getp wosi.debug.value-00`
echo "debug-01,  pid[4].cmd_d     ......" `halcmd getp wosi.debug.value-01`
echo "debug-02,                   ......" `halcmd getp wosi.debug.value-02`
echo "debug-03,  joint_vel_cmd[4] pid..." `halcmd getp wosi.debug.value-03`
echo "debug-04,  joint_vel_cmd[4] pwm..." `halcmd getp wosi.debug.value-04`
echo "debug-05,  joint_cmd[4]     ......" `halcmd getp wosi.debug.value-05`
echo "debug-06,  pwm cmd          ......" `halcmd getp wosi.debug.value-06`
echo "debug-07,  machine-on       ......" `halcmd getp wosi.debug.value-07`
