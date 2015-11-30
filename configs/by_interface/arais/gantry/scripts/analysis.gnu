# to plot trace dumped by src/emc/kinematics/tp.c
plot \
    "tp.log" using 1:2 title "state", \
    "tp.log" using 1:3 title "req_vel", \
    "tp.log" using 1:4 title "cur_accel", \
    "tp.log" using 1:5 title "cur_vel", \
    "tp.log" using 1:6 title "progress", \
    "tp.log" using 1:7 title "target", \
    "tp.log" using 1:9 title "tc_target"

plot \
    "simple_tp.log" using 1:2 title "pos_cmd", \
    "simple_tp.log" using 1:3 title "pos", \
    "simple_tp.log" using 1:4 title "vel", \
    "simple_tp.log" using 1:5 title "accel", \
    "simple_tp.log" using 1:6 title "vel_req", \
    "simple_tp.log" using 1:7 title "max_vel"

# plot J0 position-cmd and position-fb
plot \
    "wou_stepgen.log" using 1:3 title "prev_pcmd[0]", \
    "wou_stepgen.log" using 1:4 title "pos_fb[0]", \
    "wou_stepgen.log" using 1:5 title "risc_pos_cmd[0]"

#plot XY-path
plot \
    "wou_stepgen.log" using 3:7 title "xy cmd", \
    "wou_stepgen.log" using 3:11 title "xyy cmd", \
    "wou_stepgen.log" using 4:8 title "xy fb", \
    "wou_stepgen.log" using 4:12 title "xyy fb"

# plot J1/J2
plot \
    "wou_stepgen.log" using 1:($26/65536) title "debug0 J1_vel_cmd", \
    "wou_stepgen.log" using 1:($27/4) title "debug1 J1_cmd", \
    "wou_stepgen.log" using 1:($28/65536) title "debug2 J1_err",\
    "wou_stepgen.log" using 1:($29/65536) title "debug3 J2_err",\
    "wou_stepgen.log" using 1:($30/65536) title "debug4 J0_pcmd"

# plot J0   
plot \
    "wou_stepgen.log" using 1:($26/65536) title "debug0 J0_vel_cmd", \
    "wou_stepgen.log" using 1:($27/4) title "debug1 J0_cmd", \
    "wou_stepgen.log" using 1:($28/65536) title "debug2 J0_err", \
    "wou_stepgen.log" using 1:($29/65536) title "debug3 J0_pcmd", \
    "wou_stepgen.log" using 1:($30/65536) title "debug4 J0_req"

#plot XZ-path
plot \
    "wou_stepgen.log" using 3:15 title "xz cmd"


# plot debug
plot \
    "debug.log" using 1:($2/65536)        title "debug0 J0_req_vel", \
    "debug.log" using 1:($3/65536)        title "debug1 J0_cmd_d", \
    "debug.log" using 1:($4)              title "debug2 J0_error_hi", \
    "debug.log" using 1:($5/65536)        title "debug3 J0_vel_o", \
    "debug.log" using 1:($6)              title "debug4 cur_tick", \
    "debug.log" using 1:($7/65536)        title "debug5 J0_encv_lpf", \
    "debug.log" using 1:($8/65536)        title "debug6 J0_acc0", \
    "debug.log" using 1:($9/65536)        title "debug7 J0_acc1"

plot \
    "debug.log" using 1:($2/65536)        title "debug0 J0_req_vel", \
    "debug.log" using 1:($3/65536)        title "debug1 J0_cmd_d", \
    "debug.log" using 1:($4)              title "debug2 J0_error_hi", \
    "debug.log" using 1:($5/65536)        title "debug3 J0_vel_o", \
    "debug.log" using 1:($6)              title "debug4 cur_tick", \
    "debug.log" using 1:($7/65536)        title "debug5 J0_encv_lpf", \
    "debug.log" using 1:($10/16384*100)   title "debug8 J0_pwm%", \
    "debug.log" using 1:($11/65536)       title "debug9 J0_svp_vel"
