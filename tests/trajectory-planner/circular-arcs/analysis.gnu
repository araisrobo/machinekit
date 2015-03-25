# to plot trace dumped by src/emc/kinematics/tp.c
plot \
    "configs/tp.log" using 1:2 title "id", \
    "configs/tp.log" using 1:3 title "state", \
    "configs/tp.log" using 1:4 title "req_vel", \
    "configs/tp.log" using 1:5 title "cur_accel", \
    "configs/tp.log" using 1:6 title "cur_vel", \
    "configs/tp.log" using 1:7 title "progress"
