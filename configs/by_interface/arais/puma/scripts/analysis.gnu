# plot debug
# 執行完 log_filter.pl 之後，會自 mk-wosi.log 
# 取出 vel_o 較大的 10000 筆資料，存成 debug.log
plot \
    "debug.log" using 1:($2/65536)        title "debug0 J0_req_vel", \
    "debug.log" using 1:($3/65536)        title "debug1 J0_cmd_d", \
    "debug.log" using 1:($4)              title "debug2 J0_error_hi", \
    "debug.log" using 1:($5/65536)        title "debug3 J0_vel_o", \
    "debug.log" using 1:($6)              title "debug4 cur_tick", \
    "debug.log" using 1:($7/65536)        title "debug5 J0_encv_lpf", \
    "debug.log" using 1:($8/65536)        title "debug6 prev_joint_vel_cmd", \
    "debug.log" using 1:($9/65536)        title "debug7 J0_acc"

# 分析 VBC 生產記錄：
plot \
    "ll-999.csv" using 1:3 title "tangential_vel", \
    "ll-999.csv" using 1:4 title "platen_rpm", \
    "ll-999.csv" using 1:5 title "brush_rpm"

