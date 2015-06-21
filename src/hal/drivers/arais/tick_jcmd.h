#ifndef __TICK_JOINT_CMD_H__
#define __TICK_JOINT_CMD_H__

#define TICK_JOINT_NUM 6

typedef struct {
    int         _tick;
    int         cmd;
    double      pos_cmd[TICK_JOINT_NUM];
} tick_jcmd_t;

typedef enum {
    TICK_AMP_ENABLE = 0,        //<! bit-0 of cmd
    TICK_UPDATE_POS_ACK,        //<! bit-1 of cmd
} tick_cmd_t;

#endif
