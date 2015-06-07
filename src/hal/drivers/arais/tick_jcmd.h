#ifndef __TICK_JOINT_CMD_H__
#define __TICK_JOINT_CMD_H__

#define TICK_JOINT_NUM 6

typedef struct {
    int         _tick;
    int         cmd;
    double      pos_cmd[TICK_JOINT_NUM];
} tick_jcmd_t;

typedef enum {
    TICK_JCMD = 0,
    TICK_UPDATE_POS_ACK,
} tick_cmd_t;

#endif
