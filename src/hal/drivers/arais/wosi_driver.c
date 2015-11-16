/********************************************************************
 * Description:  wosi.c
 *               This file, 'wosi.c', is a HAL component that
 *               It was based on stepgen.c by John Kasunich.
 *
 * Author: Yishin Li
 * License: GPL Version 2
 *
 * Copyright (c) 2009-2015 All rights reserved.
 *
 ********************************************************************/

#ifndef ULAPI
#error This is intended as a userspace component only.
#endif

#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <unistd.h>

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "hal.h"		/* HAL public API decls */
#include "tc.h"                 /* motion state */
#include <math.h>
#include <string.h>
#include <float.h>
#include "rtapi_math.h"
#include "motion.h"

#include <wosi.h>
#include <wb_regs.h>
#include <mailtag.h>
#include "sync_cmd.h"
#include "tick_jcmd.h"
#include "inifile.h"

#define MAX_NUM_JOINT 8
#define MAX_STEP_CUR 255
#define PLASMA_ON_BIT 0x02
#define FRACTION_BITS 16
#define FIXED_POINT_SCALE   65536.0             // (double (1 << FRACTION_BITS))
#define FP_SCALE_RECIP      0.0000152587890625  // (1.0/65536.0)
#define FRACTION_MASK 0x0000FFFF

#undef PRINT_BANDWIDTH  // call wosi_status() to print WOSI bandwidth utilization

// to disable DP(): #define TRACE 0
// to dump JOINT status, #define TRACE 1
// to dump JOINT status and time statics, #define TRACE 2
#define TRACE 0
#include "tp_debug.h"
#if (TRACE!=0)
static FILE *dptrace;
#endif

/* module information */
//MODULE_AUTHOR("Yishin Li");
//MODULE_DESCRIPTION("HAL for Wishbone Over Serial Interface");
//MODULE_LICENSE("GPL");
# define GPIO_IN_NUM    96
# define GPIO_OUT_NUM   96

static const char wosi_id = 0;
static wosi_param_t w_param;

#define FIXED_POINT_SCALE       65536.0             // (double (1 << FRACTION_BITS))
#define MAX_DSIZE               127     // Maximum WOSI data size

static char reload_ini_file[80];

static int initialized = 0; //!< flag to determine if the board is initialized

//!< servo_period_ns: servo period for velocity control, unit: ns
static int servo_period_ns = 0;
//!< lsp_id: gpio pin id for limit-switch-positive(lsp)
static int lsp_id[MAX_NUM_JOINT];
//!< lsn_id: gpio pin id for limit-switch-negative(lsn)
static int lsn_id[MAX_NUM_JOINT];

/***********************************************************************
 *                STRUCTURES AND GLOBAL VARIABLES                       *
 ************************************************************************/

/** This structure contains the runtime data for a single generator. */

/* structure members are ordered to optimize caching for makepulses,
 which runs in the fastest thread */

// #pragma pack(push)  /* push current alignment to stack */
// #pragma pack(1)     /* set alignment to 1 byte boundary */
typedef struct
{
    // hal_pin_*_newf: variable has to be pointer
    // hal_param_*_newf: varaiable not necessary to be pointer
    int64_t rawcount; /* precision: 64.16; accumulated pulse sent to FPGA */
    hal_s32_t *rawcount32; /* 32-bit integer part of rawcount */
    char out_type; /* A(AB-PHASE), S(STEP-DIR), P(PWM), D(DAC) */
    char enc_type; /* A(AB-PHASE), S(STEP-DIR), L(LOOPBACK) */
    double enc_scale; /* encoder scale */
    char alr_id; /* alarm id */
    hal_s32_t *enc_pos; /* pin: encoder position from servo drive, captured from FPGA */
    hal_float_t pos_scale; /* param: steps per position unit */
    double scale_recip; /* reciprocal value used for scaling */
    double vel_cmd_t; /* velocity command (units/cycle_time) */
    double prev_pos_cmd; /* prev pos_cmd: previous position command */
    hal_float_t *probed_pos;
    hal_float_t *pos_fb; /* pin: position feedback (position units) */
    hal_float_t *risc_pos_cmd; /* pin: position command issued by RISC (position units) */
    hal_float_t *ferror; /* pin: following error (position units) */
    hal_float_t *vel_cmd; /* pin: velocity command (unit/sec) */
    hal_float_t *vel_fb;  /* pin: velocity feedback (unit/sec) */
    hal_bit_t *ferror_flag; /* following error flag from risc */
    uint32_t pulse_maxv; /* max-vel in pulse */
    uint32_t pulse_maxa; /* max-accel in pulse */
    uint32_t pulse_maxj; /* max-jerk  in pulse */
    int32_t pulse_vel; /* velocity in pulse */
    int32_t pulse_accel; /* accel in pulse */
    int32_t pulse_jerk; /* jerk in pulse */

    /* motion type be set */
    int32_t motion_type; /* motion type wrote to risc */

    hal_s32_t *cmd_pos; /* position command retained by RISC (unit: pulse) */
    hal_s32_t *enc_vel_p; /* encoder velocity in pulse per servo-period */

    hal_bit_t *homing;
    hal_float_t *risc_probe_dist;
    hal_s32_t *risc_probe_pin;
    hal_s32_t *risc_probe_type;
    uint32_t risc_probing;
    hal_float_t *uu_per_rev;
    hal_float_t prev_uu_per_rev;
    hal_float_t *jog_vel;
    hal_float_t prev_jog_vel;

    hal_bit_t *bypass_lsp;
    hal_bit_t *bypass_lsn;
    hal_bit_t prev_bypass_lsp;
    hal_bit_t prev_bypass_lsn;

} stepgen_t;
// #pragma pack(pop)   /* restore original alignment from stack */

typedef struct
{
    // Analog input: 0~4.096VDC, up to 16 channel
    hal_float_t *in[16];
    hal_float_t *out[4];        // DAC command
    hal_float_t prev_out[4];
    hal_float_t *out_fb[4];     // DAC feedback
} analog_t;

// machine_control_t:
typedef struct
{
    hal_bit_t *reload_params;
    hal_float_t *current_vel;
    hal_float_t *feed_scale;
    hal_bit_t *rt_abort; // realtime abort to FPGA
    /* sync input pins (input to motmod) */
    hal_bit_t *in[96];
    hal_bit_t *in_n[96];
    uint32_t prev_in0;
    uint32_t prev_in1;
    uint32_t prev_in2;
    hal_float_t *analog_ref_level;
    double prev_analog_ref_level;
    hal_bit_t *sync_in_trigger;
    hal_u32_t *sync_in_index; //
    hal_u32_t *wait_type;
    hal_float_t *timeout;
    double prev_timeout;

    hal_u32_t *bp_tick; /* base-period tick obtained from fetchmail */
    hal_u32_t *dout0; /* the DOUT value obtained from fetchmail */
    hal_u32_t *crc_error_counter;

    /* sync output pins (output from motmod) */
    hal_bit_t *out[GPIO_OUT_NUM];
    uint32_t prev_out[(GPIO_OUT_NUM >> 5)]; //ON or OFF

    uint32_t prev_ahc_state;
    hal_bit_t *ahc_state; // 0: disable 1:enable
    hal_float_t *ahc_level;
    hal_bit_t *motion_s3; // synchronize AHC with S3, 0: disable 1:enable
    double prev_ahc_level;
    hal_s32_t *accel_state;
    hal_s32_t prev_accel_state;
    hal_u32_t *jog_sel;
    hal_s32_t *jog_vel_scale;

    /* command channel for emc2 */
    hal_u32_t *wosi_cmd;
    uint32_t prev_wosi_cmd;
    uint32_t a_cmd_on_going;
    /* test pattern  */
    hal_s32_t *test_pattern;
    /* MPG */
    hal_s32_t *mpg_count;
    hal_s32_t *debug[32];

    hal_bit_t *teleop_mode;
    hal_bit_t *coord_mode;

    uint32_t prev_machine_ctrl; // num_joints is not included
    hal_bit_t *machine_on;

    hal_bit_t *update_pos_req;
    hal_u32_t *rcmd_seq_num_req;
    hal_u32_t *rcmd_seq_num_ack;
    hal_u32_t *rcmd_state;

    hal_u32_t *cur_tick;        // RISC cycle-time(20ns) ticks of current servo-period
    hal_u32_t *max_tick;

    // machine_status bits
    hal_bit_t *ahc_doing;
    hal_bit_t *rtp_running; // risc/remote tp running
    hal_bit_t *sfifo_empty;

    hal_u32_t *spindle_joint_id;

    hal_bit_t *gantry_en;
    uint32_t gantry_ctrl;
    uint32_t prev_gantry_ctrl;

    hal_u32_t *trigger_din;
    hal_u32_t *trigger_ain;
    hal_u32_t *trigger_type;
    hal_bit_t *trigger_cond;
    hal_u32_t *trigger_level;
    hal_bit_t *probing;
    hal_bit_t *trigger_result; // 0: disable 1:enable

    hal_bit_t prev_probing;
    double prev_trigger_level;

    hal_bit_t *pso_req;
    hal_u32_t *pso_ticks;
    hal_u32_t *pso_mode;
    hal_u32_t *pso_joint;
    hal_float_t *pso_pos;

} machine_control_t;

/* ptr to array of stepgen_t structs in shared memory, 1 per channel */
static stepgen_t *stepgen_array;
static analog_t *analog;
static machine_control_t *machine_control;

/* lookup tables for stepping types 2 and higher - phase A is the LSB */

#define MAX_STEP_TYPE 14

/* other globals */
static int comp_id; /* hal component id */
static int num_joints = 0; /* number of step generators configured */
static double dt; /* update_freq period in seconds */
static double recip_dt; /* reciprocal of period, avoids divides */

/***********************************************************************
 *                  LOCAL FUNCTION DECLARATIONS                         *
 ************************************************************************/

static int export_stepgen(int num, stepgen_t * addr);
static int export_analog(analog_t * addr);
static int export_machine_control(machine_control_t * machine_control);
static void update_rt_cmd(void);

#if (TRACE==2)
static void diff_time(struct timespec *start, struct timespec *end,
        struct timespec *diff)
{
    if ((end->tv_nsec - start->tv_nsec) < 0)
    {
        diff->tv_sec = end->tv_sec - start->tv_sec - 1;
        diff->tv_nsec = 1000000000 + end->tv_nsec - start->tv_nsec;
    } else
    {
        diff->tv_sec = end->tv_sec - start->tv_sec;
        diff->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
    return;
}
#endif

void endian_swap(uint32_t *x)
{
    *x = (*x >> 24) | ((*x << 8) & 0x00FF0000) | ((*x >> 8) & 0x0000FF00)
            | (*x << 24);
}

/************************************************************************
 * callback functions for libwosi                                        *
 ************************************************************************/
static void get_crc_error_counter(int32_t crc_error_counter)
{
    // store crc_error_counter
    return;
}

static void fetchmail(const uint8_t *buf_head)
{
    int i;
    uint16_t mail_tag;
    uint32_t *p, din[3]; //, dout[1];
    uint8_t *buf;
    stepgen_t *stepgen;
    uint32_t bp_tick; // served as previous-bp-tick
    uint32_t machine_status;

    memcpy(&mail_tag, (buf_head + 2), sizeof(uint16_t));

    // BP_TICK
    p = (uint32_t *) (buf_head + 4);
    bp_tick = *p;
    *machine_control->bp_tick = bp_tick;
#if (TRACE==2)
    DP("bp_tick(%u)\n", bp_tick);
#endif

    switch (mail_tag)
    {
    case MT_MOTION_STATUS:
        stepgen = stepgen_array;
        for (i = 0; i < num_joints; i++)
        {
            p += 1;
            *(stepgen->enc_pos) = (int32_t) *p;
            p += 1;
            *(stepgen->cmd_pos) = (int32_t) *p;
            p += 1;
            *(stepgen->enc_vel_p) = (int32_t) *p; // encoder velocity in pulses per servo-period
            stepgen += 1; // point to next joint
        }

        // digital input
        p += 1;
        din[0] = *p;
        p += 1;
        din[1] = *p;
        p += 1;
        din[2] = *p;
        // digital output
        p += 1;
        *machine_control->dout0 = *p;

        // update gpio_in[31:0]
        // compare if there's any GPIO.DIN bit got toggled
        if (machine_control->prev_in0 != din[0])
        {
            // avoid for-loop to save CPU cycles
            machine_control->prev_in0 = din[0];
            for (i = 0; i < 32; i++)
            {
                *(machine_control->in[i]) = ((machine_control->prev_in0) >> i)
                        & 0x01;
                *(machine_control->in_n[i]) = (~(*(machine_control->in[i])))
                        & 0x01;
            }
        }

        // update gpio_in[63:32]
        // compare if there's any GPIO.DIN bit got toggled
        if (machine_control->prev_in1 != din[1])
        {
            // avoid for-loop to save CPU cycles
            machine_control->prev_in1 = din[1];
            for (i = 32; i < 64; i++)
            {
                *(machine_control->in[i]) = ((machine_control->prev_in1)
                        >> (i - 32)) & 0x01;
                *(machine_control->in_n[i]) = (~(*(machine_control->in[i])))
                        & 0x01;
            }
        }

        // update gpio_in[95:64]
        // compare if there's any GPIO.DIN bit got toggled
        if (machine_control->prev_in2 != din[2])
        {
            // avoid for-loop to save CPU cycles
            machine_control->prev_in2 = din[2];
            for (i = 64; i < 96; i++)
            {
                *(machine_control->in[i]) = ((machine_control->prev_in2)
                        >> (i - 64)) & 0x01;
                *(machine_control->in_n[i]) = (~(*(machine_control->in[i])))
                        & 0x01;
            }
        }

        // copy 16 channel of 16-bit ADC value
        p += 1;
        buf = (uint8_t*) p;
        for (i = 0; i < 8; i++)
        {
            *(analog->in[i * 2]) = (hal_float_t)*(((uint16_t*) buf) + i * 2 + 1);
            *(analog->in[i * 2 + 1]) = (hal_float_t)*(((uint16_t*) buf) + i * 2);
        }
        p += i; // skip 16ch of 16-bit ADC value

        // copy 4 channel of 16-bit DAC feedback value
        buf = (uint8_t*) p;
        for (i = 0; i < 2; i++)
        {
            *(analog->out_fb[i * 2]) = (hal_float_t)*(((uint16_t*) buf) + i * 2 + 1);
            *(analog->out_fb[i * 2 + 1]) = (hal_float_t)*(((uint16_t*) buf) + i * 2);
        }
        p += i; // skip 4ch of 16-bit DAC value

        // MPG
        *(machine_control->mpg_count) = *p;
        // the MPG on my hand is 1-click for a full-AB-phase-wave.
        // therefore the mpg_count will increase by 4.
        // divide it by 4 for smooth jogging.
        // otherwise, there will be 4 units of motions for every MPG click.
        *(machine_control->mpg_count) >>= 2;
        p += 1;
        machine_status = *p;
        stepgen = stepgen_array;
        for (i = 0; i < num_joints; i++)
        {
            *stepgen->ferror_flag = machine_status & (1 << i);
            stepgen += 1; // point to next joint
        }
        *machine_control->ahc_doing = (machine_status >> AHC_DOING_BIT) & 1;
        *machine_control->rtp_running = (machine_status >> TP_RUNNING_BIT) & 1;
        *machine_control->sfifo_empty = (machine_status >> SFIFO_IS_EMPTY_BIT) & 1;
        if (*machine_control->sfifo_empty)
        {
            rtapi_print ("bp_tick(%d) sfifo_empty\n", bp_tick);
        }

        p += 1;
        *(machine_control->cur_tick) = *p;

        p += 1;
        *(machine_control->max_tick) = *p;

        p += 1;
        *machine_control->rcmd_state = *p;

        break;

    case MT_ERROR_CODE:
        // error code
        p = (uint32_t *) (buf_head + 4); // prev_bp_tick - bp_offset
        p += 1;
        bp_tick = *p;
        p += 1;
//        printf ("MT_ERROR_CODE: code(%d) bp_tick(%d) \n", *p, bp_tick);
        break;

    case MT_DEBUG:
        p = (uint32_t *) (buf_head + 4);
        bp_tick = *p;
        for (i = 0; i < 8; i++)
        {
            p += 1;
            *machine_control->debug[i] = *p;
        }
        break;

    case MT_PROBED_POS:
        stepgen = stepgen_array;
        p = (uint32_t *) (buf_head + 4);
        for (i = 0; i < num_joints; i++)
        {
            p += 1;
            *(stepgen->probed_pos) = (double) ((int32_t) *p)
                    * (stepgen->scale_recip);
            stepgen += 1; // point to next joint
        }
        p += 1;
        *machine_control->trigger_result = ((uint8_t) *p);

        p += 1;
        *machine_control->rcmd_state = *p;
        if (*p == RCMD_UPDATE_POS_REQ)
        {
            // SET update_pos_req here
            // will RESET it after sending a RCMD_UPDATE_POS_ACK packet
            *machine_control->update_pos_req = 1;
            p += 1;
            *machine_control->rcmd_seq_num_req = *p;
        }

        break;

    default:
        fprintf(stderr, "ERROR: wosi.c unknown mail tag (%d)\n", mail_tag);
        break;
    }
}

static void write_mot_param(uint32_t joint, uint32_t addr, int32_t data)
{
    uint16_t sync_cmd;
    uint8_t buf[MAX_DSIZE];
    int j;

    for (j = 0; j < sizeof(int32_t); j++)
    {
        sync_cmd = SYNC_DATA | ((uint8_t *) &data)[j];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), buf);
    }

    sync_cmd = SYNC_MOT_PARAM | PACK_MOT_PARAM_ID(joint)
            | PACK_MOT_PARAM_ADDR(addr);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), buf);
    while (wosi_flush(&w_param) == -1)
        ;

    return;
}

static void send_sync_cmd(uint16_t sync_cmd, uint32_t *data, uint32_t size)
{
    uint16_t buf;
    int i, j;

    // TODO: pack whole data into a single WB_WR_CMD
    for (i = 0; i < size; i++)
    {
        for (j = 0; j < sizeof(int32_t); j++)
        {
            buf = SYNC_DATA | ((uint8_t *) data)[j];
            wosi_cmd(&w_param, WB_WR_CMD,
                    (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), sizeof(uint16_t),
                    (const uint8_t *) &buf);
        }
        data++;
    }

    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), (const uint8_t *) &sync_cmd);
    while (wosi_flush(&w_param) == -1)
        ; // wait until all those WB_WR_CMDs are accepted by WOSI

    return;
}

static void write_machine_param(uint32_t addr, int32_t data)
{
    uint16_t sync_cmd;
    uint8_t buf[MAX_DSIZE];
    int j;

    for (j = 0; j < sizeof(int32_t); j++)
    {
        sync_cmd = SYNC_DATA | ((uint8_t *) &data)[j];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), buf);
    }
    sync_cmd = SYNC_MACH_PARAM | PACK_MACH_PARAM_ADDR(addr);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), buf);

    while (wosi_flush(&w_param) == -1)
        ;
    return;
}

static int system_initialize(FILE *fp)
{
    const char *s;
    char board[20];
    uint8_t data[MAX_DSIZE];
    int32_t immediate_data;
    int n;
    int retval;

    s = iniFind(fp, "BOARD", "ARAIS");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: can not find [ARAIS]BOARD\n");
        return -1;
    }
    strcpy(board, s);

    s = iniFind(fp, "FPGA", "ARAIS");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: can not find [ARAIS]FPGA\n");
        return -1;
    }

    // initialize FPGA with bitfile(bits)
    wosi_init(&w_param, board, wosi_id, s);
    if (wosi_connect(&w_param) != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "WOSI: ERROR: Connection failed\n");
        return -1;
    }

    s = iniFind(fp, "RISC", "ARAIS");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: can not find [ARAIS]RISC\n");
        return -1;
    }
    // programming risc with binfile([ARAIS]RISC)
    if (wosi_prog_risc(&w_param, s) != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOSI: ERROR: load RISC program filed\n");
        return -1;
    }

    // Update num_joints based on [ARAIS]JOINTS
    // JOINTS: the number of joint to be controlled by FPGA
    s = iniFind(fp, "JOINTS", "ARAIS");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOSI: ERROR: no [ARAIS]JOINTS defined\n");
        return -1;
    } else
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: [ARAIS]JOINTS=%s\n", s);
        num_joints = atoi(s);
        assert(num_joints > 0);
        assert(num_joints <= 8); // support up to 8 joints for AR11
        /* to clear PULSE/ENC/SWITCH/INDEX positions for all joints*/

        // issue a WOSI_WRITE to RESET SSIF position registers
        data[0] = (1 << num_joints) - 1; // bit-map-for-num_joints (ex. 8'b11111111)
        wosi_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
        while (wosi_flush(&w_param) == -1)
            ;

        // issue a WOSI_WRITE to clear SSIF_RST_POS register
        data[0] = 0x00;
        wosi_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, data);
        while (wosi_flush(&w_param) == -1)
            ;

        /**
         *  MACHINE_CTRL,   // [31:28]  JOG_VEL_SCALE
         *                  // [27:24]  SPINDLE_JOINT_ID
         *                  // [23:16]  NUM_JOINTS
         *                  // [l5: 8]  JOG_SEL
         *                                  [15]: MPG(1), CONT(0)
         *                                  [14]: RESERVED
         *                                  [13:8]: J[5:0], EN(1), DISABLE(0)
         *                  // [ 7: 4]  ACCEL_STATE, the FSM state of S-CURVE-VELOCITY profile
         *                  // [ 3: 1]  MOTION_MODE:
         *                                  FREE    (0)
         *                                  TELEOP  (1)
         *                                  COORD   (2)
         *                                  HOMING  (4)
         *                  // [    0]  MACHINE_ON
         **/
        // configure NUM_JOINTS after all joint parameters are set
        immediate_data = (num_joints << 16); // assume motion_mode(0) and pid_enable(0)
        write_machine_param(MACHINE_CTRL, (uint32_t) immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;

        // JCMD_CTRL:
        //  [bit-0]: BasePeriod WOSI Registers Update (1)enable (0)disable
        //  [bit-1]: SSIF_EN, servo/stepper interface enable
        //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
        data[0] = 2; // SSIF_EN = 1
        wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);
        data[0] = 1; // RISC ON
        wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | OR32_CTRL), 1, data);
        while (wosi_flush(&w_param) == -1)
            ;
    }
    // end of [KINS]JOINTS

    /* allocate memory for stepgen_array[] */
    stepgen_array = hal_malloc(num_joints * sizeof(stepgen_t));
    if (stepgen_array == 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "STEPGEN: ERROR: hal_malloc() failed\n");
        return -1;
    }

    analog = hal_malloc(sizeof(analog_t));
    if (analog == 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "ANALOG: ERROR: hal_malloc() failed\n");
        return -1;
    }

    machine_control = hal_malloc(sizeof(machine_control_t));
    if (machine_control == 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "MACHINE_CONTROL: ERROR: hal_malloc() failed\n");
        return -1;
    }
    machine_control->prev_machine_ctrl = 0; // num_joints is not included
    machine_control->prev_gantry_ctrl = 0; // gantry_brake_gpio is not included

    /* export all the variables for each pulse generator */
    for (n = 0; n < num_joints; n++)
    {
        /* export all vars */
        retval = export_stepgen(n, &(stepgen_array[n]));
        if (retval != 0)
        {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "STEPGEN: ERROR: stepgen %d var export failed\n", n);
            return -1;
        }
    }

    retval = export_analog(analog); // up to 16-ch-adc-in
    if (retval != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "ANALOG: ERROR: analog var export failed\n");
        return -1;
    }

    /* put export machine_control below */
    retval = export_machine_control(machine_control);
    if (retval != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "MACHINE_CONTROL: ERROR:  machine_control var export failed\n");
        return -1;
    }


    // set mailbox callback function
    wosi_set_mbox_cb(&w_param, fetchmail);

    // set crc counter callback function
    wosi_set_crc_error_cb(&w_param, get_crc_error_counter);

    // set rt_cmd callback function
    wosi_set_rt_cmd_cb(&w_param, update_rt_cmd);

    initialized = 1;

    return 0;
}

static int load_parameters(FILE *fp)
{
    int ret;
    const char *s;
    char *t, *token;
    int gantry_polarity;
    uint8_t data[MAX_DSIZE];
    int32_t immediate_data;
    int n;
    int32_t jogp_en;
    int32_t jogn_en;
    int32_t jogp_id;
    int32_t jogn_id;

    // JOGP_EN/JOGN_EN
    ret = iniFindInt(fp, "JOGP_EN", "ARAIS", &jogp_en);
    if (ret != 0) // can not find jogp_en
        jogp_en = 255; // default to disable jogp_en

    ret = iniFindInt(fp, "JOGN_EN", "ARAIS", &jogn_en);
    if (ret != 0) // can not find jogn_en
        jogn_en = 255; // default to disable jogn_en

    immediate_data = (MAX_NUM_JOINT << 16) | (jogp_en << 8) | (jogn_en);
    write_machine_param(JOINT_JOGP_JOGN, immediate_data);
    while (wosi_flush(&w_param) == -1)
        ;


    // GANTRY_POLARITY
    ret = iniFindInt(fp, "GANTRY_POLARITY", "ARAIS", &gantry_polarity);
    if (ret != 0) // can not find GANTRY_POLARITY
        gantry_polarity = 0; // default to disable gantry_polarity
    // gantry_polarity should either be 0, 1, or -1
    assert(abs(gantry_polarity) < 2);
    write_machine_param(GANTRY_POLARITY, gantry_polarity);
    while (wosi_flush(&w_param) == -1)
        ;


    // ALARM_EN: let ALARM/ESTOP signal to stall SSIF and reset DOUT port
    int alarm_en;
    ret = iniFindInt(fp, "ALARM_EN", "ARAIS", &alarm_en);
    if (ret != 0) // can not find ALARM_EN
        alarm_en = 1; // default to enable alarm_en
    if (alarm_en == 1)
    {
        data[0] = GPIO_ALARM_EN;
    } else if (alarm_en == 0)
    {
        data[0] = 0;
    } else
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOSI: ERROR: unknown alarm_en value: %d\n", alarm_en);
        return -1;
    }
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
            (uint8_t) 1, data);
    // end of ALARM_EN



    // configure alarm output (the GPIO-DOUT PORT while E-Stop was pressed)
    // ALR_OUTPUT_0: "DOUT[31:0]  while E-Stop is pressed";
    // ALR_OUTPUT_1: "DOUT[63:32] while E-Stop is pressed";
    s = iniFind(fp, "ALR_OUTPUT_0", "ARAIS");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOSI: ERROR: no ALR_OUTPUT_0 defined\n");
        return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: ALR_OUTPUT_0=%s\n", s);
    write_machine_param(ALR_OUTPUT_0, (uint32_t) strtoul(s, NULL, 16));
    while (wosi_flush(&w_param) == -1)
        ;

    s = iniFind(fp, "ALR_OUTPUT_1", "ARAIS");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOSI: ERROR: no ALR_OUTPUT_1 defined\n");
        return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: ALR_OUTPUT_1=%s\n", s);
    write_machine_param(ALR_OUTPUT_1, (uint32_t) strtoul(s, NULL, 16));
    while (wosi_flush(&w_param) == -1)
        ;
    // end of ALR_OUTPUT_0, ALR_OUTPUT_1

    // set DAC_CTRL_REG for up to 4 channels (AR09)
    s = iniFind(fp, "DAC_CTRL_REG", "ARAIS");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: no DAC_CTRL_REG defined\n");
    } else
    {
        uint16_t sync_cmd;
        uint32_t dac_reg;
        char buf[80];

        for (n = 0, t = (char *) s;; n++, t = NULL)
        {
            token = strtok(t, ",");
            if (token == NULL)
                break;
            else
            {
                // write DAC control register to AD5422
                sync_cmd = SYNC_DAC | (n << 8) | (SYNC_DAC_CTRL); /* DAC, ID:n, ADDR: 0x55(Control Register) */
                dac_reg = (uint32_t) strtoul(token, NULL, 16);
                send_sync_cmd(sync_cmd, &dac_reg, 1);
            }
        }

        for (int i=0; i<n; i++)
        {   // read DAC_OFFSET_n from INI file
            snprintf(buf, sizeof(buf), "DAC_OFFSET_%d", i);
            s = iniFind(fp, buf, "ARAIS");
            if (s == NULL)
            {
                rtapi_print_msg(RTAPI_MSG_ERR,
                        "WOSI: ERROR: no %s defined\n", buf);
                return -1;
            }
            sync_cmd = SYNC_DAC | (i << 8) | (SYNC_DAC_OFFSET); /* DAC, ID:n, OFFSET: 0x99 */
            dac_reg = (uint32_t) strtoul(s, NULL, 10);  // DAC_OFFSET_n, base is 10
            send_sync_cmd(sync_cmd, &dac_reg, 1);

            // to force output DAC value as DAC_OFFSET at next update cycle
            analog->prev_out[i] = (double) (dac_reg + 1) ;
            *(analog->out[i]) = (double) dac_reg;
        }
    }
    // end of DAC_CTRL_REG

    /* test for dt: servo_period_ns */
    s = iniFind(fp, "SERVO_PERIOD", "EMCMOT");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "WOSI: ERROR: no [EMCMOT]SERVO_PERIOD defined\n");
        return -1;
    } else
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: [EMCMOT]SERVO_PERIOD=%s\n", s);
        servo_period_ns = atoi(s);
        dt = (double) servo_period_ns * 0.000000001;
        recip_dt = 1.0 / dt;
    }

    /* configure motion parameters */
    for (n = 0; n < num_joints; n++)
    {
        double max_vel, max_accel, pos_scale, abs_scale, max_jerk, max_ferror;
        double enc_scale;
        char section[20];

        /**
         *  if [JOINT_%d]INPUT_SCALE exists, then
         *      use JOINT_%d as section name
         *  else
         *      use AXIS_%d as section name
         **/
        sprintf(section, "JOINT_%d", n);
        if (iniFind(fp, "INPUT_SCALE", section) == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_DBG,
                    "cannot find [JOINT_%d]INPUT_SCALE; try [AXIS_%d]*\n",
                    n, n);
            sprintf(section, "AXIS_%d", n);
        }
        pos_scale = atof(iniFind(fp, "INPUT_SCALE", section));
        max_vel = atof(iniFind(fp, "MAX_VELOCITY", section));
        max_accel = atof(iniFind(fp, "MAX_ACCELERATION", section));
        max_jerk = atof(iniFind(fp, "MAX_JERK", section));
        assert(max_vel > 0);
        assert(max_accel > 0);
        assert(max_jerk > 0);
        assert(pos_scale != 0);

        rtapi_print_msg(RTAPI_MSG_INFO,
                "j[%d] max_vel(%f) max_accel(%f) max_jerk(%f) pos_scale(%f)\n",
                n, max_vel, max_accel, max_jerk, pos_scale);

        /* config encoder scale parameter */
        enc_scale = atof(iniFind(fp, "ENC_SCALE", section));
        assert(enc_scale != 0);
        immediate_data = (int32_t) (enc_scale * FIXED_POINT_SCALE);
        write_mot_param(n, (ENC_SCALE), immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;
        stepgen_array[n].enc_scale = enc_scale;

        /* unit_pulse_scale per servo_period */
        immediate_data = (int32_t) (FIXED_POINT_SCALE * pos_scale * dt);
        rtapi_print_msg(RTAPI_MSG_INFO, "j[%d] pos_scale(%f)=>(0x%08X)\n", n,
                pos_scale, immediate_data);
        write_mot_param(n, (SCALE), immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;
        stepgen_array[n].pos_scale = pos_scale;
        stepgen_array[n].scale_recip = 1.0 / pos_scale;

        // absolute pos_scale for MAX_VEL/ACCEL/JERK/FERROR calculation
        abs_scale = fabs(pos_scale);

        /* config MAX velocity */
        immediate_data = (uint32_t) ((max_vel * abs_scale * dt
                * FIXED_POINT_SCALE + 1));
        rtapi_print_msg(RTAPI_MSG_INFO, "j[%d] max_vel(%d) = %f*%f*%f*%f\n", n,
                immediate_data, max_vel, abs_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data > 0);
        write_mot_param(n, (MAX_VELOCITY), immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;
        stepgen_array[n].pulse_maxv = immediate_data;
        *(stepgen_array[n].jog_vel) = max_vel * 0.1; // default to 10% of max_vel

        /* config acceleration */
        immediate_data = (uint32_t) ((max_accel * abs_scale * dt
                * FIXED_POINT_SCALE * dt + 1));
        rtapi_print_msg(RTAPI_MSG_INFO,
                "j[%d] max_accel(%d) = %f*%f*(%f^2)*(%f)\n", n, immediate_data,
                max_accel, abs_scale, dt, FIXED_POINT_SCALE);
        assert(immediate_data > 0);
        write_mot_param(n, (MAX_ACCEL), immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;
        stepgen_array[n].pulse_maxa = immediate_data;

        /* config max jerk */
        immediate_data = (uint32_t) (max_jerk * abs_scale * FIXED_POINT_SCALE
                * dt * dt * dt + 1);
        rtapi_print_msg(RTAPI_MSG_INFO,
                "j[%d] max_jerk(%d) = (%f * %f * %f * %f^3)))\n", n,
                immediate_data, FIXED_POINT_SCALE, max_jerk, abs_scale, dt);
        assert(immediate_data != 0);
        write_mot_param(n, (MAX_JERK), immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;
        stepgen_array[n].pulse_maxj = immediate_data;

        /* config max following error */
        // following error send with unit pulse
        max_ferror = atof(iniFind(fp, "FERROR", section));
        immediate_data = (uint32_t) (max_ferror * abs_scale);
        rtapi_print_msg(RTAPI_MSG_INFO, "max ferror(%d)\n", immediate_data);
        write_mot_param(n, (MAXFOLLWING_ERR), immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;

        int ch;
        ch = atoi(iniFind(fp, "OUT_CH", section));

        s = iniFind(fp, "OUT_DEV", section);
        if (s == NULL) {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOSI.ERROR: no OUT_DEV defined for %s\n", section);
            return -1;
        }

        if (toupper(s[0]) == 'A') {
            immediate_data = (OUT_TYPE_AB_PHASE << 28) | (ch << 24);
            stepgen_array[n].out_type = OUT_TYPE_AB_PHASE;
        } else if (toupper(s[0]) == 'S') {
            immediate_data = (OUT_TYPE_STEP_DIR << 28) | (ch << 24);
            stepgen_array[n].out_type = OUT_TYPE_STEP_DIR;
        } else if (toupper(s[0]) == 'P') {
            immediate_data = (OUT_TYPE_PWM << 28) | (ch << 24);
            stepgen_array[n].out_type = OUT_TYPE_PWM;
        } else if (toupper(s[0]) == 'D') {
            immediate_data = (OUT_TYPE_DAC << 28) | (ch << 24);
            stepgen_array[n].out_type = OUT_TYPE_DAC;
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOSI.ERROR: unsupported OUT_DEV(%s) for %s\n", s, section);
            return -1;
        }
        write_mot_param(n, (OUT_DEV), immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;

        if ((toupper(s[0]) == 'P') || (toupper(s[0]) == 'D'))
        {
            int omin, omax;
            omin = atoi(iniFind(fp, "OUT_MIN", section));
            omax = atoi(iniFind(fp, "OUT_MAX", section));
            if (omin < -32768) { omin = -32768; } //!< omin: int16_t
            if (omin > 32767) { omin = 32767; }
            if (omax < -32768) { omax = -32768; } //!< omax: int16_t
            if (omax > 32767) { omax = 32767; }
            immediate_data = ((omin) << 16) | (omax); //!< {MIN(int16_t), MAX(int16_t)}
            write_mot_param(n, (OUT_RANGE), immediate_data);
            while (wosi_flush(&w_param) == -1)
                ;

            immediate_data = atoi(iniFind(fp, "OUT_MAX_IPULSE", section));
            write_mot_param(n, (OUT_MAX_IPULSE), immediate_data);
            while (wosi_flush(&w_param) == -1)
                ;

            immediate_data = atoi(iniFind(fp, "OUT_SCALE", section));
            write_mot_param(n, (OUT_SCALE), immediate_data);
            while (wosi_flush(&w_param) == -1)
                ;

            immediate_data = atoi(iniFind(fp, "OUT_SCALE_RECIP", section));
            write_mot_param(n, (OUT_SCALE_RECIP), immediate_data);
            while (wosi_flush(&w_param) == -1)
                ;

            immediate_data = atoi(iniFind(fp, "OUT_OFFSET", section));
            write_mot_param(n, (OUT_OFFSET), immediate_data);
            while (wosi_flush(&w_param) == -1)
                ;
        }

        // begin of enc_type
        s = iniFind(fp, "ENC_TYPE", section);
        if (s == NULL) {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOSI.ERROR: no ENC_TYPE defined for %s\n", section);
            return -1;
        }
        if (toupper(s[0]) == 'L') {
            // ENC_TYPE(00): fake ENCODER counts (loop PULSE_CMD to ENCODER)
            stepgen_array[n].enc_type = ENC_TYPE_LOOPBACK;
        } else if (toupper(s[0]) == 'A') {
            // ENC_TYPE(10): real ENCODER counts, AB-Phase
            stepgen_array[n].enc_type = ENC_TYPE_AB_PHASE;
        } else if (toupper(s[0]) == 'S') {
            // ENC_TYPE(11): real ENCODER counts, STEP-DIR
            stepgen_array[n].enc_type = ENC_TYPE_STEP_DIR;
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "STEPGEN: ERROR: bad enc_type '%s' for joint_%i (must be 'a', 's', or 'l')\n",
                    s, n);
            return -1;
        }
        // end of enc_type

        // "set LSP_ID/LSN_ID for up to 8 channels"
        // LSP_ID: gpio pin id for limit-switch-positive(lsp)
        s = iniFind(fp, "LSP_ID", section);
        if (s == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOSI: ERROR: no LSP_ID defined for JOINT_%d\n", n);
            return -1;
        }
        lsp_id[n] = atoi(s);

        s = iniFind(fp, "LSN_ID", section);
        if (s == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOSI: ERROR: no LSN_ID defined for JOINT_%d\n", n);
            return -1;
        }
        lsn_id[n] = atoi(s);
        immediate_data = (n << 16) | (lsp_id[n] << 8) | (lsn_id[n]);
        write_machine_param(JOINT_LSP_LSN, immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;
        // end of LSP_ID and LSN_ID


        // "set JOG JOGP_ID/JOGN_ID for up to 8 channels"
        s = iniFind(fp, "JOGP_ID", section);
        if (s == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOSI: ERROR: no JOGP_ID defined for JOINT_%d\n", n);
            return -1;
        }
        jogp_id = atoi(s);

        s = iniFind(fp, "JOGN_ID", section);
        if (s == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOSI: ERROR: no JOGN_ID defined for JOINT_%d\n", n);
            return -1;
        }
        jogn_id = atoi(s);
        immediate_data = (n << 16) | (jogp_id << 8) | (jogn_id);
        write_machine_param(JOINT_JOGP_JOGN, immediate_data);
        while (wosi_flush(&w_param) == -1)
            ;
        // end of JOGP_ID and JOGN_ID

        // "set ALR_ID (gpio id of alarm signal) for up to 8 channels"
        s = iniFind(fp, "ALR_ID", section);
        if (s == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "WOSI: ERROR: no ALR_ID defined for JOINT_%d\n", n);
            return -1;
        }
        stepgen_array[n].alr_id = atoi(s);
        // end of ALR_ID (ALR_EN_BITS)

    } // for-loop for num_joints

    /* configure out_type */
    data[0] = 0; // SSIF_PULSE_TYPE j3 ~ j0
    data[1] = 0; // SSIF_PULSE_TYPE J7 ~ j4
    for (n = 0; n < num_joints; n++) {
        if (n < 4) {
            data[0] |= (stepgen_array[n].out_type << (n * 2));
        } else {
            data[1] |= (stepgen_array[n].out_type << ((n - 4) * 2));
        }
    } // for-loop for out_type
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (SSIF_BASE | SSIF_PULSE_TYPE),
            (uint8_t) 2, data); // for FPGA
    // end of out_type[]

    /* configure enc_type */
    data[0] = 0; // SSIF_ENC_TYPE j3 ~ j0
    data[1] = 0; // SSIF_ENC_TYPE J7 ~ j4
    for (n = 0; n < num_joints; n++) {
        if (n < 4) {
            data[0] |= (stepgen_array[n].enc_type << (n * 2));
        } else {
            data[1] |= (stepgen_array[n].enc_type << ((n - 4) * 2));
        }
    } // for-loop for enc_type
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (SSIF_BASE | SSIF_ENC_TYPE),
            (uint8_t) 2, data); // for FPGA
    // end of enc_type[]

    // "encoder polarity (POSITIVE(p) or NEGATIVE(n)) for up to 8 channels"
    rtapi_print_msg(RTAPI_MSG_INFO, "TODO: REMOVE: WOSI: SSIF_ENC_POL\n");
    data[0] = 0; // SSIF_ENC_POL j7 ~ j0
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (SSIF_BASE | SSIF_ENC_POL),
            (uint8_t) 1, data); // for FPGA
    // end of enc_scale[]

    // begin of ALR_EN_BITS
    immediate_data = alarm_en; // reset ALR_EN_BITS to ALARM_EN/ESTOP_EN/DIN[0] bit
    for (n = 0; n < num_joints; n++) {
        char alr;
        alr = stepgen_array[n].alr_id;
        if (alr != 255)
        {
            immediate_data |= (1 << alr);
            assert(alr < 32); // ALARM-ID maps to GPIO[31:1]
            assert(alr > 0);
        }
    }
    write_machine_param(ALR_EN_BITS, immediate_data);
    while (wosi_flush(&w_param) == -1)
        ;
    // end of ALR_EN_BITS

    // config PID parameter
    for (n = 0; n < MAX_NUM_JOINT; n++)
    {
        char name[20];
        int i;

        sprintf(name, "J%d_PID", n);
        s = iniFind(fp, name, "ARAIS");
        if (s == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: no value for %s\n", name);
            continue;
        }
        rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: %s=%s\n", name, s);
        rtapi_print_msg(RTAPI_MSG_INFO,
                "#   0:P 1:I 2:D 3:FF0 4:FF1 5:FF2 6:DB 7:BI 8:M_ER 9:M_EI 10:M_ED 11:MCD 12:MCDD 13:MO\n");
        // all gains (P, I, D, FF0, FF1, FF2) range from 0(0%) to 65535(100%)
        // all the others units are '1 pulse'
        for (i = 0, t = (char *) s;; i++, t = NULL)
        {
            token = strtok(t, ",");
            if (token == NULL)
                break;
            else
            {
                immediate_data = atoi(token);
                // P_GAIN: the mot_param index for P_GAIN value
                write_mot_param(n, (P_GAIN + i), immediate_data);
                while (wosi_flush(&w_param) == -1)
                    ;
                rtapi_print_msg(RTAPI_MSG_INFO, "pid(%d) = %s (%d)\n", i, token,
                        immediate_data);
            }
        }
    }

    // begin of AHC parameters
    // AHC_CH: analog input channel for auto height control
    s = iniFind(fp, "AHC_CH", "ARAIS");
    if (s == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: no AHC_CH defined\n");
    } else
    {
        double pos_scale;
        int ahc_joint;
        // set AHC related parameters when AHC_CH is defined
        rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: AHC_CH=%s\n", s);
        write_machine_param(AHC_ANALOG_CH, atoi(s));
        while (wosi_flush(&w_param) == -1)
            ;
        // end of AHC_CH

        // AHC_JNT: the joint that is controlled by AHC
        s = iniFind(fp, "AHC_JNT", "ARAIS");
        if (s == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "WOSI: ERROR: no AHC_JNT defined for AHC_CH\n");
            return -1;
        } else
        {
            rtapi_print_msg(RTAPI_MSG_INFO, "WOSI: AHC_JNT=%s\n", s);
            ahc_joint = atoi(s);
            write_machine_param(AHC_JNT, ahc_joint);
            while (wosi_flush(&w_param) == -1)
                ;
        }
        // end of AHC_JNT

        // AHC_POLARITY: the auto height control polarity between command and error
        pos_scale = stepgen_array[ahc_joint].pos_scale;
        assert (pos_scale > 0); // must be a valid joint
        s = iniFind(fp, "AHC_POLARITY", "ARAIS");
        if (s == NULL)
        {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "ERROR: no AHC_POLARITY defined for AHC_CH\n");
            return -1;
        } else
        {
            rtapi_print_msg(RTAPI_MSG_INFO, "AHC_POLARITY=%s\n", s);
            if (strcmp(s, "POSITIVE") == 0)
            {
                if (pos_scale > 0)
                {
                    write_machine_param(AHC_POLARITY, 1);
                } else
                {
                    write_machine_param(AHC_POLARITY, -1);
                }
            } else if (strcmp(s, "NEGATIVE") == 0)
            {
                if (pos_scale >= 0)
                {
                    write_machine_param(AHC_POLARITY, -1);
                } else
                {
                    write_machine_param(AHC_POLARITY, 1);
                }
            } else
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: invalid AHC_POLARITY(%s)\n", s);
                return -1;
            }
            while (wosi_flush(&w_param) == -1);
        }
    }
    // end of AHC parameters
    return 0;
}

static int read_ini(char *inifile)
{
    FILE *fp;

    fp = fopen(inifile, "r");
    if (fp != NULL)
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "read_ini(): inifile(%s) fp(%p)\n",
                inifile, fp);
        if (initialized == 0)
        {
            if (system_initialize(fp) != 0)
            {
                fclose (fp);
                return -1;
            }
        }

        if (load_parameters(fp) != 0)
        {
            fclose (fp);
            return -1;
        }
        fclose (fp);
    } else
    {
        ERRP("cant open inifile '%s'\n", inifile);
        return -1;
    }
    return 0;
}

/***********************************************************************
 *                       INIT AND EXIT CODE                            *
 ***********************************************************************/

int wosi_driver_init(int hal_comp_id, char *inifile)
{
    comp_id = hal_comp_id;

    if (inifile)
    {
        if (read_ini(inifile))
            return -1;
    } else
    {
        ERRP(
                "ERROR: no inifile - either use '--ini inifile' or set INI_FILE_NAME environment variable\n");
        return -1;
    }
    strcpy (reload_ini_file, inifile);  // save the inifile name for reloading parameters


#if (TRACE!=0)
    /* initialize file handle for logging wosi steps */
    dptrace = fopen("mk-wosi.log", "w");

    /* prepare header for gnuplot */
    DPS("#%10s", "dt");
    for (int n = 0; n < num_joints; n++)
    {
        DPS("      int_pcmd[%d]", n);
        DPS("     prev_pcmd[%d]", n);
        DPS("        pos_fb[%d]", n);
        DPS("  risc_pos_cmd[%d]", n);
    }
    DPS("\n");
#endif

    rtapi_print_msg(RTAPI_MSG_INFO,
            "STEPGEN: installed %d step pulse generators\n", num_joints);

    return 0;
}

void wosi_dirver_exit(void)
{
#if (TRACE!=0)
    fclose(dptrace);
#endif

}

/***********************************************************************
 *              REALTIME STEP PULSE GENERATION FUNCTIONS               *
 ***********************************************************************/

static void update_rt_cmd(void)
{
    uint8_t data[MAX_DSIZE]; // data[]: for wosi_cmd()
    int32_t immediate_data = 0;
    if (machine_control)
    {
        if (*machine_control->rt_abort == 1)
        {
            immediate_data = RT_ABORT;
            memcpy(data, &immediate_data, sizeof(uint32_t));
            rt_wosi_cmd(&w_param, WB_WR_CMD,
                    (uint16_t) (JCMD_BASE | OR32_RT_CMD), sizeof(uint32_t),
                    data);
            rt_wosi_flush(&w_param);
        }
    }
}

void wosi_receive()
{
    wosi_update(&w_param); // link to wosi_recv()
}

void wosi_transceive(const tick_jcmd_t *tick_jcmd)
{
    stepgen_t *stepgen;
    int n, i, j;

    uint16_t sync_cmd;
    int32_t wosi_pos_cmd, integer_pos_cmd;
    uint8_t data[MAX_DSIZE]; // data[]: for wosi_cmd()
    int homing;
    uint32_t sync_out_data;
    uint32_t tmp;
    int32_t immediate_data = 0;
#if (TRACE!=0)
    static uint32_t _dt = 0;
#endif

#ifdef PRINT_BANDWIDTH
    wosi_status(&w_param); // print bandwidth utilization
#endif

    /* begin set analog trigger level*/
    if (*machine_control->analog_ref_level
            != machine_control->prev_analog_ref_level)
    {
        write_machine_param(ANALOG_REF_LEVEL,
                (uint32_t) (*(machine_control->analog_ref_level)));
        fprintf(stderr, "wosi.c: analog_ref_level(%d) \n",
                (uint32_t) *machine_control->analog_ref_level);
    }
    machine_control->prev_analog_ref_level = *machine_control->analog_ref_level;
    /* end: */

    /* begin motion_mode */
    /**
     *  MACHINE_CTRL,   // [31:28]  JOG_VEL_SCALE
     *                  // [27:24]  SPINDLE_JOINT_ID
     *                  // [23:16]  NUM_JOINTS
     *                  // [l5: 8]  JOG_SEL
     *                                  [15]: MPG(1), CONT(0)
     *                                  [14]: RESERVED
     *                                  [13:8]: J[5:0], EN(1)
     *                  // [ 7: 4]  ACCEL_STATE
     *                  // [ 3: 1]  MOTION_MODE:
     *                                  FREE    (0)
     *                                  TELEOP  (1)
     *                                  COORD   (2)
     *                                  HOMING  (4)
     *                  // [    0]  MACHINE_ON
     **/

    homing = 0;
    for (n = 0; n < num_joints; n++)
    {
        homing |= *(stepgen_array[n].homing);
    }

    assert(abs(*machine_control->jog_vel_scale) < 8);
    tmp = (*machine_control->jog_vel_scale << 28)
            | (*machine_control->jog_sel << 8)
            | (*machine_control->accel_state << 4) | (homing << 3)
            | (*machine_control->coord_mode << 2)
            | (*machine_control->teleop_mode << 1)
            | (*machine_control->machine_on);
    if (tmp != machine_control->prev_machine_ctrl)
    {
        machine_control->prev_machine_ctrl = tmp;
        immediate_data = (num_joints << 16) | tmp;
        immediate_data = ((*machine_control->spindle_joint_id) << 24)
                | immediate_data;
        write_machine_param(MACHINE_CTRL, (uint32_t) immediate_data);
    }

    tmp = (*machine_control->gantry_en << 31);
    if (tmp != machine_control->prev_gantry_ctrl)
    {
        if (*machine_control->machine_on)
        { // only Lock/Release gantry brake after servo-on to prevent dropping
            immediate_data = tmp;
            write_machine_param(GANTRY_CTRL, (uint32_t) immediate_data);
            machine_control->prev_gantry_ctrl = tmp;
        }
    }
    /* end: */

    if (tick_jcmd->cmd & (1 << TICK_UPDATE_POS_ACK))
    {
        int32_t dbuf[2];
        dbuf[0] = RCMD_UPDATE_POS_ACK;
        dbuf[1] = *machine_control->rcmd_seq_num_req;
        send_sync_cmd((SYNC_USB_CMD | RISC_CMD_TYPE), (uint32_t *) dbuf, 2);
        // Reset update_pos_req after sending a RCMD_UPDATE_POS_ACK packet
        *machine_control->update_pos_req = 0;

        // reset prev_pos_cmd and related variables
        for (n = 0; n < num_joints; n++)
        {
            stepgen = &(stepgen_array[n]);
            stepgen->prev_pos_cmd = tick_jcmd->pos_cmd[n];
            stepgen->rawcount = stepgen->prev_pos_cmd * FIXED_POINT_SCALE
                                * stepgen->pos_scale;
        }
    }

    /* begin: handle AHC state, AHC level */
    if (*machine_control->ahc_level != machine_control->prev_ahc_level)
    {
        immediate_data = (uint32_t) (*(machine_control->ahc_level));
        write_machine_param(AHC_LEVEL, immediate_data); // 32.0
        machine_control->prev_ahc_level = *(machine_control->ahc_level);
        rtapi_print_msg(RTAPI_MSG_INFO, "wosi.c: ahc_level(%d)\n",
                (uint32_t) *(machine_control->ahc_level));
        machine_control->prev_ahc_level = *(machine_control->ahc_level);
    }

    // motion_s3 is set by M103 Q_word
    if (*machine_control->motion_s3)
    { // AHC has to synchronize with accel_state.S3
        if ((*machine_control->ahc_state)
                && (machine_control->prev_accel_state != *machine_control->accel_state))
        { // to synchronize AHC with motion.S3
            if ((*machine_control->accel_state == 3)
                    && (*machine_control->current_vel != 0))
            { // to enable AHC only when accel_state is
                write_machine_param(AHC_STATE, 1);
            } else
            { // to disable AHC
                write_machine_param(AHC_STATE, 0);
            }
            machine_control->prev_accel_state = *machine_control->accel_state;
        }
        if ((*machine_control->ahc_state == 0)
                && (machine_control->prev_accel_state
                        == *machine_control->accel_state))
        { // in motion stable, but we suddenly closed ahc
            immediate_data = (*(machine_control->ahc_state));
            write_machine_param(AHC_STATE, immediate_data);
            machine_control->prev_accel_state = 255;
        }
    } else
    { // AHC is judged by ahc_state only
        if ((*machine_control->ahc_state) != (machine_control->prev_ahc_state))
        {
            immediate_data = (*(machine_control->ahc_state));
            write_machine_param(AHC_STATE, immediate_data);
            machine_control->prev_ahc_state = *machine_control->ahc_state;
        }
    }

    /* end: handle AHC state, AHC level */

    /* begin: setup sync wait timeout */
    if (*machine_control->timeout != machine_control->prev_timeout)
    {
        immediate_data = (uint32_t) (*(machine_control->timeout)
                / (servo_period_ns * 0.000000001)); // ?? sec timeout / one tick interval
        // immediate_data = 1000; // ticks about 1000 * 0.00065536 sec
        // transmit immediate data
        fprintf(stderr, "wosi.c: setup wait timeout(%u) \n", immediate_data);
        write_machine_param(WAIT_TIMEOUT, immediate_data);
        machine_control->prev_timeout = *machine_control->timeout;
    }
    /* end: setup sync wait timeout */

    /* begin: process motion synchronized input */
    /* for M200, ARTEK's SYNC_INPUT */
    /* for G33 and G33.1, to synchronize spindle index */
    if (*(machine_control->sync_in_trigger) != 0)
    {
        assert(*(machine_control->sync_in_index) >= 0);
        assert(*(machine_control->sync_in_index) < GPIO_IN_NUM);
        // begin: trigger sync in and wait timeout
        sync_cmd = SYNC_DIN
                | PACK_IO_ID((uint32_t)*(machine_control->sync_in_index))
                | DI_TYPE((uint32_t)*(machine_control->wait_type));
        wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), (uint8_t *) &sync_cmd);
        // end: trigger sync in and wait timeout
        *(machine_control->sync_in_trigger) = 0;
    }
    /* end: process motion synchronized input */

    /* begin: process motion synchronized output */
    for (j=0; j<(GPIO_OUT_NUM >> 5); j++)
    {
        sync_out_data = 0;
        for (i=0; i<32; i++)
        {
            if (((machine_control->prev_out[j] >> i) & 0x01)
                    != (*(machine_control->out[j*32+i])))
            {
                // write a wosi frame for sync output into command FIFO
                sync_cmd = SYNC_DOUT | PACK_IO_ID(j*32+i)
                                     | DO_VAL(*(machine_control->out[j*32+i]));
                memcpy(data, &sync_cmd, sizeof(uint16_t));
                wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
                        sizeof(uint16_t), data);
            }
            sync_out_data |= ((*(machine_control->out[j*32+i])) << i);
        }
        machine_control->prev_out[j] = sync_out_data;
    }
    /* end: process motion synchronized output */

    /* DAC: fixed as 4 dac channels */
    for (i = 0; i < 4; i++)
    {
        if (analog->prev_out[i] != *(analog->out[i]))
        {
            uint32_t tmp_a;
            analog->prev_out[i] = *(analog->out[i]);
            tmp_a = analog->prev_out[i];
            sync_cmd = SYNC_DAC | (i << 8) | (SYNC_DAC_DATA); /* DAC, ID:i, ADDR: 0x01 */
            send_sync_cmd(sync_cmd, &tmp_a, 1);
            rtapi_print_msg(RTAPI_MSG_DBG, "analog->out[%d]=%f\n", i, *(analog->out[i]));
        }
    }

#if (TRACE!=0)
    _dt++;
    if (tick_jcmd->cmd & (1 << TICK_AMP_ENABLE))
    {
        DPS("%11u", _dt); // %11u: '#' + %10s
    }
#endif

    // in[0] == 1 (ESTOP released)
    // in[0] == 0 (ESTOP pressed)
    if (*(machine_control->in[0]) == 0)
    {
        // force updating prev_out and ignore "out[] for RISC" if ESTOP is pressed
        // The dout0 is forced by ALARM_OUT when ESTOP is pressed
        machine_control->prev_out[0] = *machine_control->dout0;
    }

    if ((*machine_control->pso_req == 1))
    { // PSO
        int32_t dbuf[3];
        double delta_pos;
        dbuf[0] = RCMD_PSO;
        delta_pos = *machine_control->pso_pos
                - stepgen_array[*machine_control->pso_joint].prev_pos_cmd; // delta PSO position, unit: pulse
        dbuf[1] =
                (delta_pos * (stepgen_array[*machine_control->pso_joint].pos_scale));
        dbuf[2] = ((*machine_control->pso_ticks & 0xFFFF) << 16) | // dbuf[2][31:16]
                (0x1 << 15) | // force pso_en to 1
                ((*machine_control->pso_mode & 0x3) << 12) | // dbuf[2][15:12]
                ((*machine_control->pso_joint & 0xF) << 8); // dbuf[2][11: 8]
        send_sync_cmd((SYNC_USB_CMD | RISC_CMD_TYPE), (uint32_t *) dbuf, 3);
//    	printf("wosi.c: pso_req(%d) pso_ticks(%d) pso_mode(%d) pso_joint(%d) \n",
//    			*machine_control->pso_req, *machine_control->pso_ticks,
//    			*machine_control->pso_mode, *machine_control->pso_joint);
//    	printf("wosi.c: pso_pos(%f) dbuf1[%d] delta_pos(%f) prev_pos(%f) pos_cmd(%f)\n",
//    			*machine_control->pso_pos, dbuf[1], delta_pos,
//    			stepgen_array[*machine_control->pso_joint].prev_pos_cmd,
//    			*stepgen_array[*machine_control->pso_joint].pos_cmd);
    }

    i = 0;
    for (n = 0; n < num_joints; n++)
    {
        stepgen = &(stepgen_array[n]);

        *(stepgen->rawcount32) = (int32_t) (stepgen->rawcount >> FRACTION_BITS);

        if (*stepgen->uu_per_rev != stepgen->prev_uu_per_rev)
        { // pass compensation scale
            /* set spindle_sync_motion scale parameter */
            immediate_data =
                    (int32_t) (*stepgen->uu_per_rev * FIXED_POINT_SCALE
                            * (stepgen->pos_scale)
                            / stepgen_array[(*machine_control->spindle_joint_id)].pos_scale);
            write_mot_param(n, (SSYNC_SCALE), immediate_data); // format: 16.16
            stepgen->prev_uu_per_rev = *stepgen->uu_per_rev;
        }

        if (*stepgen->jog_vel != stepgen->prev_jog_vel)
        {
            int32_t dbuf[3];
            dbuf[0] = RCMD_REMOTE_JOG;
            // jog-switch-positive
            dbuf[1] = n & 0xF;
            dbuf[2] = *stepgen->jog_vel * stepgen->pos_scale * dt
                    * FIXED_POINT_SCALE; // fixed-point 16.16
            send_sync_cmd((SYNC_USB_CMD | RISC_CMD_TYPE), (uint32_t *) dbuf, 3);
            stepgen->prev_jog_vel = *stepgen->jog_vel;
        }

        if (*stepgen->bypass_lsp != stepgen->prev_bypass_lsp)
        { // bypass limit-switch-positive
            int lsp, lsn;

            if (*stepgen->bypass_lsp)
            { /* 0 -> 1 */
                lsp = 255;
            } else
            { /* 1 -> 0 */
                lsp = lsp_id[n];
            }
            lsn = lsn_id[n];
            if (stepgen->pos_scale >= 0)
            {
                immediate_data = (n << 16) | (lsp << 8) | (lsn);
            } else
            {
                immediate_data = (n << 16) | (lsn << 8) | (lsp);
            }
            write_machine_param(JOINT_LSP_LSN, immediate_data);
            while (wosi_flush(&w_param) == -1)
                ;
            stepgen->prev_bypass_lsp = *stepgen->bypass_lsp;
        }

        if (*stepgen->bypass_lsn != stepgen->prev_bypass_lsn)
        { // bypass limit-switch-negative
            int lsp, lsn;

            if (*stepgen->bypass_lsn)
            { /* 0 -> 1 */
                lsn = 255;
            } else
            { /* 1 -> 0 */
                lsn = lsn_id[n];
            }
            lsp = lsp_id[n];
            if (stepgen->pos_scale >= 0)
            {
                immediate_data = (n << 16) | (lsp << 8) | (lsn);
            } else
            {
                immediate_data = (n << 16) | (lsn << 8) | (lsp);
            }
            write_machine_param(JOINT_LSP_LSN, immediate_data);
            while (wosi_flush(&w_param) == -1)
                ;
            stepgen->prev_bypass_lsn = *stepgen->bypass_lsn;
        }

        *(stepgen->pos_fb) = (*stepgen->enc_pos) * stepgen->scale_recip;
        *(stepgen->risc_pos_cmd) = (*stepgen->cmd_pos) * stepgen->scale_recip;
        *(stepgen->ferror) = *(stepgen->risc_pos_cmd) - *(stepgen->pos_fb);

        // update velocity-feedback based on RISC-reported encoder-velocity
        // enc_vel_p is in 16.16 pulse per servo-period format
        *(stepgen->vel_fb) = *(stepgen->enc_vel_p) * stepgen->scale_recip
                * recip_dt * FP_SCALE_RECIP;

        if ((tick_jcmd->cmd & (1 << TICK_AMP_ENABLE)) == 0)
        {
            /* AXIS not PWR-ON */
            /* reload_params only when machine-is-off */
            if (*machine_control->reload_params)
            {
                FILE *fp;

                fp = fopen(reload_ini_file, "r");
                assert (fp != NULL);
                rtapi_print_msg(RTAPI_MSG_INFO, "reload parameters: inifile(%s) fp(%p)\n", reload_ini_file, fp);
                if (load_parameters(fp) != 0)
                    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR reload parameters: inifile(%s) fp(%p)\n", reload_ini_file, fp);
                fclose(fp);
                printf ("TODO: REMOVE this msg: reload_parameters ... done\n");
                *machine_control->reload_params = 0;
            }

            /* to prevent position drift while toggeling "PWR-ON" switch */
            (stepgen->prev_pos_cmd) = tick_jcmd->pos_cmd[n];
            stepgen->rawcount = stepgen->prev_pos_cmd * FIXED_POINT_SCALE
                    * stepgen->pos_scale;
        }

        if (tick_jcmd->risc_probe_vel[n] == 0)
            stepgen->risc_probing = 0;

        if ((*stepgen->homing) && (tick_jcmd->risc_probe_vel[n] != 0)
                && (stepgen->risc_probing == 0)
                && (*machine_control->rcmd_state == RCMD_IDLE))
        {
            // do RISC_PROBE
            int32_t dbuf[4];
            dbuf[0] = RCMD_RISC_PROBE;
            dbuf[1] = n  // joint_num
                    | (*stepgen->risc_probe_type << 8)
                    | (*stepgen->risc_probe_pin << 16);
            dbuf[2] = tick_jcmd->risc_probe_vel[n] * stepgen->pos_scale * dt
                      *FIXED_POINT_SCALE; // fixed-point 16.16
            dbuf[3] = *stepgen->risc_probe_dist * stepgen->pos_scale; // distance in pulse
            send_sync_cmd((SYNC_USB_CMD | RISC_CMD_TYPE), (uint32_t *) dbuf, 4);
            assert(*stepgen->risc_probe_pin < 64);
            assert(dbuf[2] != 0);
            stepgen->risc_probing = 1;
        }

        if ((*machine_control->probing != machine_control->prev_probing))
        {
            // HOST_PROBING G38.X
            int32_t dbuf[4];
            dbuf[0] = RCMD_HOST_PROBE;
            dbuf[1] = (*machine_control->trigger_level & 0x3FFF)
                    | ((*machine_control->trigger_din & 0xFF) << 14)
                    | ((*machine_control->trigger_ain & 0x3F) << 22)
                    | ((*machine_control->trigger_type & 0x3) << 28)
                    | ((*machine_control->trigger_cond & 0x1) << 30)
                    | ((*machine_control->probing & 0x1) << 31);
            // distance in pulse
//            printf("level(%d) din(%d) ain(%d)\n type(%d) cond(%d) probing(%d)\n",
//            		*machine_control->trigger_level, *machine_control->trigger_din,
//            		*machine_control->trigger_ain, *machine_control->trigger_type,
//            		*machine_control->trigger_cond, *machine_control->probing);
            send_sync_cmd((SYNC_USB_CMD | RISC_CMD_TYPE), (uint32_t *) dbuf, 2);
            machine_control->prev_probing = *machine_control->probing;
        }

        /* pulse_type is either A(AB-PHASE) or S(STEP-DIR) */
        int32_t pulse_accel;
        int32_t pulse_jerk;

        if (tick_jcmd->cmd & (1 << TICK_UPDATE_POS_ACK))
        {
            (stepgen->prev_pos_cmd) = tick_jcmd->pos_cmd[n];
            stepgen->rawcount = stepgen->prev_pos_cmd * FIXED_POINT_SCALE
                    * stepgen->pos_scale;
        }
        stepgen->vel_cmd_t = (tick_jcmd->pos_cmd[n] - (stepgen->prev_pos_cmd));
        *stepgen->vel_cmd = stepgen->vel_cmd_t * recip_dt;

        integer_pos_cmd = (int32_t) (stepgen->vel_cmd_t * (stepgen->pos_scale)
                * FIXED_POINT_SCALE);

        // integer_pos_cmd is indeed pulse_vel (velocity in pulse)
        if (abs(integer_pos_cmd) > stepgen->pulse_maxv)
        {
            pulse_accel = integer_pos_cmd - stepgen->pulse_vel;
            pulse_jerk = pulse_accel - stepgen->pulse_accel;
            rtapi_print_msg(RTAPI_MSG_ERR, "j[%d]: assert(integer_pos_cmd(%d) > pulse_maxv(%d))\n", n, integer_pos_cmd, stepgen->pulse_maxv);
            rtapi_print_msg(RTAPI_MSG_ERR, "j[%d], pos_fb(%f) ferror(%f)\n", n, (*stepgen->pos_fb), (*stepgen->ferror));
            rtapi_print_msg(RTAPI_MSG_ERR, "j[%d], vel_cmd(%f,unit/s) pos_cmd(%f) prev_pos_cmd(%f)\n", n,
                    *stepgen->vel_cmd, tick_jcmd->pos_cmd[n],
                    (stepgen->prev_pos_cmd));
            rtapi_print_msg(RTAPI_MSG_ERR, "j[%d], pulse_vel(%d), pulse_accel(%d), pulse_jerk(%d)\n", n,
                    integer_pos_cmd, pulse_accel, pulse_jerk);
            rtapi_print_msg(RTAPI_MSG_ERR, "j[%d], PREV pulse_vel(%d), pulse_accel(%d), pulse_jerk(%d)\n",
                    n, stepgen->pulse_vel, stepgen->pulse_accel,
                    stepgen->pulse_jerk);
            rtapi_print_msg(RTAPI_MSG_ERR, "j[%d], pulse_maxv(%d), pulse_maxa(%d), pulse_maxj(%d)\n", n,
                    stepgen->pulse_maxv, stepgen->pulse_maxa,
                    stepgen->pulse_maxj);
        }
        assert(abs(integer_pos_cmd) <= stepgen->pulse_maxv);
        pulse_accel = integer_pos_cmd - stepgen->pulse_vel;
        pulse_jerk = pulse_accel - stepgen->pulse_accel;
        stepgen->pulse_vel = integer_pos_cmd;
        stepgen->pulse_accel = pulse_accel;
        stepgen->pulse_jerk = pulse_jerk;

        {
            /* extract integer part of command */
            wosi_pos_cmd = abs(integer_pos_cmd) >> FRACTION_BITS;

            if (wosi_pos_cmd >= 8192)
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "j(%d) pos_cmd(%f) prev_pos_cmd(%f) vel_cmd_t(%f)\n", n,
                        tick_jcmd->pos_cmd[n], (stepgen->prev_pos_cmd),
                        stepgen->vel_cmd_t);
                rtapi_print_msg(RTAPI_MSG_ERR, "wosi.c: wosi_pos_cmd(%d) too large\n",
                        wosi_pos_cmd);
                assert(0);
            }

            // SYNC_JNT: opcode for SYNC_JNT command
            // DIR_P: Direction, (positive(1), negative(0))
            // POS_MASK: relative position mask

            // TODO: pack sync_cmd into single-32-bit-word for each joint

            /* packing integer part of position command (16-bit) */
            if (integer_pos_cmd >= 0)
            {
                sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & wosi_pos_cmd);
            } else
            {
                sync_cmd = SYNC_JNT | DIR_N | (POS_MASK & wosi_pos_cmd);
            }
            memcpy(data + (2 * n * sizeof(uint16_t)), &sync_cmd,
                    sizeof(uint16_t));

            /* packing fraction part (16-bit) */
            wosi_pos_cmd = (abs(integer_pos_cmd)) & FRACTION_MASK;
            sync_cmd = (uint16_t) wosi_pos_cmd;
            memcpy(data + (2 * n + 1) * sizeof(uint16_t), &sync_cmd,
                    sizeof(uint16_t));

            stepgen->rawcount += (int64_t) integer_pos_cmd; // precision: 64.16
            stepgen->prev_pos_cmd = (((double) stepgen->rawcount
                    * stepgen->scale_recip) / (FIXED_POINT_SCALE));
        }

        if (n == (num_joints - 1))
        {
            // send to WOSI when all axes commands are generated
            wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
                    4 * num_joints, data);
        }

        DPS("%17d%17.7f%17.7f%17.7f",
                integer_pos_cmd, (stepgen->prev_pos_cmd), *stepgen->pos_fb, *stepgen->risc_pos_cmd);
    }

    sync_cmd = SYNC_EOF;
    memcpy(data, &sync_cmd, sizeof(uint16_t));
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), data);
    {

#if (TRACE==2)
        struct timespec t_begin, t_end, dt;
        clock_gettime(CLOCK_REALTIME, &t_begin);
#endif

        if (wosi_flush(&w_param) == -1)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: wosi_flush()\n");
        }

#if (TRACE==2)
        clock_gettime(CLOCK_REALTIME, &t_end);
        diff_time(&t_begin, &t_end, &dt);
        DP("dt.sec(%lu), dt.nsec(%lu)\n", dt.tv_sec, dt.tv_nsec);
#endif

    }

#if (TRACE!=0)
    if (tick_jcmd->cmd & (1 << TICK_AMP_ENABLE))
    {
        DPS("\n");
    }
#endif

}

/***********************************************************************
 *                   LOCAL FUNCTION DEFINITIONS                         *
 ************************************************************************/

static int export_analog(analog_t * addr)
{
    int i, retval;

    // export Analog IN
    for (i = 0; i < 16; i++)
    {
        retval = hal_pin_float_newf(HAL_OUT, &(addr->in[i]), comp_id,
                "wosi.analog.in.%d", i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->in[i]) = 0;
    }

    // export Analog OUT
    for (i = 0; i < 4; i++)
    {
        retval = hal_pin_float_newf(HAL_OUT, &(addr->out_fb[i]), comp_id,
                "wosi.analog.out.%d.fb", i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->out_fb[i]) = 0;

        retval = hal_pin_float_newf(HAL_IN, &(addr->out[i]), comp_id,
                "wosi.analog.out.%d", i);
        if (retval != 0)
        {
            return retval;
        }
        *(addr->out[i]) = 0;
        addr->prev_out[i] = 0;
    }

    return 0;
} // export_analog ()

static int export_stepgen(int num, stepgen_t * addr)
{
    int retval;

    retval = hal_pin_s32_newf(HAL_OUT, &(addr->cmd_pos), comp_id,
            "wosi.stepgen.%d.cmd-pos", num);
    if (retval != 0)
    {
        return retval;
    }

    retval = hal_pin_s32_newf(HAL_OUT, &(addr->rawcount32), comp_id,
            "wosi.stepgen.%d.rawcount32", num);
    if (retval != 0)
    {
        return retval;
    }

    retval = hal_pin_s32_newf(HAL_OUT, &(addr->enc_pos), comp_id,
            "wosi.stepgen.%d.enc_pos", num);
    if (retval != 0)
    {
        return retval;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(addr->vel_cmd), comp_id,
            "wosi.stepgen.%d.vel-cmd", num);
    if (retval != 0)
    {
        return retval;
    }

    /* export pin for pos/vel command */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->probed_pos), comp_id,
            "wosi.stepgen.%d.probed-pos", num);
    if (retval != 0)
    {
        return retval;
    }

    /* export pin for scaled position captured by update() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pos_fb), comp_id,
            "wosi.stepgen.%d.position-fb", num);
    if (retval != 0)
    {
        return retval;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(addr->risc_pos_cmd), comp_id,
            "wosi.stepgen.%d.risc-pos-cmd", num);
    if (retval != 0)
    {
        return retval;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(addr->ferror), comp_id,
            "wosi.stepgen.%d.ferror", num);
    if (retval != 0)
    {
        return retval;
    }

    /* export pin for velocity feedback (unit/sec) */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->vel_fb), comp_id,
            "wosi.stepgen.%d.vel-fb", num);
    if (retval != 0)
    {
        return retval;
    }

    /* export pin for velocity feedback (pulse) */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->enc_vel_p), comp_id,
            "wosi.stepgen.%d.enc-vel", num);
    if (retval != 0)
    {
        return retval;
    }

    /* export pin for following error */
    retval = hal_pin_bit_newf(HAL_OUT, &(addr->ferror_flag), comp_id,
            "wosi.stepgen.%d.ferror-flag", num);
    if (retval != 0)
    {
        return retval;
    }

    retval = hal_pin_bit_newf(HAL_IN, &(addr->homing), comp_id,
            "wosi.stepgen.%d.homing", num);
    if (retval != 0)
    {
        return retval;
    }
    *addr->homing = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(addr->bypass_lsp), comp_id,
            "wosi.stepgen.%d.bypass_lsp", num);
    if (retval != 0)
    {
        return retval;
    }
    *addr->bypass_lsp = 0;
    addr->prev_bypass_lsp = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(addr->bypass_lsn), comp_id,
            "wosi.stepgen.%d.bypass_lsn", num);
    if (retval != 0)
    {
        return retval;
    }
    *addr->bypass_lsn = 0;
    addr->prev_bypass_lsn = 0;

    retval = hal_pin_float_newf(HAL_IN, &(addr->risc_probe_dist), comp_id,
            "wosi.stepgen.%d.risc-probe-dist", num);
    if (retval != 0)
    {
        return retval;
    }
    *addr->risc_probe_dist = 0;

    retval = hal_pin_s32_newf(HAL_IN, &(addr->risc_probe_pin), comp_id,
            "wosi.stepgen.%d.risc-probe-pin", num);
    if (retval != 0)
    {
        return retval;
    }
    *addr->risc_probe_pin = -1;

    retval = hal_pin_s32_newf(HAL_IN, &(addr->risc_probe_type), comp_id,
            "wosi.stepgen.%d.risc-probe-type", num);
    if (retval != 0)
    {
        return retval;
    }
    *addr->risc_probe_type = -1;

    retval = hal_pin_float_newf(HAL_IN, &(addr->uu_per_rev), comp_id,
            "wosi.stepgen.%d.uu-per-rev", num);
    if (retval != 0)
    {
        return retval;
    }
    *(addr->uu_per_rev) = 0;
    (addr->prev_uu_per_rev) = 0;

    retval = hal_pin_float_newf(HAL_IN, &(addr->jog_vel), comp_id,
            "wosi.stepgen.%d.jog-vel", num);
    if (retval != 0)
    {
        return retval;
    }
    *(addr->jog_vel) = 0;
    (addr->prev_jog_vel) = 0;

    addr->pos_scale = 0.0;
    addr->scale_recip = 0.0;
    addr->out_type = 0;
    /* init the step generator core to zero output */
    addr->rawcount = 0;
    addr->prev_pos_cmd = 0;

    /* set initial pin values */
    *(addr->rawcount32) = 0;
    *(addr->enc_pos) = 0;
    *(addr->pos_fb) = 0.0;
    *(addr->ferror) = 0.0;
    *(addr->vel_fb) = 0;
    *(addr->vel_cmd) = 0.0;
    addr->pulse_vel = 0;
    addr->pulse_accel = 0;
    addr->pulse_jerk = 0;

    return 0;
}

static int export_machine_control(machine_control_t * machine_control)
{
    int i, retval;

    //!< wosi.reload-params, set by external component, reset by wosi_trans after reloading
    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->reload_params), comp_id,
            "wosi.reload-params");
    if (retval != 0)
    {
        return retval;
    }
    *machine_control->reload_params = 0;

    retval = hal_pin_float_newf(HAL_IN, &(machine_control->current_vel),
            comp_id, "wosi.motion.current-vel");
    if (retval != 0)
    {
        return retval;
    }
    retval = hal_pin_float_newf(HAL_IN, &(machine_control->feed_scale), comp_id,
            "wosi.motion.feed-scale");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->feed_scale) = 0; // pin index must not beyond index

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->machine_on), comp_id,
            "wosi.machine-on");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->machine_on) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->gantry_en), comp_id,
            "wosi.gantry-en");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->gantry_en) = 0;

    // rt_abort: realtime abort command to FPGA
    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->rt_abort), comp_id,
            "wosi.rt.abort");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->rt_abort) = 0;

    // export input status pin
    for (i = 0; i < GPIO_IN_NUM; i++)
    {
        retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->in[i]), comp_id,
                "wosi.gpio.in.%d", i);
        if (retval != 0)
        {
            return retval;
        }
        *(machine_control->in[i]) = 0;

        retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->in_n[i]), comp_id,
                "wosi.gpio.in.%d.not", i);
        if (retval != 0)
        {
            return retval;
        }
        *(machine_control->in_n[i]) = 0;
    }

    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->sync_in_trigger),
            comp_id, "wosi.sync.in.trigger");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->sync_in_trigger) = 0; // pin index must not beyond index

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->sync_in_index),
            comp_id, "wosi.sync.in.index");
    *(machine_control->sync_in_index) = 0; // pin index must not beyond index
    if (retval != 0)
    {
        return retval;
    }
    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->wait_type), comp_id,
            "wosi.sync.in.wait_type");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->wait_type) = 0;
    retval = hal_pin_float_newf(HAL_IN, &(machine_control->timeout), comp_id,
            "wosi.sync.in.timeout");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->timeout) = 0.0;

    for (i = 0; i < GPIO_OUT_NUM; i++)
    {
        retval = hal_pin_bit_newf(HAL_IN, &(machine_control->out[i]), comp_id,
                "wosi.gpio.out.%d", i);
        if (retval != 0)
        {
            return retval;
        }
        *(machine_control->out[i]) = 0;
    }

    retval = hal_pin_float_newf(HAL_IN, &(machine_control->analog_ref_level),
            comp_id, "wosi.sync.analog_ref_level");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->analog_ref_level) = 0; // pin index must not beyond index

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->ahc_state), comp_id,
            "wosi.ahc.state");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->ahc_state) = 0;

    retval = hal_pin_float_newf(HAL_IN, &(machine_control->ahc_level), comp_id,
            "wosi.ahc.level");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->ahc_level) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->motion_s3), comp_id,
            "wosi.ahc.motion_s3");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->motion_s3) = 0;

    /* wosi command */
    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->wosi_cmd), comp_id,
            "wosi.motion.cmd");
    *(machine_control->wosi_cmd) = 0; // pin index must not beyond index
    machine_control->prev_wosi_cmd = 0;
    if (retval != 0)
    {
        return retval;
    }

    retval = hal_pin_s32_newf(HAL_IN, &(machine_control->accel_state), comp_id,
            "wosi.accel-state");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->accel_state) = 0;
    machine_control->prev_accel_state = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->jog_sel), comp_id,
            "wosi.motion.jog-sel");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->jog_sel) = 0;

    retval = hal_pin_s32_newf(HAL_IN, &(machine_control->jog_vel_scale),
            comp_id, "wosi.motion.jog-vel-scale");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->jog_vel_scale) = 0;

    retval = hal_pin_s32_newf(HAL_OUT, &(machine_control->mpg_count), comp_id,
            "wosi.mpg_count");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->mpg_count) = 0;

    retval = hal_pin_s32_newf(HAL_IN, &(machine_control->test_pattern), comp_id,
            "wosi.test_pattern");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->test_pattern) = 0;

    for (i = 0; i < 32; i++)
    {
        retval = hal_pin_s32_newf(HAL_OUT, &(machine_control->debug[i]),
                comp_id, "wosi.debug.value-%02d", i);
        if (retval != 0)
        {
            return retval;
        }
        *(machine_control->debug[i]) = 0;
    }

    // dout0: the DOUT value obtained from RISC
    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->dout0), comp_id,
            "wosi.dout0");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->dout0) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->bp_tick), comp_id,
            "wosi.bp-tick");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->bp_tick) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->crc_error_counter),
            comp_id, "wosi.crc-error-counter");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->crc_error_counter) = 0;

    for (i=0; i<(GPIO_OUT_NUM >> 5); i++)
    {
        machine_control->prev_out[i] = 0;
    }

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->teleop_mode), comp_id,
            "wosi.motion.teleop-mode");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->teleop_mode) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->coord_mode), comp_id,
            "wosi.motion.coord-mode");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->coord_mode) = 0;

    // for RISC_CMD REQ and ACK
    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->update_pos_req),
            comp_id, "wosi.motion.update-pos-req");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->update_pos_req) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->rcmd_seq_num_req),
            comp_id, "wosi.motion.rcmd-seq-num-req");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->rcmd_seq_num_req) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->rcmd_state), comp_id,
            "wosi.motion.rcmd-state");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->rcmd_state) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->rcmd_seq_num_ack),
            comp_id, "wosi.motion.rcmd-seq-num-ack");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->rcmd_seq_num_ack) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->cur_tick),
            comp_id, "wosi.cur_tick");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->cur_tick) = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &(machine_control->max_tick),
            comp_id, "wosi.max_tick");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->max_tick) = 0;

    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->ahc_doing), comp_id,
            "wosi.ahc.doing");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->ahc_doing) = 0;

    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->sfifo_empty), comp_id,
            "wosi.sfifo_empty");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->sfifo_empty) = 0;
    
    retval = hal_pin_bit_newf(HAL_OUT, &(machine_control->rtp_running), comp_id,
            "wosi.motion.rtp-running");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->rtp_running) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->spindle_joint_id),
            comp_id, "wosi.motion.spindle-joint-id");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->spindle_joint_id) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->trigger_din), comp_id,
            "wosi.trigger.din");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->trigger_din) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->trigger_ain), comp_id,
            "wosi.trigger.ain");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->trigger_ain) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->trigger_type), comp_id,
            "wosi.trigger.type");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->trigger_type) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->trigger_cond), comp_id,
            "wosi.trigger.cond");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->trigger_cond) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->trigger_level),
            comp_id, "wosi.trigger.level");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->trigger_level) = 0;

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->probing), comp_id,
            "wosi.motion.probing");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->probing) = 0;
    (machine_control->prev_probing) = 0;

    retval = hal_pin_bit_newf(HAL_IO, &(machine_control->trigger_result),
            comp_id, "wosi.trigger.result");
    if (retval != 0)
    {
        return retval;
    }

    retval = hal_pin_bit_newf(HAL_IN, &(machine_control->pso_req), comp_id,
            "wosi.motion.pso_req");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->pso_req) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->pso_ticks), comp_id,
            "wosi.motion.pso_ticks");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->pso_ticks) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->pso_mode), comp_id,
            "wosi.motion.pso_mode");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->pso_mode) = 0;

    retval = hal_pin_u32_newf(HAL_IN, &(machine_control->pso_joint), comp_id,
            "wosi.motion.pso_joint");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->pso_joint) = 0;

    retval = hal_pin_float_newf(HAL_IN, &(machine_control->pso_pos), comp_id,
            "wosi.motion.pso_pos");
    if (retval != 0)
    {
        return retval;
    }
    *(machine_control->pso_pos) = 0;

    return 0;
}

//TODO: #endif	// RTAPI_SIM

// vim:sw=4:sts=4:et:
