/********************************************************************
* Description: motion.c
*   Main module initialisation and cleanup routines.
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include <stdarg.h>
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "rtapi_string.h"       /* memset */
#include "hal.h"		/* decls for HAL implementation */
#include "emcmotglb.h"
#include "motion.h"
#include "motion_debug.h"
#include "motion_struct.h"
#include "mot_priv.h"
#include "rtapi_math.h"

#ifdef USB_MOTION_ENABLE        /* enable usb motion device */
#include "sync_cmd.h"
#endif

// vtable signatures
#define VTKINS_VERSION VTKINEMATICS_VERSION1
#define VTP_VERSION    VTTP_VERSION1

// Mark strings for translation, but defer translation to userspace
#define _(s) (s)

/***********************************************************************
*                    KERNEL MODULE PARAMETERS                          *
************************************************************************/

/* module information */
/* register symbols to be modified by insmod
   see "Linux Device Drivers", Alessandro Rubini, p. 385
   (p.42-44 in 2nd edition) */
MODULE_AUTHOR("Matt Shaver/John Kasunich");
MODULE_DESCRIPTION("Motion Controller for EMC");
MODULE_LICENSE("GPL");

/* RTAPI shmem key - for comms with higher level user space stuff */
static int key = DEFAULT_MOTION_SHMEM_KEY;	/* the shared memory key, default value */
RTAPI_MP_INT(key, "shared memory key");
static long base_period_nsec = 0;	/* fastest thread period */
RTAPI_MP_LONG(base_period_nsec, "fastest thread period (nsecs)");
static int base_cpu = -1;		/* explicitly bind to CPU */
RTAPI_MP_INT(base_cpu, "CPU of base thread");
int base_thread_fp = 0;	/* default is no floating point in base thread */
RTAPI_MP_INT(base_thread_fp, "floating point in base thread?");
static long servo_period_nsec = 1000000;	/* servo thread period */
RTAPI_MP_LONG(servo_period_nsec, "servo thread period (nsecs)");
static int servo_cpu = -1;		/* explicitly bind to CPU */
RTAPI_MP_INT(servo_cpu, "CPU of servo thread");
static long traj_period_nsec = 0;	/* trajectory planner period */
RTAPI_MP_LONG(traj_period_nsec, "trajectory planner period (nsecs)");
static int num_joints = EMCMOT_MAX_JOINTS;	/* default number of joints present */
RTAPI_MP_INT(num_joints, "number of joints");
static int num_dio = DEFAULT_DIO;	/* default number of motion synched DIO */
RTAPI_MP_INT(num_dio, "number of digital inputs/outputs");
static int num_aio = DEFAULT_AIO;	/* default number of motion synched AIO */
RTAPI_MP_INT(num_aio, "number of analog inputs/outputs");
static char *kins = "trivkins";
RTAPI_MP_STRING(kins, "kinematics vtable name");
static char *tp = "tp";
RTAPI_MP_STRING(tp, "tp vtable name");

static int unlock_joints_mask = 0;/* mask to select joints for unlock pins */
RTAPI_MP_INT(unlock_joints_mask, "mask to select joints for unlock pins");
/***********************************************************************
*                  GLOBAL VARIABLE DEFINITIONS                         *
************************************************************************/

/* pointer to emcmot_hal_data_t struct in HAL shmem, with all HAL data */
emcmot_hal_data_t *emcmot_hal_data = 0;

/* pointer to joint data */
emcmot_joint_t *joints = 0;

/* pointer to axis data */
emcmot_axis_t *axes = 0;
/* Joints moved to HAL shared memory */
#if 0 // #ifndef STRUCTS_IN_SHMEM

/* allocate array for joint data */
emcmot_joint_t joint_array[EMCMOT_MAX_JOINTS];
/* allocate array for axis data */
emcmot_axis_t axis_array[EMCMOT_MAX_AXIS];
#endif

/*
  Principles of communication:

  Data is copied in or out from the various types of comm mechanisms:
  mbuff mapped memory for Linux/RT-Linux, or OS shared memory for Unixes.

  emcmotStruct is ptr to this memory.

  emcmotCommand points to emcmotStruct->command,
  emcmotStatus points to emcmotStruct->status,
  emcmotError points to emcmotStruct->error, and
 */
emcmot_struct_t *emcmotStruct = 0;
/* ptrs to either buffered copies or direct memory for command and status */
struct emcmot_command_t *emcmotCommand = 0;
struct emcmot_status_t *emcmotStatus = 0;
struct emcmot_config_t *emcmotConfig = 0;
struct emcmot_debug_t *emcmotDebug = 0;
TP_STRUCT *emcmotPrimQueue = 0; // primary planner + queues
TP_STRUCT *emcmotAltQueue = 0; // alternate planner + queues

// emcmotQueue: this was formerly &emcmotDebug->queue
TP_STRUCT *emcmotQueue = 0;     // current planner queue

struct emcmot_error_t *emcmotError = 0;	/* unused for RT_FIFO */

/***********************************************************************
*                  LOCAL VARIABLE DECLARATIONS                         *
************************************************************************/

/* RTAPI shmem ID - for comms with higher level user space stuff */
static int emc_shmem_id;	/* the shared memory ID */

static int mot_comp_id;	/* component ID for motion module */

/***********************************************************************
*                   LOCAL FUNCTION PROTOTYPES                          *
************************************************************************/

/* init_hal_io() exports HAL pins and parameters making data from
   the realtime control module visible and usable by the world
*/
static int init_hal_io(void);

/* functions called by init_hal_io() */
static int export_joint(int num, joint_hal_t * addr);
static int export_axis(char c, axis_hal_t  * addr);

/* init_comm_buffers() allocates and initializes the command,
   status, and error buffers used to communicate witht the user
   space parts of emc.
*/
static int init_comm_buffers(void);

/* functions called by init_comm_buffers() */

/* init_threads() creates realtime threads, exports functions to
   do the realtime control, and adds the functions to the threads.
*/
static int init_threads(void);

/* functions called by init_threads() */
static int setTrajCycleTime(double secs);
static int setServoCycleTime(double secs);

// init the shared state handling between tp and motion
static int init_shared(tp_shared_t *tps,
		       struct emcmot_config_t *cfg,
		       struct emcmot_status_t *status,
		       emcmot_debug_t *dbg,
		       emcmot_joint_t *joint,
		       emcmot_hal_data_t *hal);
/***********************************************************************
*                     PUBLIC FUNCTION CODE                             *
************************************************************************/

void emcmot_config_change(void)
{
    if (emcmotConfig->head == emcmotConfig->tail) {
	emcmotConfig->config_num++;
	emcmotStatus->config_num = emcmotConfig->config_num;
	emcmotConfig->head++;
    }
}

void reportError(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    emcmotErrorPutfv(emcmotError, fmt, args);
    va_end(args);
}

#ifndef va_copy
#define va_copy(dest, src) ((dest)=(src))
#endif

static rtapi_msg_handler_t old_handler = NULL;
static void emc_message_handler(msg_level_t level, const char *fmt, va_list ap)
{
    va_list apc;
    va_copy(apc, ap);
    if(level == RTAPI_MSG_ERR) emcmotErrorPutfv(emcmotError, fmt, apc);
    if(old_handler) old_handler(level, fmt, ap);
    va_end(apc);
}

int rtapi_app_main(void)
{
    int retval;
    // rtapi_set_msg_level(RTAPI_MSG_DBG);
    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_module() starting...\n");

    /* connect to the HAL and RTAPI */
    mot_comp_id = hal_init("motmod");
    if (mot_comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: hal_init() failed\n"));
	return -1;
    }
    if (( num_joints < 1 ) || ( num_joints > EMCMOT_MAX_JOINTS )) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    _("MOTION: num_joints is %d, must be between 1 and %d\n"), num_joints, EMCMOT_MAX_JOINTS);
	hal_exit(mot_comp_id);
	return -1;
    }

    if (( num_dio < 1 ) || ( num_dio > EMCMOT_MAX_DIO )) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    _("MOTION: num_dio is %d, must be between 1 and %d\n"), num_dio, EMCMOT_MAX_DIO);
	hal_exit(mot_comp_id);
	return -1;
    }
    
    if (( num_aio < 1 ) || ( num_aio > EMCMOT_MAX_AIO )) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    _("MOTION: num_aio is %d, must be between 1 and %d\n"), num_aio, EMCMOT_MAX_AIO);
	hal_exit(mot_comp_id);
	return -1;
    }

    /* initialize/export HAL pins and parameters */
    retval = init_hal_io();
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: init_hal_io() failed\n"));
	hal_exit(mot_comp_id);
	return -1;
    }

    /* allocate/initialize user space comm buffers (cmd/status/err) */
    retval = init_comm_buffers();
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: init_comm_buffers() failed\n"));
	hal_exit(mot_comp_id);
	return -1;
    }

    /* set up for realtime execution of code */
    retval = init_threads();
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: init_threads() failed\n"));
	hal_exit(mot_comp_id);
	return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_module() complete\n");

    hal_ready(mot_comp_id);

    old_handler = rtapi_get_msg_handler();
    rtapi_set_msg_handler(emc_message_handler);
    return 0;
}

void rtapi_app_exit(void)
{
    int retval;

    rtapi_set_msg_handler(old_handler);

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: cleanup_module() started.\n");

    retval = hal_stop_threads();
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    _("MOTION: hal_stop_threads() failed, returned %d\n"), retval);
    }

    // release the kinematics vtable
    hal_unreference_vtable(emcmotConfig->kins_vid);

    // release the tp vtable
    hal_unreference_vtable(emcmotConfig->tp_vid);

    /* free shared memory */
    retval = rtapi_shmem_delete(emc_shmem_id, mot_comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    _("MOTION: rtapi_shmem_delete() failed, returned %d\n"), retval);
    }
    /* disconnect from HAL and RTAPI */
    retval = hal_exit(mot_comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    _("MOTION: hal_exit() failed, returned %d\n"), retval);
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: cleanup_module() finished.\n");
}

/***********************************************************************
*                         LOCAL FUNCTION CODE                          *
************************************************************************/

/* init_hal_io() exports HAL pins and parameters making data from
   the realtime control module visible and usable by the world
*/
static int init_hal_io(void)
{
    int n, retval;
    joint_hal_t *joint_data;
    axis_hal_t  *axis_data;

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_hal_io() starting...\n");

    /* allocate shared memory for machine data */
    emcmot_hal_data = hal_malloc(sizeof(emcmot_hal_data_t));
    if (emcmot_hal_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: emcmot_hal_data malloc failed\n"));
	return -1;
    }

    // RISC_CMD REQ and ACK
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->update_pos_req), mot_comp_id, "motion.update-pos-req")) < 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->update_pos_ack), mot_comp_id, "motion.update-pos-ack")) < 0) goto error;
    if ((retval = hal_pin_u32_newf(HAL_IN, &(emcmot_hal_data->rcmd_state), mot_comp_id, "motion.rcmd-state")) < 0) goto error;
    *emcmot_hal_data->update_pos_req = 0;
    *emcmot_hal_data->update_pos_ack = 0;
#ifdef USB_MOTION_ENABLE
    *emcmot_hal_data->rcmd_state = RCMD_IDLE;
#endif // USB_MOTION_ENABLE

    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->xuu_per_rev), mot_comp_id, "motion.spindle.xuu-per-rev")) < 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->yuu_per_rev), mot_comp_id, "motion.spindle.yuu-per-rev")) < 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->zuu_per_rev), mot_comp_id, "motion.spindle.zuu-per-rev")) < 0) goto error;
    *emcmot_hal_data->xuu_per_rev = 0.0;
    *emcmot_hal_data->yuu_per_rev = 0.0;
    *emcmot_hal_data->zuu_per_rev = 0.0;

    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->usb_busy), mot_comp_id, "motion.usb-busy")) < 0) goto error;
    /* allocate shared memory for joint data */
    joints = hal_malloc(sizeof(emcmot_joint_t) * EMCMOT_MAX_JOINTS);
    if (joints == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    _("MOTION: joints malloc failed\n"));
	return -1;
    }
    /* Clear joints memory */
    memset(joints, 0, sizeof(emcmot_joint_t) * EMCMOT_MAX_JOINTS);

    /* allocate shared memory for axis data */
    axes = hal_malloc(sizeof(emcmot_axis_t) * EMCMOT_MAX_AXIS);
    if (axes == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            _("MOTION: axes malloc failed\n"));
        return -1;
    }
    /* Clear axes memory */
    memset(axes, 0, sizeof(emcmot_axis_t) * EMCMOT_MAX_AXIS);
    
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->rtp_running), mot_comp_id, "motion.rtp-running")) < 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->mpg_scale_x1), mot_comp_id, "motion.mpg-scale-x1")) < 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->mpg_scale_x10), mot_comp_id, "motion.mpg-scale-x10")) < 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->mpg_scale_x100), mot_comp_id, "motion.mpg-scale-x100")) < 0) goto error;
    *emcmot_hal_data->usb_busy = 0;
    *emcmot_hal_data->rtp_running = 0;
    *emcmot_hal_data->mpg_scale_x1 = 0;
    *emcmot_hal_data->mpg_scale_x10 = 0;
    *emcmot_hal_data->mpg_scale_x100 = 0;

    /* export machine wide hal pins */
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->probing), mot_comp_id, "motion.probing")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->probe_input), mot_comp_id, "motion.probe-input")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IO, &(emcmot_hal_data->spindle_index_enable), mot_comp_id, "motion.spindle-index-enable")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->trigger_result), mot_comp_id, "motion.trigger-result")) != 0) goto error;
    if ((retval = hal_pin_u32_newf(HAL_IN, &(emcmot_hal_data->trigger_din), mot_comp_id, "motion.trigger.din")) != 0) goto error;
    if ((retval = hal_pin_u32_newf(HAL_IN, &(emcmot_hal_data->trigger_ain), mot_comp_id, "motion.trigger.ain")) != 0) goto error;
    if ((retval = hal_pin_u32_newf(HAL_IN, &(emcmot_hal_data->trigger_type), mot_comp_id, "motion.trigger.type")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->trigger_cond), mot_comp_id, "motion.trigger.cond")) != 0) goto error;
    if ((retval = hal_pin_u32_newf(HAL_IN, &(emcmot_hal_data->trigger_level), mot_comp_id, "motion.trigger.level")) != 0) goto error;


    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->spindle_on), mot_comp_id, "motion.spindle-on")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->spindle_forward), mot_comp_id, "motion.spindle-forward")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->spindle_reverse), mot_comp_id, "motion.spindle-reverse")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->spindle_brake), mot_comp_id, "motion.spindle-brake")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->spindle_speed_out), mot_comp_id, "motion.spindle-speed-out")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->spindle_speed_out_abs), mot_comp_id, "motion.spindle-speed-out-abs")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->spindle_speed_out_rps), mot_comp_id, "motion.spindle-speed-out-rps")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->spindle_speed_out_rps_abs), mot_comp_id, "motion.spindle-speed-out-rps-abs")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->spindle_speed_cmd_rps), mot_comp_id, "motion.spindle-speed-cmd-rps")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->spindle_inhibit), mot_comp_id, "motion.spindle-inhibit")) != 0) goto error;
    *(emcmot_hal_data->spindle_inhibit) = 0;

    // spindle orient pins
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->spindle_orient_angle), mot_comp_id, "motion.spindle-orient-angle")) < 0) goto error;
    if ((retval = hal_pin_s32_newf(HAL_OUT, &(emcmot_hal_data->spindle_orient_mode), mot_comp_id, "motion.spindle-orient-mode")) < 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->spindle_orient), mot_comp_id, "motion.spindle-orient")) < 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->spindle_locked), mot_comp_id, "motion.spindle-locked")) < 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->spindle_is_oriented), mot_comp_id, "motion.spindle-is-oriented")) < 0) goto error;
    if ((retval = hal_pin_s32_newf(HAL_IN, &(emcmot_hal_data->spindle_orient_fault), mot_comp_id, "motion.spindle-orient-fault")) < 0) goto error;
    *(emcmot_hal_data->spindle_orient_angle) = 0.0;
    *(emcmot_hal_data->spindle_orient_mode) = 0;
    *(emcmot_hal_data->spindle_orient) = 0;

    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->spindle_revs), mot_comp_id, "motion.spindle-revs")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->spindle_speed_in), mot_comp_id, "motion.spindle-speed-in")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->spindle_is_atspeed), mot_comp_id, "motion.spindle-at-speed")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->adaptive_feed), mot_comp_id, "motion.adaptive-feed")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->feed_hold), mot_comp_id, "motion.feed-hold")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->feed_inhibit), mot_comp_id, "motion.feed-inhibit")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->enable), mot_comp_id, "motion.enable")) != 0) goto error;

    /* export motion-synched digital output pins */
    /* export motion-synched digital output io pins for compatibility with io signals */
    /* export motion digital input pins */
    for (n = 0; n < num_dio; n++) {
	if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->synch_do[n]), mot_comp_id, "motion.digital-out-%02d", n)) != 0) goto error;
	if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->synch_di[n]), mot_comp_id, "motion.digital-in-%02d", n)) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_IO, &(emcmot_hal_data->synch_do_io[n]), mot_comp_id, "motion.digital-out-io-%02d", n)) < 0) goto error;
    }

    /* export motion-synched analog output pins */
    /* export motion-synched analog output io pins for compatibility with io signals */
    /* export motion analog input pins */
    for (n = 0; n < num_aio; n++) {
	if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->analog_output[n]), mot_comp_id, "motion.analog-out-%02d", n)) != 0) goto error;
	if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->analog_input[n]), mot_comp_id, "motion.analog-in-%02d", n)) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_IO, &(emcmot_hal_data->analog_output_io[n]), mot_comp_id, "motion.analog-out-io-%02d", n)) < 0) goto error;
    }

    /* export machine wide hal pins */
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->motion_enabled), mot_comp_id, "motion.motion-enabled")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->in_position), mot_comp_id, "motion.in-position")) != 0) goto error;
    if ((retval = hal_pin_s32_newf(HAL_OUT, &(emcmot_hal_data->motion_type), mot_comp_id, "motion.motion-type")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->coord_mode), mot_comp_id, "motion.coord-mode")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->teleop_mode), mot_comp_id, "motion.teleop-mode")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->coord_error), mot_comp_id, "motion.coord-error")) != 0) goto error;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->on_soft_limit), mot_comp_id, "motion.on-soft-limit")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->current_vel), mot_comp_id, "motion.current-vel")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->requested_vel), mot_comp_id, "motion.requested-vel")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->distance_to_go), mot_comp_id, "motion.distance-to-go")) != 0) goto error;
    if ((retval = hal_pin_s32_newf(HAL_OUT, &(emcmot_hal_data->program_line), mot_comp_id, "motion.program-line")) != 0) goto error;


    if ((retval = hal_pin_s32_newf(HAL_OUT, &(emcmot_hal_data->accel_state), mot_comp_id, "motion.accel-state")) != 0) goto error;

    /* export debug parameters */
    /* these can be used to view any internal variable, simply change a line
       in control.c:output_to_hal() and recompile */
    if ((retval = hal_param_bit_newf(HAL_RO, &(emcmot_hal_data->debug_bit_0), mot_comp_id, "motion.debug-bit-0")) != 0) goto error;
    if ((retval = hal_param_bit_newf(HAL_RO, &(emcmot_hal_data->debug_bit_1), mot_comp_id, "motion.debug-bit-1")) != 0) goto error;
    if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->debug_float_0), mot_comp_id, "motion.debug-float-0")) != 0) goto error;
    if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->debug_float_1), mot_comp_id, "motion.debug-float-1")) != 0) goto error;
    if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->debug_float_2), mot_comp_id, "motion.debug-float-2")) != 0) goto error;
    if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->debug_float_3), mot_comp_id, "motion.debug-float-3")) != 0) goto error;
    if ((retval = hal_param_s32_newf(HAL_RO, &(emcmot_hal_data->debug_s32_0), mot_comp_id, "motion.debug-s32-0")) != 0) goto error;
    if ((retval = hal_param_s32_newf(HAL_RO, &(emcmot_hal_data->debug_s32_1), mot_comp_id, "motion.debug-s32-1")) != 0) goto error;

    // FIXME - debug only, remove later
    // export HAL parameters for some trajectory planner internal variables
    // so they can be scoped
    if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->traj_pos_out), mot_comp_id, "traj.pos_out")) != 0) goto error;
    if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->traj_vel_out), mot_comp_id, "traj.vel_out")) != 0) goto error;
    if ((retval = hal_param_u32_newf(HAL_RO, &(emcmot_hal_data->traj_active_tc), mot_comp_id, "traj.active_tc")) != 0) goto error;

    for ( n = 0 ; n < 4 ; n++ ) {
        if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->tc_pos[n]), mot_comp_id, "tc.%d.pos", n)) != 0) goto error;
        if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->tc_vel[n]), mot_comp_id, "tc.%d.vel", n)) != 0) goto error;
        if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->tc_acc[n]), mot_comp_id, "tc.%d.acc", n)) != 0) goto error;
    }
    // end of exporting trajectory planner internals

    // export timing related HAL parameters so they can be scoped
    if ((retval = hal_param_u32_newf(HAL_RO, &(emcmot_hal_data->last_period), mot_comp_id, "motion.servo.last-period")) != 0) goto error;
#ifdef HAVE_CPU_KHZ
    if ((retval = hal_param_float_newf(HAL_RO, &(emcmot_hal_data->last_period_ns), mot_comp_id, "motion.servo.last-period-ns")) != 0) goto error;
#endif
    if ((retval = hal_param_u32_newf(HAL_RO, &(emcmot_hal_data->overruns), mot_comp_id, "motion.servo.overruns")) != 0) goto error;

    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_x), mot_comp_id, "motion.tooloffset.x")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_y), mot_comp_id, "motion.tooloffset.y")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_z), mot_comp_id, "motion.tooloffset.z")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_a), mot_comp_id, "motion.tooloffset.a")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_b), mot_comp_id, "motion.tooloffset.b")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_c), mot_comp_id, "motion.tooloffset.c")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_u), mot_comp_id, "motion.tooloffset.u")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_v), mot_comp_id, "motion.tooloffset.v")) != 0) goto error;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->tooloffset_w), mot_comp_id, "motion.tooloffset.w")) != 0) goto error;

    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->tp_reverse_input),
                                   mot_comp_id, "motion.tp-reverse-input")) < 0) return retval;

    if ((retval = hal_pin_s32_newf(HAL_OUT, &(emcmot_hal_data->pause_state),
				   mot_comp_id, "motion.pause-state")) < 0) return retval;
    // feedhold-offset related pins
    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->pause_return_path),
                                   mot_comp_id, "motion.pause-return-path")) < 0) return retval;

    if ((retval = hal_pin_bit_newf(HAL_IN, &(emcmot_hal_data->pause_offset_enable),
				   mot_comp_id, "motion.pause-offset-enable")) < 0) return retval;

    if ((retval = hal_pin_s32_newf(HAL_OUT, &(emcmot_hal_data->paused_at_motion_type),
				   mot_comp_id, "motion.paused-at-motion")) < 0) return retval;

    if ((retval = hal_pin_s32_newf(HAL_OUT, &(emcmot_hal_data->current_motion_type),
				   mot_comp_id, "motion.current-motion")) < 0) return retval;

    if ((retval = hal_pin_bit_newf(HAL_OUT, &(emcmot_hal_data->pause_offset_in_range),
				   mot_comp_id, "motion.pause-offset-in-range")) < 0) return retval;


    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_jog_vel),
				     mot_comp_id, "motion.pause-jog-feed")) < 0) return retval;

    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_x),
				     mot_comp_id, "motion.pause-offset-x")) < 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_y),
				     mot_comp_id, "motion.pause-offset-y")) < 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_z),
				     mot_comp_id, "motion.pause-offset-z")) < 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_a),
				     mot_comp_id, "motion.pause-offset-a")) < 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_b),
				     mot_comp_id, "motion.pause-offset-b")) < 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_c),
				     mot_comp_id, "motion.pause-offset-c")) < 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_u),
				     mot_comp_id, "motion.pause-offset-u")) < 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_v),
				     mot_comp_id, "motion.pause-offset-v")) < 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(emcmot_hal_data->pause_offset_w),
				     mot_comp_id, "motion.pause-offset-w")) < 0) return retval;


    /* initialize machine wide pins and parameters */
    *emcmot_hal_data->spindle_is_atspeed = 1;
    *(emcmot_hal_data->adaptive_feed) = 1.0;
    *(emcmot_hal_data->feed_hold) = 0;
    *(emcmot_hal_data->feed_inhibit) = 0;

    *(emcmot_hal_data->probe_input) = 0;
    /* default value of enable is TRUE, so simple machines
       can leave it disconnected */
    *(emcmot_hal_data->enable) = 1;
    
    /* motion synched dio, init to not enabled */
    for (n = 0; n < num_dio; n++) {
	 *(emcmot_hal_data->synch_do[n]) = 0;
	 *(emcmot_hal_data->synch_di[n]) = 0;
     *(emcmot_hal_data->synch_do_io[n]) = 0;
    }

    for (n = 0; n < num_aio; n++) {
	 *(emcmot_hal_data->analog_output[n]) = 0.0;
	 *(emcmot_hal_data->analog_input[n]) = 0.0;
     *(emcmot_hal_data->analog_output_io[n]) = 0.0;
    }
    
    /*! \todo FIXME - these don't really need initialized, since they are written
       with data from the emcmotStatus struct */
    *(emcmot_hal_data->motion_enabled) = 0;
    *(emcmot_hal_data->in_position) = 0;
    *(emcmot_hal_data->motion_type) = 0;
    *(emcmot_hal_data->coord_mode) = 0;
    *(emcmot_hal_data->teleop_mode) = 0;
    *(emcmot_hal_data->coord_error) = 0;
    *(emcmot_hal_data->on_soft_limit) = 0;

    /* init debug parameters */
    emcmot_hal_data->debug_bit_0 = 0;
    emcmot_hal_data->debug_bit_1 = 0;
    emcmot_hal_data->debug_float_0 = 0.0;
    emcmot_hal_data->debug_float_1 = 0.0;
    emcmot_hal_data->debug_float_2 = 0.0;
    emcmot_hal_data->debug_float_3 = 0.0;

    emcmot_hal_data->overruns = 0;
    emcmot_hal_data->last_period = 0;

    /* export joint pins and parameters */
    for (n = 0; n < num_joints; n++) {
	/* point to axis data */
	joint_data = &(emcmot_hal_data->joint[n]);
	/* export all vars */
        retval = export_joint(n, joint_data);
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: joint %d pin/param export failed\n"), n);
	    return -1;
	}
	/* init axis pins and parameters */
	*(joint_data->amp_enable) = 0;
	*(joint_data->home_state) = HOME_IDLE;
        *(joint_data->home_sw_id) = -1; //!< for usb_homing.c        
	/* We'll init the index model to EXT_ENCODER_INDEX_MODEL_RAW for now,
	   because it is always supported. */
    }

    /* export axis pins and parameters */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        char c = "xyzabcuvw"[n];
        axis_data = &(emcmot_hal_data->axis[n]);
        if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->axis[n].pos_cmd),
           mot_comp_id, "axis.%c.pos-cmd",         c)) != 0) goto error;
        if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->axis[n].vel_cmd),
           mot_comp_id, "axis.%c.vel-cmd",         c)) != 0) goto error;
        if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->axis[n].teleop_pos_cmd),
           mot_comp_id, "axis.%c.teleop-pos-cmd",  c)) != 0) goto error;
        if ((retval = hal_pin_float_newf(HAL_OUT, &(emcmot_hal_data->axis[n].teleop_vel_lim),
           mot_comp_id, "axis.%c.teleop-vel-lim",  c)) != 0) goto error;
        if ((retval = hal_pin_bit_newf(HAL_OUT,   &(emcmot_hal_data->axis[n].teleop_tp_enable),
           mot_comp_id, "axis.%c.teleop-tp-enable",c)) != 0) goto error;

        retval = export_axis(c, axis_data);
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: axis %c pin/param export failed\n"), c);
	    return -1;
	}
    }

    /* Done! */
    rtapi_print_msg(RTAPI_MSG_INFO,
	"MOTION: init_hal_io() complete, %d axes.\n", n);
    return 0;

    error:
	return retval;

}

static int export_joint(int num, joint_hal_t * addr)
{
    int retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    /* export joint pins */
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->coarse_pos_cmd), mot_comp_id, "joint.%d.coarse-pos-cmd", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->joint_pos_cmd), mot_comp_id, "joint.%d.pos-cmd", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->joint_pos_fb), mot_comp_id, "joint.%d.pos-fb", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->motor_pos_cmd), mot_comp_id, "joint.%d.motor-pos-cmd", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(addr->motor_pos_fb), mot_comp_id, "joint.%d.motor-pos-fb", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->motor_offset), mot_comp_id, "joint.%d.motor-offset", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(addr->blender_offset), mot_comp_id, "joint.%d.blender-offset", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(addr->pos_lim_sw), mot_comp_id, "joint.%d.pos-lim-sw-in", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(addr->neg_lim_sw), mot_comp_id, "joint.%d.neg-lim-sw-in", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(addr->home_sw), mot_comp_id, "joint.%d.home-sw-in", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IO, &(addr->index_enable), mot_comp_id, "joint.%d.index-enable", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->amp_enable), mot_comp_id, "joint.%d.amp-enable-out", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(addr->amp_fault), mot_comp_id, "joint.%d.amp-fault-in", num)) != 0) return retval;
    if ((retval = hal_pin_s32_newf(HAL_IN,   &(addr->jjog_counts), mot_comp_id, "joint.%d.jog-counts", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IN,   &(addr->jjog_enable), mot_comp_id, "joint.%d.jog-enable", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(addr->jjog_scale), mot_comp_id, "joint.%d.jog-scale", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IN,   &(addr->jjog_vel_mode), mot_comp_id, "joint.%d.jog-vel-mode", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->joint_vel_cmd), mot_comp_id, "joint.%d.vel-cmd", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->backlash_corr), mot_comp_id, "joint.%d.backlash-corr", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->backlash_filt), mot_comp_id, "joint.%d.backlash-filt", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->backlash_vel), mot_comp_id, "joint.%d.backlash-vel", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->f_error), mot_comp_id, "joint.%d.f-error", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->f_error_lim), mot_comp_id, "joint.%d.f-error-lim", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->free_pos_cmd), mot_comp_id, "joint.%d.free-pos-cmd", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->free_vel_lim), mot_comp_id, "joint.%d.free-vel-lim", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->free_tp_enable), mot_comp_id, "joint.%d.free-tp-enable", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->kb_jjog_active), mot_comp_id, "joint.%d.kb-jog-active", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->wheel_jjog_active), mot_comp_id, "joint.%d.wheel-jog-active", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->in_position), mot_comp_id, "joint.%d.in-position", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->phl), mot_comp_id, "joint.%d.pos-hard-limit", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->nhl), mot_comp_id, "joint.%d.neg-hard-limit", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->active), mot_comp_id, "joint.%d.active", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->error), mot_comp_id, "joint.%d.error", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->f_errored), mot_comp_id, "joint.%d.f-errored", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->faulted), mot_comp_id, "joint.%d.faulted", num)) != 0) return retval;
    /* for usb_homing.c, TODO: rename usb_homing.c as fpga_homing.c */
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->homed), mot_comp_id, "joint.%d.homed", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->homing), mot_comp_id, "joint.%d.homing", num)) != 0) return retval;
    if ((retval = hal_pin_s32_newf(HAL_OUT, &(addr->home_state), mot_comp_id, "joint.%d.home-state", num)) != 0) return retval;

    if ( unlock_joints_mask & (1 << num) ) {
        // these pins may be needed for rotary joints
        rtapi_print_msg(RTAPI_MSG_WARN,"motion.c: Creating unlock hal pins for joint %d\n",num);
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->unlock), mot_comp_id, "joint.%d.unlock", num)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IN, &(addr->is_unlocked), mot_comp_id, "joint.%d.is-unlocked", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(addr->probed_pos), mot_comp_id, "joint.%d.probed-pos", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->risc_probe_vel), mot_comp_id, "joint.%d.risc-probe-vel", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_OUT, &(addr->risc_probe_dist), mot_comp_id, "joint.%d.risc-probe-dist", num)) != 0) return retval;
    if ((retval = hal_pin_s32_newf(HAL_OUT, &(addr->risc_probe_pin), mot_comp_id, "joint.%d.risc-probe-pin", num)) != 0) return retval;
    if ((retval = hal_pin_s32_newf(HAL_OUT, &(addr->risc_probe_type), mot_comp_id, "joint.%d.risc-probe-type", num)) != 0) return retval;
    if ((retval = hal_pin_s32_newf(HAL_IN, &(addr->home_sw_id), mot_comp_id, "joint.%d.home-sw-id", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(addr->index_pos_pin), mot_comp_id, "joint.%d.index-pos", num)) != 0) return retval;
    /* for fpga_motion */
    if ((retval = hal_pin_bit_newf(HAL_IN, &(addr->usb_ferror_flag), mot_comp_id, "joint.%d.usb-ferror-flag", num)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN, &(addr->risc_pos_cmd), mot_comp_id, "joint.%d.risc-pos-cmd", num)) != 0) return retval;
    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}

static int export_axis(char c, axis_hal_t * addr)
{
    int retval, msg;
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    if ((retval = hal_pin_bit_newf(HAL_IN,  &(addr->ajog_enable),      mot_comp_id,"axis.%c.jog-enable", c)) != 0) return retval;
    if ((retval = hal_pin_float_newf(HAL_IN,&(addr->ajog_scale),       mot_comp_id,"axis.%c.jog-scale", c)) != 0) return retval;
    if ((retval = hal_pin_s32_newf(HAL_IN,  &(addr->ajog_counts),      mot_comp_id,"axis.%c.jog-counts", c)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_IN,  &(addr->ajog_vel_mode),    mot_comp_id,"axis.%c.jog-vel-mode", c)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->kb_ajog_active),   mot_comp_id,"axis.%c.kb-jog-active", c)) != 0) return retval;
    if ((retval = hal_pin_bit_newf(HAL_OUT, &(addr->wheel_ajog_active),mot_comp_id,"axis.%c.wheel-jog-active", c)) != 0) return retval;

    rtapi_set_msg_level(msg);
    return 0;
}

/* init_comm_buffers() allocates and initializes the command,
   status, and error buffers used to communicate with the user
   space parts of emc.
*/
static int init_comm_buffers(void)
{
    int joint_num, n;
    emcmot_joint_t *joint;
    int retval;

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_comm_buffers() starting...\n");

    emcmotStruct = 0;
    emcmotDebug = 0;
    emcmotStatus = 0;
    emcmotCommand = 0;
    emcmotConfig = 0;

    /* allocate and initialize the shared memory structure */
    emc_shmem_id = rtapi_shmem_new(key, mot_comp_id, sizeof(emcmot_struct_t));
    if (emc_shmem_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_new failed, returned %d\n", emc_shmem_id);
	return -1;
    }
    retval = rtapi_shmem_getptr(emc_shmem_id, (void **) &emcmotStruct, 0);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_getptr failed, returned %d\n", retval);
	return -1;
    }

    /* zero shared memory before doing anything else. */
    memset(emcmotStruct, 0, sizeof(emcmot_struct_t));

    /* we'll reference emcmotStruct directly */
    emcmotCommand = &emcmotStruct->command;
    emcmotStatus = &emcmotStruct->status;
    emcmotConfig = &emcmotStruct->config;

    // bind kinematics vtable
    emcmotConfig->kins_vid = hal_reference_vtable(kins, VTKINS_VERSION,
						  (void **)&emcmotConfig->vtk);
    if (emcmotConfig->kins_vid < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"MOTION: hal_reference_vtable(%s,%d) failed: %d\n",
			kins, VTKINS_VERSION, emcmotConfig->kins_vid);
	return -1;
    }

    /* record the kinematics type of the machine */
    emcmotConfig->kinType = emcmotConfig->vtk->kinematicsType();

    // bind the tp vtable
    emcmotConfig->tp_vid = hal_reference_vtable(tp, VTP_VERSION,
						(void **)&emcmotConfig->vtp);
    if (emcmotConfig->tp_vid < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"MOTION: hal_reference_vtable(%s,%d) failed: %d\n",
			tp, VTP_VERSION, emcmotConfig->tp_vid);
	return -1;
    }


    emcmotDebug = &emcmotStruct->debug;

    emcmotPrimQueue = &emcmotStruct->debug.tp;     // primary motion queue
    emcmotAltQueue = &emcmotStruct->debug.altqueue;   // alternate motion queue

    // emcmotQueue: this was formerly &emcmotDebug->queue
    emcmotQueue = emcmotPrimQueue;   // start on primary motion queue

    emcmotError = &emcmotStruct->error;

    /* init error struct */
    emcmotErrorInit(emcmotError);

    /* init command struct */
    emcmotCommand->head = 0;
    emcmotCommand->command = 0;
    emcmotCommand->commandNum = 0;
    emcmotCommand->tail = 0;
    emcmotCommand->spindlesync = 0.0;

    /* init status struct */
    emcmotStatus->head = 0;
    emcmotStatus->commandEcho = 0;
    emcmotStatus->commandNumEcho = 0;
    emcmotStatus->commandStatus = 0;
    emcmotStatus->wait_risc = 0;

    /* init more stuff */
    emcmotDebug->head = 0;
    emcmotConfig->head = 0;

    emcmotStatus->motionFlag = 0;
    SET_MOTION_ERROR_FLAG(0);
    SET_MOTION_COORD_FLAG(0);
    SET_MOTION_TELEOP_FLAG(0);
    emcmotDebug->split = 0;
    emcmotStatus->heartbeat = 0;
    emcmotConfig->numJoints = num_joints;
    emcmotConfig->numDIO = num_dio;
    emcmotConfig->numAIO = num_aio;

    ZERO_EMC_POSE(emcmotStatus->carte_pos_cmd);
    ZERO_EMC_POSE(emcmotStatus->carte_pos_fb);
    emcmotStatus->vel = VELOCITY;
    emcmotConfig->limitVel = VELOCITY;
    emcmotStatus->acc = ACCELERATION;
    emcmotStatus->feed_scale = 1.0;
    emcmotStatus->rapid_scale = 1.0;
    emcmotStatus->spindle_scale = 1.0;
    emcmotStatus->net_feed_scale = 1.0;
    /* adaptive feed is off by default, feed override, spindle 
       override, and feed hold are on */
    emcmotStatus->enables_new = FS_ENABLED | SS_ENABLED | FH_ENABLED;
    emcmotStatus->enables_queued = emcmotStatus->enables_new;
    emcmotStatus->id = 0;
    emcmotStatus->depth = 0;
    emcmotStatus->activeDepth = 0;
    emcmotStatus->pause_state  = PS_RUNNING;
    emcmotStatus->resuming = 0;
    emcmotStatus->overrideLimitMask = 0;
    emcmotStatus->spindle.speed = 0.0;
    SET_MOTION_INPOS_FLAG(1);
    SET_MOTION_ENABLE_FLAG(0);
    /* record the kinematics type of the machine */
    emcmotConfig->kinType = emcmotConfig->vtk->kinematicsType();
    emcmot_config_change();

    /* *joints, *axes are already initialized in init_hal_io() */

    /* init per-joint stuff */
    for (joint_num = 0; joint_num < emcmotConfig->numJoints; joint_num++) {
	/* point to structure for this joint */
	joint = &joints[joint_num];

	/* Export some HAL parameters */
	if ((retval = hal_pin_float_newf(HAL_IN, &(joint->home), mot_comp_id, "joint.%d.home", joint_num)) != 0) return retval;
	if ((retval = hal_pin_float_newf(HAL_IN, &(joint->home_offset), mot_comp_id, "joint.%d.home-offset", joint_num)) != 0) return retval;

	/* init the config fields with some "reasonable" defaults" */
        joint->id = joint_num;          //!< to be used for INDEX homing, usb_homing.c
	joint->type = 0;
	joint->max_pos_limit = 1.0;
	joint->min_pos_limit = -1.0;
	joint->en_soft_limit = 0;
	joint->vel_limit = 1.0;
	joint->acc_limit = 1.0;
        joint->jerk_limit = 1.0;
	joint->min_ferror = 0.01;
	joint->max_ferror = 1.0;
	joint->home_search_vel = 0.0;
	joint->home_latch_vel = 0.0;
	joint->home_final_vel = -1;
	*(joint->home_offset) = 0.0;
	*(joint->home) = 0.0;
	joint->home_flags = 0;
	joint->home_sequence = -1;
	joint->home_state = HOME_IDLE;
	joint->backlash = 0.0;

	joint->comp.entries = 0;
	joint->comp.entry = &(joint->comp.array[0]);
	/* the compensation code has -DBL_MAX at one end of the table
	   and +DBL_MAX at the other so _all_ commanded positions are
	   guaranteed to be covered by the table */
	joint->comp.array[0].nominal = -DBL_MAX;
	joint->comp.array[0].fwd_trim = 0.0;
	joint->comp.array[0].rev_trim = 0.0;
	joint->comp.array[0].fwd_slope = 0.0;
	joint->comp.array[0].rev_slope = 0.0;
	for ( n = 1 ; n < EMCMOT_COMP_SIZE+2 ; n++ ) {
	    joint->comp.array[n].nominal = DBL_MAX;
	    joint->comp.array[n].fwd_trim = 0.0;
	    joint->comp.array[n].rev_trim = 0.0;
	    joint->comp.array[n].fwd_slope = 0.0;
	    joint->comp.array[n].rev_slope = 0.0;
	}

	/* init joint flags */
	joint->flag = 0;
	SET_JOINT_INPOS_FLAG(joint, 1);

	/* init status info */
	joint->coarse_pos = 0.0;
	joint->pos_cmd = 0.0;
	joint->vel_cmd = 0.0;
	joint->backlash_corr = 0.0;
	joint->backlash_filt = 0.0;
	joint->backlash_vel = 0.0;
        joint->blender_offset = 0.0;
        joint->motor_pos_cmd = 0.0;
        joint->motor_pos_fb = 0.0;
	joint->pos_fb = 0.0;
	joint->ferror = 0.0;
	joint->ferror_limit = joint->min_ferror;
	joint->ferror_high_mark = 0.0;

        /* init usb_homing info */
        joint->risc_probe_vel = 0;
        joint->risc_probe_dist = 0;
        joint->risc_probe_pin = -1;
        joint->risc_probe_type = -1;

	/* init internal info */
	cubicInit(&(joint->cubic));
    }

    /*! \todo FIXME-- add emcmotError */

    emcmotDebug->cur_time = emcmotDebug->last_time = 0.0;
    emcmotDebug->start_time = etime();
    emcmotDebug->running_time = 0.0;

    emcmotPrimQueue = &emcmotStruct->debug.tp;     // primary motion queue
    emcmotAltQueue = &emcmotStruct->debug.altqueue;   // alternate motion queue


    // init the shared data between tp and using code
    emcmotDebug->tps = hal_malloc(sizeof(tp_shared_t));
    if (!emcmotDebug->tps) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"MOTION: failed to create tp_shared in HAL memory");
	return -1;
    }
    init_shared(emcmotDebug->tps,
		emcmotConfig,
		emcmotStatus,
		emcmotDebug,
		joints, // internal joint data
		emcmot_hal_data); // HAL exorted part of joint data

    /* init motion emcmotDebug->queue */
    if (-1 == emcmotConfig->vtp->tpCreate(emcmotPrimQueue, DEFAULT_TC_QUEUE_SIZE,
					  emcmotDebug->queueTcSpace,
					  emcmotDebug->tps)) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: failed to create motion emcmotPrimQueue\n");
	return -1;
    }

    // and the alternate queue
    if (-1 == emcmotConfig->vtp->tpCreate(emcmotAltQueue, DEFAULT_ALT_TC_QUEUE_SIZE,
					  emcmotDebug->altqueueTcSpace,
					  emcmotDebug->tps)) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: failed to create motion emcmotAltQueue\n");
	return -1;
    }


//    tpInit(&emcmotDebug->queue); // tpInit called from tpCreate
    emcmotConfig->vtp->tpSetCycleTime(emcmotQueue, emcmotConfig->trajCycleTime);
    emcmotConfig->vtp->tpSetPos(emcmotQueue, &emcmotStatus->carte_pos_cmd);
    emcmotConfig->vtp->tpSetVmax(emcmotQueue, emcmotStatus->vel, emcmotStatus->vel);
    emcmotConfig->vtp->tpSetAmax(emcmotQueue, emcmotStatus->acc);
    emcmotConfig->vtp->tpSetJmax(emcmotQueue, emcmotStatus->jerk);

    // the emcmotAltQueue parameters as per above are cloned
    // by tpSnapshot() during switching queues

    emcmotStatus->tail = 0;

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_comm_buffers() complete\n");
    return 0;
}

/* init_threads() creates realtime threads, exports functions to
   do the realtime control, and adds the functions to the threads.
*/
static int init_threads(void)
{
    double base_period_sec, servo_period_sec;
    int servo_base_ratio;
    int retval;

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_threads() starting...\n");

    /* if base_period not specified, assume same as servo_period */
    if (base_period_nsec == 0) {
	base_period_nsec = servo_period_nsec;
    }
    if (traj_period_nsec == 0) {
	traj_period_nsec = servo_period_nsec;
    }
    /* servo period must be greater or equal to base period */
    if (servo_period_nsec < base_period_nsec) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: bad servo period %ld nsec\n", servo_period_nsec);
	return -1;
    }
    /* convert desired periods to floating point */
    base_period_sec = base_period_nsec * 0.000000001;
    servo_period_sec = servo_period_nsec * 0.000000001;
    /* calculate period ratios, round to nearest integer */
    servo_base_ratio = (servo_period_sec / base_period_sec) + 0.5;
    /* revise desired periods to be integer multiples of each other */
    servo_period_nsec = base_period_nsec * servo_base_ratio;
    /* create HAL threads for each period */
    /* only create base thread if it is faster than servo thread */
    if (servo_base_ratio > 1) {

	retval = hal_create_thread("base-thread", base_period_nsec, base_thread_fp, base_cpu);
	if (retval < 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"MOTION: failed to create %ld nsec base thread\n",
		base_period_nsec);
	    return -1;
	}
    }
    retval = hal_create_thread("servo-thread", servo_period_nsec, 1, servo_cpu);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: failed to create %ld nsec servo thread\n",
	    servo_period_nsec);
	return -1;
    }
    /* export realtime functions that do the real work */
    retval = hal_export_funct("motion-controller", emcmotController, 0	/* arg 
	 */ , 1 /* uses_fp */ , 0 /* reentrant */ , mot_comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: failed to export controller function\n");
	return -1;
    }
    retval = hal_export_funct("motion-command-handler", emcmotCommandHandler, 0	/* arg 
	 */ , 1 /* uses_fp */ , 0 /* reentrant */ , mot_comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: failed to export command handler function\n");
	return -1;
    }
/*! \todo Another #if 0 */
#if 0
    /*! \todo FIXME - currently the traj planner is called from the controller */
    /* eventually it will be a separate function */
    retval = hal_export_funct("motion-traj-planner", emcmotTrajPlanner, 0	/* arg 
	 */ , 1 /* uses_fp */ ,
	0 /* reentrant */ , mot_comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: failed to export traj planner function\n");
	return -1;
    }
#endif

    // if we don't set cycle times based on these guesses, emc doesn't
    // start up right
    setServoCycleTime(servo_period_nsec * 1e-9);
    setTrajCycleTime(traj_period_nsec * 1e-9);

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_threads() complete\n");
    return 0;
}

void emcmotSetCycleTime(unsigned long nsec)
{
    int servo_mult;
    servo_mult = traj_period_nsec / nsec;
    if(servo_mult < 0) servo_mult = 1;
    setTrajCycleTime(nsec * 1e-9);
    setServoCycleTime(nsec * servo_mult * 1e-9);
}

void emcmotSetUuPerRev(PmCartesian uu_per_rev)
{
    *(emcmot_hal_data->xuu_per_rev) = uu_per_rev.x;
    *(emcmot_hal_data->yuu_per_rev) = uu_per_rev.y;
    *(emcmot_hal_data->zuu_per_rev) = uu_per_rev.z;
}

/* call this when setting the trajectory cycle time */
static int setTrajCycleTime(double secs)
{
    static int t;

    rtapi_print_msg(RTAPI_MSG_INFO,
	"MOTION: setting Traj cycle time to %ld nsecs\n", (long) (secs * 1e9));

    /* make sure it's not zero */
    if (secs <= 0.0) {
	return -1;
    }

    emcmot_config_change();

    /* compute the interpolation rate as nearest integer to traj/servo */
    if(emcmotConfig->servoCycleTime)
        emcmotConfig->interpolationRate =
            (int) (secs / emcmotConfig->servoCycleTime + 0.5);
    else
        emcmotConfig->interpolationRate = 1;

    /* set traj planner */
    emcmotConfig->vtp->tpSetCycleTime(emcmotPrimQueue, secs);

    /* set the free planners, cubic interpolation rate and segment time */
    for (t = 0; t < emcmotConfig->numJoints; t++) {
	cubicSetInterpolationRate(&(joints[t].cubic),
	    emcmotConfig->interpolationRate);
    }

    /* copy into status out */
    emcmotConfig->trajCycleTime = secs;

    return 0;
}

/* call this when setting the servo cycle time */
static int setServoCycleTime(double secs)
{
    static int t;

    rtapi_print_msg(RTAPI_MSG_INFO,
	"MOTION: setting Servo cycle time to %ld nsecs\n", (long) (secs * 1e9));

    /* make sure it's not zero */
    if (secs <= 0.0) {
	return -1;
    }

    emcmot_config_change();

    /* compute the interpolation rate as nearest integer to traj/servo */
    emcmotConfig->interpolationRate =
	(int) (emcmotConfig->trajCycleTime / secs + 0.5);

    /* set the cubic interpolation rate and PID cycle time */
    for (t = 0; t < emcmotConfig->numJoints; t++) {
	cubicSetInterpolationRate(&(joints[t].cubic),
	    emcmotConfig->interpolationRate);
	cubicSetSegmentTime(&(joints[t].cubic), secs);
    }

    /* copy into status out */
    emcmotConfig->servoCycleTime = secs;

    return 0;
}

static int init_shared(tp_shared_t *tps,
		       struct emcmot_config_t *cfg,
		       struct emcmot_status_t *status,
		       emcmot_debug_t *dbg,
		       emcmot_joint_t *joint,
		       emcmot_hal_data_t *hal) // hal not used yet
{
    // global module param
    tps->num_dio = &num_dio;
    tps->num_aio = &num_aio;

    // from emcmotConfig
    tps->arcBlendGapCycles = &cfg->arcBlendGapCycles;
    tps->arcBlendOptDepth = &cfg->arcBlendOptDepth;
    tps->arcBlendEnable = &cfg->arcBlendEnable;
    tps->arcBlendRampFreq = &cfg->arcBlendRampFreq;
    tps->arcBlendTangentKinkRatio = &cfg->arcBlendTangentKinkRatio;
    tps->arcBlendFallbackEnable = &cfg->arcBlendFallbackEnable;
    tps->maxFeedScale = &cfg->maxFeedScale;

    // from emcmotStatus
    tps->net_feed_scale = &status->net_feed_scale;
    tps->spindle_direction = &status->spindle.direction;
    tps->spindle_speed = &status->spindle.speed;
    tps->spindleRevs = &status->spindleRevs;
    tps->spindleSpeedIn = &status->spindleSpeedIn;
    tps->spindle_index_enable = &status->spindle_index_enable;
    tps->spindle_is_atspeed = &status->spindle_is_atspeed; // or pin
    tps->spindleSync = &status->spindleSync;
    tps->current_vel = &status->current_vel;
    tps->requested_vel = &status->requested_vel;
    tps->distance_to_go = &status->distance_to_go;
    tps->enables_new = &status->enables_new;
    tps->enables_queued = &status->enables_queued;
    tps->tcqlen = &status->tcqlen;

    tps->dtg[0] = &status->dtg.tran.x;
    tps->dtg[1] = &status->dtg.tran.y;
    tps->dtg[2] = &status->dtg.tran.z;
    tps->dtg[3] = &status->dtg.a;
    tps->dtg[4] = &status->dtg.b;
    tps->dtg[5] = &status->dtg.c;
    tps->dtg[6] = &status->dtg.u;
    tps->dtg[7] = &status->dtg.v;
    tps->dtg[8] = &status->dtg.w;

    // from joints array
    tps->jerk_limit[0] = &joint[0].jerk_limit;
    tps->jerk_limit[1] = &joint[1].jerk_limit;
    tps->jerk_limit[2] = &joint[2].jerk_limit;

    tps->acc_limit[0] = &joint[0].acc_limit;
    tps->acc_limit[1] = &joint[1].acc_limit;
    tps->acc_limit[2] = &joint[2].acc_limit;

    tps->vel_limit[0] = &joint[0].vel_limit;
    tps->vel_limit[1] = &joint[1].vel_limit;
    tps->vel_limit[2] = &joint[2].vel_limit;

    // from  emcmot_debug_t
    tps->stepping = &dbg->stepping;

    // M6x pin setters
    tps->dioWrite = emcmotDioWrite;
    tps->aioWrite = emcmotAioWrite;

    // rotary setter/getters
    tps->SetRotaryUnlock = emcmotSetRotaryUnlock;
    tps->GetRotaryIsUnlocked = emcmotGetRotaryIsUnlocked;

    // RISC@FPGA signals
    tps->rtp_running = hal->rtp_running;        //!< for G38.X
    tps->probing = hal->probing;                //!< for G38.X
    tps->update_pos_req = hal->update_pos_req;  //!< for RISC-JOG, AHC, etc
    tps->SetUuPerRev = emcmotSetUuPerRev;

    return 0;
}
