/********************************************************************
* Description: motion.h
*   Data structures used throughout emc2.
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved
*
* Last change:
********************************************************************/

/* jmk says: This file is a mess! */

/*

Misc ramblings:

The terms axis and joint are used inconsistently throughout EMC.
For all new code, the usages are as follows:

    axis - one of the nine degrees of freedom, x, y, z, a, b, c, u, v, w
	these refer to axes in Cartesian space, which may or
	may not match up with joints (see below). On Cartesian
	machines they do match up, but for hexapods, robots, and
	other non-Cartesian machines they don't.
    joint - one of the physical degrees of freedom of the machine
	these might be linear (leadscrews) or rotary (rotary
	tables, robot arm joints).  There can be any number of
	joints.  The kinematics code is responsible for translating
	from axis space to joint space and back.

There are three main kinds of data needed by the motion controller

1) data shared with higher level stuff - commands, status, etc.
2) data that is local to the motion controller
3) data shared with lower level stuff - hal pins

In addition, some internal data (2) should be shared for trouble
shooting purposes, even though it is "internal" to the motion
controller.  Depending on the type of data, it can either be
treated as type (1), and made available to the higher level
code, or it can be treated as type (3), and made available to
the hal, so that halscope can monitor it.

This file should ONLY contain structures and declarations for
type (1) items - those that are shared with higher level code.

Type (2) items should be declared in mot_priv.h, along
with type (3) items.

In the interest of retaining my sanity, I'm not gonna attempt
to move everything to its proper location yet....

However, all new items will be defined in the proper place,
and some existing items may be moved from one struct definition
to another.

*/

/* the following line can be used to control where some of the
   "internal" motion controller data is stored.  By default,
   it is stored in staticlly allocated kernel memory.  However,
   if STRUCTS_IN_SHMEM is defined, it will be stored in the
   emcmotStruct shared memory area, for debugging purposes.
*/

// enable this for now just so we have a single copy of the joints struct
// not within emcmotdebug, since we'll have two of emcmotdebug
// eventually this should be separated out into emcmotinternal or so
// #define STRUCTS_IN_SHMEM

#define STRUCTS_IN_SHMEM


#ifndef MOTION_H
#define MOTION_H

#include "hal.h"
#include "posemath.h"		/* PmCartesian, PmPose, pmCartMag() */
#include "emcpos.h"		/* EmcPose */
#include "cubic.h"		/* CUBIC_STRUCT, CUBIC_COEFF */
#include "emcmotcfg.h"		/* EMCMOT_MAX_JOINTS */
#include "kinematics.h"
#include "simple_tp.h"
#include "rtapi_limits.h"
#include "motion_id.h"
#include "tp.h"
#include <stdarg.h>
#include "state_tag.h"

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

    enum pause_state {
        PS_RUNNING=0,  // aka 'not paused'
        PS_PAUSING=1,  // looking for the first pausable motion (e.g. not spindle-synced)
        PS_PAUSED=2,   // motion stopped, and ok to unpause
        PS_PAUSED_IN_OFFSET=3, // motion stopped, but not where we can return to primary queue
        PS_JOGGING=4,  // coord mode motion in progress
        PS_RETURNING=5, // executing a return move to return to running
        PS_PAUSING_FOR_STEP=6,
    };

    typedef struct _EMC_TELEOP_DATA {
	EmcPose currentVel;
	EmcPose currentAccel;
	EmcPose desiredVel;
	EmcPose desiredAccel;
    } EMC_TELEOP_DATA;

/* This enum lists all the possible commands */

    typedef enum {
    EMCMOT_ABORT = 1,              /* abort all motion */
    EMCMOT_AXIS_ABORT = 2,         /* abort one axis */ //FIXME-AJ: replace command name to EMCMOT_JOINT_ABORT
    EMCMOT_ENABLE = 3,             /* enable servos for active joints */
    EMCMOT_DISABLE = 4,            /* disable servos for active joints */
    EMCMOT_ENABLE_AMPLIFIER = 5,   /* enable amp outputs */
    EMCMOT_DISABLE_AMPLIFIER = 6,  /* disable amp outputs */
    EMCMOT_ENABLE_WATCHDOG = 7,    /* enable watchdog sound, parport */
    EMCMOT_DISABLE_WATCHDOG = 8,   /* enable watchdog sound, parport */
    EMCMOT_ACTIVATE_JOINT = 9,     /* make joint active */
    EMCMOT_DEACTIVATE_JOINT = 10,  /* make joint inactive */

    EMCMOT_PAUSE = 11,             /* pause motion */
    EMCMOT_RESUME = 12,            /* resume motion */
    EMCMOT_STEP = 13,              /* resume motion until id encountered */
    EMCMOT_FREE = 14,              /* set mode to free (joint) motion */
    EMCMOT_COORD = 15,             /* set mode to coordinated motion */
    EMCMOT_TELEOP = 16,            /* set mode to teleop */

    EMCMOT_SPINDLE_SCALE = 17,     /* set scale factor for spindle speed */
    EMCMOT_SS_ENABLE = 18,         /* enable/disable scaling the spindle speed */
    EMCMOT_FEED_SCALE = 19,        /* set scale factor for feedrate */
    EMCMOT_FS_ENABLE = 20,         /* enable/disable scaling feedrate */
    EMCMOT_FH_ENABLE = 21,         /* enable/disable feed_hold */
    EMCMOT_AF_ENABLE = 22,         /* enable/disable adaptive feedrate */
    EMCMOT_OVERRIDE_LIMITS = 23,   /* temporarily ignore limits until jog done */

    EMCMOT_HOME = 24,              /* home a joint or all joints */
    EMCMOT_UNHOME = 25,            /* unhome a joint or all joints*/
    EMCMOT_JOG_CONT = 26,          /* continuous jog */
    EMCMOT_JOG_INCR = 27,          /* incremental jog */
    EMCMOT_JOG_ABS = 28,           /* absolute jog */
    EMCMOT_SET_LINE = 29,          /* queue up a linear move */
    EMCMOT_SET_CIRCLE = 30,        /* queue up a circular move */
    EMCMOT_SET_TELEOP_VECTOR = 31, /* Move at a given velocity but in
                                      world Cartesian coordinates, not
                                      in joint space like EMCMOT_JOG_* */

    EMCMOT_CLEAR_PROBE_FLAGS = 32, /* clears probeTripped flag */
    EMCMOT_PROBE = 33,             /* go to pos, stop if probe trips, record
                                      trip pos */

    EMCMOT_SPINDLE_SYNC_MOTION = 34,  /* CSS,
                                       * RIGID_TAP: go to pos, with sync to spindle speed,
                                       *            then return to initial pos
                                       */

    EMCMOT_SET_POSITION_LIMITS = 35,  /* set the joint position +/- limits */
    EMCMOT_SET_BACKLASH = 36,         /* set the joint backlash */
    EMCMOT_SET_MIN_FERROR = 37,       /* minimum following error, input units */
    EMCMOT_SET_MAX_FERROR = 38,       /* maximum following error, input units */
    EMCMOT_SET_VEL = 39,              /* set the velocity for subsequent moves */
    EMCMOT_SET_VEL_LIMIT = 40,        /* set the max vel for all moves (tooltip) */
    EMCMOT_SET_JOINT_VEL_LIMIT = 41,  /* set the max joint vel */
    EMCMOT_SET_JOINT_ACC_LIMIT = 42,  /* set the max joint accel */
    EMCMOT_SET_ACC = 43,              /* set the max accel for moves (tooltip) */
    EMCMOT_SET_TERM_COND = 44,        /* set termination condition (stop, blend) */
    EMCMOT_SET_NUM_AXES = 45,         /* set the number of joints */ //FIXME-AJ: function needs to get renamed
    EMCMOT_SET_WORLD_HOME = 46,       /* set pose for world home */
    EMCMOT_SET_HOMING_PARAMS = 47,    /* sets joint homing parameters */
    EMCMOT_SET_DEBUG = 48,            /* sets the debug level */
    EMCMOT_SET_DOUT = 49,             /* sets or unsets a DIO, this can be imediate or synched with motion */
    EMCMOT_SET_AOUT = 50,             /* sets or unsets a AIO, this can be imediate or synched with motion */
    EMCMOT_SET_SPINDLESYNC = 51,      /* synchronize motion to spindle encoder */

    EMCMOT_SPINDLE_ON = 52,               /* start the spindle */
    EMCMOT_SPINDLE_OFF = 53,              /* stop the spindle */
    EMCMOT_SPINDLE_INCREASE = 54,         /* spindle faster */
    EMCMOT_SPINDLE_DECREASE = 55,         /* spindle slower */
    EMCMOT_SPINDLE_BRAKE_ENGAGE = 56,     /* engage the spindle brake */
    EMCMOT_SPINDLE_BRAKE_RELEASE = 57,    /* release the spindle brake */
    EMCMOT_SPINDLE_ORIENT = 58,           /* orient the spindle */
    EMCMOT_SET_MOTOR_OFFSET = 59,         /* set the offset between joint and motor */
    EMCMOT_SET_JOINT_COMP = 60,           /* set a compensation triplet for a joint (nominal, forw., rev.) */
    EMCMOT_SET_OFFSET = 61,               /* set tool offsets */
    EMCMOT_SET_MAX_FEED_OVERRIDE = 62,
    EMCMOT_SETUP_ARC_BLENDS = 63,
    EMCMOT_SET_JOINT_JERK_LIMIT,	  /* set the max joint jerk */
    EMCMOT_SET_AXIS_JERK_LIMIT,           /* set the max axis jerk */
    EMCMOT_SET_JERK,	                  /* set the max jerk for moves (tooltip) */
    EMCMOT_SETUP_USBMOT,                  /* setup the USB-FPGA motion control parameters */
    EMCMOT_END_PROBE,                     /* after probe*/
    } cmd_code_t;

/* this enum lists the possible results of a command */

    typedef enum {
	EMCMOT_COMMAND_OK = 0,	/* cmd honored */
	EMCMOT_COMMAND_UNKNOWN_COMMAND,	/* cmd not understood */
	EMCMOT_COMMAND_INVALID_COMMAND,	/* cmd can't be handled now */
	EMCMOT_COMMAND_INVALID_PARAMS,	/* bad cmd params */
	EMCMOT_COMMAND_BAD_EXEC	/* error trying to initiate */
    } cmd_status_t;

/* termination conditions for queued motions */
#define EMCMOT_TERM_COND_STOP 1
#define EMCMOT_TERM_COND_BLEND 2
#define EMCMOT_TERM_COND_TANGENT 3

/*********************************
       COMMAND STRUCTURE
*********************************/

/* This is the command structure.  There is one of these in shared
   memory, and all commands from higher level code come thru it.
*/
    typedef struct emcmot_command_t {
	unsigned char head;	/* flag count for mutex detect */
	cmd_code_t command;	/* command code (enum) */
	int commandNum;		/* increment this for new command */
	double motor_offset;    /* offset from joint to motor position */
	double maxLimit;	/* pos value for position limit, output */
	double minLimit;	/* neg value for position limit, output */
	EmcPose pos;		/* line/circle endpt, or teleop vector */
	PmCartesian center;	/* center for circle */
	PmCartesian normal;	/* normal vec for circle */
	int turn;		/* turns for circle or which rotary to unlock for a line */
	double vel;		/* max velocity */
        double ini_maxvel;      /* max velocity allowed by machine
                                   constraints (the ini file) */
        double ini_maxjerk;     /* max jerk allowed by machine
                                   constraints (the ini file) */
        int motion_type;        /* this move is because of traverse, feed, arc, or toolchange */
        double spindlesync;     /* user units per spindle revolution, 0 = no sync */
    	double acc;		/* max acceleration */
    	double jerk;		/* max jerk */
	double backlash;	/* amount of backlash */
	int id;			/* id for motion */
	int termCond;		/* termination condition */
	double tolerance;	/* tolerance for path deviation in CONTINUOUS mode */
	int axis;		/* which index to use for below */ //FIXME-AJ: replace with joint
	double scale;		/* velocity scale or spindle_speed scale arg */
	double offset;		/* input, output, or home offset arg */
	double home;		/* joint home position */
        double home_final_vel;  /* joint velocity for moving from OFFSET to HOME */
        double home_enc_pos;    /* joint initial encoder to HOME */
	double search_vel;	/* home search velocity */
	double latch_vel;	/* home latch velocity */
	int flags;		/* homing config flags, other boolean args */
	int home_sequence;      /* order in homing sequence */
        int volatile_home;      /* joint should get unhomed when we get unhome -2 
                                   (generated by task upon estop, etc) */
	double minFerror;	/* min following error */
	double maxFerror;	/* max following error */
	int wdWait;		/* cycle to wait before toggling wd */
	int debug;		/* debug level, from DEBUG in .ini file */
        unsigned int out; /* these are related to synched AOUT/DOUT. out = which gets set */
	unsigned char now, start, end;	/* now=wether now or synched, start=start value, end=end value */
	unsigned char mode;	/* used for turning overrides etc. on/off */
	double comp_nominal, comp_forward, comp_reverse; /* compensation triplet, nominal, forward, reverse */
        unsigned char probe_type; /* ~1 = error if probe operation is unsuccessful (ngc default)
                                     |1 = suppress error, report in # instead
                                     ~2 = move until probe trips (ngc default)
                                     |2 = move until probe clears */
        EmcPose tool_offset;        /* TLO */
	double  orientation;    /* angle for spindle orient */
	char    direction;      /* CANON_DIRECTION flag for spindle orient */
	double  timeout;        /* of wait for spindle orient to complete */
	unsigned char tail;	/* flag count for mutex detect */
        int arcBlendOptDepth;
        hal_bit_t arcBlendEnable;
        hal_bit_t arcBlendFallbackEnable;
        hal_s32_t arcBlendGapCycles;
        double arcBlendRampFreq;
        double arcBlendTangentKinkRatio;
        double maxFeedScale;
        struct state_tag_t tag;
        double dist;            /* distance for EMC_MOTION_TYPE_JOINT */

        int usbmotEnable;       /* enable USB-FPGA motion control device */
        int ssm_mode;           /* spindle sync motion mode: G33(0):CSS/THREADING, G33.1(1):RIGID_TAP */

        double css_factor;      /* for spindle */
        double xoffset;         /* for spindle, the x-offset to spindle-center */
        double yoffset;         /* for spindle, the y-offset to spindle-center */
    } emcmot_command_t;

/*! \todo FIXME - these packed bits might be replaced with chars
   memory is cheap, and being able to access them without those
   damn macros would be nice
*/

/* motion flag type */
    typedef unsigned short EMCMOT_MOTION_FLAG;

/*
  motion status flag structure-- looks like:

  MSB                             LSB
  v---------------v------------------v
  |   |   |   | T | CE | C | IP | EN |
  ^---------------^------------------^

  where:

  EN is 1 if calculations are enabled, 0 if not
  IP is 1 if all joints in position, 0 if not
  C is 1 if coordinated mode, 0 if in free mode
  CE is 1 if coordinated mode error, 0 if not
  T is 1 if we are in teleop mode.
  */

/* bit masks */
#define EMCMOT_MOTION_ENABLE_BIT      0x0001
#define EMCMOT_MOTION_INPOS_BIT       0x0002
#define EMCMOT_MOTION_COORD_BIT       0x0004
#define EMCMOT_MOTION_ERROR_BIT       0x0008
#define EMCMOT_MOTION_TELEOP_BIT      0x0010

/* joint flag type */
    typedef unsigned short EMCMOT_JOINT_FLAG;
/*
  joint status flag structure-- looks like:

  MSB                                                          LSB
  ----------v-----------------v--------------------v-------------------v
  | AF | FE | AH | HD | H | HS | NHL | PHL | - | - | ER | IP | AC | EN |
  ----------^-----------------^--------------------^-------------------^
               

  x = unused

  where:

  EN  is 1 if joint amplifier is enabled, 0 if not
  AC  is 1 if joint is active for calculations, 0 if not
  IP  is 1 if joint is in position, 0 if not (free mode only)
  ER  is 1 if joint has an error, 0 if not

  PHL is 1 if joint is on maximum hardware limit, 0 if not
  NHL is 1 if joint is on minimum hardware limit, 0 if not

  HS  is 1 if joint home switch is tripped, 0 if not
  H   is 1 if joint is homing, 0 if not
  HD  is 1 if joint has been homed, 0 if not
  AH  is 1 if joint is at home position, 0 if not

  FE  is 1 if joint exceeded following error, 0 if not
  AF  is 1 if amplifier is faulted, 0 if not

Suggestion: Split this in to an Error and a Status flag register..
             Then a simple test on each of the two flags can be performed
             rather than testing each bit... Saving on a global per joint
             fault and ready status flag.
  */

/* bit masks */
#define EMCMOT_JOINT_ENABLE_BIT         0x0001
#define EMCMOT_JOINT_ACTIVE_BIT         0x0002
#define EMCMOT_JOINT_INPOS_BIT          0x0004
#define EMCMOT_JOINT_ERROR_BIT          0x0008

#define EMCMOT_JOINT_MAX_HARD_LIMIT_BIT 0x0040
#define EMCMOT_JOINT_MIN_HARD_LIMIT_BIT 0x0080

#define EMCMOT_JOINT_HOME_SWITCH_BIT    0x0100
#define EMCMOT_JOINT_HOMING_BIT         0x0200
#define EMCMOT_JOINT_HOMED_BIT          0x0400

/*! \todo FIXME - I'm not sure AT_HOME is being reported correctly.
   AT_HOME is cleared when you jog in free mode, but not if
   you do a coordinated move... perhaps that is the intended
   behavior.
*/
#define EMCMOT_JOINT_AT_HOME_BIT        0x0800

#define EMCMOT_JOINT_FERROR_BIT         0x1000
#define EMCMOT_JOINT_FAULT_BIT          0x2000

/*! \todo FIXME - the terms "teleop", "coord", and "free" are poorly
   documented.  This is my feeble attempt to understand exactly
   what they mean.

   According to Fred, teleop is never used with machine tools,
   although that may not be true for machines with non-trivial
   kinematics.

   "coord", or coordinated mode, means that all the joints are
   synchronized, and move together as commanded by the higher
   level code.  It is the normal mode when machining.  In
   coordinated mode, commands are assumed to be in the cartesean
   reference frame, and if the machine is non-cartesean, the
   commands are translated by the kinematics to drive each
   joint in joint space as needed.

   "free" mode means commands are interpreted in joint space.
   It is used for jogging individual joints, although
   it does not preclude multiple joints moving at once (I think).
   Homing is also done in free mode, in fact machines with
   non-trivial kinematics must be homed before they can go
   into either coord or teleop mode.

   'teleop' is what you probably want if you are 'jogging'
   a hexapod.  The jog commands as implemented by the motion
   controller are joint jogs, which work in free mode.  But
   if you want to jog a hexapod or similar machine along
   one particular cartesean axis, you need to operate more
   than one joint.  That's what 'teleop' is for.

*/

/* compensation structures */
    typedef struct {
	double nominal;		/* nominal (command) position */
	float fwd_trim;		/* correction for forward movement */
	float rev_trim;		/* correction for reverse movement */
	float fwd_slope;	/* slopes between here and next pt */
	float rev_slope;
    } emcmot_comp_entry_t; 


#define EMCMOT_COMP_SIZE 256
    typedef struct {
	int entries;		/* number of entries in the array */
	emcmot_comp_entry_t *entry;  /* current entry in array */
	emcmot_comp_entry_t array[EMCMOT_COMP_SIZE+2];
	/* +2 because array has -HUGE_VAL and +HUGE_VAL entries at the ends */
    } emcmot_comp_t;

/* motion controller states */

    typedef enum {
	EMCMOT_MOTION_DISABLED = 0,
	EMCMOT_MOTION_FREE,
	EMCMOT_MOTION_TELEOP,
	EMCMOT_MOTION_COORD
    } motion_state_t;

/* states for homing */
    typedef enum {
	HOME_IDLE = 0,
	HOME_START,			// 1 
	HOME_UNLOCK,			// 2 
	HOME_UNLOCK_WAIT,		// 3 
	HOME_INITIAL_BACKOFF_START,	// 4 
	HOME_INITIAL_BACKOFF_WAIT,	// 5 
	HOME_INITIAL_SEARCH_START,	// 6 
	HOME_INITIAL_SEARCH_WAIT,	// 7 
	HOME_SET_COARSE_POSITION,	// 8 
	HOME_FINAL_BACKOFF_START,	// 9 
	HOME_FINAL_BACKOFF_WAIT,	// 10
	HOME_RISE_SEARCH_START,		// 11
	HOME_RISE_SEARCH_WAIT,		// 12
	HOME_FALL_SEARCH_START,		// 13
	HOME_FALL_SEARCH_WAIT,		// 14
	HOME_SET_SWITCH_POSITION,	// 15
	HOME_INDEX_ONLY_START,		// 16
	HOME_INDEX_SEARCH_START,	// 17
	HOME_INDEX_SEARCH_WAIT,		// 18
	HOME_SET_INDEX_POSITION,	// 19
	HOME_FINAL_MOVE_START,		// 20
	HOME_FINAL_MOVE_WAIT,		// 21
	HOME_LOCK,			// 22
	HOME_LOCK_WAIT,			// 23
	HOME_FINISHED,			// 24
	HOME_ABORT,			// 25
	HOME_INDEX_WAIT_GANTRY          // 26
    } home_state_t;

    typedef enum {
	HOME_SEQUENCE_IDLE = 0,
	HOME_SEQUENCE_START,
	HOME_SEQUENCE_START_JOINTS,
	HOME_SEQUENCE_WAIT_JOINTS,
    } home_sequence_state_t;

    typedef enum {
	EMCMOT_ORIENT_NONE = 0,
	EMCMOT_ORIENT_IN_PROGRESS,
	EMCMOT_ORIENT_COMPLETE,
	EMCMOT_ORIENT_FAULTED,
    } orient_state_t;

/* flags for homing */
#define HOME_IGNORE_LIMITS	1
#define HOME_USE_INDEX		2
#define HOME_IS_SHARED		4
#define HOME_UNLOCK_FIRST       8
/* flags for usb_homing */
#define HOME_GANTRY_MASTER      0x10    // home_flag[4] for gantry_master(1), gantry_slave(0)
#define HOME_GANTRY_JOINT       0x20    // home_flag[5] for gantry_master/gantry_slave:(1), others:(0)

/* flags for enabling spindle scaling, feed scaling,
   adaptive feed, and feed hold */

#define SS_ENABLED 0x01
#define FS_ENABLED 0x02
#define AF_ENABLED 0x04
#define FH_ENABLED 0x08

/* This structure contains all of the data associated with
   a single joint.  Note that this structure does not need
   to be in shared memory (but it can, if desired for debugging
   reasons).  The portions of this structure that are considered
   "status" and need to be made available to user space are
   copied to a much smaller struct called emcmot_joint_status_t
   which is located in shared memory.

*/
    typedef struct {

	/* configuration info - changes rarely */
	int type;		/* 0 = linear, 1 = rotary */
	double max_pos_limit;	/* upper soft limit on joint pos */
	double min_pos_limit;	/* lower soft limit on joint pos */
	double max_jog_limit;	/* jog limits change when not homed */
	double min_jog_limit;
	int    en_soft_limit;   /* set both max_pos_limit and min_pos_limit to 0 to disable software limit */
	double vel_limit;	/* upper limit of joint speed */
	double acc_limit;	/* upper limit of joint accel */
	double jerk_limit;	/* upper limit of joint jerk */
	double min_ferror;	/* zero speed following error limit */
	double max_ferror;	/* max speed following error limit */
	double home_search_vel;	/* dir/spd to look for home switch */
        double home_final_vel;  /* speed to travel from OFFSET to HOME position */
        double home_enc_pos;    /* initial encoder to HOME position */
	double home_latch_vel;	/* dir/spd to latch switch/index pulse */
	hal_float_t *home_offset;	/* dir/dist from switch to home point */
	hal_float_t *home;	/* joint coordinate of home point */
	int home_flags;		/* flags for various homing options */
	int volatile_home;      /* joint should get unhomed when we get unhome -2 
                                   (generated by task upon estop, etc) */
	double backlash;	/* amount of backlash */
	int home_sequence;      /* Order in homing sequence */
	emcmot_comp_t comp;	/* leadscrew correction data */

	/* status info - changes regularly */
	/* many of these need to be made available to higher levels */
	/* they can either be copied to the status struct, or an array of
	   joint structs can be made part of the status */
	EMCMOT_JOINT_FLAG flag;	/* see above for bit details */
	double coarse_pos;	/* trajectory point, before interp */
	double pos_cmd;		/* commanded joint position */
	double vel_cmd;		/* comanded joint velocity */
	double backlash_corr;	/* correction for backlash */
	double backlash_filt;	/* filtered backlash correction */
	double backlash_vel;	/* backlash velocity variable */
	double motor_pos_cmd;	/* commanded position, with comp */
        double motor_pos_fb;    /* position feedback, with comp */
	double pos_fb;		/* position feedback, comp removed */
        double risc_pos_cmd;    /* position command issued by RISC */
	double ferror;		/* following error */
	double ferror_limit;	/* limit depends on speed */
	double ferror_high_mark;	/* max following error */
	double free_pos_cmd;	/* position command for free mode TP */
	double free_vel_lim;	/* velocity limit for free mode TP */
	int free_tp_enable;	/* if zero, joint stops ASAP */
	int free_tp_active;	/* if non-zero, move in progress */
	int kb_jog_active;	/* non-zero during a keyboard jog */
	int wheel_jog_active;	/* non-zero during a wheel jog */

	/* internal info - changes regularly, not usually accessed from user
	   space */
	CUBIC_STRUCT cubic;	/* cubic interpolator data */

	int on_pos_limit;	/* non-zero if on limit */
	int on_neg_limit;	/* non-zero if on limit */
	double home_sw_pos;	/* latched position of home sw */
	int home_pause_timer;	/* used to delay between homing states */
	int index_enable;	/* current state of index enable pin */

	home_state_t home_state;	/* state machine for homing */
	double motor_offset;	/* diff between internal and motor pos, used
				   to set position to zero during homing */
	int old_jog_counts;	/* prior value, used for deltas */

	/* stuff moved from the other structs that might be needed (or might
	   not!) */
	double big_vel;		/* used for "debouncing" velocity */
        
        double  probed_pos;
        
        /**
         * for usb_homing.c
         **/
        home_state_t prev_home_state;   /* state machine for homing */
        double  risc_probe_vel; 	/* velocity for RISC probing */
        double  risc_probe_dist;
        int     risc_probe_pin;
        int     risc_probe_type;
        int     home_sw_id;
        hal_u32_t *joint_id;            /* joint_id that maps to this axis */

	double  blender_offset;         /* offset created by realtime component, blender.comp */
    } emcmot_joint_t;

/* This structure contains only the "status" data associated with
   a joint.  "Status" data is that data that should be reported to
   user space on a continuous basis.  An array of these structs is
   part of the main status structure, and is filled in with data
   copied from the emcmot_joint_t structs every servo period.

   For now this struct contains more data than it really needs, but
   paring it down will take time (and probably needs to be done one
   or two items at a time, with much testing).  My main goal right
   now is to get get the large joint struct out of status.

*/
    typedef struct {

	EMCMOT_JOINT_FLAG flag;	/* see above for bit details */
	double pos_cmd;		/* commanded joint position */
	double pos_fb;		/* position feedback, comp removed */
	double vel_cmd;         /* current velocity */
	double ferror;		/* following error */
	double ferror_high_mark;	/* max following error */

/*! \todo FIXME - the following are not really "status", but taskintf.cc expects
   them to be in the status structure.  I don't know how or if they are
   used by the user space code.  Ideally they will be removed from here,
   but each one will need to be investigated individually.
*/
	double backlash;	/* amount of backlash */
	double max_pos_limit;	/* upper soft limit on joint pos */
	double min_pos_limit;	/* lower soft limit on joint pos */
	double min_ferror;	/* zero speed following error limit */
	double max_ferror;	/* max speed following error limit */
	double home_offset;	/* dir/dist from switch to home point */
    } emcmot_joint_status_t;


    typedef struct {
	hal_float_t speed;	// spindle speed in RPMs
        hal_float_t curr_vel_rps; // current speed in RPS
        hal_float_t css_error;  // the velocity error for CSS motion
	double css_factor;
	double xoffset;         // x-offset to spindle-center
	double yoffset;         // y-offset to spindle-center
	hal_s32_t direction;	// 0 stopped, 1 forward, -1 reverse
	int brake;		// 0 released, 1 engaged
	int locked;             // spindle lock engaged after orient
	int orient_fault;       // fault code from motion.spindle-orient-fault
	int orient_state;       // orient_state_t
    } spindle_status;
    

/*********************************
        STATUS STRUCTURE
*********************************/

/* This is the status structure.  There is one of these in shared
   memory, and it reports motion controller status to higher level
   code in user space.  For the most part, this structure contains
   higher level variables - low level stuff is made visible to the
   HAL and troubleshooting, etc, is done using the HAL oscilliscope.
*/

/*! \todo FIXME - this struct is broken into two parts... at the top are
   structure members that I understand, and that are needed for emc2.
   Other structure members follow.  All the later ones need to be
   evaluated - either they move up, or they go away.
*/

    typedef struct emcmot_status_t {
        uint32_t wait_risc;
        uint32_t update_pos_ack;    /* for RCMD_FSM inside RISC */
        uint32_t update_pos_req;    /* for RCMD_FSM inside RISC */

        unsigned char head;	/* flag count for mutex detect */
	/* these three are updated only when a new command is handled */
	cmd_code_t commandEcho;	/* echo of input command */
	int commandNumEcho;	/* echo of input command number */
	cmd_status_t commandStatus;	/* result of most recent command */
	/* these are config info, updated when a command changes them */
	double feed_scale;	/* velocity scale factor for all motion */
	double spindle_scale;	/* velocity scale factor for spindle speed */
	hal_u32_t enables_new;	/* flags for FS, SS, etc */
		/* the above set is the enables in effect for new moves */
	/* the rest are updated every cycle */
	double net_feed_scale;	/* net scale factor for all motion */
	double net_spindle_scale;	/* net scale factor for spindle */
	hal_u32_t enables_queued;	/* flags for FS, SS, etc */
		/* the above set is the enables in effect for the
		   currently executing move */
	motion_state_t motion_state; /* operating state: FREE, COORD, etc. */
	EMCMOT_MOTION_FLAG motionFlag;	/* see above for bit details */
	EmcPose carte_pos_cmd;	/* commanded Cartesian position */
	int carte_pos_cmd_ok;	/* non-zero if command is valid */
	EmcPose carte_pos_fb;	/* actual Cartesian position */
	int carte_pos_fb_ok;	/* non-zero if feedback is valid */
	EmcPose world_home;	/* cartesean coords of home position */
	int homing_active;	/* non-zero if any joint is homing */
	home_sequence_state_t homingSequenceState;
	emcmot_joint_status_t joint_status[EMCMOT_MAX_JOINTS];	/* all joint status data */

	int on_soft_limit;	/* non-zero if any joint is on soft limit */

	int probeVal;		/* debounced value of probe input */

	int probeTripped;	/* Has the probe signal changed since start
				   of probe command? */
	int probing;		/* Currently looking for a probe signal? */
        unsigned char probe_type;
	EmcPose probedPos;	/* Axis positions stored as soon as possible
				   after last probeTripped */
        hal_bit_t spindle_index_enable;  /* hooked to a canon encoder index-enable */
        hal_bit_t spindleSync;        /* we are doing spindle-synced motion */
        hal_float_t spindleRevs;     /* position of spindle in revolutions */
        hal_float_t spindleSpeedIn;  /* velocity of spindle in revolutions per minute */

	spindle_status spindle;	/* data types for spindle status */
	
	int synch_di[EMCMOT_MAX_DIO]; /* inputs to the motion controller, queried by g-code */
	int synch_do[EMCMOT_MAX_DIO]; /* outputs to the motion controller, queried by g-code */
	double analog_input[EMCMOT_MAX_AIO]; /* inputs to the motion controller, queried by g-code */
	double analog_output[EMCMOT_MAX_AIO]; /* outputs to the motion controller, queried by g-code */

        struct state_tag_t tag; /* Current interp state corresponding to motion line */

/*! \todo FIXME - all structure members beyond this point are in limbo */

	/* dynamic status-- changes every cycle */
	unsigned int heartbeat;
	int config_num;		/* incremented whenever configuration
				   changed. */
	double computeTime;
	int id;			/* id for executing motion */
	int depth;		/* motion queue depth */
	int activeDepth;	/* depth of active blend elements */
	int queueFull;		/* Flag to indicate the tc queue is full */
	int pause_state;	/* state of the motion pause FSM */
	int tp_reverse_input;   //!< the hal input signal for tp_reversed mode
	int resuming;	        /* resume operation in progress */
	int overrideLimitMask;	/* non-zero means one or more limits ignored */
				/* 1 << (joint-num*2) = ignore neg limit */
				/* 2 << (joint-num*2) = ignore pos limit */

	/* static status-- only changes upon input commands, e.g., config */
	double vel;		/* scalar max vel */
	double acc;		/* scalar max accel */
	double jerk;	        /* scalar max jerk */
        int32_t accelState;     /* s-curve acceleration state */

	int level;
        int motionType;         /* line, arc, tap, ... */
        double distance_to_go;  /* in this move */
        double prim_dtg;        /* distance_to_go of emcmotPrimQueue */
        double prim_progress;   //!< progress of current-tc of emcmotPrimQueue
        EmcPose dtg;
        double current_vel;
        double requested_vel;

        hal_u32_t tcqlen;
        EmcPose tool_offset;
        int atspeed_next_feed;  /* at next feed move, wait for spindle to be at speed  */
        hal_bit_t spindle_is_atspeed; /* hal input */

	EmcPose pause_carte_pos;	// initial pause point (ipp) - where we switched to the altQueue
	EmcPose pause_offset_carte_pos;	// ipp + current offset values, set by update_offset_pose()
	int current_request;    // one of enum pause_request

	unsigned char tail;	/* flag count for mutex detect */

    } emcmot_status_t;

    enum pause_request { REQ_NONE,
			 REQ_STEP,
			 REQ_RESUME,
			 REQ_PAUSE
    };


/*********************************
        CONFIG STRUCTURE
*********************************/

/* This is the config structure.  This is currently in shared memory,
   but I have no idea why... there are commands to set most of the
   items in this structure.  It seems we should either put the struct
   in private memory and manipulate it with commands, or we should
   put it in shared memory and manipulate it directly - not both.
   The structure contains static or rarely changed information that
   describes the machine configuration.

   later: I think I get it now - the struct is in shared memory so
   user space can read the config at any time, but commands are used
   to change the config so they only take effect when the realtime
   code processes the command.
*/

/*! \todo FIXME - this struct is broken into two parts... at the top are
   structure members that I understand, and that are needed for emc2.
   Other structure members follow.  All the later ones need to be
   evaluated - either they move up, or they go away.
*/
    typedef struct emcmot_config_t {
	unsigned char head;	/* flag count for mutex detect */

/*! \todo FIXME - all structure members beyond this point are in limbo */

	int config_num;		/* Incremented everytime configuration
				   changed, should match status.config_num */
	int numJoints;		/* The number of joints in the system (which
				   must be between 1 and EMCMOT_MAX_JOINTS,
				   inclusive). Allegedly, holds a copy of the
				   global num_joints - seems daft to maintain
				   duplicates ! */
	int spindleAxis;        /* the axis that maps as spindle */
	double trajCycleTime;	/* the rate at which the trajectory loop
				   runs.... (maybe) */
	double servoCycleTime;	/* the rate of the servo loop - Not the same
				   as the traj time */

	int interpolationRate;	/* grep control.c for an explanation....
				   approx line 50 */

	double limitVel;	/* scalar upper limit on vel */
	KINEMATICS_TYPE kinematics_type;
	vtkins_t *vtk;          // pointer to kinematics vtable
	vtp_t    *vtp;          // pointer to tp vtable
	int kins_vid;           // HAL id of kins vtable
	int tp_vid;             // HAL id of tp vtable
	int debug;		/* copy of DEBUG, from .ini file */
	unsigned char tail;	/* flag count for mutex detect */
        hal_s32_t arcBlendOptDepth;
        hal_bit_t arcBlendEnable;
        hal_bit_t arcBlendFallbackEnable;
        hal_s32_t arcBlendGapCycles;
        double arcBlendRampFreq;
        double arcBlendTangentKinkRatio;
        double maxFeedScale;
        int usbmotEnable;       /* enable usb based motion control device */
    } emcmot_config_t;

/*********************************
      INTERNAL STRUCTURE
*********************************/

/* This is the internal structure.  It contains stuff that is used
   internally by the motion controller that does not need to be in
   shared memory.  It will wind up with a lot of the stuff that got
   tossed into the debug structure.

   FIXME - so far most if the stuff that was tossed in here got
   moved back out, maybe don't need it after all?
*/

    typedef struct emcmot_internal_t {
	unsigned char head;	/* flag count for mutex detect */

	int probe_debounce_cntr;
	unsigned char tail;	/* flag count for mutex detect */
    } emcmot_internal_t;

/* error structure - A ring buffer used to pass formatted printf stings to usr space */
    typedef struct emcmot_error_t {
	unsigned char head;	/* flag count for mutex detect */
	char error[EMCMOT_ERROR_NUM][EMCMOT_ERROR_LEN];
	int start;		/* index of oldest error */
	int end;		/* index of newest error */
	int num;		/* number of items */
	unsigned char tail;	/* flag count for mutex detect */
    } emcmot_error_t;
    
    enum tp_dir_state { TP_FORWARD=0,
                        TP_REVERSE=1
    };

/*
  function prototypes for emcmot code
*/

/* error ring buffer access functions */
    extern int emcmotErrorInit(emcmot_error_t * errlog);
    extern int emcmotErrorPut(emcmot_error_t * errlog, const char *error);
    extern int emcmotErrorPutfv(emcmot_error_t * errlog, const char *fmt, va_list ap);
    extern int emcmotErrorPutf(emcmot_error_t * errlog, const char *fmt, ...);
    extern int emcmotErrorGet(emcmot_error_t * errlog, char *error);

    int setTrajCycleTime(double secs);
    int setServoCycleTime(double secs);

#ifdef __cplusplus
}
#endif
#endif	/* MOTION_H */
