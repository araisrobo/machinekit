/********************************************************************
* Description: taskintf.cc
*   Interface functions for motion.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/

#include "rtapi_math.h"		// isnan()
#include <float.h>		// DBL_MAX
#include <string.h>		// memcpy() strncpy()
#include <unistd.h>             // unlink()

#include "usrmotintf.h"		// usrmotInit(), usrmotReadEmcmotStatus(),
				// etc.
#include "motion.h"		// emcmot_command_t,STATUS, etc.
#include "motion_debug.h"
#include "emc.hh"
#include "emcglb.h"		// EMC_INIFILE
#include "emc_nml.hh"
#include "rcs_print.hh"
#include "timer.hh"
#include "inifile.hh"
#include "iniaxis.hh"
#include "initraj.hh"
#include "inihal.hh"

value_inihal_data old_inihal_data;

/* define this to catch isnan errors, for rtlinux FPU register 
   problem testing */
#define ISNAN_TRAP

// MOTION INTERFACE

/*! \todo FIXME - this decl was originally much later in the file, moved
here temporarily for debugging */
static emcmot_status_t emcmotStatus;

/*
  Implementation notes:

  Initing:  the emcmot interface needs to be inited once, but nml_traj_init()
  and nml_servo_init() can be called in any order. Similarly, the emcmot
  interface needs to be exited once, but nml_traj_exit() and nml_servo_exit()
  can be called in any order. They can also be called multiple times. Flags
  are used to signify if initing has been done, or if the final exit has
  been called.
  */

static emcmot_command_t emcmotCommand;

static int emcmotTrajInited = 0;	// non-zero means traj called init
/* FIXME is this supposed to be axes, or joints? */
static int emcmotAxisInited[EMCMOT_MAX_JOINTS] = { 0 };	// non-zero means axis called init

__attribute__ ((unused))
static int emcmotIoInited = 0;	// non-zero means io called init
static int emcmotion_initialized = 0;	// non-zero means both
						// emcMotionInit called.

// EMC_AXIS functions

// local status data, not provided by emcmot
static unsigned long localMotionHeartbeat = 0;
static int localMotionCommandType = 0;
static int localMotionEchoSerialNumber = 0;

/* FIXME axes or joints? */
static unsigned char localEmcAxisAxisType[EMCMOT_MAX_JOINTS];
static double localEmcAxisUnits[EMCMOT_MAX_JOINTS];
static double localEmcMaxAcceleration = DBL_MAX;
static double localEmcMaxJerk = DBL_MAX;

// axes are numbered 0..NUM-1

/*
  In emcmot, we need to set the cycle time for traj, and the interpolation
  rate, in any order, but both need to be done. 
 */
/* FIXME: all of this stuff is really JOINTS not AXES!!! */

int emcAxisSetAxis(int axis, unsigned char axisType)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    localEmcAxisAxisType[axis] = axisType;

    return 0;
}

int emcAxisSetUnits(int axis, double units)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    localEmcAxisUnits[axis] = units;

    return 0;
}

int emcAxisSetBacklash(int axis, double backlash, double input_scale)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(backlash)) {
	printf("isnan error for backlash in emcAxisSetBacklash()\n");
	return -1;
    }
    if (rtapi_isnan(input_scale)) {
        printf("isnan error for input_scale in emcAxisSetBacklash()\n");
        return -1;
    }
#endif

    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_BACKLASH;
    emcmotCommand.axis = axis;
    emcmotCommand.backlash = backlash;
    emcmotCommand.scale = input_scale;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

// saved values of limits, since emcmot expects them to be set in
// pairs and we set them individually.
static double saveMinLimit[EMCMOT_MAX_JOINTS];
static double saveMaxLimit[EMCMOT_MAX_JOINTS];

int emcAxisSetMinPositionLimit(int axis, double limit)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(limit)) {
	printf("isnan error in emcAxisSetMinPosition()\n");
	return -1;
    }
#endif

    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_POSITION_LIMITS;
    emcmotCommand.axis = axis;
    emcmotCommand.maxLimit = saveMaxLimit[axis];
    emcmotCommand.minLimit = limit;
    saveMinLimit[axis] = limit;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetMaxPositionLimit(int axis, double limit)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(limit)) {
	printf("isnan error in emcAxisSetMaxPosition()\n");
	return -1;
    }
#endif

    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_POSITION_LIMITS;
    emcmotCommand.axis = axis;
    emcmotCommand.minLimit = saveMinLimit[axis];
    emcmotCommand.maxLimit = limit;
    saveMaxLimit[axis] = limit;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetMotorOffset(int axis, double offset) 
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(offset)) {
	printf("isnan error in emcAxisSetMotorOffset()\n");
	return -1;
    }
#endif

    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    emcmotCommand.command = EMCMOT_SET_MOTOR_OFFSET;
    emcmotCommand.axis = axis;
    emcmotCommand.motor_offset = offset;
    
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetFerror(int axis, double ferror)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(ferror)) {
	printf("isnan error in emcAxisSetFerror()\n");
	return -1;
    }
#endif

    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_MAX_FERROR;
    emcmotCommand.axis = axis;
    emcmotCommand.maxFerror = ferror;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetMinFerror(int axis, double ferror)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(ferror)) {
	printf("isnan error in emcAxisSetMinFerror()\n");
	return -1;
    }
#endif

    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    emcmotCommand.command = EMCMOT_SET_MIN_FERROR;
    emcmotCommand.axis = axis;
    emcmotCommand.minFerror = ferror;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetHomingParams(int axis, double home, double offset, double home_final_vel,
                            double home_enc_pos, double search_vel, double latch_vel,
                            int use_index, int ignore_limits, int is_shared,
                            int sequence,int volatile_home, int locking_indexer)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(home) || rtapi_isnan(offset) || rtapi_isnan(home_final_vel) ||
	rtapi_isnan(search_vel) || rtapi_isnan(latch_vel) || rtapi_isnan(home_enc_pos)) {
	printf("isnan error in emcAxisSetHoming()\n");
	return -1;
    }
#endif

    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_HOMING_PARAMS;
    emcmotCommand.axis = axis;
    emcmotCommand.home = home;
    emcmotCommand.offset = offset;
    emcmotCommand.home_final_vel = home_final_vel;
    emcmotCommand.home_enc_pos = home_enc_pos;
    emcmotCommand.search_vel = search_vel;
    emcmotCommand.latch_vel = latch_vel;
    emcmotCommand.flags = 0;
    emcmotCommand.home_sequence = sequence;
    emcmotCommand.volatile_home = volatile_home;
    if (use_index) {
	emcmotCommand.flags |= HOME_USE_INDEX;
    }
    if (ignore_limits) {
	emcmotCommand.flags |= HOME_IGNORE_LIMITS;
    }
    if (is_shared) {
	emcmotCommand.flags |= HOME_IS_SHARED;
    }
    if (locking_indexer) {
        emcmotCommand.flags |= HOME_UNLOCK_FIRST;
    }
    
    if (axis_sync_id[axis] >= 0) {
        if (axis < axis_sync_id[axis]) {
            // the axis with lower SYNC_ID is GANTRY_MASTER
            emcmotCommand.flags |= HOME_GANTRY_MASTER;
            emcmotCommand.flags |= HOME_GANTRY_JOINT;
        } else {
            // the axis with greater SYNC_ID is GANTRY_SLAVE
            emcmotCommand.flags |= HOME_GANTRY_JOINT;
        }
    }

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetMaxVelocity(int axis, double vel)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    if (vel < 0.0) {
	vel = 0.0;
    }

    axis_max_velocity[axis] = vel;

    emcmotCommand.command = EMCMOT_SET_JOINT_VEL_LIMIT;
    emcmotCommand.axis = axis;
    emcmotCommand.vel = vel;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetMaxAcceleration(int axis, double acc)
{

    if (axis < 0 || axis >= EMC_AXIS_MAX) {
	return 0;
    }
    if (acc < 0.0) {
	acc = 0.0;
    }
    axis_max_acceleration[axis] = acc;
    emcmotCommand.command = EMCMOT_SET_JOINT_ACC_LIMIT;
    emcmotCommand.axis = axis;
    emcmotCommand.acc = acc;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetMaxJerk(int axis, double jerk)
{

    if (axis < 0 || axis >= EMC_AXIS_MAX) {
	return 0;
    }
    if (jerk < 0.0) {
    	jerk = 0.0;
    }
    axis_max_jerk[axis] = jerk;
    emcmotCommand.command = EMCMOT_SET_JOINT_JERK_LIMIT;
    emcmotCommand.axis = axis;
    emcmotCommand.jerk = jerk;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisSetSyncId(int axis, int sync_id)
{
    if (axis < 0 || axis >= EMC_AXIS_MAX) {
        rcs_print_error("%s (%s:%d) Invalid axis(%d)\n", __FILE__, __FUNCTION__, __LINE__,
                        axis);
        return -1;
    }
    if ((sync_id == axis) || (sync_id >= EMC_AXIS_MAX)) {
        rcs_print_error("%s (%s:%d) Invalid sync_id(%d) for axis(%d)\n", __FILE__, __FUNCTION__, __LINE__,
                        sync_id, axis);
        return -1;
    }

    if (sync_id < 0) {
        /* invalid sync_id, disable synchronized motion for this axis */
        axis_sync_en[axis] = 0;
        axis_sync_id[axis] = -1;
    } else {
        /* enable and specify synchronized axis */
        axis_sync_en[axis] = 1;
        axis_sync_id[axis] = sync_id;
    }

    return 0;
}

/* This function checks to see if any axis or the traj has
   been inited already.  At startup, if none have been inited,
   usrmotIniLoad and usrmotInit must be called first.  At
   shutdown, after all have been halted, the usrmotExit must
   be called.
*/

static int AxisOrTrajInited(void)
{
    int axis;

    for (axis = 0; axis < EMCMOT_MAX_JOINTS; axis++) {
	if (emcmotAxisInited[axis]) {
	    return 1;
	}
    }
    if (emcmotTrajInited) {
	return 1;
    }
    return 0;
}

int emcAxisInit(int axis)
{
    int retval = 0;
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    // init emcmot interface
    if (!AxisOrTrajInited()) {
	usrmotIniLoad(emc_inifile);
	if (0 != usrmotInit("emc2_task")) {
	    return -1;
	}
    }
    emcmotAxisInited[axis] = 1;
    if (0 != iniAxis(axis, emc_inifile)) {
	retval = -1;
    }
    return retval;
}

int emcAxisHalt(int axis)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    /*! \todo FIXME-- refs global emcStatus; should make EMC_AXIS_STAT an arg here */
    if (NULL != emcStatus && emcmotion_initialized
	&& emcmotAxisInited[axis]) {
	dumpAxis(axis, emc_inifile, &emcStatus->motion.axis[axis]);
    }
    emcmotAxisInited[axis] = 0;

    if (!AxisOrTrajInited()) {
	usrmotExit();		// ours is final exit
    }

    return 0;
}

int emcAxisAbort(int axis)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    if (axis_sync_en[axis] && (emcmotStatus.motion_state == EMCMOT_MOTION_FREE)) {
        // synchronize jog for gantry joint
        emcmotCommand.command = EMCMOT_AXIS_ABORT;
        emcmotCommand.axis = axis_sync_id[axis];
        usrmotWriteEmcmotCommand(&emcmotCommand);
    }

    emcmotCommand.command = EMCMOT_AXIS_ABORT;
    emcmotCommand.axis = axis;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisActivate(int axis)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_ACTIVATE_JOINT;
    emcmotCommand.axis = axis;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisDeactivate(int axis)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_DEACTIVATE_JOINT;
    emcmotCommand.axis = axis;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisOverrideLimits(int axis)
{
    // can have axis < 0, for resuming normal limit checking
    if (axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_OVERRIDE_LIMITS;
    emcmotCommand.axis = axis;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisEnable(int axis)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_ENABLE_AMPLIFIER;
    emcmotCommand.axis = axis;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisDisable(int axis)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_DISABLE_AMPLIFIER;
    emcmotCommand.axis = axis;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisHome(int axis)
{
    if (axis < -1 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_HOME;
    emcmotCommand.axis = axis;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisUnhome(int axis)
{
	if (axis < -2 || axis >= EMCMOT_MAX_JOINTS) {
		return 0;
	}

	emcmotCommand.command = EMCMOT_UNHOME;
	emcmotCommand.axis = axis;

	return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisJog(int axis, double vel)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    if (vel > axis_max_velocity[axis]) {
	vel = axis_max_velocity[axis];
    } else if (vel < -axis_max_velocity[axis]) {
	vel = -axis_max_velocity[axis];
    }

    if (axis_sync_en[axis] && (emcmotStatus.motion_state == EMCMOT_MOTION_FREE)) {
        // synchronize jog for gantry joint
        emcmotCommand.command = EMCMOT_JOG_CONT;
        emcmotCommand.axis = axis_sync_id[axis];
        emcmotCommand.vel = vel;
        usrmotWriteEmcmotCommand(&emcmotCommand);
    }

    emcmotCommand.command = EMCMOT_JOG_CONT;
    emcmotCommand.axis = axis;
    emcmotCommand.vel = vel;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisIncrJog(int axis, double incr, double vel)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    if (vel > axis_max_velocity[axis]) {
	vel = axis_max_velocity[axis];
    } else if (vel < -axis_max_velocity[axis]) {
	vel = -axis_max_velocity[axis];
    }
    
    if (axis_sync_en[axis] && (emcmotStatus.motion_state == EMCMOT_MOTION_FREE)) {
        // synchronize jog for gantry joint
        emcmotCommand.command = EMCMOT_JOG_INCR;
        emcmotCommand.axis = axis_sync_id[axis];
        emcmotCommand.vel = vel;
        emcmotCommand.offset = incr;
        usrmotWriteEmcmotCommand(&emcmotCommand);
    }

    emcmotCommand.command = EMCMOT_JOG_INCR;
    emcmotCommand.axis = axis;
    emcmotCommand.vel = vel;
    emcmotCommand.offset = incr;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisAbsJog(int axis, double pos, double vel)
{
    if (axis < 0 || axis >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    if (vel > axis_max_velocity[axis]) {
	vel = axis_max_velocity[axis];
    } else if (vel < -axis_max_velocity[axis]) {
	vel = -axis_max_velocity[axis];
    }

    if (axis_sync_en[axis] && (emcmotStatus.motion_state == EMCMOT_MOTION_FREE)) {
        // synchronize jog for gantry joint
        emcmotCommand.command = EMCMOT_JOG_ABS;
        emcmotCommand.axis = axis_sync_id[axis];
        emcmotCommand.vel = vel;
        emcmotCommand.offset = pos;
        usrmotWriteEmcmotCommand(&emcmotCommand);
    }

    emcmotCommand.command = EMCMOT_JOG_ABS;
    emcmotCommand.axis = axis;
    emcmotCommand.vel = vel;
    emcmotCommand.offset = pos;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcAxisLoadComp(int axis, const char *file, int type)
{
    return usrmotLoadComp(axis, file, type);
}

static emcmot_config_t emcmotConfig;
int get_emcmot_debug_info = 0;

/*
  these globals are set in emcMotionUpdate(), then referenced in
  emcAxisUpdate(), emcTrajUpdate() to save calls to usrmotReadEmcmotStatus
 */
static emcmot_debug_t emcmotDebug;
static char errorString[EMCMOT_ERROR_LEN];
static int new_config = 0;

/*! \todo FIXME - debugging - uncomment the following line to log changes in
   AXIS_FLAG */
// #define WATCH_FLAGS 1

int emcAxisUpdate(EMC_AXIS_STAT stat[], int numAxes)
{
/*! \todo FIXME - this function accesses data that has been
   moved.  Once I know what it is used for I'll fix it */

    int axis;
    emcmot_joint_status_t *joint;
#ifdef WATCH_FLAGS
    static int old_joint_flag[8];
#endif

    // check for valid range
    if (numAxes <= 0 || numAxes > EMCMOT_MAX_JOINTS) {
	return -1;
    }

    for (axis = 0; axis < numAxes; axis++) {
	/* point to joint data */

	joint = &(emcmotStatus.joint_status[axis]);

	stat[axis].axisType = localEmcAxisAxisType[axis];
	stat[axis].units = localEmcAxisUnits[axis];
	if (new_config) {
	    stat[axis].backlash = joint->backlash;
	    stat[axis].minPositionLimit = joint->min_pos_limit;
	    stat[axis].maxPositionLimit = joint->max_pos_limit;
	    stat[axis].minFerror = joint->min_ferror;
	    stat[axis].maxFerror = joint->max_ferror;
	}
	stat[axis].output = joint->pos_cmd;
	stat[axis].input = joint->pos_fb;
	stat[axis].velocity = joint->vel_cmd;
        stat[axis].ferrorCurrent = joint->ferror;
        stat[axis].ferrorHighMark = joint->ferror_high_mark;

	stat[axis].homing = (joint->flag & EMCMOT_JOINT_HOMING_BIT ? 1 : 0);
	stat[axis].homed = (joint->flag & EMCMOT_JOINT_HOMED_BIT ? 1 : 0);
	stat[axis].fault = (joint->flag & EMCMOT_JOINT_FAULT_BIT ? 1 : 0);
	stat[axis].enabled = (joint->flag & EMCMOT_JOINT_ENABLE_BIT ? 1 : 0);
	stat[axis].inpos = (joint->flag & EMCMOT_JOINT_INPOS_BIT ? 1 : 0);

/* FIXME - soft limits are now applied to the command, and should never
   happen */
	stat[axis].minSoftLimit = 0;
	stat[axis].maxSoftLimit = 0;
	stat[axis].minHardLimit =
	    (joint->flag & EMCMOT_JOINT_MIN_HARD_LIMIT_BIT ? 1 : 0);
	stat[axis].maxHardLimit =
	    (joint->flag & EMCMOT_JOINT_MAX_HARD_LIMIT_BIT ? 1 : 0);
	stat[axis].overrideLimits = !!(emcmotStatus.overrideLimitMask);	// one
	// for
	// all

/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - per-axis Vscale temporarily? removed */
	stat[axis].scale = emcmotStatus.axVscale[axis];
#endif
#ifdef WATCH_FLAGS
	if (old_joint_flag[axis] != joint->flag) {
	    printf("joint %d flag: %04X -> %04X\n", axis,
		   old_joint_flag[axis], joint->flag);
	    old_joint_flag[axis] = joint->flag;
	}
#endif
	if (joint->flag & EMCMOT_JOINT_ERROR_BIT) {
	    if (stat[axis].status != RCS_ERROR) {
		rcs_print_error("Error on axis %d, command number %d\n",
				axis, emcmotStatus.commandNumEcho);
		stat[axis].status = RCS_ERROR;
	    }
	} else if (joint->flag & EMCMOT_JOINT_INPOS_BIT) {
	    stat[axis].status = RCS_DONE;
	} else {
	    stat[axis].status = RCS_EXEC;
	}
    }
    return 0;
}

// EMC_TRAJ functions

// local status data, not provided by emcmot
static int localEmcTrajAxes = 0;
static int localEmcTrajAxisMask = 0;
static double localEmcTrajLinearUnits = 1.0;
static double localEmcTrajAngularUnits = 1.0;
static int localEmcTrajMotionId = 0;
static int localSpindleAxis = 5;        // default to AXIS_5
//FIXME if you can't beat em...
static struct state_tag_t localEmcTrajTag;

int emcTrajUpdateTag(StateTag const &tag) {
    localEmcTrajTag = tag.get_state_tag();
    return 0;
}

int emcTrajSetAxes(int axes, int axismask)
{
    if(axes == 0) {
	if(axismask & 256) axes = 9;
	else if(axismask & 128) axes = 8;
	else if(axismask & 64) axes = 7;
	else if(axismask & 32) axes = 6;
	else if(axismask & 16) axes = 5;
	else if(axismask & 8) axes = 4;
	else if(axismask & 4) axes = 3;
	else if(axismask & 2) axes = 2;
	else if(axismask & 1) axes = 1;
    }
    if (axes <= 0 || axes > EMCMOT_MAX_JOINTS || axismask >= (1<<axes)) {
	rcs_print("emcTrajSetAxes failing: axes=%d axismask=%x\n",
		axes, axismask);
	return -1;
    }

    localEmcTrajAxes = axes;
    localEmcTrajAxisMask = axismask;
    emcmotCommand.command = EMCMOT_SET_NUM_AXES;
    emcmotCommand.axis = axes;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetUnits(double linearUnits, double angularUnits)
{
    if (linearUnits <= 0.0 || angularUnits <= 0.0) {
	return -1;
    }

    localEmcTrajLinearUnits = linearUnits;
    localEmcTrajAngularUnits = angularUnits;

    return 0;
}

int emcTrajSetMode(int mode)
{
    switch (mode) {
    case EMC_TRAJ_MODE_FREE:
	emcmotCommand.command = EMCMOT_FREE;
	return usrmotWriteEmcmotCommand(&emcmotCommand);

    case EMC_TRAJ_MODE_COORD:
	emcmotCommand.command = EMCMOT_COORD;
	return usrmotWriteEmcmotCommand(&emcmotCommand);

    case EMC_TRAJ_MODE_TELEOP:
	emcmotCommand.command = EMCMOT_TELEOP;
	return usrmotWriteEmcmotCommand(&emcmotCommand);

    default:
	return -1;
    }
}

int emcTrajSetTeleopVector(EmcPose vel)
{
    emcmotCommand.command = EMCMOT_SET_TELEOP_VECTOR;
    emcmotCommand.pos = vel;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetVelocity(double vel, double ini_maxvel)
{
    int retval;

    if (vel < 0.0) {
	vel = 0.0;
    } else if (vel > traj_max_velocity) {
	vel = traj_max_velocity;
    }

    if (ini_maxvel < 0.0) {
	    ini_maxvel = 0.0;
    } else if (vel > traj_max_velocity) {
	    ini_maxvel = traj_max_velocity;
    }

    emcmotCommand.command = EMCMOT_SET_VEL;
    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;

    retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    return retval;
}

int emcTrajSetAcceleration(double acc)
{
    if (acc < 0.0) {
	acc = 0.0;
    } else if (acc > localEmcMaxAcceleration) {
	acc = localEmcMaxAcceleration;
    }

    emcmotCommand.command = EMCMOT_SET_ACC;
    emcmotCommand.acc = acc;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetJerk(double jerk)
{
    if (jerk < 0.0) {
    	jerk = 0.0;
    } else if (jerk > localEmcMaxJerk) {
    	jerk = localEmcMaxJerk;
    }

    emcmotCommand.command = EMCMOT_SET_JERK;
    emcmotCommand.jerk = jerk;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

/*
  emcmot has no limits on max velocity, acceleration so we'll save them
  here and apply them in the functions above
  */
int emcTrajSetMaxVelocity(double vel)
{
    if (vel < 0.0) {
	vel = 0.0;
    }

    traj_max_velocity = vel;

    emcmotCommand.command = EMCMOT_SET_VEL_LIMIT;
    emcmotCommand.vel = vel;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetMaxAcceleration(double acc)
{
    if (acc < 0.0) {
	acc = 0.0;
    }

    localEmcMaxAcceleration = acc;

    return 0;
}

int emcTrajSetMaxJerk(double jerk)
{
    if (jerk < 0.0) {
    	jerk = 0.0;
    }

    localEmcMaxJerk = jerk;

    return 0;
}

int emcTrajSetHome(EmcPose home)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(home.tran.x) || rtapi_isnan(home.tran.y) || rtapi_isnan(home.tran.z) ||
	rtapi_isnan(home.a) || rtapi_isnan(home.b) || rtapi_isnan(home.c) ||
	rtapi_isnan(home.u) || rtapi_isnan(home.v) || rtapi_isnan(home.w)) {
	printf("isnan error in emcTrajSetHome()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_SET_WORLD_HOME;
    emcmotCommand.pos = home;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetScale(double scale)
{
    if (scale < 0.0) {
	scale = 0.0;
    }

    emcmotCommand.command = EMCMOT_FEED_SCALE;
    emcmotCommand.scale = scale;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetSpindleScale(double scale)
{
    if (scale < 0.0) {
	scale = 0.0;
    }

    emcmotCommand.command = EMCMOT_SPINDLE_SCALE;
    emcmotCommand.scale = scale;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetFOEnable(unsigned char mode)
{
    emcmotCommand.command = EMCMOT_FS_ENABLE;
    emcmotCommand.mode = mode;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetFHEnable(unsigned char mode)
{
    emcmotCommand.command = EMCMOT_FH_ENABLE;
    emcmotCommand.mode = mode;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetSOEnable(unsigned char mode)
{
    emcmotCommand.command = EMCMOT_SS_ENABLE;
    emcmotCommand.mode = mode;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetAFEnable(unsigned char enable)
{
    emcmotCommand.command = EMCMOT_AF_ENABLE;

    if ( enable ) {
	emcmotCommand.flags = 1;
    } else {
	emcmotCommand.flags = 0;
    }
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetMotionId(int id)
{

    if (EMC_DEBUG_MOTION_TIME & emc_debug) {
	if (id != localEmcTrajMotionId) {
	    rcs_print("Outgoing motion id is %d.\n", id);
	}
    }

    localEmcTrajMotionId = id;

    return 0;
}

int emcTrajInit()
{
    int retval = 0;

    // init emcmot interface
    if (!AxisOrTrajInited()) {
	usrmotIniLoad(emc_inifile);
	if (0 != usrmotInit("emc2_task")) {
	    return -1;
	}
    }
    emcmotTrajInited = 1;
    // initialize parameters from ini file
    if (0 != iniTraj(emc_inifile)) {
	retval = -1;
    }
    return retval;
}

int emcTrajHalt()
{
    emcmotTrajInited = 0;

    if (!AxisOrTrajInited()) {
	usrmotExit();		// ours is final exit
    }

    return 0;
}

int emcTrajEnable()
{
    emcmotCommand.command = EMCMOT_ENABLE;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajDisable()
{
    emcmotCommand.command = EMCMOT_DISABLE;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajAbort()
{
    emcmotCommand.command = EMCMOT_ABORT;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajPause()
{
    emcmotCommand.command = EMCMOT_PAUSE;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajStep()
{
    emcmotCommand.command = EMCMOT_STEP;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajResume()
{
    emcmotCommand.command = EMCMOT_RESUME;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajDelay(double delay)
{
    /* nothing need be done here - it's done in task controller */

    return 0;
}

double emcTrajGetLinearUnits()
{
    return localEmcTrajLinearUnits;
}

double emcTrajGetAngularUnits()
{
    return localEmcTrajAngularUnits;
}

int emcTrajSetOffset(EmcPose tool_offset)
{
    emcmotCommand.command = EMCMOT_SET_OFFSET;
    emcmotCommand.tool_offset = tool_offset;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetSpindleSync(double fpr, bool wait_for_index) 
{
    emcmotCommand.command = EMCMOT_SET_SPINDLESYNC;
    emcmotCommand.spindlesync = fpr;
    emcmotCommand.flags = wait_for_index;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetTermCond(int cond, double tolerance)
{
    emcmotCommand.command = EMCMOT_SET_TERM_COND;
    // Direct passthrough since TP can handle the distinction now
    emcmotCommand.termCond = cond;
    emcmotCommand.tolerance = tolerance;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajLinearMove(EmcPose end, int type, double vel, double ini_maxvel, double acc, double jerk,
                      int indexrotary)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(end.tran.x) || rtapi_isnan(end.tran.y) || rtapi_isnan(end.tran.z) ||
        rtapi_isnan(end.a) || rtapi_isnan(end.b) || rtapi_isnan(end.c) ||
        rtapi_isnan(end.u) || rtapi_isnan(end.v) || rtapi_isnan(end.w)) {
	printf("isnan error in emcTrajLinearMove()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_SET_LINE;

    emcmotCommand.pos = end;

    emcmotCommand.id = localEmcTrajMotionId;
    emcmotCommand.tag = localEmcTrajTag;
    emcmotCommand.motion_type = type;
    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;
    emcmotCommand.acc = acc;
    emcmotCommand.ini_maxjerk = jerk;
    emcmotCommand.turn = indexrotary;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajCircularMove(EmcPose end, PM_CARTESIAN center,
			PM_CARTESIAN normal, int turn, int type, double vel, double ini_maxvel, double acc, double ini_maxjerk)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(end.tran.x) || rtapi_isnan(end.tran.y) || rtapi_isnan(end.tran.z) ||
	rtapi_isnan(end.a) || rtapi_isnan(end.b) || rtapi_isnan(end.c) ||
	rtapi_isnan(end.u) || rtapi_isnan(end.v) || rtapi_isnan(end.w) ||
	rtapi_isnan(center.x) || rtapi_isnan(center.y) || rtapi_isnan(center.z) ||
	rtapi_isnan(normal.x) || rtapi_isnan(normal.y) || rtapi_isnan(normal.z)) {
	printf("isnan error in emcTrajCircularMove()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_SET_CIRCLE;

    emcmotCommand.pos = end;
    emcmotCommand.motion_type = type;

    emcmotCommand.center.x = center.x;
    emcmotCommand.center.y = center.y;
    emcmotCommand.center.z = center.z;

    emcmotCommand.normal.x = normal.x;
    emcmotCommand.normal.y = normal.y;
    emcmotCommand.normal.z = normal.z;

    emcmotCommand.turn = turn;
    emcmotCommand.id = localEmcTrajMotionId;
    emcmotCommand.tag = localEmcTrajTag;

    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;
    emcmotCommand.acc = acc;
    emcmotCommand.ini_maxjerk = ini_maxjerk;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajClearProbeTrippedFlag()
{
    emcmotCommand.command = EMCMOT_CLEAR_PROBE_FLAGS;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajEndProbe(unsigned char probe_type)
{
    emcmotCommand.command = EMCMOT_END_PROBE;
    emcmotCommand.probe_type = probe_type;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajProbe(EmcPose pos, int type, double vel, double ini_maxvel, double acc, double ini_maxjerk, unsigned char probe_type)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(pos.tran.x) || rtapi_isnan(pos.tran.y) || rtapi_isnan(pos.tran.z) ||
        rtapi_isnan(pos.a) || rtapi_isnan(pos.b) || rtapi_isnan(pos.c) ||
        rtapi_isnan(pos.u) || rtapi_isnan(pos.v) || rtapi_isnan(pos.w)) {
	printf("isnan error in emcTrajProbe()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_PROBE;
    emcmotCommand.pos = pos;
    emcmotCommand.id = localEmcTrajMotionId;
    emcmotCommand.tag = localEmcTrajTag;
    emcmotCommand.motion_type = type;
    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;
    emcmotCommand.acc = acc;
    emcmotCommand.ini_maxjerk = ini_maxjerk;
    emcmotCommand.probe_type = probe_type;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSpindleSyncMotion(EmcPose pos, int type, double vel, double ini_maxvel, double acc, double ini_maxjerk, int ssm_mode)
{
#ifdef ISNAN_TRAP
    if (rtapi_isnan(pos.tran.x) || rtapi_isnan(pos.tran.y) || rtapi_isnan(pos.tran.z)) {
	printf("isnan error in emcSpindleSyncMotion()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif
    emcmotCommand.command = EMCMOT_SPINDLE_SYNC_MOTION;
    emcmotCommand.pos = pos;
    emcmotCommand.id = localEmcTrajMotionId;
    emcmotCommand.tag = localEmcTrajTag;
    emcmotCommand.motion_type = type;
    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;
    emcmotCommand.acc = acc;
    emcmotCommand.ini_maxjerk = ini_maxjerk;
    emcmotCommand.ssm_mode = ssm_mode;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

static int last_id = 0;
static int last_id_printed = 0;
static int last_status = 0;
static double last_id_time;

int emcTrajUpdate(EMC_TRAJ_STAT * stat)
{
    int axis, enables;

    stat->axes = localEmcTrajAxes;
    stat->axis_mask = localEmcTrajAxisMask;
    stat->linearUnits = localEmcTrajLinearUnits;
    stat->angularUnits = localEmcTrajAngularUnits;

    stat->mode =
	emcmotStatus.
	motionFlag & EMCMOT_MOTION_TELEOP_BIT ? EMC_TRAJ_MODE_TELEOP
	: (emcmotStatus.
	   motionFlag & EMCMOT_MOTION_COORD_BIT ? EMC_TRAJ_MODE_COORD :
	   EMC_TRAJ_MODE_FREE);

    /* enabled if motion enabled and all axes enabled */
    stat->enabled = 0;		/* start at disabled */
    if (emcmotStatus.motionFlag & EMCMOT_MOTION_ENABLE_BIT) {
	for (axis = 0; axis < localEmcTrajAxes; axis++) {
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - the axis flag has been moved to the joint struct */
	    if (!emcmotStatus.axisFlag[axis] & EMCMOT_JOINT_ENABLE_BIT) {
		break;
	    }
#endif
	    /* got here, then all are enabled */
	    stat->enabled = 1;
	}
    }

    stat->inpos = emcmotStatus.motionFlag & EMCMOT_MOTION_INPOS_BIT;
    stat->queue = emcmotStatus.depth;
    stat->activeQueue = emcmotStatus.activeDepth;
    stat->queueFull = emcmotStatus.queueFull;
    stat->id = emcmotStatus.id;
    StateTag newtag(emcmotStatus.tag);
    //TODO assignment operator
    stat->tag = newtag;
    stat->motion_type = emcmotStatus.motionType;
    stat->distance_to_go = emcmotStatus.distance_to_go;
    stat->prim_dtg = emcmotStatus.prim_dtg;
    stat->prim_progress = emcmotStatus.prim_progress;
    stat->dtg = emcmotStatus.dtg;
    stat->current_vel = emcmotStatus.current_vel;
    if (EMC_DEBUG_MOTION_TIME & emc_debug) {
	if (stat->id != last_id) {
	    if (last_id != last_id_printed) {
		rcs_print("Motion id %d took %f seconds (current %d)\n", last_id,
			  etime() - last_id_time, stat->id);
		last_id_printed = last_id;
	    }
	    last_id = stat->id;
	    last_id_time = etime();
	}
    }

    stat->paused = emcmotStatus.pause_state;
    stat->pause_state = emcmotStatus.pause_state;
    stat->tp_reverse_input = emcmotStatus.tp_reverse_input;
    stat->scale = emcmotStatus.feed_scale;
    stat->spindle_scale = emcmotStatus.spindle_scale;

    stat->position = emcmotStatus.carte_pos_cmd;

    stat->actualPosition = emcmotStatus.carte_pos_fb;

    stat->velocity = emcmotStatus.vel;
    stat->acceleration = emcmotStatus.acc;
    stat->maxAcceleration = localEmcMaxAcceleration;
    stat->maxJerk = localEmcMaxJerk;

    if (emcmotStatus.motionFlag & EMCMOT_MOTION_ERROR_BIT) {
	stat->status = RCS_ERROR;
    } else if (stat->inpos && (stat->queue == 0)) {
	stat->status = RCS_DONE;
    } else {
	stat->status = RCS_EXEC;
    }

    if (EMC_DEBUG_MOTION_TIME & emc_debug) {
	if (stat->status == RCS_DONE && last_status != RCS_DONE
	    && stat->id != last_id_printed) {
	    rcs_print("Motion id %d took %f seconds (current %d).\n", last_id,
		      etime() - last_id_time, stat->id );
	    last_id_printed = last_id = stat->id;
	    last_id_time = etime();
	}
    }

    stat->probedPosition = emcmotStatus.probedPos;

    stat->probeval = emcmotStatus.probeVal;
    stat->probing = emcmotStatus.probing;
    stat->probe_tripped = emcmotStatus.probeTripped;
    
#ifdef USB_MOTION_ENABLE
    stat->update_pos_req = emcmotStatus.update_pos_req;   // update position request from RISC
#endif

    if (emcmotStatus.motionFlag & EMCMOT_MOTION_COORD_BIT)
        enables = emcmotStatus.enables_queued;
    else
        enables = emcmotStatus.enables_new;
    
    stat->feed_override_enabled = enables & FS_ENABLED;
    stat->spindle_override_enabled = enables & SS_ENABLED;
    stat->adaptive_feed_enabled = enables & AF_ENABLED;
    stat->feed_hold_enabled = enables & FH_ENABLED;

    if (new_config) {
	stat->cycleTime = emcmotConfig.trajCycleTime;
	stat->kinematics_type = emcmotConfig.kinematics_type;
	stat->maxVelocity = emcmotConfig.limitVel;
    }

    return 0;
}

int emcPositionLoad() {
    double positions[EMCMOT_MAX_JOINTS];
    IniFile ini;
    ini.Open(emc_inifile);
    const char *posfile = ini.Find("POSITION_FILE", "TRAJ");
    ini.Close();
    if(!posfile || !posfile[0]) return 0;
    FILE *f = fopen(posfile, "r");
    if(!f) return 0;
    for(int i=0; i<EMCMOT_MAX_JOINTS; i++) {
	int r = fscanf(f, "%lf", &positions[i]);
	if(r != 1) { fclose(f); return -1; }
    }
    fclose(f);
    int result = 0;
    for(int i=0; i<EMCMOT_MAX_JOINTS; i++) {
	if(emcAxisSetMotorOffset(i, -positions[i]) != 0) result = -1;;
    }
    return result;
}


int emcPositionSave() {
    IniFile ini;
    const char *posfile;

    ini.Open(emc_inifile);
    try {
        posfile = ini.Find("POSITION_FILE", "TRAJ");
    } catch (IniFile::Exception e) {
        ini.Close();
        return -1;
    }
    ini.Close();

    if(!posfile || !posfile[0]) return 0;
    // like the var file, make sure the posfile is recreated according to umask
    unlink(posfile);
    FILE *f = fopen(posfile, "w");
    if(!f) return -1;
    for(int i=0; i<EMCMOT_MAX_JOINTS; i++) {
	int r = fprintf(f, "%.17f\n", emcmotStatus.joint_status[i].pos_fb);
	if(r < 0) { fclose(f); return -1; }
    }
    fclose(f);
    return 0;
}

// EMC_MOTION functions
int emcMotionInit()
{
    int r1;
    int r2;
    int r3;
    int axis;

    r2 = emcTrajInit(); // we want to check Traj first, the sane defaults for units are there

    r1 = 0;
    for (axis = 0; axis < localEmcTrajAxes; axis++) {
	if (0 != emcAxisInit(axis)) {
	    r1 = -1;		// at least one is busted
	}
    }

    r3 = emcPositionLoad();

    if (r1 == 0 && r2 == 0 && r3 == 0) {
	emcmotion_initialized = 1;
    }

    if (ini_hal_init()) {
	rcs_print("emcMotionInit: ini_hal_init fail\n");
	return -1;
    }

    if (ini_hal_init_pins()) {
	rcs_print("emcMotionInit: ini_hal_init_pins fail\n");
    }

    return (r1 == 0 && r2 == 0 && r3 == 0) ? 0 : -1;
}

int emcMotionHalt()
{
    int r1, r2, r3, r4;
    int t;

    r1 = -1;
    for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	if (0 == emcAxisHalt(t)) {
	    r1 = 0;		// at least one is okay
	}
    }

    r2 = emcTrajDisable();
    r3 = emcTrajHalt();
    r4 = emcPositionSave();
    emcmotion_initialized = 0;

    return (r1 == 0 && r2 == 0 && r3 == 0 && r4 == 0) ? 0 : -1;
}

int emcMotionAbort()
{
    int r1;
    int r2;
    int r3 = 0;
    int t;

    r1 = -1;
    for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	if (0 == emcAxisAbort(t)) {
	    r1 = 0;		// at least one is okay
	}
    }

    r2 = emcTrajAbort();

    return (r1 == 0 && r2 == 0 && r3 == 0) ? 0 : -1;
}

int emcMotionSetDebug(int debug)
{
    emcmotCommand.command = EMCMOT_SET_DEBUG;
    emcmotCommand.debug = debug;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

/*! \function emcMotionSetAout()
    
    This function sends a EMCMOT_SET_AOUT message to the motion controller.
    That one plans a AOUT command when motion starts or right now.

    @parameter	index	which output gets modified
    @parameter	now	wheather change is imediate or synched with motion
    @parameter	start	value set at start of motion
    @parameter	end	value set at end of motion
*/
int emcMotionSetAout(unsigned int index, double start, double end, unsigned char now)
{
    emcmotCommand.command = EMCMOT_SET_AOUT;
    emcmotCommand.now = now;
    emcmotCommand.out = index;
  /*! \todo FIXME-- if this works, set up some dedicated cmd fields instead of
     borrowing these */
    emcmotCommand.minLimit = start;
    emcmotCommand.maxLimit = end;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

/*! \function emcMotionSetDout()
    
    This function sends a EMCMOT_SET_DOUT message to the motion controller.
    That one plans a DOUT command when motion starts or right now.

    @parameter	index	which output gets modified
    @parameter	now	wheather change is imediate or synched with motion
    @parameter	start	value set at start of motion
    @parameter	end	value set at end of motion
*/
int emcMotionSetDout(unsigned int index, unsigned char start,
		     unsigned char end, unsigned char now)
{
    emcmotCommand.command = EMCMOT_SET_DOUT;
    emcmotCommand.now = now;
    emcmotCommand.out = index;
    emcmotCommand.start = start;
    emcmotCommand.end = end;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleAbort(void)
{
    return emcSpindleOff();
}

int emcSpindleSpeed(double speed, double css_factor, double offset)
{

    if (emcmotStatus.spindle.speed == 0)
	return 0; //spindle stopped, not updating speed

    return emcSpindleOn(speed, css_factor, offset);
}

int emcSpindleOrient(double orientation, int mode) 
{
    emcmotCommand.command = EMCMOT_SPINDLE_ORIENT;
    emcmotCommand.orientation = orientation;
    emcmotCommand.mode = mode;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}


int emcSpindleOn(double speed, double css_factor, double offset)
{

    emcmotCommand.command = EMCMOT_SPINDLE_ON;
    emcmotCommand.vel = speed;
    emcmotCommand.css_factor = css_factor;
    emcmotCommand.xoffset = offset;
    emcmotCommand.axis = localSpindleAxis;
    emcmotCommand.ini_maxvel = axis_max_velocity[localSpindleAxis];
    emcmotCommand.acc = axis_max_acceleration[localSpindleAxis];
    emcmotCommand.ini_maxjerk = axis_max_jerk[localSpindleAxis];
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleOff()
{
    emcmotCommand.command = EMCMOT_SPINDLE_OFF;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleBrakeRelease()
{
    emcmotCommand.command = EMCMOT_SPINDLE_BRAKE_RELEASE;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleBrakeEngage()
{
    emcmotCommand.command = EMCMOT_SPINDLE_BRAKE_ENGAGE;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleIncrease()
{
    emcmotCommand.command = EMCMOT_SPINDLE_INCREASE;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleDecrease()
{
    emcmotCommand.command = EMCMOT_SPINDLE_DECREASE;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleConstant()
{
    return 0; // nothing to do
}



int emcMotionUpdate(EMC_MOTION_STAT * stat)
{
    int r1;
    int r2;
    int axis;
    int error;
    int exec;
    int dio, aio;

    // read the emcmot status
    if (0 != usrmotReadEmcmotStatus(&emcmotStatus)) {
	return -1;
    }

    new_config = 0;
    if (emcmotStatus.config_num != emcmotConfig.config_num) {
	if (0 != usrmotReadEmcmotConfig(&emcmotConfig)) {
	    return -1;
	}
	new_config = 1;
    }

    if (get_emcmot_debug_info) {
	if (0 != usrmotReadEmcmotDebug(&emcmotDebug)) {
	    return -1;
	}
    }
    // read the emcmot error
    if (0 != usrmotReadEmcmotError(errorString)) {
	// no error, so ignore
    } else {
	// an error to report
	emcOperatorError(0, "%s", errorString);
    }

    // save the heartbeat and command number locally,
    // for use with emcMotionUpdate
    localMotionHeartbeat = emcmotStatus.heartbeat;
    localMotionCommandType = emcmotStatus.commandEcho;	/*! \todo FIXME-- not NML one! */
    localMotionEchoSerialNumber = emcmotStatus.commandNumEcho;

    r1 = emcAxisUpdate(&stat->axis[0], EMCMOT_MAX_JOINTS);
    r2 = emcTrajUpdate(&stat->traj);
    stat->heartbeat = localMotionHeartbeat;
    stat->command_type = localMotionCommandType;
    stat->echo_serial_number = localMotionEchoSerialNumber;
    stat->debug = emcmotConfig.debug;
    
    stat->spindle.enabled = emcmotStatus.spindle.speed != 0;
    stat->spindle.speed = emcmotStatus.spindle.speed;
    stat->spindle.brake = emcmotStatus.spindle.brake;
    stat->spindle.direction = emcmotStatus.spindle.direction;
    stat->spindle.orient_state = emcmotStatus.spindle.orient_state;
    stat->spindle.orient_fault = emcmotStatus.spindle.orient_fault;

    for (dio = 0; dio < EMC_MAX_DIO; dio++) {
	stat->synch_di[dio] = emcmotStatus.synch_di[dio];
	stat->synch_do[dio] = emcmotStatus.synch_do[dio];
    }

    for (aio = 0; aio < EMC_MAX_AIO; aio++) {
	stat->analog_input[aio] = emcmotStatus.analog_input[aio];
	stat->analog_output[aio] = emcmotStatus.analog_output[aio];
    }

    // set the status flag
    error = 0;
    exec = 0;

    for (axis = 0; axis < stat->traj.axes; axis++) {
	if (stat->axis[axis].status == RCS_ERROR) {
	    error = 1;
	    break;
	}
	if (stat->axis[axis].status == RCS_EXEC) {
	    exec = 1;
	    break;
	}
    }
    if (stat->traj.status == RCS_ERROR) {
	error = 1;
    } else if (stat->traj.status == RCS_EXEC) {
	exec = 1;
    }

    if (error) {
	stat->status = RCS_ERROR;
    } else if (exec) {
	stat->status = RCS_EXEC;
    } else {
	stat->status = RCS_DONE;
    }
    return (r1 == 0 && r2 == 0) ? 0 : -1;
}

int emcSetupArcBlends(int arcBlendEnable,
        int arcBlendFallbackEnable,
        int arcBlendOptDepth,
        int arcBlendGapCycles,
        double arcBlendRampFreq,
        double arcBlendTangentKinkRatio) {

    emcmotCommand.command = EMCMOT_SETUP_ARC_BLENDS;
    emcmotCommand.arcBlendEnable = arcBlendEnable;
    emcmotCommand.arcBlendFallbackEnable = arcBlendFallbackEnable;
    emcmotCommand.arcBlendOptDepth = arcBlendOptDepth;
    emcmotCommand.arcBlendGapCycles = arcBlendGapCycles;
    emcmotCommand.arcBlendRampFreq = arcBlendRampFreq;
    emcmotCommand.arcBlendTangentKinkRatio = arcBlendTangentKinkRatio;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSetupUsbMot(int usbmotEnable)
{
    emcmotCommand.command = EMCMOT_SETUP_USBMOT;
    emcmotCommand.usbmotEnable = usbmotEnable;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSetMaxFeedOverride(double maxFeedScale) {
    emcmotCommand.command = EMCMOT_SET_MAX_FEED_OVERRIDE;
    emcmotCommand.maxFeedScale = maxFeedScale;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

void emcSetSpindleAxis(int spindleAxis)
{
    localSpindleAxis = spindleAxis;
    return;
}

int emcGetSpindleAxis(void)
{
    return localSpindleAxis;
}
