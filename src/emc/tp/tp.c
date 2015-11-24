/********************************************************************
* Description: tp.c
*   Trajectory planner based on TC elements
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/
#include "rtapi.h"              /* rtapi_print_msg */
#include "posemath.h"           /* Geometry types & functions */
#include "tc.h"
#include "tp.h"
#include "tp_private.h"
#include "tp_shared.h"
#include "emcpose.h"
#include "rtapi_math.h"
#include "motion_types.h"
#include "motion_id.h"
#include "spherical_arc.h"
#include "blendmath.h"
#include <assert.h>
#include <stdint.h>

//KLUDGE Don't include all of emc.hh here, just hand-copy the TERM COND
//definitions until we can break the emc constants out into a separate file.
//#include "emc.hh"
#define EMC_TRAJ_TERM_COND_STOP  0
#define EMC_TRAJ_TERM_COND_EXACT 1
#define EMC_TRAJ_TERM_COND_BLEND 2

/**
 * @section tpdebugflags TP debugging flags
 * Enable / disable various debugging functions here.
 * These flags control debug printing from RTAPI. These functions are
 * admittedly kludged on top of the existing rtapi_print framework. As written,
 * though, it's an easy way to selectively compile functions as static or not,
 * and selectively compile in assertions and debug printing.
 */
#define TRACE 0

#include "tp_debug.h"
#if (TRACE!=0)
static FILE* dptrace = 0;
static uint32_t _dt = 0;
#endif

#define TP_SHOW_BLENDS
#define TP_OPTIMIZATION_LAZY

/** static function primitives (ugly but less of a pain than moving code around)*/
STATIC int tpComputeBlendVelocity(
        TP_STRUCT const * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT * const nexttc);

STATIC int tpCheckEndCondition(
        TP_STRUCT * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT * const nexttc);

STATIC int tpUpdateCycle(
        TP_STRUCT * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT * const nexttc);

STATIC int tpRunOptimization(
        TP_STRUCT * const tp);

STATIC inline int tpAddSegmentToQueue(
        TP_STRUCT * const tp,
        TC_STRUCT * const tc,
        int inc_id);

STATIC inline double tpGetMaxTargetVel(
        TP_STRUCT const * const tp,
        TC_STRUCT const * const tc);


STATIC int tpAdjustAccelForTangent(TP_STRUCT const * const,
        TC_STRUCT * const tc,
        double normal_acc_scale);
/**
 * @section tpcheck Internal state check functions.
 * These functions compartmentalize some of the messy state checks.
 * Hopefully this makes changes easier to track as much of the churn will be on small functions.
 */


/**
 * Check if the tail of the queue has a parabolic blend condition and update tc appropriately.
 * This sets flags so that accelerations are correct due to the current segment
 * having to blend with the previous.
 */
STATIC int tcCheckLastParabolic(TC_STRUCT * const tc,
        TC_STRUCT const * const prev_tc) {
    if (prev_tc && prev_tc->term_cond == TC_TERM_COND_PARABOLIC) {
        tp_debug_print("prev segment parabolic, flagging blend_prev\n");
        tc->blend_prev = 1;
    }
    return TP_ERR_OK;
}

/**
 * Returns true if there is motion along ABC or UVW axes, false otherwise.
 */
STATIC int tpRotaryMotionCheck(TP_STRUCT const * const tp, TC_STRUCT const * const tc) {
    switch (tc->motion_type) {
        //Note lack of break statements due to every path returning
        case TC_SPINDLE_SYNC_MOTION:
            return false;
        case TC_LINEAR:
            if (tc->coords.line.abc.tmag_zero && tc->coords.line.uvw.tmag_zero) {
                return false;
            } else {
                return true;
            }
        case TC_CIRCULAR:
            if (tc->coords.circle.abc.tmag_zero && tc->coords.circle.uvw.tmag_zero) {
                return false;
            } else {
                return true;
            }
        case TC_SPHERICAL:
            return true;
        default:
            tp_debug_print("Unknown motion type!\n");
            return false;
    }
}


/**
 * @section tpgetset Internal Get/Set functions
 * @brief Calculation / status functions for commonly used values.
 * These functions return the "actual" values of things like a trajectory
 * segment's feed override, while taking into account the status of tp itself.
 */


STATIC int tpGetMachineAccelBounds(TP_STRUCT const * const tp, PmCartesian  * const acc_bound)
{
    if (!acc_bound) {
        return TP_ERR_FAIL;
    }

    /* acc_bound->x = joints[0].acc_limit; */
    /* acc_bound->y = joints[1].acc_limit; */
    /* acc_bound->z = joints[2].acc_limit; */
    acc_bound->x = get_acc_limit(tp->shared, 0);
    acc_bound->y = get_acc_limit(tp->shared, 1);
    acc_bound->z = get_acc_limit(tp->shared, 2);
    return TP_ERR_OK;
}


STATIC int tpGetMachineVelBounds(TP_STRUCT const * const tp, PmCartesian  * const vel_bound)
{
    if (!vel_bound) {
        return TP_ERR_FAIL;
    }

    /* vel_bound->x = joints[0].vel_limit; */
    /* vel_bound->y = joints[1].vel_limit; */
    /* vel_bound->z = joints[2].vel_limit; */
    vel_bound->x = get_vel_limit(tp->shared, 0);
    vel_bound->y = get_vel_limit(tp->shared, 1);
    vel_bound->z = get_vel_limit(tp->shared, 2);
    return TP_ERR_OK;
}

STATIC int tpGetMachineActiveLimit(double * const act_limit, PmCartesian const * const bounds) {
    if (!act_limit) {
        return TP_ERR_FAIL;
    }
    //Start with max accel value
    *act_limit = rtapi_fmax(rtapi_fmax(bounds->x,bounds->y),bounds->z);

    // Compare only with active axes
    if (bounds->x > 0) {
        *act_limit = rtapi_fmin(*act_limit, bounds->x);
    }
    if (bounds->y > 0) {
        *act_limit = rtapi_fmin(*act_limit, bounds->y);
    }
    if (bounds->z > 0) {
        *act_limit = rtapi_fmin(*act_limit, bounds->z);
    }
    tp_debug_print(" arc blending a_max=%f\n", *act_limit);
    return TP_ERR_OK;
}


/**
 * Get a segment's feed scale based on the current planner state and tp_shared_t.
 */
STATIC double tpGetFeedScale(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc) {
    //All reasons to disable feed override go here
    bool pausing = tp->pausing && tc->synchronized == TC_SYNC_NONE;
    bool aborting = tp->aborting;
    if (pausing)  {
        tc_debug_print("pausing\n");
        return 0.0;
    } else if (aborting) {
        tc_debug_print("aborting\n");
        return 0.0;
    } else if (tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE ||
            tc->synchronized == TC_SYNC_POSITION ) {
        return 1.0;
    } else if (tc->is_blending) {
        //KLUDGE: Don't allow feed override to keep blending from overruning max velocity
        return rtapi_fmin(get_net_feed_scale(tp->shared), 1.0);
    } else {
        return get_net_feed_scale(tp->shared);
    }
}


/**
 * Get target velocity for a tc based on the trajectory planner state.
 * This gives the requested velocity, capped by the segments maximum velocity.
 */
STATIC inline double tpGetRealTargetVel(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc)
{

    // Start with the scaled target velocity based on the current feed scale
    double v_target = tc->synchronized ? tc->target_vel : tc->reqvel;
    tc_debug_print("%d: Initial v_target(%f) tc->synchronized(%d) target_vel(%f) reqvel(%f)\n", __LINE__,
                    v_target,
                    tc->synchronized,
                    tc->target_vel,
                    tc->reqvel);

    // Get the maximum allowed target velocity, and make sure we're below it
    return rtapi_fmin(v_target * tpGetFeedScale(tp,tc), tpGetMaxTargetVel(tp, tc));
}


/**
 * Get the worst-case target velocity for a segment based on the trajectory
 * planner state.
 */
STATIC inline double tpGetMaxTargetVel(
        TP_STRUCT const * const tp,
        TC_STRUCT const * const tc)
{
#ifdef TP_PEDANTIC
    if (!tp || !tc) {
        return TP_ERR_MISSING_INPUT;
    }
#endif

    double max_scale = get_maxFeedScale(tp->shared);
    if (tc->is_blending) {
        //KLUDGE: Don't allow feed override to keep blending from overruning max velocity
        max_scale = rtapi_fmin(max_scale, 1.0);
    }
    // Get maximum reachable velocity from max feed override
    double v_max_target = tc->target_vel * max_scale;

    /* Check if the cartesian velocity limit applies and clip the maximum
     * velocity. The vLimit is from the max velocity slider, and should
     * restrict the maximum velocity during non-synced moves and velocity
     * synchronization. However, position-synced moves have the target velocity
     * computed in the TP, so it would disrupt position tracking to apply this
     * limit here.
     */
    if (!tcPureRotaryCheck(tc) && (tc->synchronized != TC_SYNC_POSITION)){
        /*tc_debug_print("Cartesian velocity limit active\n");*/
        v_max_target = rtapi_fmin(v_max_target,tp->vLimit);
    }

    // Clip maximum velocity by the segment's own maximum velocity
    return rtapi_fmin(v_max_target, tc->maxvel/tc->cycle_time);
}


/**
 * Get final velocity for a tc based on the trajectory planner state.
 * This function factors in the feed override and TC limits. It clamps the
 * final velocity to the maximum velocity and the next segment's target velocity
 */
STATIC inline double tpGetRealFinalVel(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc,
        TC_STRUCT const * const nexttc)
{
    /* If we're stepping, then it doesn't matter what the optimization says, we
     * want to end at a stop.  If the term_cond gets changed out from under us,
     * detect this and force final velocity to zero.
     */
    if (get_stepping(tp->shared) || tc->term_cond != TC_TERM_COND_TANGENT) {
        return 0.0;
    } 
    
    // Get target velocities for this segment and next segment
    double v_target_this = tpGetRealTargetVel(tp, tc);
    double v_target_next = 0.0;
    if (nexttc) {
        v_target_next = tpGetRealTargetVel(tp, nexttc);
    }

    tc_debug_print("v_target_next = %f\n",v_target_next);
    // Limit final velocity to minimum of this and next target velocities
    double v_target = rtapi_fmin(v_target_this, v_target_next);
    double finalvel = rtapi_fmin(tc->finalvel, v_target);
    return finalvel;
}

/**
 * Get acceleration for a tc based on the trajectory planner state.
 */
STATIC inline double tpGetScaledAccel(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc) {
    double a_scale = tc->maxaccel;
    /* Parabolic blending conditions: If the next segment or previous segment
     * has a parabolic blend with this one, acceleration is scaled down by 1/2
     * so that the sum of the two does not exceed the maximum.
     */
    if (tc->term_cond == TC_TERM_COND_PARABOLIC || tc->blend_prev) {
        a_scale *= 0.5;
    }
    if (tc->motion_type == TC_CIRCULAR || tc->motion_type == TC_SPHERICAL) {
        //Limit acceleration for cirular arcs to allow for normal acceleration
        a_scale *= BLEND_ACC_RATIO_TANGENTIAL;
    }
    return a_scale;
}

/**
 * Cap velocity based on trajectory properties
 */
STATIC inline double tpGetSampleVelocity(double vel, double length, double dt) {
    //FIXME div by zero check
    double v_sample = length / dt;
    return rtapi_fmin(vel,v_sample);
}

/**
 * Convert the 2-part spindle position and sign to a signed double.
 */
STATIC inline double tpGetSignedSpindlePosition(double spindle_pos, int spindle_dir) {
    if (spindle_dir < 0.0) {
        spindle_pos*=-1.0;
    }
    return spindle_pos;
}

/**
 * @section tpaccess tp class-like API
 */

/**
 * Create the trajectory planner structure with an empty queue.
 */
int tpCreate(TP_STRUCT * const tp, int _queueSize, TC_STRUCT * const tcSpace,
	     tp_shared_t *shared)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (_queueSize <= 0) {
        tp->queueSize = TP_DEFAULT_QUEUE_SIZE;
    } else {
        tp->queueSize = _queueSize;
    }

    tp->shared = shared;

    /* create the queue */
    if (-1 == tcqCreate(&tp->queue, tp->queueSize, tcSpace)) {
        return TP_ERR_FAIL;
    }

#if (TRACE!=0)
    if (!dptrace) {
        dptrace = fopen("tp.log", "w");
        /* prepare header for gnuplot */
        DPS ("%11s%5s%6s%15s%15s%15s%15s%15s%15s%15s%15s%15s%15s\n",
                "#dt", "id", "state", "req_vel", "cur_accel",
                "cur_vel", "progress%", "progress", "dist_to_go",
                "lookahead", "tc->jerk", "currentPos.x", "currentPos.y");
        _dt = 0;
    }
#endif

    /* init the rest of our data */
    return tpInit(tp);
}

/**
 * Clears any potential DIO toggles and anychanged.
 * If any DIOs need to be changed: dios[i] = 1, DIO needs to get turned on, -1
 * = off
 */
int tpClearDIOs(TP_STRUCT * const tp) {
    //XXX: All IO's will be flushed on next synced aio/dio! Is it ok?
    unsigned int i;
    tp->syncdio.anychanged = 0;
    RTAPI_ZERO_BITMAP(tp->syncdio.dio_mask, EMCMOT_MAX_DIO);
    tp->syncdio.aio_mask = 0ull;
    for (i = 0; i < get_num_dio(tp->shared); i++) {
        tp->syncdio.dios[i] = 0;
    }
    for (i = 0; i < get_num_aio(tp->shared); i++) {
        tp->syncdio.aios[i] = 0;
    }

    return TP_ERR_OK;
}

/**
 *    "Soft initialize" the trajectory planner tp.
 *    This is a "soft" initialization in that TP_STRUCT configuration
 *    parameters (cycleTime, vMax, and aMax) are left alone, but the queue is
 *    cleared, and the flags are set to an empty, ready queue. The currentPos
 *    is left alone, and goalPos is set to this position.  This function is
 *    intended to put the motion queue in the state it would be if all queued
 *    motions finished at the current position.
 */
int tpClear(TP_STRUCT * const tp)
{
    tcqInit(&tp->queue);
    tp->queueSize = 0;
    tp->goalPos = tp->currentPos;
    tp->nextId = 0;
    tp->execId = 0;
    tp->motionType = 0;
    tp->termCond = TC_TERM_COND_PARABOLIC;
    tp->tolerance = 0.0;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->pausing = 0;
    tp->synchronized = 0;
    tp->uu_per_rev = 0.0;
    tp->distance_to_go = 0.0;
    tp->progress = 0.0;
    set_spindleSync(tp->shared, 0);
    set_current_vel(tp->shared, 0.0);
    set_requested_vel(tp->shared, 0.0);
    set_distance_to_go(tp->shared, 0.0);
    zero_dtg(tp->shared);

    return tpClearDIOs(tp);
}

/**
 * Fully initialize the tp structure.
 * Sets tp configuration to default values and calls tpClear to create a fresh,
 * empty queue.
 */
int tpInit(TP_STRUCT * const tp)
{
    tp->cycleTime = 0.0;
    //Velocity limits
    tp->vLimit = 0.0;
    tp->ini_maxvel = 0.0;
    //Accelerations
    tp->aLimit = 0.0;
    PmCartesian acc_bound;
    //FIXME this acceleration bound isn't valid (nor is it used)
    tpGetMachineAccelBounds(tp, &acc_bound);
    tpGetMachineActiveLimit(&tp->aMax, &acc_bound);
    //Angular limits
    tp->wMax = 0.0;
    tp->wDotMax = 0.0;

    tp->spindle.waiting_for_index = MOTION_INVALID_ID;
    tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;

    ZERO_EMC_POSE(tp->currentPos);

    PmCartesian vel_bound;
    tpGetMachineVelBounds(tp, &vel_bound);
    tpGetMachineActiveLimit(&tp->vMax, &vel_bound);

    tp->old_spindlepos = 0.0; // sanity - just a temporary
    return tpClear(tp);
}

/**
 * Set the cycle time for the trajectory planner.
 */
int tpSetCycleTime(TP_STRUCT * const tp, double secs)
{
    if (0 == tp || secs <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->cycleTime = secs;

    return TP_ERR_OK;
}

/**
 * Set requested velocity and absolute maximum velocity (bounded by machine).
 * This is called before adding lines or circles, specifying vMax (the velocity
 * requested by the F word) and ini_maxvel, the max velocity possible before
 * meeting a machine constraint caused by an AXIS's max velocity.  (the TP is
 * allowed to go up to this high when feed override >100% is requested)  These
 * settings apply to subsequent moves until changed.
 */
int tpSetVmax(TP_STRUCT * const tp, double vMax, double ini_maxvel)
{
    if (0 == tp || vMax <= 0.0 || ini_maxvel <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->vMax = vMax;
    tp->ini_maxvel = ini_maxvel;

    return TP_ERR_OK;
}

/**
 * (?) Set the tool tip maximum velocity.
 * I think this is the [TRAJ] max velocity. This should be the max velocity of
 * const the TOOL TIP, not necessarily any particular axis. This applies to
 * subsequent moves until changed.
 */
int tpSetVlimit(TP_STRUCT * const tp, double vLimit)
{
    if (!tp) return TP_ERR_FAIL;

    if (vLimit < 0.)
        tp->vLimit = 0.;
    else
        tp->vLimit = vLimit;

    return TP_ERR_OK;
}

/** Sets the max acceleration for the trajectory planner. */
int tpSetAmax(TP_STRUCT * const tp, double aMax)
{
    if (0 == tp || aMax <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->aMax = aMax;

    return TP_ERR_OK;
}
/** Sets the max jerk for the trajectory planner. */
int tpSetJmax(TP_STRUCT * const tp, double jMax)
{
    if (0 == tp || jMax <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->jMax = jMax;

    return TP_ERR_OK;
}
/**
 * Sets the id that will be used for the next appended motions.
 * nextId is incremented so that the next time a motion is appended its id will
 * be one more than the previous one, modulo a signed int. If you want your own
 * ids for each motion, call this before each motion you append and stick what
 * you want in here.
 */
int tpSetId(TP_STRUCT * const tp, int id)
{

    if (!MOTION_ID_VALID(id)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpSetId: invalid motion id %d\n", id);
        return TP_ERR_FAIL;
    }

    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    tp->nextId = id;

    return TP_ERR_OK;
}

/** Returns the id of the last motion that is currently
  executing.*/
int tpGetExecId(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    return tp->execId;
}

struct state_tag_t tpGetExecTag(TP_STRUCT * const tp)
{
    if (0 == tp) {
        struct state_tag_t empty = {0};
        return empty;
    }

    return tp->execTag;
}


/**
 * Sets the termination condition for all subsequent queued moves.
 * If cond is TC_TERM_COND_STOP, motion comes to a stop before a subsequent move
 * begins. If cond is TC_TERM_COND_PARABOLIC, the following move is begun when the
 * current move slows below a calculated blend velocity.
 */
int tpSetTermCond(TP_STRUCT * const tp, int cond, double tolerance)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }

    switch (cond) {
        //Purposeful waterfall for now
        case TC_TERM_COND_PARABOLIC:
        case TC_TERM_COND_TANGENT:
        case TC_TERM_COND_EXACT:
        case TC_TERM_COND_STOP:
            tp->termCond = cond;
            tp->tolerance = tolerance;
            break;
        default:
            //Invalid condition
            return  -1;
    }

    return TP_ERR_OK;
}

/**
 * Used to tell the tp the initial position.
 * It sets the current position AND the goal position to be the same.  Used
 * only at TP initialization and when switching modes.
 */
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    int res_invalid = tpSetCurrentPos(tp, pos);
    if (res_invalid) {
        return TP_ERR_FAIL;
    }

    tp->goalPos = *pos;
    return TP_ERR_OK;
}


/**
 * Set current position.
 * It sets the current position AND the goal position to be the same.  Used
 * only at TP initialization and when switching modes.
 */
int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (emcPoseValid(pos)) {
        tp->currentPos = *pos;
        return TP_ERR_OK;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Tried to set invalid pose in tpSetCurrentPos on id %d!"
                "pos is %.12g, %.12g, %.12g\n",
                tp->execId,
                pos->tran.x,
                pos->tran.y,
                pos->tran.z);
        return TP_ERR_INVALID;
    }
}


int tpAddCurrentPos(TP_STRUCT * const tp, EmcPose const * const disp)
{
    if (!tp || !disp) {
        return TP_ERR_MISSING_INPUT;
    }

    if (emcPoseValid(disp)) {
        emcPoseSelfAdd(&tp->currentPos, disp);
        return TP_ERR_OK;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Tried to set invalid pose in tpAddCurrentPos on id %d!"
                "disp is %.12g, %.12g, %.12g\n",
                tp->execId,
                disp->tran.x,
                disp->tran.y,
                disp->tran.z);
        return TP_ERR_INVALID;
    }
}


/**
 * Check for valid tp before queueing additional moves.
 */
int tpErrorCheck(TP_STRUCT const * const tp) {

    if (!tp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is null\n");
        return TP_ERR_FAIL;
    }
    if (tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is aborting\n");
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}


/**
 * Find the "peak" velocity a segment can acheive if its velocity profile is triangular.
 * This is used to estimate blend velocity, though by itself is not enough
 * (since requested velocity and max velocity could be lower).
 */
STATIC double tpCalculateTriangleVel(TP_STRUCT const * const tp, TC_STRUCT * const tc) {
    //Compute peak velocity for blend calculations
    double acc_scaled = tpGetScaledAccel(tp, tc);
    double length = tc->target;
    if (!tc->finalized) {
        // blending may remove up to 1/2 of the segment
        length /= 2.0;
    }
    double triangle_vel = pmSqrt( acc_scaled * length);
    tp_debug_print("triangle vel for segment %d is %f\n", tc->id, triangle_vel);

    return triangle_vel;
}


/**
 * Handles the special case of blending into an unfinalized segment.
 * The problem here is that the last segment in the queue can always be cut
 * short by a blend to the next segment. However, we can only ever consume at
 * most 1/2 of the segment. This function computes the worst-case final
 * velocity the previous segment can have, if we want to exactly stop at the
 * halfway point.
 */
STATIC double tpCalculateOptimizationInitialVel(TP_STRUCT const * const tp, TC_STRUCT * const tc)
{
    double acc_scaled = tpGetScaledAccel(tp, tc);
    //FIXME this is defined in two places!
    double triangle_vel = pmSqrt( acc_scaled * tc->target * BLEND_DIST_FRACTION);
    double max_vel = tpGetMaxTargetVel(tp, tc);
    tp_debug_print("optimization initial vel for segment %d is %f\n", tc->id, triangle_vel);
    return rtapi_fmin(triangle_vel, max_vel);
}


/**
 * Initialize a blend arc from its parent lines.
 * This copies and initializes properties from the previous and next lines to
 * initialize a blend arc. This function does not handle connecting the
 * segments together, however.
 */
STATIC int tpInitBlendArcFromPrev(TP_STRUCT const * const tp,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT* const blend_tc,
        double vel,
        double ini_maxvel,
        double acc) {

#ifdef TP_SHOW_BLENDS
    int canon_motion_type = EMC_MOTION_TYPE_ARC;
#else
    int canon_motion_type = prev_tc->canon_motion_type;
#endif

    tcInit(blend_tc,
            TC_SPHERICAL,
            canon_motion_type,
            tp->cycleTime,
            prev_tc->enables,
            prev_tc->atspeed);
    //FIXME refactor into Init
    blend_tc->tag = prev_tc->tag;

    // Copy over state data from TP
    tcSetupState(blend_tc, tp);
    
    // Set kinematics parameters from blend calculations
    tcSetupMotion(blend_tc,
            vel,
            ini_maxvel,
            acc,
            0,  //!< will update blend_tc->jrek at next step
            tp->cycleTime);
    // TODO: confirm if we should calculate JERK for BLENDER
    blend_tc->jerk = prev_tc->jerk;

    // Skip syncdio setup since this blend extends the previous line
    blend_tc->syncdio = prev_tc->syncdio; //enqueue the list of DIOs that need toggling

    // find "helix" length for target
    double length;
    arcLength(&blend_tc->coords.arc.xyz, &length);
    tp_info_print("blend tc length = %f\n",length);
    blend_tc->target = length;
    blend_tc->nominal_length = length;

    // Set the blend arc to be tangent to the next segment
    tcSetTermCond(blend_tc, TC_TERM_COND_TANGENT);

    //NOTE: blend arc radius and everything else is finalized, so set this to 1.
    //In the future, radius may be adjustable.
    tcFinalizeLength(blend_tc);

    return TP_ERR_OK;
}

STATIC int tcSetLineXYZ(TC_STRUCT * const tc, PmCartLine const * const line)
{

    //Update targets with new arc length
    if (!line || tc->motion_type != TC_LINEAR) {
        return TP_ERR_FAIL;
    }
    if (!tc->coords.line.abc.tmag_zero || !tc->coords.line.uvw.tmag_zero) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SetLineXYZ does not supportABC or UVW motion\n");
        return TP_ERR_FAIL;
    }

    tc->coords.line.xyz = *line;
    tc->target = line->tmag;
    return TP_ERR_OK;
}



/**
 * Compare performance of blend arc and equivalent tangent speed.
 * If we can go faster by assuming the segments are already tangent (and
 * slowing down some), then prefer this over using the blend arc. This is
 * mostly useful for some odd arc-to-arc cases where the blend arc becomes very
 * short (and therefore slow).
 */
STATIC int tpCheckTangentPerformance(TP_STRUCT const * const tp,
        TC_STRUCT * const prev_tc,
        TC_STRUCT * const tc,
        TC_STRUCT * const blend_tc)
{
    tcFinalizeLength(blend_tc);
    if (blend_tc->maxvel < tc->kink_vel) {
        tp_debug_print("segment maxvel %f is less than kink_vel %f, falling back to simple tangent\n",
                blend_tc->maxvel, tc->kink_vel);

        // Fall back to tangent, using kink_vel as final velocity
        tcSetTermCond(prev_tc, TC_TERM_COND_TANGENT);

        // Finally, reduce acceleration proportionally to prevent violations during "kink"
        const double kink_ratio = get_arcBlendTangentKinkRatio(tp->shared);
        tpAdjustAccelForTangent(tp, tc, kink_ratio);
        tpAdjustAccelForTangent(tp, prev_tc, kink_ratio);
        return TP_ERR_NO_ACTION;
    } else {
        tp_debug_print("segment maxvel %f is greater than kink_vel %f, using blend arc\n",
                blend_tc->maxvel, tc->kink_vel);
        // Since we get near-exact tangency with the blend arc, throw out the kink velocity since it's not relevant
        tc->kink_vel = -1.0;
    }

    return TP_ERR_OK;
}


STATIC int tpCreateLineArcBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{
    tp_debug_print("-- Starting LineArc blend arc --\n");

    PmCartesian acc_bound, vel_bound;
    
    //Get machine limits
    tpGetMachineAccelBounds(tp, &acc_bound);
    tpGetMachineVelBounds(tp, &vel_bound);

    //Populate blend geometry struct
    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;

    int res_init = blendInit3FromLineArc(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            get_maxFeedScale(tp->shared));

    if (res_init != TP_ERR_OK) {
        tp_debug_print("blend init failed with code %d, aborting blend arc\n",
                res_init);
        return res_init;
    }

    // Check for coplanarity based on binormal and tangents
    int coplanar = pmCartCartParallel(&geom.binormal,
            &tc->coords.circle.xyz.normal,
            TP_ANGLE_EPSILON);

    if (!coplanar) {
        tp_debug_print("aborting arc, not coplanar\n");
        return TP_ERR_FAIL;
    }

    int res_param = blendComputeParameters(&param);

    int res_points = blendFindPoints3(&points_approx, &geom, &param);
    
    int res_post = blendLineArcPostProcess(&points_exact,
            &points_approx,
            &param, 
            &geom, &prev_tc->coords.line.xyz,
            &tc->coords.circle.xyz);

    //Catch errors in blend setup
    if (res_init || res_param || res_points || res_post) {
        tp_debug_print("Got %d, %d, %d, %d for init, param, points, post, aborting arc\n",
                res_init,
                res_param,
                res_points,
                res_post);
        return TP_ERR_FAIL;
    }
    
    /* If blend calculations were successful, then we're ready to create the
     * blend arc.
     */

    if (points_exact.trim2 > param.phi2_max) {
        tp_debug_print("trim2 %f > phi2_max %f, aborting arc...\n",
                points_exact.trim2,
                param.phi2_max);
        return TP_ERR_FAIL;
    }

    blendCheckConsume(&param, &points_exact, prev_tc, get_arcBlendGapCycles(tp->shared));
    //Store working copies of geometry
    PmCartLine line1_temp = prev_tc->coords.line.xyz;
    PmCircle circ2_temp = tc->coords.circle.xyz;

    // Change lengths of circles
    double new_len1 = line1_temp.tmag - points_exact.trim1;
    int res_stretch1 = pmCartLineStretch(&line1_temp,
            new_len1,
            false);

    double phi2_new = tc->coords.circle.xyz.angle - points_exact.trim2;

    tp_debug_print("phi2_new = %f\n",phi2_new);
    int res_stretch2 = pmCircleStretch(&circ2_temp,
            phi2_new,
            true);
    //TODO create blends
    if (res_stretch1 || res_stretch2) {
        tp_debug_print("segment resize failed, aborting arc\n");
        return TP_ERR_FAIL;
    }

    //Get exact start and end points to account for spiral in arcs
    pmCartLinePoint(&line1_temp,
            line1_temp.tmag,
            &points_exact.arc_start);
    pmCirclePoint(&circ2_temp,
            0.0,
            &points_exact.arc_end);
    //TODO deal with large spiral values, or else detect and fall back?

    blendPoints3Print(&points_exact);
    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz,
            &points_exact,
            &geom,
            &param);
    if (res_arc < 0) {
        tp_debug_print("arc creation failed, aborting arc\n");
        return TP_ERR_FAIL;
    }

    // Note that previous restrictions don't allow ABC or UVW movement, so the
    // end and start points should be identical
    blend_tc->coords.arc.abc = prev_tc->coords.line.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.line.uvw.end;

    //set the max velocity to v_plan, since we'll violate constraints otherwise.
    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);
    blend_tc->target_vel = param.v_actual;

    int res_tangent = checkTangentAngle(&circ2_temp,
            &blend_tc->coords.arc.xyz,
            &geom,
            &param,
            tp->cycleTime,
            true);

    if (res_tangent < 0) {
        tp_debug_print("failed tangent check, aborting arc...\n");
        return TP_ERR_FAIL;
    }

    if (tpCheckTangentPerformance(tp, prev_tc, tc, blend_tc) == TP_ERR_NO_ACTION) {
        return TP_ERR_NO_ACTION;
    }

    tp_debug_print("Passed all tests, updating segments\n");

    //Cleanup any mess from parabolic
    tc->blend_prev = 0;

    //TODO refactor to pass consume to connect function
    if (param.consume) {
        //Since we're consuming the previous segment, pop the last line off of the queue
        int res_pop = tcqPopBack(&tp->queue);
        if (res_pop) {
            tp_debug_print("failed to pop segment, aborting arc\n");
            return TP_ERR_FAIL;
        }
    } else {
        tcSetLineXYZ(prev_tc, &line1_temp);
    }
    tcSetCircleXYZ(tc, &circ2_temp);

    tcSetTermCond(prev_tc, TC_TERM_COND_TANGENT);

    return TP_ERR_OK;
}


STATIC int tpCreateArcLineBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{

    tp_debug_print("-- Starting ArcLine blend arc --\n");

    PmCartesian acc_bound, vel_bound;
    
    //Get machine limits
    tpGetMachineAccelBounds(tp, &acc_bound);
    tpGetMachineVelBounds(tp, &vel_bound);

    //Populate blend geometry struct
    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;
    param.consume = 0;

    int res_init = blendInit3FromArcLine(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            get_maxFeedScale(tp->shared));

    if (res_init != TP_ERR_OK) {
        tp_debug_print("blend init failed with code %d, aborting blend arc\n",
                res_init);
        return res_init;
    }

    // Check for coplanarity based on binormal
    int coplanar = pmCartCartParallel(&geom.binormal,
            &prev_tc->coords.circle.xyz.normal,
            TP_ANGLE_EPSILON);

    if (!coplanar) {
        tp_debug_print("aborting arc, not coplanar\n");
        return TP_ERR_FAIL;
    }

    int res_param = blendComputeParameters(&param);

    int res_points = blendFindPoints3(&points_approx, &geom, &param);
    
    int res_post = blendArcLinePostProcess(&points_exact,
            &points_approx,
            &param, 
            &geom, &prev_tc->coords.circle.xyz,
            &tc->coords.line.xyz);

    //Catch errors in blend setup
    if (res_init || res_param || res_points || res_post) {
        tp_debug_print("Got %d, %d, %d, %d for init, param, points, post\n",
                res_init,
                res_param,
                res_points,
                res_post);
        return TP_ERR_FAIL;
    }
    
    blendCheckConsume(&param, &points_exact, prev_tc, get_arcBlendGapCycles(tp->shared));

    /* If blend calculations were successful, then we're ready to create the
     * blend arc.
     */

    // Store working copies of geometry
    PmCircle circ1_temp = prev_tc->coords.circle.xyz;
    PmCartLine line2_temp = tc->coords.line.xyz;

    // Update start and end points of segment copies
    double phi1_new = circ1_temp.angle - points_exact.trim1;

    if (points_exact.trim1 > param.phi1_max) {
        tp_debug_print("trim1 %f > phi1_max %f, aborting arc...\n",
                points_exact.trim1,
                param.phi1_max);
        return TP_ERR_FAIL;
    }

    int res_stretch1 = pmCircleStretch(&circ1_temp,
            phi1_new,
            false);
    if (res_stretch1 != TP_ERR_OK) {
        return TP_ERR_FAIL;
    }

    double new_len2 = tc->target - points_exact.trim2;
    int res_stretch2 = pmCartLineStretch(&line2_temp,
            new_len2,
            true);

    if (res_stretch1 || res_stretch2) {
        tp_debug_print("segment resize failed, aborting arc\n");
        return TP_ERR_FAIL;
    }

    pmCirclePoint(&circ1_temp,
            circ1_temp.angle,
            &points_exact.arc_start);

    pmCartLinePoint(&line2_temp,
            0.0,
            &points_exact.arc_end);

    blendPoints3Print(&points_exact);

    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points_exact, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    // Note that previous restrictions don't allow ABC or UVW movement, so the
    // end and start points should be identical
    blend_tc->coords.arc.abc = tc->coords.line.abc.start;
    blend_tc->coords.arc.uvw = tc->coords.line.uvw.start;

    //set the max velocity to v_plan, since we'll violate constraints otherwise.
    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);
    blend_tc->target_vel = param.v_actual;

    int res_tangent = checkTangentAngle(&circ1_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, false);
    if (res_tangent) {
        tp_debug_print("failed tangent check, aborting arc...\n");
        return TP_ERR_FAIL;
    }

    if (tpCheckTangentPerformance(tp, prev_tc, tc, blend_tc) == TP_ERR_NO_ACTION) {
        return TP_ERR_NO_ACTION;
    }

    tp_debug_print("Passed all tests, updating segments\n");

    tcSetCircleXYZ(prev_tc, &circ1_temp);
    tcSetLineXYZ(tc, &line2_temp);

    //Cleanup any mess from parabolic
    tc->blend_prev = 0;
    tcSetTermCond(prev_tc, TC_TERM_COND_TANGENT);
    return TP_ERR_OK;
}

STATIC int tpCreateArcArcBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{

    tp_debug_print("-- Starting ArcArc blend arc --\n");
    //TODO type checks
    int colinear = pmCartCartParallel(&prev_tc->coords.circle.xyz.normal,
            &tc->coords.circle.xyz.normal, TP_ANGLE_EPSILON);
    if (!colinear) {
        // Fail out if not collinear
        tp_debug_print("arc abort: not coplanar\n");
        return TP_ERR_FAIL;
    }

    PmCartesian acc_bound, vel_bound;
    
    //Get machine limits
    tpGetMachineAccelBounds(tp, &acc_bound);
    tpGetMachineVelBounds(tp, &vel_bound);

    //Populate blend geometry struct
    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;

    int res_init = blendInit3FromArcArc(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            get_maxFeedScale(tp->shared));

    if (res_init != TP_ERR_OK) {
        tp_debug_print("blend init failed with code %d, aborting blend arc\n",
                res_init);
        return res_init;
    }

    int res_param = blendComputeParameters(&param);
    int res_points = blendFindPoints3(&points_approx, &geom, &param);
    
    int res_post = blendArcArcPostProcess(&points_exact,
            &points_approx,
            &param, 
            &geom, &prev_tc->coords.circle.xyz,
            &tc->coords.circle.xyz);

    //Catch errors in blend setup
    if (res_init || res_param || res_points || res_post) {
        tp_debug_print("Got %d, %d, %d, %d for init, param, points, post\n",
                res_init,
                res_param,
                res_points,
                res_post);

        return TP_ERR_FAIL;
    }

    blendCheckConsume(&param, &points_exact, prev_tc, get_arcBlendGapCycles(tp->shared));
    
    /* If blend calculations were successful, then we're ready to create the
     * blend arc. Begin work on temp copies of each circle here:
     */

    double phi1_new = prev_tc->coords.circle.xyz.angle - points_exact.trim1;
    double phi2_new = tc->coords.circle.xyz.angle - points_exact.trim2;

    // TODO pare down this debug output
    tp_debug_print("phi1_new = %f, trim1 = %f\n", phi1_new, points_exact.trim1);
    tp_debug_print("phi2_new = %f, trim2 = %f\n", phi2_new, points_exact.trim2);

    if (points_exact.trim1 > param.phi1_max) {
        tp_debug_print("trim1 %f > phi1_max %f, aborting arc...\n",
                points_exact.trim1,
                param.phi1_max);
        return TP_ERR_FAIL;
    }

    if (points_exact.trim2 > param.phi2_max) {
        tp_debug_print("trim2 %f > phi2_max %f, aborting arc...\n",
                points_exact.trim2,
                param.phi2_max);
        return TP_ERR_FAIL;
    }

    //Store working copies of geometry
    PmCircle circ1_temp = prev_tc->coords.circle.xyz;
    PmCircle circ2_temp = tc->coords.circle.xyz;

    int res_stretch1 = pmCircleStretch(&circ1_temp,
            phi1_new,
            false);
    if (res_stretch1 != TP_ERR_OK) {
        return TP_ERR_FAIL;
    }

    int res_stretch2 = pmCircleStretch(&circ2_temp,
            phi2_new,
            true);
    if (res_stretch1 || res_stretch2) {
        tp_debug_print("segment resize failed, aborting arc\n");
        return TP_ERR_FAIL;
    }

    //Get exact start and end points to account for spiral in arcs
    pmCirclePoint(&circ1_temp,
            circ1_temp.angle,
            &points_exact.arc_start);
    pmCirclePoint(&circ2_temp,
            0.0,
            &points_exact.arc_end);

    tp_debug_print("Modified arc points\n");
    blendPoints3Print(&points_exact);
    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points_exact, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    // Note that previous restrictions don't allow ABC or UVW movement, so the
    // end and start points should be identical
    blend_tc->coords.arc.abc = prev_tc->coords.circle.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.circle.uvw.end;

    //set the max velocity to v_plan, since we'll violate constraints otherwise.
    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);
    blend_tc->target_vel = param.v_actual;

    int res_tangent1 = checkTangentAngle(&circ1_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, false);
    int res_tangent2 = checkTangentAngle(&circ2_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, true);
    if (res_tangent1 || res_tangent2) {
        tp_debug_print("failed tangent check, aborting arc...\n");
        return TP_ERR_FAIL;
    }

    if (tpCheckTangentPerformance(tp, prev_tc, tc, blend_tc) == TP_ERR_NO_ACTION) {
        return TP_ERR_NO_ACTION;
    }

    tp_debug_print("Passed all tests, updating segments\n");

    tcSetCircleXYZ(prev_tc, &circ1_temp);
    tcSetCircleXYZ(tc, &circ2_temp);

    //Cleanup any mess from parabolic
    tc->blend_prev = 0;
    tcSetTermCond(prev_tc, TC_TERM_COND_TANGENT);

    return TP_ERR_OK;
}


STATIC int tpCreateLineLineBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc,
        TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{

    tp_debug_print("-- Starting LineLine blend arc --\n");
    PmCartesian acc_bound, vel_bound;
    
    //Get machine limits
    tpGetMachineAccelBounds(tp, &acc_bound);
    tpGetMachineVelBounds(tp, &vel_bound);
    
    // Setup blend data structures
    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points;

    int res_init = blendInit3FromLineLine(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            get_maxFeedScale(tp->shared));

    if (res_init != TP_ERR_OK) {
        tp_debug_print("blend init failed with code %d, aborting blend arc\n",
                res_init);
        return res_init;
    }

    int res_blend = blendComputeParameters(&param);
    if (res_blend != TP_ERR_OK) {
        return res_blend;
    }

    blendFindPoints3(&points, &geom, &param);

    blendCheckConsume(&param, &points, prev_tc, get_arcBlendGapCycles(tp->shared));

    // Set up actual blend arc here
    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    // Note that previous restrictions don't allow ABC or UVW movement, so the
    // end and start points should be identical
    blend_tc->coords.arc.abc = prev_tc->coords.line.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.line.uvw.end;

    //set the max velocity to v_plan, since we'll violate constraints otherwise.
    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);
    blend_tc->target_vel = param.v_actual;

    if (tpCheckTangentPerformance(tp, prev_tc, tc, blend_tc) == TP_ERR_NO_ACTION) {
        return TP_ERR_NO_ACTION;
    }

    int retval = TP_ERR_FAIL;

    //TODO refactor to pass consume to connect function
    if (param.consume) {
        //Since we're consuming the previous segment, pop the last line off of the queue
        retval = tcqPopBack(&tp->queue);
        if (retval) {
            //This is unrecoverable since we've already changed the line. Something is wrong if we get here...
            rtapi_print_msg(RTAPI_MSG_ERR, "PopBack failed\n");
            return TP_ERR_FAIL;
        }
        //Since the blend arc meets the end of the previous line, we only need
        //to "connect" to the next line
        retval = tcConnectBlendArc(NULL, tc, &points.arc_start, &points.arc_end);
    } else {
        //TODO refactor connect function to stretch lines and check for bad stretching
        tp_debug_print("keeping previous line\n");
        retval = tcConnectBlendArc(prev_tc, tc, &points.arc_start, &points.arc_end);
    }
    return retval;
}


/**
 * Add a newly created motion segment to the tp queue.
 * Returns an error code if the queue operation fails, otherwise adds a new
 * segment to the queue and updates the end point of the trajectory planner.
 */
STATIC inline int tpAddSegmentToQueue(TP_STRUCT * const tp, TC_STRUCT * const tc, int inc_id) {

    tc->id = tp->nextId;
    if (tcqPut(&tp->queue, tc) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
        return TP_ERR_FAIL;
    }
    if (inc_id) {
        tp->nextId++;
    }

    // Store end of current move as new final goal of TP
    tcGetEndpoint(tc, &tp->goalPos);
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    //Fixing issue with duplicate id's?
    tp_debug_print("Adding TC id %d of type %d, total length %0.08f\n",tc->id,tc->motion_type,tc->target);

    return TP_ERR_OK;
}

STATIC int tpCheckCanonType(TC_STRUCT * const prev_tc, TC_STRUCT const * const tc)
{
    if (!tc || !prev_tc) {
        return TP_ERR_FAIL;
    }
    if ((prev_tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE) ^
            (tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE)) {
        tp_debug_print("Can't blend between rapid and feed move, aborting arc\n");
        tcSetTermCond(prev_tc, TC_TERM_COND_STOP);
    }
    return TP_ERR_OK;
}

STATIC int tpSetupSyncedIO(TP_STRUCT * const tp, TC_STRUCT * const tc) {
    if (tp->syncdio.anychanged != 0) {
        tc->syncdio = tp->syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(tp); // clear out the list, in order to prepare for the next time we need to use it
        return TP_ERR_OK;
    } else {
        tc->syncdio.anychanged = 0;
        return TP_ERR_NO_ACTION;
    }
}

/**
 * SPINDLE_SYNC_MOTION:
 *      -- RIGID_TAPPING(G33.1)
 *      -- CSS(G33 w/ G96)
 *      -- THREADING(G33 w/ G97)
 */
int tpAddSpindleSyncMotion(
        TP_STRUCT *tp,
        EmcPose end,
        double vel,
        double ini_maxvel,
        double acc,
        double jerk,
        int ssm_mode,
        unsigned char enables,
        struct state_tag_t tag)
{
    PmCartLine line_xyz;
    PmCartesian start_xyz, end_xyz;
    PmCartesian abc, uvw;
    int atspeed;

    if (tpErrorCheck(tp)) {
        return TP_ERR_FAIL;
    }

    tp_info_print("== AddSpindleSyncMotion ==\n");

    if(!tp->synchronized) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Cannot add unsynchronized SpindleSyncMotion.\n");
        return TP_ERR_FAIL;
    }

    TC_STRUCT tc = {0};

    assert (ssm_mode < 2);
    assert (ssm_mode >= 0);
    if (ssm_mode == 0){
        atspeed = 1; // G33 needs to wait for spindle atspeed
    } else {
        atspeed = 0; // G33.1(RIGID_TAP) does not wait for atspeed
    }
    tcInit(&tc,
            TC_SPINDLE_SYNC_MOTION,
            0,
            tp->cycleTime,
            enables,
            atspeed);

    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    // Copy in motion parameters
    tcSetupMotion(&tc,
            vel,
            ini_maxvel,
            acc,
            jerk,
            tp->cycleTime);

    // Setup SpindleSyncMotion Geometry
    start_xyz = tp->goalPos.tran;
    end_xyz = end.tran;

    // abc cannot move
    abc.x = tp->goalPos.a;
    abc.y = tp->goalPos.b;
    abc.z = tp->goalPos.c;

    uvw.x = tp->goalPos.u;
    uvw.y = tp->goalPos.v;
    uvw.z = tp->goalPos.w;

    pmCartLineInit(&line_xyz, &start_xyz, &end_xyz);

    // tc.target: set as revolutions of spindle
    tc.target = line_xyz.tmag / tp->uu_per_rev;
    tp_info_print("jerk(%f) req_vel(%f) req_acc(%f) ini_maxvel(%f)\n",
                   jerk, vel, acc, ini_maxvel);
    tp_info_print("target(%f) uu_per_rev(%f)\n", tc.target, tp->uu_per_rev);

    tc.distance_to_go = tc.target;
    tc.feed_override = 0.0;
    tc.active = 0;

    tc.coords.spindle_sync.xyz = line_xyz;
    tc.coords.spindle_sync.abc = abc;
    tc.coords.spindle_sync.uvw = uvw;
    // updated spindle speed constrain based on spindleSyncMotionMsg.vel of emccanon.cc
    tc.coords.spindle_sync.spindle_reqvel = vel;

    tc.motion_type = TC_SPINDLE_SYNC_MOTION;

    tc.canon_motion_type = 0;
    tc.tolerance = tp->tolerance;

    tc.synchronized = tp->synchronized;
    tc.uu_per_rev = tp->uu_per_rev;
    tc.enables = enables;
    tc.indexrotary = -1;
    tc.coords.spindle_sync.mode = ssm_mode;

    if (vel > 0)        // vel is requested spindle velocity
    {
        tc.coords.spindle_sync.spindle_dir = 1.0;
    } else
    {
        tc.coords.spindle_sync.spindle_dir = -1.0;
    }

    // Force exact stop mode after rigid tapping regardless of TP setting
    tcSetTermCond(&tc, TC_TERM_COND_STOP);

    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);
    tcFinalizeLength(prev_tc);
    tcFlagEarlyStop(prev_tc, &tc);
    int retval = tpAddSegmentToQueue(tp, &tc, true); /* will update tp->goalPos */
    tpRunOptimization(tp);
    if (retval != TP_ERR_OK)
        return (retval);

    if (ssm_mode == 1)  // for G33.1
    {   // REVERSING
        pmCartLineInit(&line_xyz, &end_xyz, &start_xyz);  // reverse the line direction
        tc.coords.spindle_sync.xyz = line_xyz;
        if (vel > 0)
        {
            tc.coords.spindle_sync.spindle_dir = -1.0;
        } else
        {
            tc.coords.spindle_sync.spindle_dir = 1.0;
        }

        prev_tc = tcqLast(&tp->queue);
        tcFinalizeLength(prev_tc);
        tcFlagEarlyStop(prev_tc, &tc);
        retval = tpAddSegmentToQueue(tp, &tc, true); /* will update tp->goalPos */
        tpRunOptimization(tp);
    }
    return retval;
}

STATIC blend_type_t tpCheckBlendArcType(TP_STRUCT const * const tp,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc) {

    if (!prev_tc || !tc) {
        tp_debug_print("prev_tc or tc doesn't exist\n");
        return BLEND_NONE;
    }

    //If exact stop, we don't compute the arc
    if (prev_tc->term_cond != TC_TERM_COND_PARABOLIC) {
        tp_debug_print("Wrong term cond = %u\n", prev_tc->term_cond);
        return BLEND_NONE;
    }

    //If we have any rotary axis motion, then don't create a blend arc
    if (tpRotaryMotionCheck(tp, tc) || tpRotaryMotionCheck(tp, prev_tc)) {
        tp_debug_print("One of the segments has rotary motion, aborting blend arc\n");
        return BLEND_NONE;
    }

    tp_debug_print("Motion types: prev_tc = %u, tc = %u\n",
            prev_tc->motion_type,tc->motion_type);
    //If not linear blends, we can't easily compute an arc
    if ((prev_tc->motion_type == TC_LINEAR) && (tc->motion_type == TC_LINEAR)) {
        return BLEND_LINE_LINE;
    } else if (prev_tc->motion_type == TC_LINEAR && tc->motion_type == TC_CIRCULAR) {
        return BLEND_LINE_ARC;
    } else if (prev_tc->motion_type == TC_CIRCULAR && tc->motion_type == TC_LINEAR) {
        return BLEND_ARC_LINE;
    } else if (prev_tc->motion_type == TC_CIRCULAR && tc->motion_type == TC_CIRCULAR) {
        return BLEND_ARC_ARC;
    } else {
        return BLEND_NONE;
    }
}

/**
 * Do "rising tide" optimization to update lookahead_target length for each
 * queued segment.Walk along the queue from the back to the front. Based on the
 * "current" segment's target, calculate the previous segment's lookahead_target.
 * The depth we walk along the queue is controlled by the TP_LOOKAHEAD_DEPTH
 * constant for now. The process safetly aborts early due to a short queue or
 * other conflicts.
 */
STATIC int tpRunOptimization(TP_STRUCT * const tp) {
    // Pointers to the "current", previous, and 2nd previous trajectory
    // components. Current in this context means the segment being optimized,
    // NOT the currently excecuting segment.

    TC_STRUCT *tc;
    TC_STRUCT *prev1_tc;

    int ind, x;
    int len = tcqLen(&tp->queue);

    /* Starting at the 2nd to last element in the queue, work backwards towards
     * the front. We can't do anything with the very last element because its
     * length may change if a new line is added to the queue.*/

    for (x = 1; x < get_arcBlendOptDepth(tp->shared) + 2; ++x) {
        tp_info_print("==== Optimization step %d ====\n",x);

        // Update the pointers to the trajectory segments in use
        ind = len-x;
        tc = tcqItem(&tp->queue, ind);
        prev1_tc = tcqItem(&tp->queue, ind-1);

        if ( !prev1_tc || !tc) {
            tp_debug_print(" Reached end of queue in optimization\n");
            return TP_ERR_OK;
        }

        // stop optimizing if we hit a non-tangent segment (final velocity
        // stays zero)
        if (prev1_tc->term_cond != TC_TERM_COND_TANGENT) {
            prev1_tc->lookahead_target = 0;
            tp_debug_print("Found non-tangent segment, stopping optimization\n");
            return TP_ERR_OK;
        }
        
        rtapi_print_msg(RTAPI_MSG_DBG, "FIXME: emcmotStatus->net_feed_scale = 100%%, will let velocity down to zero\n");
        // for circular motion, its maxvel is limited by tangential velocity
        if (((tc->motion_type == TC_CIRCULAR) || (tc->motion_type == TC_SPHERICAL)) &&
            ((get_net_feed_scale(tp->shared) * tc->reqvel * tc->cycle_time) > tc->maxvel))
        {   // sharp arc that is not allowed for requested-velocity
            prev1_tc->lookahead_target = tc->target;
            tp_debug_print("Found sharp arc, stopping optimization\n");
            return TP_ERR_OK;
        }

        //Somewhat pedantic check for other conditions that would make blending unsafe
        if (prev1_tc->splitting || prev1_tc->blending_next) {
            tp_debug_print("segment %d is already blending, cannot optimize safely!\n",
                    ind-1);
            return TP_ERR_OK;
        }

        prev1_tc->lookahead_target = tc->target + tc->lookahead_target;
        tp_info_print("  current term = %u, motion_type = %u, id = %u, accel_mode = %d target(%f) lookahead(%f)\n",
                tc->term_cond, tc->motion_type, tc->id, tc->accel_mode, tc->target, tc->lookahead_target);
        tp_info_print("  prev term = %u, motion_type = %u, id = %u, accel_mode = %d target(%f) lookahead(%f)\n",
                prev1_tc->term_cond, prev1_tc->motion_type, prev1_tc->id, prev1_tc->accel_mode, tc->target, tc->lookahead_target);
        if (!tc->finalized) {
            tp_debug_print("Segment %d, type %d not finalized, continuing\n",tc->id,tc->motion_type);
            // use worst-case final velocity that allows for up to 1/2 of a segment to be consumed.
            prev1_tc->finalvel = rtapi_fmin(prev1_tc->maxvel, tpCalculateOptimizationInitialVel(tp,tc));
            tc->finalvel = 0.0;
        }

//        tpComputeOptimalVelocity(tp, tc, prev1_tc);
//
//        tc->active_depth = x - 2 - hit_peaks;
//#ifdef TP_OPTIMIZATION_LAZY
//        if (tc->optimization_state == TC_OPTIM_AT_MAX) {
//            hit_peaks++;
//        }
//        if (hit_peaks > TP_OPTIMIZATION_CUTOFF) {
//            return TP_ERR_OK;
//        }
//#endif

    }
    tp_debug_print("Reached optimization depth limit\n");
    return TP_ERR_OK;
}

STATIC double pmCartAbsMax(PmCartesian const * const v)
{
    return rtapi_fmax(rtapi_fmax(rtapi_fabs(v->x),rtapi_fabs(v->y)),rtapi_fabs(v->z));
}

STATIC int tpAdjustAccelForTangent(TP_STRUCT const * const tp,
        TC_STRUCT * const tc,
        double normal_acc_scale)
{
        if (normal_acc_scale >= 1.0) {
            rtapi_print_msg(RTAPI_MSG_ERR,"Can't have acceleration scale %f > 1.0\n",normal_acc_scale);
            return TP_ERR_FAIL;
        }
        double a_reduction_ratio = 1.0 - normal_acc_scale;
        tp_debug_print(" acceleration reduction ratio is %f\n", a_reduction_ratio);
        tc->maxaccel *= a_reduction_ratio;
        return TP_ERR_OK;
}

/**
 * Check for tangency between the current segment and previous segment.
 * If the current and previous segment are tangent, then flag the previous
 * segment as tangent, and limit the current segment's velocity by the sampling
 * rate.
 */
STATIC int tpSetupTangent(TP_STRUCT const * const tp,
        TC_STRUCT * const prev_tc, TC_STRUCT * const tc) {
    if (!tc || !prev_tc) {
        tp_debug_print("missing tc or prev tc in tangent check\n");
        return TP_ERR_FAIL;
    }
    //If we have ABCUVW movement, then don't check for tangency
    if (tpRotaryMotionCheck(tp, tc) || tpRotaryMotionCheck(tp, prev_tc)) {
        tp_debug_print("found rotary axis motion\n");
        return TP_ERR_FAIL;
    }

    if (get_arcBlendOptDepth(tp->shared) < 2) {
        tp_debug_print("Optimization depth %d too low for tangent optimization\n",
                get_arcBlendOptDepth(tp->shared));
        return TP_ERR_FAIL;
    }

    if (prev_tc->term_cond == TC_TERM_COND_STOP) {
        tp_debug_print("Found exact stop condition\n");
        return TP_ERR_FAIL;
    }

    PmCartesian prev_tan, this_tan;

    int res_endtan = tcGetEndTangentUnitVector(prev_tc, &prev_tan);
    int res_starttan = tcGetStartTangentUnitVector(tc, &this_tan);
    if (res_endtan || res_starttan) {
        tp_debug_print("Got %d and %d from tangent vector calc\n",
                res_endtan, res_starttan);
    }

    tp_debug_print("prev tangent vector: %f %f %f\n", prev_tan.x, prev_tan.y, prev_tan.z);
    tp_debug_print("this tangent vector: %f %f %f\n", this_tan.x, this_tan.y, this_tan.z);

    double dot = -1.0;
    const double SHARP_CORNER_DEG = 2.0;
    const double SHARP_CORNER_THRESHOLD = rtapi_cos(PM_PI * (1.0 - SHARP_CORNER_DEG / 180.0));
    pmCartCartDot(&prev_tan, &this_tan, &dot);
    if (dot < SHARP_CORNER_THRESHOLD) {
        tp_debug_print("Found sharp corner\n");
        tcSetTermCond(prev_tc, TC_TERM_COND_STOP);
        return TP_ERR_FAIL;
    }

    // Calculate instantaneous acceleration required for change in direction
    // from v1 to v2, assuming constant speed
    double v_max1 = rtapi_fmin(prev_tc->maxvel, prev_tc->reqvel * get_maxFeedScale(tp->shared));
    double v_max2 = rtapi_fmin(tc->maxvel, tc->reqvel * get_maxFeedScale(tp->shared));
    double v_max = rtapi_fmin(v_max1, v_max2);
    tp_debug_print("tangent v_max = %f\n",v_max);

    double a_inst = v_max / tp->cycleTime;
    // Set up worst-case final velocity
    PmCartesian acc1, acc2, acc_diff;
    pmCartScalMult(&prev_tan, a_inst, &acc1);
    pmCartScalMult(&this_tan, a_inst, &acc2);
    pmCartCartSub(&acc2,&acc1,&acc_diff);

    //TODO store this in TP struct instead?
    PmCartesian acc_bound;
    tpGetMachineAccelBounds(tp, &acc_bound);

    PmCartesian acc_scale;
    findAccelScale(&acc_diff,&acc_bound,&acc_scale);
    tp_debug_print("acc_diff: %f %f %f\n",
            acc_diff.x,
            acc_diff.y,
            acc_diff.z);
    tp_debug_print("acc_scale: %f %f %f\n",
            acc_scale.x,
            acc_scale.y,
            acc_scale.z);

    //FIXME this ratio is arbitrary, should be more easily tunable
    double acc_scale_max = pmCartAbsMax(&acc_scale);
    //KLUDGE lumping a few calculations together here
    if (prev_tc->motion_type == TC_CIRCULAR || tc->motion_type == TC_CIRCULAR) {
        acc_scale_max /= BLEND_ACC_RATIO_TANGENTIAL;
    }

    const double kink_ratio = get_arcBlendTangentKinkRatio(tp->shared);
    if (acc_scale_max < kink_ratio) {
        tp_debug_print(" Kink acceleration within %g, treating as tangent\n", kink_ratio);
        tcSetTermCond(prev_tc, TC_TERM_COND_TANGENT);
        tc->kink_vel = v_max;
        tpAdjustAccelForTangent(tp, tc, acc_scale_max);
        tpAdjustAccelForTangent(tp, prev_tc, acc_scale_max);

        return TP_ERR_OK;
    } else {
        tc->kink_vel = v_max * kink_ratio / acc_scale_max;
        tp_debug_print("Kink acceleration scale %f above %f, kink vel = %f, blend arc may be faster\n",
                acc_scale_max,
                kink_ratio,
                tc->kink_vel);
        return TP_ERR_NO_ACTION;
    }

}


/**
 * Handle creating a blend arc when a new line segment is about to enter the queue.
 * This function handles the checks, setup, and calculations for creating a new
 * blend arc. Essentially all of the blend arc functions are called through
 * here to isolate the process.
 */
STATIC int tpHandleBlendArc(TP_STRUCT * const tp, TC_STRUCT * const tc) {

    tp_debug_print("*****************************************\n** Handle Blend Arc **\n");

    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);

    //If the previous segment has already started, then don't create a blend
    //arc for the next pair.
    // TODO May be able to lift this restriction if we can ensure that we leave
    // 1 timestep's worth of distance in prev_tc
    if ( !prev_tc) {
        tp_debug_print(" queue empty\n");
        return TP_ERR_FAIL;
    }

    // Check for tangency between segments and handle any errors
    // TODO possibly refactor this into a macro?
    int res_tan = tpSetupTangent(tp, prev_tc, tc);
    switch (res_tan) {
        // Abort blend arc creation in these cases
        case TP_ERR_FAIL:
            tp_debug_print(" tpSetupTangent failed, aborting blend arc\n");
        case TP_ERR_OK:
            return res_tan;
            break;
        case TP_ERR_NO_ACTION:
        default:
            //Continue with creation
            break;
    }

    TC_STRUCT blend_tc = {0};

    blend_type_t type = tpCheckBlendArcType(tp, prev_tc, tc);
    int res_create;
    switch (type) { 
        case BLEND_LINE_LINE:
            res_create = tpCreateLineLineBlend(tp, prev_tc, tc, &blend_tc);
            break;
        case BLEND_LINE_ARC:
            res_create = tpCreateLineArcBlend(tp, prev_tc, tc, &blend_tc);
            break;
        case BLEND_ARC_LINE:
            res_create = tpCreateArcLineBlend(tp, prev_tc, tc, &blend_tc);
            break;
        case BLEND_ARC_ARC:
            res_create = tpCreateArcArcBlend(tp, prev_tc, tc, &blend_tc);
            break;
        default:
            tp_debug_print("intersection type not recognized, aborting arc\n");
            res_create = TP_ERR_FAIL;
            break;
    }

    if (res_create == TP_ERR_OK) {
        //Need to do this here since the length changed
        tpAddSegmentToQueue(tp, &blend_tc, false);
    } else {
        return res_create;
    }

    return TP_ERR_OK;
}

//TODO final setup steps as separate functions
//
/**
 * Add a straight line to the tc queue.
 * end of the previous move to the new end specified here at the
 * currently-active accel and vel settings from the tp struct.
 */
int tpAddLine(TP_STRUCT * const tp, EmcPose end, int canon_motion_type, double vel, double ini_maxvel,
        double acc, double ini_maxjerk, unsigned char enables, char atspeed, int indexrotary, struct state_tag_t tag)
{
    if (tpErrorCheck(tp) < 0) {
        return TP_ERR_FAIL;
    }
    tp_info_print("== AddLine == req-vel(%f) max-vel(%f) acc(%f) jerk(%f)\n", vel, ini_maxvel, acc, ini_maxjerk);

    // Initialize new tc struct for the line segment
    TC_STRUCT tc = {0};
    tcInit(&tc,
            TC_LINEAR,
            canon_motion_type,
            tp->cycleTime,
            enables,
            atspeed);
    tc.tag = tag;

    // Copy in motion parameters
    tcSetupMotion(&tc,
            vel,
            ini_maxvel,
            acc,
            ini_maxjerk,
            tp->cycleTime);

    tp_info_print("%d: tc.maxvel(%f) tc.jerk(%.20f)\n", __LINE__, tc.maxvel, tc.jerk);

    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    tp_info_print("%d: tc.maxvel(%f) tc.jerk(%.20f)\n", __LINE__, tc.maxvel, tc.jerk);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    tp_info_print("%d: tc.maxvel(%f) tc.jerk(%.20f)\n", __LINE__, tc.maxvel, tc.jerk);

    // Setup line geometry
    pmLine9Init(&tc.coords.line,
            &tp->goalPos,
            &end);
    tc.target = pmLine9Target(&tc.coords.line);
    if (tc.target < TP_POS_EPSILON) {
        rtapi_print_msg(RTAPI_MSG_DBG,"failed to create line id %d, zero-length segment\n",tp->nextId);
        return TP_ERR_ZERO_LENGTH;
    }
    tc.nominal_length = tc.target;
    tc.distance_to_go = tc.target;

    //Reduce max velocity to match sample rate
    double sample_maxvel = tc.target / (tp->cycleTime * TP_MIN_SEGMENT_CYCLES);
    tc.maxvel = rtapi_fmin(tc.maxvel, sample_maxvel);

    tp_info_print("distance_to_go(%f) tc.maxvel(%f) tc.jerk(%f)\n", tc.target, tc.maxvel, tc.jerk);

    // For linear move, set rotary axis settings 
    tc.indexrotary = indexrotary;

    //TODO refactor this into its own function
    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);
    tpCheckCanonType(prev_tc, &tc);
    if (get_arcBlendEnable(tp->shared)){
        tpHandleBlendArc(tp, &tc);
    }
    tcCheckLastParabolic(&tc, prev_tc);
    tcFinalizeLength(prev_tc);
    tcFlagEarlyStop(prev_tc, &tc);

    int retval = tpAddSegmentToQueue(tp, &tc, true);
    //Run speed optimization (will abort safely if there are no tangent segments)
    tpRunOptimization(tp);

    return retval;
}


/**
 * Adds a circular (circle, arc, helix) move from the end of the
 * last move to this new position.
 *
 * @param end is the xyz/abc point of the destination.
 *
 * see pmCircleInit for further details on how arcs are specified. Note that
 * degenerate arcs/circles are not allowed. We are guaranteed to have a move in
 * xyz so the target is always the circle/arc/helical length.
 */
int tpAddCircle(TP_STRUCT * const tp,
        EmcPose end,
        PmCartesian center,
        PmCartesian normal,
        int turn,
        int canon_motion_type,
        double vel,
        double ini_maxvel,
        double acc,
        double ini_maxjerk,
        unsigned char enables,
        char atspeed,
        struct state_tag_t tag)
{
    if (tpErrorCheck(tp)<0) {
        return TP_ERR_FAIL;
    }

    tp_info_print("== AddCircle ==\n");
    tp_debug_print("ini_maxvel(%f) tcqLen(%d)\n", ini_maxvel, tcqLen(&tp->queue));

    TC_STRUCT tc = {0};

    tcInit(&tc,
            TC_CIRCULAR,
            canon_motion_type,
            tp->cycleTime,
            enables,
            atspeed);
    tc.tag = tag;
    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    // Setup circle geometry
    int res_init = pmCircle9Init(&tc.coords.circle,
            &tp->goalPos,
            &end,
            &center,
            &normal,
            turn);

    if (res_init) return res_init;

    // Update tc target with existing circular segment
    tc.target = pmCircle9Target(&tc.coords.circle);
    if (tc.target < TP_POS_EPSILON) {
        return TP_ERR_FAIL;
    }
    tp_debug_print("tc.target = %f\n",tc.target);
    tc.nominal_length = tc.target;

    //Reduce max velocity to match sample rate
    tcClampVelocityByLength(&tc);

    double v_max_actual = pmCircleActualMaxVel(&tc.coords.circle.xyz, ini_maxvel, acc, false);

    // Copy in motion parameters
    tcSetupMotion(&tc,
            vel,
            v_max_actual,
            acc,
            ini_maxjerk,
            tp->cycleTime);

    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);

    tpCheckCanonType(prev_tc, &tc);
    if (get_arcBlendEnable(tp->shared)){
        tpHandleBlendArc(tp, &tc);
        findSpiralArcLengthFit(&tc.coords.circle.xyz, &tc.coords.circle.fit);
    }
    tcCheckLastParabolic(&tc, prev_tc);
    tcFinalizeLength(prev_tc);  // FIXME: is tcFinalizeLength() necessary?
    tcFlagEarlyStop(prev_tc, &tc);

    int retval = tpAddSegmentToQueue(tp, &tc, true);

    tpRunOptimization(tp);
    return retval;
}


/**
 * Adjusts blend velocity and acceleration to safe limits.
 * If we are blending between tc and nexttc, then we need to figure out what a
 * safe blend velocity is based on the known trajectory parameters. This
 * function updates the TC_STRUCT data with a safe blend velocity.
 */
STATIC int tpComputeBlendVelocity(TP_STRUCT const * const tp,
        TC_STRUCT * const tc, TC_STRUCT * const nexttc)
{
    /* Pre-checks for valid pointers */
    if (!nexttc || !tc) {
        return TP_ERR_FAIL;
    }

    if (tc->term_cond != TC_TERM_COND_PARABOLIC) {
        return TP_ERR_NO_ACTION;
    }

    double acc_this = tpGetScaledAccel(tp, tc);
    double acc_next = tpGetScaledAccel(tp, nexttc);

    // cap the blend velocity at the current requested speed (factoring in feed override)
    double target_vel_this;
    double target_vel_next;
    target_vel_this = tpGetRealTargetVel(tp, tc);
    target_vel_next = tpGetRealTargetVel(tp, nexttc);

    double v_reachable_this = rtapi_fmin(tpCalculateTriangleVel(tp,tc), target_vel_this);
    double v_reachable_next = rtapi_fmin(tpCalculateTriangleVel(tp,nexttc), target_vel_next);

    /* Compute the maximum allowed blend time for each segment.
     * This corresponds to the minimum acceleration that will just barely reach
     * max velocity as we are 1/2 done the segment.
     */

    double t_max_this = tc->target / v_reachable_this;
    double t_max_next = nexttc->target / v_reachable_next;
    double t_max_reachable = rtapi_fmin(t_max_this, t_max_next);

    // How long the blend phase would be at maximum acceleration
    double t_min_blend_this = v_reachable_this / acc_this;
    double t_min_blend_next = v_reachable_next / acc_next;

    double t_max_blend = rtapi_fmax(t_min_blend_this, t_min_blend_next);
    // The longest blend time we can get that's still within the 1/2 segment restriction
    double t_blend = rtapi_fmin(t_max_reachable, t_max_blend);

    // Now, use this blend time to find the best acceleration / velocity for each segment
    double v_blend_this = rtapi_fmin(v_reachable_this, t_blend * acc_this);
    double v_blend_next = rtapi_fmin(v_reachable_next, t_blend * acc_next);

    double theta;
    if (tc->tolerance > 0) {
        /* see diagram blend.fig.  T (blend tolerance) is given, theta
         * is calculated from dot(s1, s2)
         *
         * blend criteria: we are decelerating at the end of segment s1
         * and we pass distance d from the end.
         * find the corresponding velocity v when passing d.
         *
         * in the drawing note d = 2T/cos(theta)
         *
         * when v1 is decelerating at a to stop, v = at, t = v/a
         * so required d = .5 a (v/a)^2
         *
         * equate the two expressions for d and solve for v
         */
        double tblend_vel;
        PmCartesian v1, v2;

        tcGetEndAccelUnitVector(tc, &v1);
        tcGetStartAccelUnitVector(nexttc, &v2);
        findIntersectionAngle(&v1, &v2, &theta);
        /* Minimum value of cos(theta) to prevent numerical instability */
        const double min_cos_theta = rtapi_cos(PM_PI / 2.0 - TP_MIN_ARC_ANGLE);
        if (rtapi_cos(theta) > min_cos_theta) {
            tblend_vel = 2.0 * pmSqrt(acc_this * tc->tolerance / rtapi_cos(theta));
            v_blend_this = rtapi_fmin(v_blend_this, tblend_vel);
            v_blend_next = rtapi_fmin(v_blend_next, tblend_vel);
        }
    }

    tc->blend_vel = v_blend_this;
    nexttc->blend_vel = v_blend_next;
    return TP_ERR_OK;
}

/*
 Continuous form
 PT = P0 + V0T + 1/2A0T2 + 1/6JT3
 VT = V0 + A0T + 1/2 JT2
 AT = A0 + JT

 Discrete time form (let T be 1, then T^2 == 1, T^3 == 1)
 PT = PT + VT + 1/2AT + 1/6J
 VT = VT + AT + 1/2JT
 AT = AT + JT
 */
/**
 * S-curve Velocity Profiler FSM
 * Yishin Li <ysli@araisrobo.com>
 * ARAIS ROBOT TECHNOLOGY, http://www.araisrobo.com/
 **/
void tcRunCycle(TP_STRUCT *tp, TC_STRUCT *tc)
{

    double t, t1, vel, acc, v1, dist, req_vel;

    static double ts, ti;
    static double k, s6_a, s6_v, s6_p, error_d, prev_s, prev_v;
    static double c1, c2, c3, c4, c5, c6;
    static double prev_tc_target;

    double pi = 3.14159265359;
    int immediate_state;
    double tc_target;

    // Find maximum allowed velocity from feed and machine limits
    double tc_target_vel = tpGetRealTargetVel(tp, tc);

    tp_info_print("(%s:%d) tc_target_vel(%f) maxvel(%f) jerk(%.20f)\n", __FUNCTION__, __LINE__, tc_target_vel, tc->maxvel, tc->jerk);

    tc_target = tc->target + tc->lookahead_target;
    if ((prev_tc_target != tc_target) && (tc->accel_state == ACCEL_S7)) {
        /* re-evaluate ACCEL_S7 condition if lookahead_target changed */
        tc->accel_state = ACCEL_S4;
    }
    prev_tc_target = tc_target;

    immediate_state = 0;
    do {
        switch (tc->accel_state) {
        case ACCEL_S0:
            // AT = AT + JT
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->cur_accel = tc->cur_accel + tc->jerk;
            tc->currentvel = tc->currentvel + tc->cur_accel + 0.5 * tc->jerk;
            tc->progress = tc->progress + tc->currentvel + 0.5 * tc->cur_accel + 1.0/6.0 * tc->jerk;

            // check if we hit accel limit at next BP
            if ((tc->cur_accel + tc->jerk) >= tc->maxaccel) {
                tc->cur_accel = tc->maxaccel;
                tc->accel_state = ACCEL_S1;
                break;
            }

            // check if we will hit velocity limit;
            // assume we start decel from here to "accel == 0"
            //
            // AT = A0 + JT (let AT = 0 to calculate T)
            // VT = V0 + A0T + 1/2JT2
            t = rtapi_ceil(tc->cur_accel / tc->jerk);
            req_vel = tc_target_vel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            vel = req_vel - tc->cur_accel * t + 0.5 * tc->jerk * t * t;
            if (tc->currentvel >= vel) {
                tc->accel_state = ACCEL_S2;
                break;
            }

            // check if we will hit progress limit
            // AT = AT + JT
            // VT = V0 + A0T + 1/2 JT^2
            // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
            // distance for S2
            t = rtapi_ceil(tc->cur_accel/tc->jerk);
            dist = tc->progress + tc->currentvel * t + 0.5 * tc->cur_accel * t * t
                    - 1.0/6.0 * tc->jerk * t * t * t;
            vel = tc->currentvel + tc->cur_accel * t - 0.5 * tc->jerk * t * t;
            // distance for S3
            dist += (vel);

            /*
            0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
            t1 = sqrt(vel/j)
             */
            t = rtapi_ceil(rtapi_sqrt(vel/tc->jerk));
            // AT = AT + JT
            t1 = rtapi_ceil(tc->maxaccel / tc->jerk);   // max time for S4
            if (t > t1) {
                // S4 -> S5 -> S6
                dist += t1 * vel;    // dist of (S4 + S6)
                // calc decel.dist for ACCEL_S5
                // t: time for S5
                t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                v1 = vel - 0.5 * tc->jerk * t1 * t1;
                // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
            } else {
                // S4 -> S6
                dist += t * vel;    // dist of (S4 + S6)
            }

            if (tc_target < dist) {
                tc->accel_state = ACCEL_S2;
                break;
            }

            break;

        case ACCEL_S1:
            // jerk is 0 at this state
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->currentvel = tc->currentvel + tc->cur_accel;
            tc->progress = tc->progress + tc->currentvel + 0.5 * tc->cur_accel;

            // check if we will hit velocity limit;
            // assume we start decel from here to "accel == 0"
            //
            // AT = A0 + JT (let AT = 0 to calculate T)
            // VT = V0 + A0T + 1/2JT2
            // t = rtapi_ceil(tc->cur_accel / tc->jerk);
            t = tc->cur_accel / tc->jerk;
            req_vel = tc_target_vel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            vel = req_vel - tc->cur_accel * t + 0.5 * tc->jerk * t * t;
            if (tc->currentvel >= vel) {
                tc->accel_state = ACCEL_S2;
                break;
            }

            // check if we will hit progress limit
            // distance for S2
            t = rtapi_ceil(tc->cur_accel/tc->jerk);
            dist = tc->progress + tc->currentvel * t + 0.5 * tc->cur_accel * t * t
                    - 1.0/6.0 * tc->jerk * t * t * t;
            vel = tc->currentvel + tc->cur_accel * t - 0.5 * tc->jerk * t * t;
            // distance for S3
            dist += (vel);

            /*
            0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
            t1 = sqrt(vel/j)
             */
            t = rtapi_ceil(rtapi_sqrt(vel/tc->jerk));
            // AT = AT + JT
            t1 = rtapi_ceil(tc->maxaccel / tc->jerk);   // max time for S4
            if (t > t1) {
                // S4 -> S5 -> S6
                dist += t1 * vel;    // dist of (S4 + S6)
                // calc decel.dist for ACCEL_S5
                // t: time for S5
                t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                v1 = vel - 0.5 * tc->jerk * t1 * t1;
                // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
            } else {
                // S4 -> S6
                dist += t * vel;    // dist of (S4 + S6)
            }

            if (tc_target < dist) {
                tc->accel_state = ACCEL_S2;
                break;
            }
            break;

        case ACCEL_S2:
            // to DECELERATE to ACCEL==0

            // AT = AT + JT
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->cur_accel = tc->cur_accel - tc->jerk;
            tc->currentvel = tc->currentvel + tc->cur_accel - 0.5 * tc->jerk;
            tc->progress = tc->progress + tc->currentvel + 0.5 * tc->cur_accel - 1.0/6.0 * tc->jerk;

            // check if we will hit velocity limit at next BP
            req_vel = tc_target_vel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            // vel: velocity at next BP
            vel = tc->currentvel + tc->cur_accel - 1.5 * tc->jerk;
            if (vel > req_vel) {
                tc->currentvel = req_vel;
                tc->accel_state = ACCEL_S3;
                break;
            }

            // check if (accel <= 0) at next BP
            acc = tc->cur_accel - tc->jerk;
            if (acc <= 0) {
                tc->accel_state = ACCEL_S3;
                break;
            }

            // check if we will hit progress limit
            // refer to 2011-10-17 ysli design note
            // AT = AT + JT
            // VT = V0 + A0T + 1/2 JT^2
            // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
            vel = tc->currentvel;
            // distance for S3
            dist = tc->progress + (vel);

            /*
            0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
            t1 = sqrt(vel/j)
             */
            t = rtapi_ceil(rtapi_sqrt(vel/tc->jerk));
            // AT = AT + JT
            t1 = rtapi_ceil(tc->maxaccel / tc->jerk);   // max time for S4
            if (t > t1) {
                // S4 -> S5 -> S6
                dist += t1 * vel;    // dist of (S4 + S6)
                // calc decel.dist for ACCEL_S5
                // t: time for S5
                t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                v1 = vel - 0.5 * tc->jerk * t1 * t1;
                // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
            } else {
                // S4 -> S6
                dist += t * vel;    // dist of (S4 + S6)
            }

            if (tc_target < dist) {
                tc->accel_state = ACCEL_S3;
                break;
            }

            break;

        case ACCEL_S3:
            // PT = PT + VT + 1/2AT + 1/6JT
            // , where (jerk == 0) and (accel == 0)
            tc->cur_accel = 0;
            tc->progress = tc->progress + tc->currentvel;

            // check if we will hit progress limit
            // refer to 2011-10-17 ysli design note
            // AT = AT + JT
            // VT = V0 + A0T + 1/2 JT^2
            // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
            /*
            0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
            t1 = sqrt(vel/j)
             */
            vel = tc->currentvel;
            t = rtapi_sqrt(vel/tc->jerk);
            // AT = AT + JT
            t1 = tc->maxaccel / tc->jerk;   // max time for S4
            if (t > t1) {
                // S4 -> S5 -> S6
                dist = tc->progress + t1 * vel;    // dist of (S4 + S6)
                // calc decel.dist for ACCEL_S5
                // t: time for S5
                t = (vel - tc->maxaccel * t1) / tc->maxaccel - 0.5;
                // t = (vel - tc->maxaccel * t1) / tc->maxaccel;
                v1 = vel - 0.5 * tc->jerk * t1 * t1;
                // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
            } else {
                // S4 -> S6
                dist = tc->progress + t * vel;    // dist of (S4 + S6)
            }

            // check if dist would be greater than tc_target at next cycle
            if (tc_target < (dist - vel)) {
                tc->accel_state = ACCEL_S4;
                break;
            }

            // check for changes of feed_override and request-velocity
            req_vel = tc_target_vel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            if ((tc->currentvel + 1.5 * tc->jerk) < req_vel) {
                tc->accel_state = ACCEL_S0;
                break;
            } else if ((tc->currentvel - 1.5 * tc->jerk) > req_vel) {
                tc->accel_state = ACCEL_S4;
                break;
            }
            tc->currentvel = req_vel;
            break;


        case ACCEL_S4:
            // AT = AT + JT
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->cur_accel = tc->cur_accel - tc->jerk;
            tc->currentvel = tc->currentvel + tc->cur_accel - 0.5 * tc->jerk;
            if (tc->currentvel <= 0) {
                tc->currentvel = 0;
                tc->accel_state = ACCEL_S3;
                break;
            }
            tc->progress = tc->progress + tc->currentvel + 0.5 * tc->cur_accel - 1.0/6.0 * tc->jerk;

            // (accel < 0) and (jerk < 0)
            assert (tc->cur_accel < 0);

            // check if we hit accel limit at next BP
            if ((tc->cur_accel - tc->jerk) <= -tc->maxaccel) {
                tc->cur_accel = -tc->maxaccel;
                tc->accel_state = ACCEL_S5;
                break;
            }

            // should we stay in S4 and keep decel?
            // calculate dist for S4 -> (maybe S5) -> S6
            t = - tc->cur_accel / tc->jerk;
            // target dist after switching to S6 (jerk is positive for S6)
            dist = tc->progress + tc->currentvel * t
                    + 0.5 * tc->cur_accel * t * t
                    + 1.0 / 6.0 * tc->jerk * t * t * t;
            // VT = V0 + A0T + 1/2JT2
            // obtain vel for S6 -> S3
            vel = tc->currentvel + tc->cur_accel * t + 0.5 * tc->jerk * t * t;
            if (vel > 0) {
                /*
                0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
                t1 = sqrt(vel/j)
                 */
                t = rtapi_ceil(rtapi_sqrt(vel/tc->jerk));
                // AT = AT + JT
                t1 = rtapi_ceil(tc->maxaccel / tc->jerk);   // max time for S4
                if (t > t1) {
                    // S4 -> S5 -> S6
                    dist += t1 * vel;    // dist of (S4 + S6)
                    // calc decel.dist for ACCEL_S5
                    // t: time for S5
                    t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                    v1 = vel - 0.5 * tc->jerk * t1 * t1;
                    // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                    dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
                } else {
                    // S4 -> S6
                    dist += t * vel;    // dist of (S4 + S6)
                }
            }

            if (tc_target < (dist - (tc->currentvel + 1.5 * tc->cur_accel - 2.1666667 * tc->jerk))) {
                tc->accel_state = ACCEL_S4;
                break;
            }

            // check if we will approaching requested velocity
            // vel should not be greater than "request velocity" after
            // starting acceleration to "accel == 0".
            //
            // AT = A0 + JT (let AT = 0 to calculate T)
            // VT = V0 + A0T + 1/2JT2
            t = - tc->cur_accel / tc->jerk;
            req_vel = tc_target_vel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            if ((tc->currentvel + tc->cur_accel * t + 0.5 * tc->jerk * t * t) <= req_vel) {
                if ((tc->progress/tc_target) < 0.9) {
                    tc->accel_state = ACCEL_S6;
                }
                else
                {
                    tc->accel_state = ACCEL_S7;
                    s6_v = tc->currentvel;
                    s6_a = rtapi_fabs(tc->cur_accel);
                    s6_p = tc->progress;
                    ts = rtapi_floor((2*s6_v)/s6_a);
                    k = s6_a*pi/(4*s6_v);
                    error_d = tc_target - tc->progress - s6_v * s6_v / s6_a * (1-4/(pi*pi));
                    prev_s = 0;
                    prev_v = s6_v;
                    c1 = -s6_a/4;
                    c2 = s6_v+((error_d*s6_a)/(2*s6_v));
                    c3 = s6_a/(8*k*k);
                    c4 = 2*k;
                    c5 = -(error_d*s6_a)/(8*k*s6_v);
                    c6 = 4*k;
                    ti = 1;
                    break;
                }
            }

            break;

        case ACCEL_S5:
            // jerk is 0 at this state
            // accel < 0
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            tc->currentvel = tc->currentvel + tc->cur_accel;
            tc->progress = tc->progress + tc->currentvel + 0.5 * tc->cur_accel;

            // should we stay in S5 and keep decel?
            // calculate dist for S6 -> S4 -> (maybe S5) -> S6
            t = - tc->cur_accel / tc->jerk;
            // target dist after switching to S6 (jerk is positive for S6)
            dist = tc->progress + tc->currentvel * t
                    + 0.5 * tc->cur_accel * t * t
                    + 1.0 / 6.0 * tc->jerk * t * t * t;
            // VT = V0 + A0T + 1/2JT2
            // obtain vel for S6 -> S3
            vel = tc->currentvel + tc->cur_accel * t + 0.5 * tc->jerk * t * t;

            if (vel > 0) {
                /* S6 -> S3 -> S4 -> S5(maybe) -> S6 */
                /*
                0.5 * vel = vel + 0 * t1 - 0.5 * j * t1 * t1;
                t1 = sqrt(vel/j)
                 */
                t = rtapi_ceil(rtapi_sqrt(vel/tc->jerk));
                // AT = AT + JT
                t1 = rtapi_ceil(tc->maxaccel / tc->jerk);   // max time for S4
                if (t > t1) {
                    // S4 -> S5 -> S6
                    dist += t1 * vel;    // dist of (S4 + S6)
                    // calc decel.dist for ACCEL_S5
                    // t: time for S5
                    t = (vel - tc->jerk * t1 * t1) / tc->maxaccel;
                    v1 = vel - 0.5 * tc->jerk * t1 * t1;
                    // PT = P0 + V0T + 1/2A0T^2 + 1/6JT^3
                    dist += (v1 * t - 0.5 * tc->maxaccel * t * t);
                } else {
                    // S4 -> S6
                    dist += t * vel;    // dist of (S4 + S6)
                }
            }

            // check if dist would be greater than tc_target at next cycle
            if (tc_target < (dist - (tc->currentvel + 1.5 * tc->cur_accel))) {
                tc->accel_state = ACCEL_S5;
//                DPS("should stay in S5 and keep decel\n");
                break;
            }

            // check if we will approaching requested velocity
            // vel should not be greater than "request velocity" after
            // starting acceleration to "accel == 0".
            //
            // AT = A0 + JT (let AT = 0 to calculate T)
            // VT = V0 + A0T + 1/2JT2
            // t: cycles for accel to decel to 0
            t = - tc->cur_accel / tc->jerk;
            req_vel = tc_target_vel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }
            if ((tc->currentvel + tc->cur_accel * t + 0.5 * tc->jerk * t * t) <= req_vel) {
                if(tc->progress/tc_target < 0.9){
                    tc->accel_state = ACCEL_S6;
//                    DPS("S5 move to S6\n");
                }
                else
                {
                    tc->accel_state = ACCEL_S7;
                    s6_v = tc->currentvel;
                    s6_a = rtapi_fabs(tc->cur_accel);
                    s6_p = tc->progress;
                    ts = rtapi_floor((2*s6_v)/s6_a);
                    k = s6_a*pi/(4*s6_v);
                    error_d = tc_target - tc->progress - s6_v * s6_v / s6_a * (1-4/(pi*pi));
                    prev_s = 0;
                    prev_v = s6_v;
                    c1 = -s6_a/4;
                    c2 = s6_v+((error_d*s6_a)/(2*s6_v));
                    c3 = s6_a/(8*k*k);
                    c4 = 2*k;
                    c5 = -(error_d*s6_a)/(8*k*s6_v);
                    c6 = 4*k;
                    ti = 1;
//                    DPS("S5 move to S7\n");
                    break;
                }
            }

            break;

        case ACCEL_S6:
            // for approaching to req_vel
            // AT = AT + JT
            // VT = VT + AT + 1/2JT
            // PT = PT + VT + 1/2AT + 1/6JT
            req_vel = tc_target_vel * tc->feed_override * tc->cycle_time;
            if (req_vel > tc->maxvel) {
                req_vel = tc->maxvel;
            }

            tc->cur_accel = tc->cur_accel + tc->jerk;
            tc->currentvel = tc->currentvel + tc->cur_accel + 0.5 * tc->jerk;

            if (tc->currentvel <= req_vel) {
                tc->accel_state = ACCEL_S3;
                if ((req_vel - tc->currentvel) < 1.5*tc->jerk) {
                    // align to req_vel only when not changing feed_override
                    tc->currentvel = req_vel;
                }
            }
            dist = tc->currentvel + 0.5 * tc->cur_accel + 1.0/6.0 * tc->jerk;
            tc->progress = tc->progress + dist;

            if (tc->cur_accel >= 0) {
                tc->accel_state = ACCEL_S3;
            }

            break;

        case ACCEL_S7:
            // decel to target position based on Jofey's algorithm
            if (ti <= ts) {
                dist = c1*ti*ti + c2*ti + c3*rtapi_cos(c4*ti) + c5*rtapi_cos(c6*ti-0.5*pi) - c3;
                tc->currentvel = dist - prev_s;
                tc->cur_accel = tc->currentvel - prev_v;
                prev_s = dist;
                prev_v = tc->currentvel;
                tc->progress = s6_p + dist;
                ti = ti + 1;
            } else {
                tc->currentvel = 0;
                tc->cur_accel = 0;
                tc->progress = tc->target;
                tc->accel_state = ACCEL_S3;
            }
            break;

        default:
            assert(0);
        } // switch (tc->accel_state)
    } while (immediate_state);

//    if (tc->seamless_blend_mode != SMLBLND_ENABLE) {
//        if (tc->progress >= tc->target) {
//            // finished
//            // DPS("hit target, cur_accel(%f), currentvel(%f)\n", tc->cur_accel, tc->currentvel);
//            tc->progress = tc->target;
//            tc->cur_accel = 0;
//            tc->currentvel = 0;
//        }
//    }

//    if (tc->pso.enable)
//    {
//        if (tc->progress > tc->pso.next_progress)
//        {
//            double temp_progress;
//            DP ("TODO: resolve blending two segments: may cause double entrance of tcRunCycle ()\n");
//            DP ("tc->progress(%f) tc->pso.next_progress(%f)\n", tc->progress, tc->pso.next_progress);
//            temp_progress = tc->progress;
//            tc->progress = tc->pso.next_progress;
//            emcmotStatus->pso_pos = tcGetPos(tc);
//            tc->progress = temp_progress;
//            tc->pso.next_progress += tc->pso.pitch;
//            emcmotStatus->pso_mode = tc->pso.mode;
//            emcmotStatus->pso_tick = tc->pso.tick;
//            emcmotStatus->pso_req = 1;
//            DP ("tp.c: pso_pos x(%f) y(%f)\n", emcmotStatus->pso_pos.tran.x, emcmotStatus->pso_pos.tran.y);
//            DP ("tp.c: turn pso_req ON, pso_req(%d)\n", emcmotStatus->pso_req);
//        }
//        else
//        {
//            emcmotStatus->pso_req = 0;
//        }
//    }

    DPS("%11u%5d%6d%15.8f%15.8f%15.8f%15.8f%15.8f%15.8f%15.8f%15.8f%15.8f%15.8f\n",
            _dt, tc->id, tc->accel_state, tc_target_vel * tc->feed_override * tc->cycle_time,
            tc->cur_accel, tc->currentvel, tc->progress/tc->target, tc->progress,
            tc->target - tc->progress, tc_target, tc->jerk, tp->currentPos.tran.x, tp->currentPos.tran.y);
    DPS("target_vel(%f) feed_override(%f) cycle_time(%f)\n", tc_target_vel, tc->feed_override, tc->cycle_time);

    tc->distance_to_go = tc->target - tc->progress;
}


void tpToggleDIOs(TP_STRUCT const * const tp,
		  TC_STRUCT * const tc) {

    unsigned int i = 0;
    if (tc->syncdio.anychanged != 0) { // we have DIO's to turn on or off
        for (i=0; i < get_num_dio(tp->shared); i++) {
            if (!(RTAPI_BIT_TEST(tc->syncdio.dio_mask, i))) continue;
            if (tc->syncdio.dios[i] > 0) dioWrite(tp->shared, i, 1); // turn DIO[i] on
            if (tc->syncdio.dios[i] < 0) dioWrite(tp->shared, i, 0); // turn DIO[i] off
        }
        for (i=0; i < get_num_aio(tp->shared); i++) {
            if (!(tc->syncdio.aio_mask & (1ull << i))) continue;
            aioWrite(tp->shared, i, tc->syncdio.aios[i]); // set AIO[i]
        }
        tc->syncdio.anychanged = 0; //we have turned them all on/off, nothing else to do for this TC the next time
    }
}

STATIC void tpSetUuPerRev(TP_STRUCT * const tp, TC_STRUCT * const tc)
{
    if(tc->uu_updated == 0)
    {
        PmCartesian uu;
        tc->uu_updated = 1;

        if (tc->uu_per_rev != 0.0) {
            uu.x = tc->uu_per_rev * tc->coords.spindle_sync.xyz.uVec.x;
            uu.y = tc->uu_per_rev * tc->coords.spindle_sync.xyz.uVec.y;
            uu.z = tc->uu_per_rev * tc->coords.spindle_sync.xyz.uVec.z;
        } else {
            uu = (PmCartesian){0.0, 0.0, 0.0};
        }
        SetUuPerRev(tp->shared, uu);
    }
    return;
}


/**
 * Update emcMotStatus with information about trajectory motion.
 * Based on the specified trajectory segment tc, read its progress and status
 * flags. Then, update upper level data through tp_shared_t.
 */
STATIC int tpUpdateMovementStatus(TP_STRUCT * const tp, TC_STRUCT * const tc ) {


    if (!tp) {
        return TP_ERR_FAIL;
    }

    if (!tc) {
        // Assume that we have no active segment, so we should clear out the status fields
        set_requested_vel(tp->shared, 0);
        set_current_vel(tp->shared, 0);
        set_distance_to_go(tp->shared, 0);

        emcPoseZero2fp(tp->shared->dtg);

        tp->motionType = 0;
        tp->activeDepth = 0;
        return TP_ERR_STOPPED;
    }

    EmcPose tc_pos;
    tcGetEndpoint(tc, &tc_pos);

    tc_debug_print("tc id = %u canon_type = %u mot type = %u\n",
            tc->id, tc->canon_motion_type, tc->motion_type);
    tp->motionType = tc->canon_motion_type;
    tp->accelState = tc->accel_state;
    tp->activeDepth = tc->active_depth;
    tp->distance_to_go = tc->target - tc->progress;
    set_distance_to_go(tp->shared, tp->distance_to_go);
    set_enables_queued(tp->shared, tc->enables);
    tp->progress = tc->progress;
    // report our line number to the guis
    tp->execId = tc->id;
    set_requested_vel(tp->shared, tc->reqvel);
    set_current_vel(tp->shared, tc->currentvel / tc->cycle_time);
    emcPoseSub2fp(&tc_pos, &tp->currentPos, tp->shared->dtg);
    return TP_ERR_OK;
}


/**
 * Do a parabolic blend by updating the nexttc.
 * Perform the actual blending process by updating the target velocity for the
 * next segment, then running a cycle update.
 */
STATIC void tpUpdateBlend(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {

    double save_vel = nexttc->target_vel;

    if (tpGetFeedScale(tp, nexttc) > TP_VEL_EPSILON) {
        double dv = tc->vel_at_blend_start - tc->currentvel;
        double vel_start = rtapi_fmax(tc->vel_at_blend_start, TP_VEL_EPSILON);
        // Clip the ratio at 1 and 0
        double blend_progress = rtapi_fmax(rtapi_fmin(dv / vel_start, 1.0), 0.0);
        double blend_scale = tc->vel_at_blend_start / tc->blend_vel;
        nexttc->target_vel = blend_progress * nexttc->blend_vel * blend_scale;
        // Mark the segment as blending so we handle the new target velocity properly
        nexttc->is_blending = true;
    } else {
        // Drive the target velocity to zero since we're stopping
        nexttc->target_vel = 0.0;
    }

    tpUpdateCycle(tp, nexttc, NULL);
    //Restore the original target velocity
    nexttc->target_vel = save_vel;

}


/**
 * Cleanup if tc is not valid (empty queue).
 * If the program ends, or we hit QUEUE STARVATION, do a soft reset on the trajectory planner.
 * TODO merge with tpClear?
 */
STATIC void tpHandleEmptyQueue(TP_STRUCT * const tp)
{

    tcqInit(&tp->queue);
    tp->goalPos = tp->currentPos;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->motionType = 0;

    tpUpdateMovementStatus(tp, NULL);

    tpResume(tp);
    // when not executing a move, use the current enable flags
    set_enables_queued(tp->shared,
		       get_enables_new(tp->shared));
}

/** Wrapper function to unlock rotary axes */
STATIC void tpSetRotaryUnlock(TP_STRUCT * const tp, int axis, int unlock) {
    SetRotaryUnlock(tp->shared, axis, unlock);
}

/** Wrapper function to check rotary axis lock */
STATIC int tpGetRotaryIsUnlocked(TP_STRUCT * const tp, int axis) {
    return GetRotaryIsUnlocked(tp->shared, axis);
}


/**
 * Cleanup after a trajectory segment is complete.
 * If the current move is complete and we're not waiting on the spindle for
 * const this move, then pop if off the queue and perform cleanup operations.
 * Finally, get the next move in the queue.
 */
STATIC int tpCompleteSegment(TP_STRUCT * const tp,
        TC_STRUCT const * const tc) {

    if (tp->spindle.waiting_for_atspeed == tc->id) {
        return TP_ERR_FAIL;
    }

    if(tc->indexrotary != -1) {
        // this was an indexing move, so before we remove it we must
        // relock the axis
        tpSetRotaryUnlock(tp, tc->indexrotary, 0);
        // if it is now locked, fall through and remove the finished move.
        // otherwise, just come back later and check again
        if(tpGetRotaryIsUnlocked(tp, tc->indexrotary)) {
            return TP_ERR_FAIL;
        }
    }

    // done with this move
    tcqRemove(&tp->queue, 1);
    tp->depth = tcqLen(&tp->queue);
    tp_debug_print("Finished tc id %d\n", tc->id);

    return TP_ERR_OK;
}


/**
 * Handle an abort command.
 * Based on the current motion state, handle the consequences of an abort command.
 */
STATIC int tpHandleAbort(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {

    if(!tp->aborting) {
        //Don't need to do anything if not aborting
        return TP_ERR_NO_ACTION;
    }
    //If the motion has stopped, then it's safe to reset the TP struct.
    if( MOTION_ID_VALID(tp->spindle.waiting_for_index) ||
            MOTION_ID_VALID(tp->spindle.waiting_for_atspeed) ||
            (tc->currentvel == 0.0 && (!nexttc || nexttc->currentvel == 0.0))) {
        tcqInit(&tp->queue);
        tp->goalPos = tp->currentPos;
        tp->done = 1;
        tp->depth = tp->activeDepth = 0;
        tp->aborting = 0;
        tp->motionType = 0;
        tp->synchronized = 0;
        tp->spindle.waiting_for_index = MOTION_INVALID_ID;
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
	set_spindleSync(tp->shared, 0);
        tpResume(tp);
        return TP_ERR_STOPPED;
    }  //FIXME consistent error codes
    return TP_ERR_SLOWING;
}


/**
 * Check if the spindle has reached the required speed for a move.
 * Returns a "wait" code if the spindle needs to spin up before a move and it
 * has not reached the requested speed, or the spindle index has not been
 * detected.
 */
STATIC int tpCheckAtSpeed(TP_STRUCT * const tp, TC_STRUCT * const tc)
{

    // this is no longer the segment we were waiting_for_index for
    if (MOTION_ID_VALID(tp->spindle.waiting_for_index) && tp->spindle.waiting_for_index != tc->id)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Was waiting for index on motion id %d, but reached id %d\n",
                tp->spindle.waiting_for_index, tc->id);
        tp->spindle.waiting_for_index = MOTION_INVALID_ID;
    }

    if (MOTION_ID_VALID(tp->spindle.waiting_for_atspeed) && tp->spindle.waiting_for_atspeed != tc->id)
    {

        rtapi_print_msg(RTAPI_MSG_ERR,
                "Was waiting for atspeed on motion id %d, but reached id %d\n",
                tp->spindle.waiting_for_atspeed, tc->id);
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
    }

    if (MOTION_ID_VALID(tp->spindle.waiting_for_atspeed)) {
        if(!get_spindle_is_atspeed(tp->shared)) {
            // spindle is still not at the right speed, so wait another cycle
            return TP_ERR_WAITING;
        } else {
            tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->spindle.waiting_for_index)) {
        if (get_spindle_index_enable(tp->shared)) {
            /* haven't passed index yet */
            return TP_ERR_WAITING;
        } else {
            /* passed index, start the move */
            set_spindleSync(tp->shared, 1);
            tp->spindle.waiting_for_index = MOTION_INVALID_ID;
        }
    }

    return TP_ERR_OK;
}


/**
 * "Activate" a segment being read for the first time.
 * This function handles initial setup of a new segment read off of the queue
 * for the first time.
 */
STATIC int tpActivateSegment(TP_STRUCT * const tp, TC_STRUCT * const tc) {

    //Check if already active
    if (!tc || tc->active) {
        return TP_ERR_OK;
    }

    if (!tp) {
        return TP_ERR_MISSING_INPUT;
    }

    // Do at speed checks that only happen once
    int needs_atspeed = tc->atspeed ||
        (tc->synchronized == TC_SYNC_POSITION && !(get_spindleSync(tp->shared)));

    if ( needs_atspeed && !(get_spindle_is_atspeed(tp->shared))) {
        tp->spindle.waiting_for_atspeed = tc->id;
        return TP_ERR_WAITING;
    }

    if (tc->indexrotary != -1) {
        // request that the axis unlock
        tpSetRotaryUnlock(tp, tc->indexrotary, 1);
        // if it is unlocked, fall through and start the move.
        // otherwise, just come back later and check again
        if (!tpGetRotaryIsUnlocked(tp, tc->indexrotary))
            return TP_ERR_WAITING;
    }

    // Temporary debug message
    tp_debug_print("%d: Activate tc id = %d target_vel = %f req_vel = %f final_vel = %f length = %f acc(%f) jerk(%.20f)\n",
            __LINE__,
            tc->id,
            tc->target_vel,
            tc->reqvel,
            tc->finalvel,
            tc->target,
            tc->maxaccel,
            tc->jerk);

    tc->active = 1;

    tc->currentvel = 0;

    //Do not change initial velocity here, since tangent blending already sets this up
    tp->motionType = tc->canon_motion_type;
    tc->blending_next = 0;
    tc->on_final_decel = 0;

    tp_debug_print("TODO: implement waiting_for_index for spindle@tp\n");
//    if (TC_SYNC_POSITION == tc->synchronized && !(get_spindleSync(tp->shared))) {
//        tp_debug_print("Setting up position sync\n");
//        // if we aren't already synced, wait
//        tp->spindle.waiting_for_index = tc->id;
//        // ask for an index reset
//        set_spindle_index_enable(tp->shared, 1);
//        rtapi_print_msg(RTAPI_MSG_DBG, "Waiting on sync...\n");
//        return TP_ERR_WAITING;
//    }

    // Update the modal state displayed by the TP
    tp->execTag = tc->tag;

    return TP_ERR_OK;
}


/**
 * Run velocity mode synchronization.
 * Update requested velocity to follow the spindle's velocity (scaled by feed rate).
 */
STATIC void tpSyncVelocityMode(TP_STRUCT * const tp, TC_STRUCT * const tc, TC_STRUCT const * nexttc) {
    double speed = get_spindleSpeedIn(tp->shared);
    double pos_error = rtapi_fabs(speed) * tc->uu_per_rev;
    // Account for movement due to parabolic blending with next segment
    if(nexttc) {
        pos_error -= nexttc->progress;
    }
    tc->target_vel = pos_error;
}

/* spindle speed calculator for CSS motion */
STATIC double tpSyncSpindleSpeed (TP_STRUCT * tp)
{
    if (tp->spindle.on) {
        // spindle.css_factor is css_numerator in emccanon.cc
        if(tp->spindle.css_factor)
        {
            // for G96
            tp_debug_print ("TODO: add yoffset for VBC, bypass yoffset for MEINAN\n");
    //        double denom =  sqrt(pow(emcmotStatus->spindle.xoffset - emcmotStatus->carte_pos_cmd.tran.x, 2) +
    //                             pow(emcmotStatus->g5x_offset.tran.y - emcmotStatus->carte_pos_cmd.tran.y, 2));
            double denom =  (tp->spindle.xoffset - tp->currentPos.tran.x);
            double speed;           // speed for major spindle (spindle-s)
            double maxspeed;        // absolute max spindle speed
            int positive = ((tp->spindle.xoffset - tp->currentPos.tran.x) > 0)? 1: -1;
            // css_factor: unit(mm or inch)/min
            if(denom != 0)
            {
                speed = tp->spindle.css_factor / (denom * 60.0); // rps
            }
            else
            {
                speed = tp->spindle.speed_rps;
            }
    // for VBC:
    //        if(emcmotStatus->spindle.dynamic_speed_mode == 0)
    //        {
    //        }
    //        else
    //        {
    //            // dynamic_spindle_mode == 1
    //            // adjust spindle speed dynamically to maintain constant tangential velocity
    //            // the spindle speed is constant if (r <= const_speed_radius)
    //            double csr;
    //            csr = emcmotStatus->spindle.const_speed_radius;
    //
    //            if ((fabs(denom) >= csr) && (csr > 0))
    //            {
    //                           以下做法相當於將S(線速度) 設為 (D(max-RPM) * csr) :
    //                speed = (emcmotStatus->spindle.speed_rps * csr) / denom; // rps
    //            }
    //            else
    //            {
    //                speed = emcmotStatus->spindle.speed_rps;
    //            }
    //        }
            speed = rtapi_fabs(speed);
            maxspeed = rtapi_fabs(tp->spindle.speed_rps);
            if(speed > maxspeed) speed = maxspeed;
            tp->spindle.speed_req_rps = speed * tp->spindle.direction;
            // css_error: to be compensated by another spindle
            tp->spindle.css_error =
                            (tp->spindle.css_factor / 60.0
                             - denom * positive * rtapi_fabs(tp->spindle.curr_vel_rps))
                            * tp->spindle.direction; // (unit/(2*PI*sec)
            tp_debug_print ("speed_req_rps(%f)\n", tp->spindle.speed_req_rps);
            tp_debug_print ("css_req(%f)(unit/sec)\n", denom * tp->spindle.speed_req_rps * 2 * M_PI);
            tp_debug_print ("css_cur(%f)\n", denom * tp->spindle.curr_vel_rps * 2 * M_PI);
            tp_debug_print ("css_error(%f)(unit/(2*PI*sec))\n", tp->spindle.css_error);
            tp_debug_print ("speed(%f) denom(%f) s.curr_vel(%f) css_factor(%f)\n",
                                 speed, denom, tp->spindle.curr_vel_rps, tp->spindle.css_factor);
            tp_debug_print ("spindle.direction(%d)\n", tp->spindle.direction);
    //        DP ("synched-joint-vel(%f)(unit/sec)\n", tp->spindle.curr_vel_rps * tp->uu_per_rev);
    #if (CSS_TRACE!=0)
            /* prepare data for gnuplot */
            fprintf (csstrace, "%11d%15.9f%15.9f%19.9f%19.9f%19.9f\n"
                    , _dt
                    , denom
                    , tp->progress
                    , tp->spindle.css_factor / 60.0 * 2 * M_PI
                    , denom * tp->spindle.curr_vel_rps * 2 * M_PI
                    , (tp->spindle.css_factor / 60.0 * 2 * M_PI - denom * tp->spindle.curr_vel_rps * 2 * M_PI));
            _dt += 1;
    #endif
        }
        else
        {
            // G97, G33 w/ G97 or G33.1
            tp->spindle.speed_req_rps = tp->spindle.speed_rps;
            tp->spindle.css_error = 0;
        }
    } else {
        // spindle is OFF
        tp->spindle.speed_req_rps = 0;
        tp->spindle.css_error = 0;
    }

    return (tp->spindle.speed_req_rps);
}

/**
 * Run position mode synchronization.
 * Updates requested velocity for a trajectory segment to track the spindle's position.
 */
STATIC void tpSyncPositionMode(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc ) {

    if ((tc->motion_type == TC_SPINDLE_SYNC_MOTION)) {
        // calc spindle target velocity of spindle for SpindleSyncMotion
        if (tc->coords.spindle_sync.mode < 2)
        {   // G33, G33.1
            tc->target_vel = rtapi_fabs(tpSyncSpindleSpeed(tp));
        }
    }

    if (nexttc && nexttc->synchronized) {
        //If the next move is synchronized too, then match it's
        //requested velocity to the current move
        nexttc->target_vel = tc->target_vel;
    }
}


/**
 * Perform parabolic blending if needed between segments and handle status updates.
 * This isolates most of the parabolic blend stuff to make the code path
 * between tangent and parabolic blends easier to follow.
 */
STATIC int tpDoParabolicBlending(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {

    tc_debug_print("in DoParabolicBlend\n");
    tpUpdateBlend(tp,tc,nexttc);

    /* Status updates */
    //Decide which segment we're in depending on which is moving faster
    if(tc->currentvel > nexttc->currentvel) {
        tpUpdateMovementStatus(tp, tc);
    } else {
        tpToggleDIOs(tp, nexttc);
        tpUpdateMovementStatus(tp, nexttc);
    }
#ifdef TP_SHOW_BLENDS
    // hack to show blends in axis
    tp->motionType = 0;
#endif

    //Update velocity status based on both tc and nexttc
    set_current_vel(tp->shared, tc->currentvel + nexttc->currentvel);

    return TP_ERR_OK;
}


/**
 * Do a complete update on one segment.
 * Handles the majority of updates on a single segment for the current cycle.
 */
STATIC int tpUpdateCycle(TP_STRUCT * const tp,
        TC_STRUCT * const tc, TC_STRUCT * const nexttc) {

    //placeholders for position for this update
    EmcPose before;

    //Store the current position due to this TC
    tcGetPos(tc, &before);

    // Update the start velocity if we're not blending yet
    if (!tc->blending_next) {
        tc->vel_at_blend_start = tc->currentvel;
    }

    // Run cycle update with cycle time
    tcRunCycle(tp, tc);

    //Check if we're near the end of the cycle and set appropriate changes
    tpCheckEndCondition(tp, tc, nexttc);

    EmcPose displacement;

    // Calculate displacement
    tcGetPos(tc, &displacement);
    emcPoseSelfSub(&displacement, &before);

    // for CSS, update spindle displacement to corresponding spindleAxis
    tcUpdateSpindleAxisCSS(tp, tc, &displacement);

    //Store displacement (checking for valid pose)
    int res_set = tpAddCurrentPos(tp, &displacement);

#ifdef TC_DEBUG
    double mag;
    emcPoseMagnitude(&displacement, &mag);
    tc_debug_print("cycle movement = %f\n", mag);
#endif

    return res_set;
}


/**
 * Send default values to status structure.
 */
STATIC int tpUpdateInitialStatus(TP_STRUCT const * const tp) {
    // Update queue length
    set_tcqlen(tp->shared, tcqLen(&tp->queue));
    // Set default value for requested speed
    set_requested_vel(tp->shared, 0.0);
    return TP_ERR_OK;
}


/**
 * Flag a segment as needing a split cycle.
 * In addition to flagging a segment as splitting, do any preparations to store
 * data for the next cycle.
 */
STATIC inline int tcSetSplitCycle(TC_STRUCT * const tc, double split_time,
        double v_f)
{
    tp_debug_print("split time for id %d is %.16g\n", tc->id, split_time);
    if (tc->splitting != 0 && split_time > 0.0) {
        rtapi_print_msg(RTAPI_MSG_ERR,"already splitting on id %d with cycle time %.16g, dx = %.16g, split time %.12g\n",
                tc->id,
                tc->cycle_time,
                tc->target-tc->progress,
                split_time);
        return TP_ERR_FAIL;
    }
    tc->splitting = 1;
    tc->cycle_time = split_time;
    tc->term_vel = v_f;
    return 0;
}


/**
 * Check remaining time in a segment and calculate split cycle if necessary.
 * This function estimates how much time we need to complete the next segment.
 * If it's greater than one timestep, then we do nothing and carry on. If not,
 * then we flag the segment as "splitting", so that during the next cycle,
 * it handles the transition to the next segment.
 */
STATIC int tpCheckEndCondition(
        TP_STRUCT * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT * const nexttc)
{
    //Assume no split time unless we find otherwise
    tc->cycle_time = tp->cycleTime;

    assert(tc->remove != 1);

    //Initial guess at dt for next round
    double dx = tc->target - tc->progress;
    tc_debug_print("tpCheckEndCondition: dx = %e\n",dx);

    if (dx <= TP_POS_EPSILON) {
        if (get_probing(tp->shared) && get_rtp_running(tp->shared))
        {
            // G38.X: 等待 RISC 處理完所有 TP 命令才結束
            return TP_ERR_NO_ACTION;
        }

        tc->progress = tc->target;
        tc->remove = 1;
        if (tc->term_cond == TC_TERM_COND_TANGENT) {
            // BLEND with NEXT-TC
            if (nexttc) {
                assert (nexttc->active == 0);
                nexttc->active          = 1;
                nexttc->currentvel      = tc->currentvel;
                nexttc->cur_accel       = tc->cur_accel;
                nexttc->accel_state     = tc->accel_state;
                nexttc->progress        = tc->progress - tc->target;
                tp->motionType          = nexttc->canon_motion_type;
                // FIXME: need algorighm for flushing a bounch of small segments
                assert (nexttc->progress < nexttc->target);
            }
        }
        return TP_ERR_OK;
    } else {
        return TP_ERR_NO_ACTION;
    }
#if 0
    if (dx <= TP_POS_EPSILON) {
        //If the segment is close to the target position, then we assume that it's done.
        tp_debug_print("close to target, dx = %.12f\n",dx);
        //Force progress to land exactly on the target to prevent numerical errors.
        tc->progress = tc->target;
        tcSetSplitCycle(tc, 0.0, tc->currentvel);
        if (tc->term_cond == TC_TERM_COND_STOP || tc->term_cond == TC_TERM_COND_EXACT) {
            tc->remove = 1;
        }
        return TP_ERR_OK;
    } else if (tc->term_cond == TC_TERM_COND_STOP || tc->term_cond == TC_TERM_COND_EXACT) {
        return TP_ERR_NO_ACTION;
    }


    double v_f = tpGetRealFinalVel(tp, tc, nexttc);
    double v_avg = (tc->currentvel + v_f) / 2.0;

    //Check that we have a non-zero "average" velocity between now and the
    //finish. If not, it means that we have to accelerate from a stop, which
    //will take longer than the minimum 2 timesteps that each segment takes, so
    //we're safely far form the end.

    //Get dt assuming that we can magically reach the final velocity at
    //the end of the move.
    //
    //KLUDGE: start with a value below the cutoff
    double dt = TP_TIME_EPSILON / 2.0;
    if (v_avg > TP_VEL_EPSILON) {
        //Get dt from distance and velocity (avoid div by zero)
        dt = rtapi_fmax(dt, dx / v_avg);
    } else {
        if ( dx > (v_avg * tp->cycleTime) && dx > TP_POS_EPSILON) {
            tc_debug_print(" below velocity threshold, assuming far from end\n");
            return TP_ERR_NO_ACTION;
        }
    }

    //Calculate the acceleration this would take:

    double dv = v_f - tc->currentvel;
    double a_f = dv / dt;

    //If this is a valid acceleration, then we're done. If not, then we solve
    //for v_f and dt given the max acceleration allowed.
    double a_max = tpGetScaledAccel(tp,tc);

    //If we exceed the maximum acceleration, then the dt estimate is too small.
    double a = a_f;
    int recalc = sat_inplace(&a, a_max);

    //Need to recalculate vf and above
    if (recalc) {
        tc_debug_print(" recalculating with a_f = %f, a = %f\n", a_f, a);
        double disc = pmSq(tc->currentvel / a) + 2.0 / a * dx;
        if (disc < 0) {
            //Should mean that dx is too big, i.e. we're not close enough
            tc_debug_print(" dx = %f, too large, not at end yet\n",dx);
            return TP_ERR_NO_ACTION;
        }

        if (disc < TP_TIME_EPSILON * TP_TIME_EPSILON) {
            tc_debug_print("disc too small, skipping sqrt\n");
            dt =  -tc->currentvel / a;
        } else if (a > 0) {
            tc_debug_print("using positive sqrt\n");
            dt = -tc->currentvel / a + pmSqrt(disc);
        } else {
            tc_debug_print("using negative sqrt\n");
            dt = -tc->currentvel / a - pmSqrt(disc);
        }

        tc_debug_print(" revised dt = %f\n", dt);
        //Update final velocity with actual result
        v_f = tc->currentvel + dt * a;
    }

    if (dt < TP_TIME_EPSILON) {
        //Close enough, call it done
        tc_debug_print("revised dt small, finishing tc\n");
        tc->progress = tc->target;
        tcSetSplitCycle(tc, 0.0, v_f);
    } else if (dt < tp->cycleTime ) {
        tc_debug_print(" corrected v_f = %f, a = %f\n", v_f, a);
        tcSetSplitCycle(tc, dt, v_f);
    } else {
        tc_debug_print(" dt = %f, not at end yet\n",dt);
    }

    return TP_ERR_OK;
#endif
}

STATIC int tpHandleRegularCycle(TP_STRUCT * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT * const nexttc)
{
    if (tc->remove) {
        //Don't need to update since this segment is flagged for removal
        return TP_ERR_NO_ACTION;
    }
    //Run with full cycle time
    tc_debug_print("Normal cycle\n");
    tc->cycle_time = tp->cycleTime;
    tpUpdateCycle(tp, tc, nexttc);

    /* Parabolic blending */

    tpComputeBlendVelocity(tp, tc, nexttc);
    if (nexttc && tcIsBlending(tc)) {
        tpDoParabolicBlending(tp, tc, nexttc);
    } else {
        //Update status for a normal step
        tpToggleDIOs(tp, tc);
        tpUpdateMovementStatus(tp, tc);
    }
    tp_info_print("%d: tc reqvel(%f) target_vel(%f) maxvel(%f) jerk(%.20f)\n", __LINE__, tc->reqvel, tc->target_vel, tc->maxvel, tc->jerk);

    return TP_ERR_OK;
}


/* calculate spindle displacement(delta-s) and velocity */
STATIC void tpSpindleCycle(TP_STRUCT * const tp, EmcPose * const displacement)
{
    /*Velocity Control Algorithm is taking from simple_tp.c */
    double max_da, vel_err, acc_req;

    max_da = tp->spindle.max_da;

    /**
     * calculate an acceleration that tends to drive vel_err to zero,
     * but allows for move without velocity overshoot
     **/
    vel_err = tp->spindle.speed_rps - tp->spindle.curr_vel_rps;

    /* positive and negative errors require some sign flipping to
           avoid sqrt(negative) */
    if (vel_err > tp->spindle.tiny_dv) {
        acc_req = -max_da +
                rtapi_sqrt(2.0 * tp->spindle.max_jerk * vel_err + max_da * max_da);
    } else if (vel_err < -tp->spindle.tiny_dv) {
        acc_req =  max_da -
                rtapi_sqrt(-2.0 * tp->spindle.max_jerk * vel_err + max_da * max_da);
    } else {
        /* within 'tiny_dv' of desired vel, no need to accel */
        acc_req = 0;
        tp->spindle.curr_vel_rps = tp->spindle.speed_rps;
    }

    /* limit acceleration request */
    if (acc_req > tp->spindle.max_acc) {
        acc_req = tp->spindle.max_acc;
    } else if (acc_req < -tp->spindle.max_acc) {
        acc_req = -tp->spindle.max_acc;
    }

    /* ramp acceleration toward request at jerk limit */
    if (acc_req > (tp->spindle.curr_acc + max_da)) {
        tp->spindle.curr_acc += max_da;
    } else if (acc_req < (tp->spindle.curr_acc - max_da)) {
        tp->spindle.curr_acc -= max_da;
    } else {
        tp->spindle.curr_acc = acc_req;
    }

    /* integrate acceleration to get new velocity */
    tp->spindle.curr_vel_rps += tp->spindle.curr_acc * tp->cycleTime;
    displacement->s = tp->spindle.curr_vel_rps * tp->cycleTime;

    return;
}

void tpUpdateSpindleAxis(TP_STRUCT const * const tp, EmcPose * const pos)
{
    switch (tp->spindle.axis) {
        case -1: /* do not specify spindleAxis */
            break;
        case 3:
            pos->a = pos->s;
            break;
        case 4:
            pos->b = pos->s;
            break;
        case 5:
            pos->c = pos->s;
            break;
        default:
            rtapi_print_msg (RTAPI_MSG_ERR, "(%s:%d) incorrect spindleAxis(%d)\n", __FUNCTION__, __LINE__,
                                            tp->spindle.axis);
            break;
    }
    return;
}

STATIC int tpHandelSpindle(TP_STRUCT * const tp)
{
    /* for CSS motion, update its spindle position at tcUpdateSpindleAxisCSS() */
    if (tp->spindle.css_factor == 0)
    {
        if ((tp->spindle.on) || ((tp->spindle.on == 0) && (tp->spindle.curr_vel_rps != 0)))
        {
            EmcPose displacement;
            ZERO_EMC_POSE(displacement);
            tpSpindleCycle(tp, &displacement);  // calculate spindle displacement(delta-s) and velocity
            // update spindle displacement to corresponding spindleAxis
            tpUpdateSpindleAxis(tp, &displacement);
            //Store displacement (checking for valid pose)
            int res_set = tpAddCurrentPos(tp, &displacement);
            return (res_set);
        }
    }
    return TP_ERR_OK;
}

/**
 * Calculate an updated goal position for the next timestep.
 * This is the brains of the operation. It's called every TRAJ period and is
 * expected to set tp->currentPos to the new machine position. Lots of other
 * const tp fields (depth, done, etc) have to be twiddled to communicate the
 * status; I think those are spelled out here correctly and I can't clean it up
 * without breaking the API that the TP presents to motion.
 */
int tpRunCycle(TP_STRUCT * const tp, long period)
{
#if (TRACE!=0)
    _dt += 1;
#endif
    
    // handle spindle velocity control for non-CSS motions
    tpHandelSpindle(tp);

    //Pointers to current and next trajectory component
    TC_STRUCT *tc;
    TC_STRUCT *nexttc;

    /* Get pointers to current and relevant future segments. It's ok here if
     * future segments don't exist (NULL pointers) as we check for this later).
     */
    tc = tcqItem(&tp->queue, 0);
    nexttc = tcqItem(&tp->queue, 1);

    //Set GUI status to "zero" state
    tpUpdateInitialStatus(tp);

    //If we have a NULL pointer, then the queue must be empty, so we're done.
    if(!tc) {
        tpHandleEmptyQueue(tp);
        return TP_ERR_WAITING;
    }
    
    //Update motion.spindle.{x,y,z}uu_per_rev
    tpSetUuPerRev(tp, tc);

    tc_debug_print("(%s:%d)-------------------\n", __FUNCTION__, __LINE__);

#ifdef TC_DEBUG
    //Hack debug output for timesteps
    static double time_elapsed = 0;
    time_elapsed+=tp->cycleTime;
#endif

    /* If the queue empties enough, assume that the program is near the end.
     * This forces the last segment to be "finalized" to let the optimizer run.*/
    /*tpHandleLowQueue(tp);*/

    /* If we're aborting or pausing and the velocity has reached zero, then we
     * don't need additional planning and can abort here. */
    if (tpHandleAbort(tp, tc, nexttc) == TP_ERR_STOPPED) {
        return TP_ERR_STOPPED;
    }

    //Return early if we have a reason to wait (i.e. not ready for motion)
    if (tpCheckAtSpeed(tp, tc) != TP_ERR_OK){
        return TP_ERR_WAITING;
    }

    if(!tc->active) {
        int res = tpActivateSegment(tp, tc);
        // Need to wait to continue motion, end planning here
        if (res == TP_ERR_WAITING) {
            return TP_ERR_WAITING;
        }
    }

    /** If synchronized with spindle, calculate requested velocity to track
     * spindle motion.*/
    switch (tc->synchronized) {
        case TC_SYNC_NONE:
            set_spindleSync(tp->shared, 0);
            break;
        case TC_SYNC_VELOCITY:
            /* comes from START_SPEED_FEED_SYNCH() of interp_convert.cc */
            /* at 2015-08-22, there's no motion that is with TC_SYNC_VELOCITY */
            assert(0);
            tp_debug_print("sync velocity\n");
            tpSyncVelocityMode(tp, tc, nexttc);
            break;
        case TC_SYNC_POSITION:
            tp_debug_print("sync position\n");
            tpSyncPositionMode(tp, tc, nexttc);
            break;
        default:
            tp_debug_print("unrecognized spindle sync state!\n");
            break;
    }

#ifdef TC_DEBUG
    EmcPose pos_before = tp->currentPos;
#endif

    tc->feed_override = get_net_feed_scale(tp->shared);
    if(nexttc) {
        nexttc->feed_override = get_net_feed_scale(tp->shared);

    tcClearFlags(tc);
    tcClearFlags(nexttc);
    }

    /* handle pausing */
    if(tp->pausing && (!tc->synchronized)) {
        tc->feed_override = 0.0;
        if(nexttc) {
            nexttc->feed_override = 0.0;
        }
    }

    tpHandleRegularCycle(tp, tc, nexttc);

#ifdef TC_DEBUG
    double mag;
    EmcPose disp;
    emcPoseSub(&tp->currentPos, &pos_before, &disp);
    emcPoseMagnitude(&disp, &mag);
    tc_debug_print("time: %.12e total movement = %.12e vel = %.12e\n",
		   time_elapsed,
		   mag, get_current_vel(tp->shared));

    tc_debug_print("tp_displacement = %.12e %.12e %.12e time = %.12e\n",
            disp.tran.x,
            disp.tran.y,
            disp.tran.z,
            time_elapsed);
#endif

    // If TC is complete, remove it from the queue.
    if (tc->remove) {
        tpCompleteSegment(tp, tc);
    }

    return TP_ERR_OK;
}

int tpSetSpindleSync(TP_STRUCT * const tp, double sync, int mode) {
    if(sync) {
        if (mode) {
            tp->synchronized = TC_SYNC_VELOCITY;
        } else {
            tp->synchronized = TC_SYNC_POSITION;
        }
        tp->uu_per_rev = sync;
    } else
        tp->synchronized = 0;

    return TP_ERR_OK;
}

int tpPause(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 1;
    return TP_ERR_OK;
}

int tpResume(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 0;
    return TP_ERR_OK;
}

int tpTcqInit(TP_STRUCT * const tp)
{
    int ret;

    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    ret = tcqInit(&tp->queue);
    return ret;
}

int tpAbort(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (!tp->aborting) {
        /* const to abort, signal a pause and set our abort flag */
        tpPause(tp);
        tp->aborting = 1;
    }
    return tpClearDIOs(tp); //clears out any already cached DIOs
}

int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->motionType;
}

int tpGetAccelState(TP_STRUCT * const tp)
{
    return tp->accelState;
}

int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{

    if (0 == tp) {
        ZERO_EMC_POSE((*pos));
        return TP_ERR_FAIL;
    } else {
        *pos = tp->currentPos;
    }

    return TP_ERR_OK;
}

int tpIsDone(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->done;
}

int tpQueueDepth(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->depth;
}

int tpActiveDepth(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->activeDepth;
}

int tpSetAout(TP_STRUCT * const tp, unsigned int index, double start, double end) {
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->syncdio.anychanged = 1; //something has changed
    tp->syncdio.aio_mask |= (1ull << index);
    tp->syncdio.aios[index] = start;
    return TP_ERR_OK;
}

/* tpSetSpindle(): setup speed_rps, direction, ... etc. */
int tpSetSpindle(TP_STRUCT * tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    tp_debug_print("(%s:%d) spindle.speed(%f) max_vel(%f) max_acc(%f) max_jerk(%f)\n", __FUNCTION__, __LINE__,
            tp->spindle.speed, tp->spindle.max_vel, tp->spindle.max_acc, tp->spindle.max_jerk);
    tp->spindle.speed_rps = tp->spindle.speed / 60.0;
    tp->spindle.max_da = tp->spindle.max_jerk * tp->cycleTime;
    tp->spindle.tiny_dv = tp->spindle.max_da * tp->cycleTime * 0.001;

    if (tp->spindle.speed > 0) {
        tp->spindle.direction = 1;      // direction: 0 stopped, 1 forward, -1 reverse
    } else if (tp->spindle.speed == 0) {
        tp->spindle.direction = 0;
    } else {
        tp->spindle.direction = -1;
    }

    return TP_ERR_OK;
}


int tpSetDout(TP_STRUCT * const tp, unsigned int index, unsigned char start, unsigned char end) {
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->syncdio.anychanged = 1; //something has changed
    RTAPI_BIT_SET(tp->syncdio.dio_mask, index);
    if (start > 0)
        tp->syncdio.dios[index] = 1; // the end value can't be set from canon currently, and has the same value as start
    else
        tp->syncdio.dios[index] = -1;
    return TP_ERR_OK;
}

// test whether a tpPause() has actually resulted in motion stopped
// needed since a synchronized motion in progress will not be paused
int tpIsPaused(TP_STRUCT * tp)
{
    TC_STRUCT *tc;

    if (0 == tp) { // I assume this would be a fatal error?
        //rtapi_print_msg(RTAPI_MSG_DBG, " tpIsPaused(): tp == NULL\n");
        return 0;
    }
    tc = tcqItem(&tp->queue, 0);
    if (!tc) {  // motion queue empty.
        //rtapi_print_msg(RTAPI_MSG_DBG, " tpIsPaused(): motion queue empty\n");
        return tp->pausing;
    }
    //rtapi_print_msg(RTAPI_MSG_DBG, " tpIsPaused(): pausing=%d synced=%d velmode=%d\n",
    //                tp->pausing, tc->synchronized, tc->velocity_mode);

    //If the machine is still moving, then it's not actually paused yet
    if ( get_current_vel(tp->shared) > TP_VEL_EPSILON) {
        tp_debug_print("IsPaused: still slowing");
        return 0;
    }

    return (tp->pausing && (!tc->synchronized || tp->velocity_mode) && (get_update_pos_req(tp->shared) == 0));
}

// prime an alternate motion queue with current parameters
// this avoids tracking calls in parallel
// to tpSetCycleTime(), tpSetVmax(),tpSetAmax(),tpSetVlimit()
// it does not tpSetPos(), tpSetExeciId(),or completely tpClear() the altq

int tpSnapshot(TP_STRUCT * from, TP_STRUCT * to)
{
    if (from == 0 || to == 0) {
	return -1;
    }

    to->vMax = from->vMax;
    to->ini_maxvel = from->ini_maxvel;
    to->vLimit = from->vLimit;
    to->aMax = from->aMax;
    to->cycleTime = from->cycleTime;

    return 0;
}

// vim:sw=4:sts=4:et:
