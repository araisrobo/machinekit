/*!
********************************************************************
* Description: tc.c
*\brief Discriminate-based trajectory planning
*
*\author Derived from a work by Fred Proctor & Will Shackleford
*\author rewritten by Chris Radek
*
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include "rtapi.h"		/* rtapi_print_msg */
#include "rtapi_math.h"
#include "posemath.h"
#include "blendmath.h"
#include "emcpose.h"
#include "tc.h"
#include "tp_types.h"
#include "spherical_arc.h"
#include "motion_types.h"
#include <assert.h>

//Debug output
#include "tp_debug.h"


int tcCircleStartAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out)
{
    PmCartesian startpoint;
    PmCartesian radius;
    PmCartesian tan, perp;

    pmCirclePoint(&tc->coords.circle.xyz, 0.0, &startpoint);
    pmCartCartSub(&startpoint, &tc->coords.circle.xyz.center, &radius);
    pmCartCartCross(&tc->coords.circle.xyz.normal, &radius, &tan);
    pmCartUnitEq(&tan);
    //The unit vector's actual direction is adjusted by the normal
    //acceleration here. This unit vector is NOT simply the tangent
    //direction.
    pmCartCartSub(&tc->coords.circle.xyz.center, &startpoint, &perp);
    pmCartUnitEq(&perp);

    pmCartScalMult(&tan, tc->maxaccel, &tan);
    pmCartScalMultEq(&perp, pmSq(0.5 * tc->reqvel)/tc->coords.circle.xyz.radius);
    pmCartCartAdd(&tan, &perp, out);
    pmCartUnitEq(out);
    return 0;
}

int tcCircleEndAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out)
{
    PmCartesian endpoint;
    PmCartesian radius;

    pmCirclePoint(&tc->coords.circle.xyz, tc->coords.circle.xyz.angle, &endpoint);
    pmCartCartSub(&endpoint, &tc->coords.circle.xyz.center, &radius);
    pmCartCartCross(&tc->coords.circle.xyz.normal, &radius, out);
    pmCartUnitEq(out);
    return 0;
}

/**
 * Get the acceleration direction unit vector for blend velocity calculations.
 * This calculates the direction of acceleration at the start of a segment.
 */
int tcGetStartAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out) {

    switch (tc->motion_type) {
        case TC_LINEAR:
        case TC_RIGIDTAP:
            *out=tc->coords.line.xyz.uVec;
            break;
        case TC_CIRCULAR:
            tcCircleStartAccelUnitVector(tc,out);
            break;
        case TC_SPHERICAL:
            return -1;
        default:
            return -1;
    }
    return 0;
}

/**
 * Get the acceleration direction unit vector for blend velocity calculations.
 * This calculates the direction of acceleration at the end of a segment.
 */
int tcGetEndAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out) {

    switch (tc->motion_type) {
        case TC_LINEAR:
            *out=tc->coords.line.xyz.uVec;
            break;
        case TC_RIGIDTAP:
            pmCartScalMult(&tc->coords.line.xyz.uVec, -1.0, out);
            break;
        case TC_CIRCULAR:
            tcCircleEndAccelUnitVector(tc,out);
            break;
       case TC_SPHERICAL:
            return -1;
       default:
            return -1;
    }
    return 0;
}

int tcGetIntersectionPoint(TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc, PmCartesian * const point)
{
    // TODO NULL pointer check?
    // Get intersection point from geometry
    if (tc->motion_type == TC_LINEAR) {
        *point = tc->coords.line.xyz.start;
    } else if (prev_tc->motion_type == TC_LINEAR) {
        *point = prev_tc->coords.line.xyz.end;
    } else if (tc->motion_type == TC_CIRCULAR){
        pmCirclePoint(&tc->coords.circle.xyz, 0.0, point);
    } else {
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}


/**
 * Check if a segment can be consumed without disrupting motion or synced IO.
 */
int tcCanConsume(TC_STRUCT const * const tc)
{
    if (!tc) {
        return false;
    }

    if (tc->syncdio.anychanged || tc->blend_prev || tc->atspeed) {
        //TODO add other conditions here (for any segment that should not be consumed by blending
        return false;
    }

    return true;

}

/**
 * Find the geometric tangent vector to a helical arc.
 * Unlike the acceleration vector, the result of this calculation is a vector
 * tangent to the helical arc. This is called by wrapper functions for the case of a circular or helical arc.
 */
int pmCircleTangentVector(PmCircle const * const circle,
        double angle_in, PmCartesian * const out)
{

    PmCartesian startpoint;
    PmCartesian radius;
    PmCartesian uTan, dHelix, dRadial;

    // Get vector in radial direction
    pmCirclePoint(circle, angle_in, &startpoint);
    pmCartCartSub(&startpoint, &circle->center, &radius);

    /* Find local tangent vector using planar normal. Assuming a differential
     * angle dtheta, the tangential component of the tangent vector is r *
     * dtheta. Since we're normalizing the vector anyway, assume dtheta = 1.
     */
    pmCartCartCross(&circle->normal, &radius, &uTan);

    // find dz/dtheta and get differential movement along helical axis
    double h;
    pmCartMag(&circle->rHelix, &h);

    /* the binormal component of the tangent vector is (dz / dtheta) * dtheta.
     */
    double dz = 1.0 / circle->angle;
    pmCartScalMult(&circle->rHelix, dz, &dHelix);

    pmCartCartAddEq(&uTan, &dHelix);

    /* The normal component is (dr / dtheta) * dtheta.
     */
    double dr = circle->spiral / circle->angle;
    pmCartUnit(&radius, &dRadial);
    pmCartScalMultEq(&dRadial, dr);
    pmCartCartAddEq(&uTan, &dRadial);

    //Normalize final output vector
    pmCartUnit(&uTan, out);

    return 0;
}


/**
 * Calulate the unit tangent vector at the start of a move for any segment.
 */
int tcGetStartTangentUnitVector(TC_STRUCT const * const tc, PmCartesian * const out) {

    switch (tc->motion_type) {
        case TC_LINEAR:
            *out=tc->coords.line.xyz.uVec;
            break;
        case TC_RIGIDTAP:
            *out=tc->coords.rigidtap.xyz.uVec;
            break;
        case TC_CIRCULAR:
            pmCircleTangentVector(&tc->coords.circle.xyz, 0.0, out);
            break;
        default:
            rtapi_print_msg(RTAPI_MSG_ERR, "Invalid motion type %d!\n",tc->motion_type);
            return -1;
    }
    return 0;
}

/**
 * Calulate the unit tangent vector at the end of a move for any segment.
 */
int tcGetEndTangentUnitVector(TC_STRUCT const * const tc, PmCartesian * const out) {

    switch (tc->motion_type) {
        case TC_LINEAR:
            *out=tc->coords.line.xyz.uVec;
            break;
        case TC_RIGIDTAP:
            pmCartScalMult(&tc->coords.rigidtap.xyz.uVec, -1.0, out);
            break;
        case TC_CIRCULAR:
            pmCircleTangentVector(&tc->coords.circle.xyz,
                    tc->coords.circle.xyz.angle, out);
            break;
        default:
            rtapi_print_msg(RTAPI_MSG_ERR, "Invalid motion type %d!\n",tc->motion_type);
            return -1;
    }
    return 0;
}

/*! tcGetPos() function
 *
 * \brief This function calculates the machine position along the motion's path.
 *
 * As we move along a TC, from zero to its length, we call this function repeatedly,
 * with an increasing tc->progress.
 * This function calculates the machine position along the motion's path
 * corresponding to the current progress.
 * It gets called at the end of tpRunCycle()
 *
 * @param    tc    the current TC that is being planned
 *
 * @return	 EmcPose   returns a position (\ref EmcPose = datatype carrying XYZABC information
 */

int tcGetPos(TC_STRUCT const * const tc, EmcPose * const out) {
    tcGetPosReal(tc, TC_GET_PROGRESS, out);
    return 0;
}

int tcGetStartpoint(TC_STRUCT const * const tc, EmcPose * const out) {
    tcGetPosReal(tc, TC_GET_STARTPOINT, out);
    return 0;
}

int tcGetEndpoint(TC_STRUCT const * const tc, EmcPose * const out) {
    tcGetPosReal(tc, TC_GET_ENDPOINT, out);
    return 0;
}

int tcGetPosReal(TC_STRUCT const * const tc, int of_point, EmcPose * const pos)
{
    PmCartesian xyz;
    PmCartesian abc;
    PmCartesian uvw;
    double progress=0.0;

    switch (of_point) {
        case TC_GET_PROGRESS:
            progress = tc->progress;
            break;
        case TC_GET_ENDPOINT:
            progress = tc->target;
            break;
        case TC_GET_STARTPOINT:
            progress = 0.0;
            break;
    }


    // Used for arc-length to angle conversion with spiral segments
    double angle = 0.0;

    switch (tc->motion_type){
        case TC_RIGIDTAP:
            if(tc->coords.rigidtap.state > REVERSING) {
                pmCartLinePoint(&tc->coords.rigidtap.aux_xyz, progress, &xyz);
            } else {
                pmCartLinePoint(&tc->coords.rigidtap.xyz, progress, &xyz);
            }
            // no rotary move allowed while tapping
            abc = tc->coords.rigidtap.abc;
            uvw = tc->coords.rigidtap.uvw;
            break;
        case TC_LINEAR:
            pmCartLinePoint(&tc->coords.line.xyz,
                    progress, &xyz);
            pmCartLinePoint(&tc->coords.line.uvw,
                    progress * tc->coords.line.uvw.tmag / tc->target,
                    &uvw);
            pmCartLinePoint(&tc->coords.line.abc,
                    progress * tc->coords.line.abc.tmag / tc->target,
                    &abc);
            break;
        case TC_CIRCULAR:
            angle = pmCircleAngleFromProgress(&tc->coords.circle.xyz,
                    &tc->coords.circle.fit,
                    progress);
            pmCirclePoint(&tc->coords.circle.xyz,
                    angle,
                    &xyz);
            pmCartLinePoint(&tc->coords.circle.abc,
                    progress * tc->coords.circle.abc.tmag / tc->target,
                    &abc);
            pmCartLinePoint(&tc->coords.circle.uvw,
                    progress * tc->coords.circle.uvw.tmag / tc->target,
                    &uvw);
            break;
        case TC_SPHERICAL:
            arcPoint(&tc->coords.arc.xyz,
                    progress,
                    &xyz);
            abc = tc->coords.arc.abc;
            uvw = tc->coords.arc.uvw;
            break;
    }

    pmCartesianToEmcPose(&xyz, &abc, &uvw, pos);
    return 0;
}


/**
 * Set the terminal condition of a segment.
 * This function will eventually handle state changes associated with altering a terminal condition.
 */
int tcSetTermCond(TC_STRUCT * const tc, int term_cond) {
    tp_debug_print("setting term condition %d on tc id %d, type %d\n", term_cond, tc->id, tc->motion_type);
    tc->term_cond = term_cond;
    return 0;
}


/**
 * Connect a blend arc to the two line segments it blends.
 * Starting with two adjacent line segments, this function shortens each
 * segment to connect them with the newly created blend arc. The "previous"
 * segment gets a new end point, while the next segment gets a new start point.
 * After the operation is complete the result is a set of 3 connected segments
 * (line-arc-line).
 */
int tcConnectBlendArc(TC_STRUCT * const prev_tc, TC_STRUCT * const tc,
        PmCartesian const * const circ_start,
        PmCartesian const * const circ_end) {

    /* Only shift XYZ for now*/
    if (prev_tc) {
        tp_debug_print("connect: keep prev_tc\n");
        //Have prev line, need to shorten it
        pmCartLineInit(&prev_tc->coords.line.xyz,
                &prev_tc->coords.line.xyz.start, circ_start);
        tp_debug_print("Old target = %f\n", prev_tc->target);
        prev_tc->target = prev_tc->coords.line.xyz.tmag;
        tp_debug_print("Target = %f\n",prev_tc->target);
        //Setup tangent blending constraints
        tcSetTermCond(prev_tc, TC_TERM_COND_TANGENT);
        tp_debug_print(" L1 end  : %f %f %f\n",prev_tc->coords.line.xyz.end.x,
                prev_tc->coords.line.xyz.end.y,
                prev_tc->coords.line.xyz.end.z);
    } else {
        tp_debug_print("connect: consume prev_tc\n");
    }

    //Shorten next line
    pmCartLineInit(&tc->coords.line.xyz, circ_end, &tc->coords.line.xyz.end);

    tp_info_print(" L2: old target = %f\n", tc->target);
    tc->target = tc->coords.line.xyz.tmag;
    tp_info_print(" L2: new target = %f\n", tc->target);
    tp_debug_print(" L2 start  : %f %f %f\n",tc->coords.line.xyz.start.x,
            tc->coords.line.xyz.start.y,
            tc->coords.line.xyz.start.z);

    //Disable flag for parabolic blending with previous
    tc->blend_prev = 0;

    tp_info_print("       Q1: %f %f %f\n",circ_start->x,circ_start->y,circ_start->z);
    tp_info_print("       Q2: %f %f %f\n",circ_end->x,circ_end->y,circ_end->z);

    return 0;
}


/**
 * Check if the current segment is actively blending.
 * Checks if a blend should start based on acceleration and velocity criteria.
 * Also saves this status so that the blend continues until the segment is
 * done.
 */
int tcIsBlending(TC_STRUCT * const tc) {
    //FIXME Disabling blends for rigid tap cycle until changes can be verified.
    int is_blending_next = (tc->term_cond == TC_TERM_COND_PARABOLIC ) &&
        tc->on_final_decel && (tc->currentvel < tc->blend_vel) &&
        tc->motion_type != TC_RIGIDTAP;

    //Latch up the blending_next status here, so that even if the prev conditions
    //aren't necessarily true we still blend to completion once the blend
    //starts.
    tc->blending_next |= is_blending_next;

    return tc->blending_next;
}

int tcFindBlendTolerance(TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc, double * const T_blend, double * const nominal_tolerance)
{
    const double tolerance_ratio = 0.25;
    double T1 = prev_tc->tolerance;
    double T2 = tc->tolerance;
    //Detect zero tolerance = no tolerance and force to reasonable maximum
    if (T1 == 0) {
        /* calculate MAX arc radius based on request-velocity of prev_tc */
        T1 = prev_tc->reqvel * prev_tc->cycle_time * prev_tc->reqvel * prev_tc->cycle_time / prev_tc->maxaccel;
    }
    if (T2 == 0) {
        /* calculate MAX arc radius based on request-velocity of tc */
        T2 = tc->reqvel * tc->cycle_time * tc->reqvel * tc->cycle_time / tc->maxaccel;
    }
    *nominal_tolerance = rtapi_fmin(T1,T2);
    //Blend tolerance is the limit of what we can reach by blending alone,
    //consuming half a segment or less (parabolic equivalent)
    double blend_tolerance = rtapi_fmin(rtapi_fmin(*nominal_tolerance, 
                prev_tc->nominal_length * tolerance_ratio),
            tc->nominal_length * tolerance_ratio);
    *T_blend = blend_tolerance;
    return 0;
}


/**
 * Check for early stop conditions.
 * If a variety of conditions are true, then we can't do blending as we expect.
 * This function checks for any conditions that force us to stop on the current
 * segment. This is different from pausing or aborting, which can happen any
 * time.
 */
int tcFlagEarlyStop(TC_STRUCT * const tc,
        TC_STRUCT * const nexttc)
{

    if (!tc || !nexttc) {
        return TP_ERR_NO_ACTION;
    }

    if(tc->synchronized != TC_SYNC_POSITION && nexttc->synchronized == TC_SYNC_POSITION) {
        // we'll have to wait for spindle sync; might as well
        // stop at the right place (don't blend)
        tp_debug_print("waiting on spindle sync for tc %d\n", tc->id);
        tcSetTermCond(tc, TC_TERM_COND_STOP);
    }

    if(nexttc->atspeed) {
        // we'll have to wait for the spindle to be at-speed; might as well
        // stop at the right place (don't blend), like above
        // FIXME change the values so that 0 is exact stop mode
        tp_debug_print("waiting on spindle atspeed for tc %d\n", tc->id);
        tcSetTermCond(tc, TC_TERM_COND_STOP);
    }

    return TP_ERR_OK;
}

double pmLine9Target(PmLine9 * const line9)
{
    if (!line9->xyz.tmag_zero) {
        return line9->xyz.tmag;
    } else if (!line9->uvw.tmag_zero) {
        return line9->uvw.tmag;
    } else if (!line9->abc.tmag_zero) {
        return line9->abc.tmag;
    } else {
        rtapi_print_msg(RTAPI_MSG_DBG,
                "line can't have zero length! "
                "xyz start = %.12e,%.12e,%.12e, end = %.12e,%.12e,%.12e\n",
                line9->xyz.start.x,
                line9->xyz.start.y,
                line9->xyz.start.z,
                line9->xyz.end.x,
                line9->xyz.end.y,
                line9->xyz.end.z);
        //FIXME yet it does return zero...
        return 0;
    }
}


/**
 * Initialize a new trajectory segment with common parameters.
 *
 * NOTE: this function only sets default values that are non-zero. Make sure
 * the struct is properly initialized BEFORE calling this function.
 */
int tcInit(TC_STRUCT * const tc,
        int motion_type,
        int canon_motion_type,
        double cycle_time,
        unsigned char enables,
        char atspeed)
{

    /** Motion type setup */
    tc->motion_type = motion_type;
    tc->canon_motion_type = canon_motion_type;
    tc->atspeed = atspeed;

    /** Segment settings passed down from interpreter*/
    tc->enables = enables;
    tc->cycle_time = cycle_time;

    tc->id = -1; //ID to be set when added to queue (may change before due to blend arcs)

    /** Segment settings (given values later during setup / optimization) */
    tc->indexrotary = -1;

    tc->active_depth = 1;

    tc->lookahead_target = 0.0;
    tc->progress = 0.0;
    tc->accel_state = ACCEL_S3;
    tc->feed_override = 0.0;
    tc->cur_accel = 0.0;
    tc->currentvel = 0.0;
    return TP_ERR_OK;
}


/**
 * Set kinematic properties for a trajectory segment.
 */
int tcSetupMotion(TC_STRUCT * const tc,
        double vel,
        double ini_maxvel,
        double acc,
        double ini_maxjerk,
        double cycle_time)
{

    if (ini_maxjerk == 0) {
        rtapi_print_msg(RTAPI_MSG_WARN, "jerk is not provided or jerk is 0\n");
        ini_maxjerk = 1e99; //!< force JERK to unlimited
    }

    tc->jerk = ini_maxjerk * cycle_time * cycle_time * cycle_time;      // unit: dt^3
    // FIXME: ini_maxjerk suddenly becomes abnormal
    if (tc->jerk > 1){
        assert(0);
        ini_maxjerk = 10000;
        tc->jerk = ini_maxjerk * cycle_time * cycle_time * cycle_time;
    }

    tc->maxaccel = acc * cycle_time * cycle_time;                       // unit: dt^2

    tc->maxvel = ini_maxvel * cycle_time;                               // unit: dt

    tc->reqvel = vel;                                                   // unit: sec
    // Initial guess at target velocity is just the requested velocity
    tc->target_vel = vel;                                               // unit: sec

    return TP_ERR_OK;
}


int tcSetupState(TC_STRUCT * const tc, TP_STRUCT const * const tp)
{
    tcSetTermCond(tc, tp->termCond);
    tc->tolerance = tp->tolerance;
    tc->synchronized = tp->synchronized;
    tc->uu_per_rev = tp->uu_per_rev;
    return TP_ERR_OK;
}

int pmLine9Init(PmLine9 * const line9,
        EmcPose const * const start,
        EmcPose const * const end)
{
    // Scratch variables
    PmCartesian start_xyz, end_xyz;
    PmCartesian start_uvw, end_uvw;
    PmCartesian start_abc, end_abc;

    // Convert endpoint to cartesian representation
    emcPoseToPmCartesian(start, &start_xyz, &start_abc, &start_uvw);
    emcPoseToPmCartesian(end, &end_xyz, &end_abc, &end_uvw);

    // Initialize cartesian line members
    int xyz_fail = pmCartLineInit(&line9->xyz, &start_xyz, &end_xyz);
    int abc_fail = pmCartLineInit(&line9->abc, &start_abc, &end_abc);
    int uvw_fail = pmCartLineInit(&line9->uvw, &start_uvw, &end_uvw);

    if (xyz_fail || abc_fail || uvw_fail) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Failed to initialize Line9, err codes %d, %d, %d\n",
                xyz_fail,abc_fail,uvw_fail);
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

int pmCircle9Init(PmCircle9 * const circ9,
        EmcPose const * const start,
        EmcPose const * const end,
        PmCartesian const * const center,
        PmCartesian const * const normal,
        int turn)
{
    PmCartesian start_xyz, end_xyz;
    PmCartesian start_uvw, end_uvw;
    PmCartesian start_abc, end_abc;

    emcPoseToPmCartesian(start, &start_xyz, &start_abc, &start_uvw);
    emcPoseToPmCartesian(end, &end_xyz, &end_abc, &end_uvw);

    int xyz_fail = pmCircleInit(&circ9->xyz, &start_xyz, &end_xyz, center, normal, turn);
    //Initialize line parts of Circle9
    int abc_fail = pmCartLineInit(&circ9->abc, &start_abc, &end_abc);
    int uvw_fail = pmCartLineInit(&circ9->uvw, &start_uvw, &end_uvw);

    int res_fit = findSpiralArcLengthFit(&circ9->xyz,&circ9->fit);

    if (xyz_fail || abc_fail || uvw_fail || res_fit) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Failed to initialize Circle9, err codes %d, %d, %d, %d\n",
                xyz_fail, abc_fail, uvw_fail, res_fit);
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

double pmCircle9Target(PmCircle9 const * const circ9)
{

    double h2;
    pmCartMagSq(&circ9->xyz.rHelix, &h2);
    double helical_length = pmSqrt(pmSq(circ9->fit.total_planar_length) + h2);

    return helical_length;
}

/**
 * "Finalizes" a segment so that its length can't change.
 * By setting the finalized flag, we tell the optimizer that this segment's
 * length won't change anymore. Since any blends are already set up, we can
 * trust that the length will be the same, and so can use the length in the
 * velocity optimization.
 */
int tcFinalizeLength(TC_STRUCT * const tc)
{
    //Apply velocity corrections
    if (!tc) {
        tp_debug_print("Missing prev_tc in finalize!\n");
        return TP_ERR_FAIL;
    }

    if (tc->finalized) {
        tp_debug_print("tc %d already finalized\n", tc->id);
        return TP_ERR_NO_ACTION;
    }

    tp_debug_print("Finalizing tc id %d, type %d\n", tc->id, tc->motion_type);
    //TODO function to check for parabolic?
    int parabolic = (tc->blend_prev || tc->term_cond == TC_TERM_COND_PARABOLIC);
    tp_debug_print("blend_prev = %d, term_cond = %d\n",tc->blend_prev, tc->term_cond);

    if (tc->motion_type == TC_CIRCULAR) {
        tc->maxvel = pmCircleActualMaxVel(&tc->coords.circle.xyz, tc->maxvel, tc->maxaccel, parabolic);
    }
    tc->finalized = 1;
    return TP_ERR_OK;
}

/**
 * compute the total arc length of a circle segment
 */
int tcUpdateTargetFromCircle(TC_STRUCT * const tc)
{
    if (!tc || tc->motion_type !=TC_CIRCULAR) {
        return TP_ERR_FAIL;
    }

    double h2;
    pmCartMagSq(&tc->coords.circle.xyz.rHelix, &h2);
    double helical_length = pmSqrt(pmSq(tc->coords.circle.fit.total_planar_length) + h2);

    tc->target = helical_length;
    return TP_ERR_OK;
}



int pmRigidTapInit(PmRigidTap * const tap,
        EmcPose const * const start,
        EmcPose const * const end)
{
    PmCartesian start_xyz, end_xyz;
    PmCartesian abc, uvw;

    //Slightly more allocation this way, but much easier to read
    emcPoseToPmCartesian(start, &start_xyz, &abc, &uvw);
    emcPoseGetXYZ(end, &end_xyz);

    // Setup XYZ motion
    pmCartLineInit(&tap->xyz, &start_xyz, &end_xyz);

    // Copy over fixed ABC and UVW points
    tap->abc = abc;
    tap->uvw = uvw;

    // Setup initial tap state
    tap->reversal_target = tap->xyz.tmag;
    tap->state = TAPPING;
    return TP_ERR_OK;

}

double pmRigidTapTarget(PmRigidTap * const tap, double uu_per_rev)
{
    // allow 10 turns of the spindle to stop - we don't want to just go on forever
    double overrun = 10. * uu_per_rev;
    double target = tap->xyz.tmag + overrun;
    tp_debug_print("initial tmag = %.12g, added %.12g for overrun, target = %.12g\n",
            tap->xyz.tmag, overrun,target);
    return target;
}

/** Returns true if segment has ONLY rotary motion, false otherwise. */
int tcPureRotaryCheck(TC_STRUCT const * const tc)
{
    return (tc->motion_type == TC_LINEAR) &&
        (tc->coords.line.xyz.tmag_zero) &&
        (tc->coords.line.uvw.tmag_zero);
}


/**
 * Given a PmCircle and a circular segment, copy the circle in as the XYZ portion of the segment, then update the motion parameters.
 * NOTE: does not yet support ABC or UVW motion!
 */
int tcSetCircleXYZ(TC_STRUCT * const tc, PmCircle const * const circ)
{

    //Update targets with new arc length
    if (!circ || tc->motion_type != TC_CIRCULAR) {
        return TP_ERR_FAIL;
    }
    if (!tc->coords.circle.abc.tmag_zero || !tc->coords.circle.uvw.tmag_zero) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SetCircleXYZ does not supportABC or UVW motion\n");
        return TP_ERR_FAIL;
    }

    // Store the new circular segment (or use the current one)

    if (!circ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SetCircleXYZ missing new circle definition\n");
        return TP_ERR_FAIL;
    }

    tc->coords.circle.xyz = *circ;
    // Update the arc length fit to this new segment
    findSpiralArcLengthFit(&tc->coords.circle.xyz, &tc->coords.circle.fit);

    // compute the new total arc length using the fit and store as new
    // target distance
    tc->target = pmCircle9Target(&tc->coords.circle);

    return TP_ERR_OK;
}

int tcClearFlags(TC_STRUCT * const tc)
{
    if (!tc) {
        return TP_ERR_MISSING_INPUT;
    }

    //KLUDGE this will need to be updated manually if any other flags are added.
    tc->is_blending = false;

    return TP_ERR_OK;
}


