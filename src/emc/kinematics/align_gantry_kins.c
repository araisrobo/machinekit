/********************************************************************
* Description: align_gantry_kins.c
*   Simple example kinematics for thita alignment in software
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: Yishin Li, ARAIS ROBOT TECHNOLOGY
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2011 All rights reserved.
*
********************************************************************/

#include "kinematics.h"         /* these decls */
#include "rtapi.h"              /* RTAPI realtime OS API */
#include "rtapi_app.h"          /* RTAPI realtime module decls */
#include "hal.h"

#define VTVERSION VTKINEMATICS_VERSION1

typedef struct {
    hal_float_t *yy_offset;
} align_pins_t;

static align_pins_t *align_pins;

#define YY_OFFSET       (*(align_pins->yy_offset))

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{

    pos->tran.x = joints[0];
    pos->tran.y = joints[1];
    pos->tran.z = joints[3];
    pos->a = joints[4];
    pos->b = joints[5];

    // DP("kFWD: x(%f), y(%f), j0(%f), j1(%f), j2(%f), yy_offset(%f)\n",
    //     pos->tran.x, pos->tran.y, joints[0], joints[1], joints[2], YY_OFFSET);
    // DP("kFWD: s(%f), j5(%f)\n", pos->s, joints[5]);

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    joints[0] = pos->tran.x;
    joints[1] = pos->tran.y;
    joints[2] = pos->tran.y - YY_OFFSET;  // YY
    joints[3] = pos->tran.z;
    joints[4] = pos->a;
    joints[5] = pos->b;

    // DP("kINV: x(%f), y(%f), j0(%f), j1(%f), j2(%f), yy_offset(%f)\n",
    //    pos->tran.x, pos->tran.y, joints[0], joints[1], joints[2], YY_OFFSET);
    // DP("kINV: s(%f), j5(%f)\n", pos->s, joints[5]);

    return 0;
}

/* implemented for these kinematics as giving joints preference */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

MODULE_LICENSE("GPL");

static vtkins_t vtk = {
    .kinematicsForward = kinematicsForward,
    .kinematicsInverse  = kinematicsInverse,
    // .kinematicsHome = kinematicsHome,
    .kinematicsType = kinematicsType
};

static int comp_id, vtable_id;
static const char *name = "align_gantry_kins";

int rtapi_app_main(void) 
{
    comp_id = hal_init(name);
    if(comp_id > 0) {
        vtable_id = hal_export_vtable(name, VTVERSION, &vtk, comp_id);

        if (vtable_id < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "%s: ERROR: hal_export_vtable(%s,%d,%p) failed: %d\n",
                            name, name, VTVERSION, &vtk, vtable_id );
            return -ENOENT;
        }

        align_pins = hal_malloc(sizeof(align_pins_t));
        if (!align_pins) goto error;
        if ((hal_pin_float_new("align-gantry-kins.yy-offset", HAL_IN, &(align_pins->yy_offset), comp_id)) < 0) goto error;
        YY_OFFSET = 0;

        hal_ready(comp_id);
        return 0;

error:
        hal_exit(comp_id);
    }
    return comp_id;
    
}

void rtapi_app_exit(void)
{
    hal_remove_vtable(vtable_id);
    hal_exit(comp_id);
}
