/*****************************************************************
 * Description: ra605kins.c
 *   Kinematics for puma typed robots
 *   Set the params using HAL to fit your robot
 *
 *   Derived from a work by Fred Proctor
 *
 * Author:
 * License: GPL Version 2
 * System: Linux
 *
 * Copyright (c) 2004 All rights reserved.
 *
 * Last change:
 *******************************************************************
 */

#include "rtapi_math.h"
#include "gotypes.h"            /* go_result, go_integer */
#include "gomath.h"             /* go_pose */
#include "genserkins.h"         /* these decls */
#include "kinematics.h"

#ifdef RTAPI
#include "rtapi.h"
#include "rtapi_app.h"
#endif

#define VTVERSION VTKINEMATICS_VERSION1

#include "hal.h"
struct haldata {
    hal_float_t *a[GENSER_MAX_JOINTS];
    hal_float_t *alpha[GENSER_MAX_JOINTS];
    hal_float_t *d[GENSER_MAX_JOINTS];
    genser_struct *kins;
    go_pose *pos;               // used in various functions, we malloc it
                                // only once in rtapi_app_main
} *haldata = 0;

double j[GENSER_MAX_JOINTS];

#define A(i) (*(haldata->a[i]))
#define ALPHA(i) (*(haldata->alpha[i]))
#define D(i) (*(haldata->d[i]))

#define KINS_PTR (haldata->kins)

#if GENSER_MAX_JOINTS < 6
#error GENSER_MAX_JOINTS must be at least 6; fix genserkins.h
#endif

enum { GENSER_DEFAULT_MAX_ITERATIONS = 100 };

//MODULE_LICENSE("GPL");

int kinematicsForward(const double * joint,
        EmcPose * world,
        const KINEMATICS_FORWARD_FLAGS * fflags,
        KINEMATICS_INVERSE_FLAGS * iflags)
{

    double s1, s2, s3, s4, s5, s6;
    double c1, c2, c3, c4, c5, c6;
    double s23;
    double c23;
    double t1, t2, t3, t4, t5;
//    double sumSq, k;
    PmHomogeneous hom;
    PmPose worldPose;
    PmRpy rpy;

    /* Calculate sin of joints for future use */
    s1 = rtapi_sin(joint[0]*PM_PI/180);
    s2 = rtapi_sin(joint[1]*PM_PI/180);
    s3 = rtapi_sin(joint[2]*PM_PI/180);
    s4 = rtapi_sin(joint[3]*PM_PI/180);
    s5 = rtapi_sin(joint[4]*PM_PI/180);
    s6 = rtapi_sin(joint[5]*PM_PI/180);

    /* Calculate cos of joints for future use */
    c1 = rtapi_cos(joint[0]*PM_PI/180);
    c2 = rtapi_cos(joint[1]*PM_PI/180);
    c3 = rtapi_cos(joint[2]*PM_PI/180);
    c4 = rtapi_cos(joint[3]*PM_PI/180);
    c5 = rtapi_cos(joint[4]*PM_PI/180);
    c6 = rtapi_cos(joint[5]*PM_PI/180);

    s23 = c2 * s3 + s2 * c3;
    c23 = c2 * c3 - s2 * s3;

    /* Calculate terms to be used in definition of... */
    /* first column of rotation matrix.               */
    t1 = c4 * c5 * c6 - s4 * s6;
    t2 = s23 * s5 * c6;
    t3 = -s4 * c5 * c6 - c4 * s6;
    t4 = c23 * t1 - t2;
    t5 = c23 * s5 * c6;

    /* Define first column of rotation matrix */
    hom.rot.x.x = c1 * t4 - s1 * t3;
    hom.rot.x.y = s1 * t4 + c1 * t3;
    hom.rot.x.z = -s23 * t1 - t5;

    /* Calculate terms to be used in definition of...  */
    /* second column of rotation matrix.               */
    t1 = -c4 * c5 * s6 - s4 * c6;
    t2 = s23 * s5 * s6;
    t3 = s4 * c5 * s6 - c4 * c6;
    t4 = c23 * t1 + t2;
    t5 = c23 * s5 * s6;

    /* Define second column of rotation matrix */
    hom.rot.y.x = c1 * t4 - s1 * t3;
    hom.rot.y.y = s1 * t4 + c1 * t3;
    hom.rot.y.z = -s23 * t1 + t5;

    /* Calculate term to be used in definition of... */
    /* third column of rotation matrix.              */
    t1 = c23 * c4 * s5 + s23 * c5;
    t2 = s4 * s5;

    /* Define third column of rotation matrix */
    hom.rot.z.x = -c1 * t1 - s1 * t2;
    hom.rot.z.y = -s1 * t1 + c1 * t2;
    hom.rot.z.z = s23 * c4 * s5 - c23 * c5;

    /* Calculate term to be used in definition of...  */
    /* position vector.                               */

    // t2 = s4 * s5; (previous defined)
    t4 = A(3) - c4 * D(5) * s5;
    t3 = c5 * D(5) + D(3);
    t1 = c23 * t4 - s23 * t3 + A(2) * c2 + A(1);

    /* Define position vector */
    hom.tran.x = c1 * t1 - D(5) * s1 * t2;
    hom.tran.y = s1 * t1 + D(5) * c1 * t2;
    hom.tran.z = -s23 * t4 - A(2) * s2 - c23 * t3;

    /* Calculate terms to be used to...   */
    /* determine flags.                   */
    // TODO:...
    //   sumSq = hom.tran.x * hom.tran.x + hom.tran.y * hom.tran.y -
    //           PUMA_D3 * PUMA_D3;
    //   k = (sumSq + hom.tran.z * hom.tran.z - PUMA_A2 * PUMA_A2 -
    //       PUMA_A3 * PUMA_A3 - PUMA_D4 * PUMA_D4) /
    //       (2.0 * PUMA_A2);
    //
    //   /* reset flags */
    //   *iflags = 0;
    //
    //   /* Set shoulder-up flag if necessary */
    //   if (rtapi_fabs(joint[0]*PM_PI/180 - rtapi_atan2(hom.tran.y, hom.tran.x) +
    //       rtapi_atan2(PUMA_D3, -rtapi_sqrt(sumSq))) < FLAG_FUZZ)
    //   {
    //     *iflags |= PUMA_SHOULDER_RIGHT;
    //   }
    //
    //   /* Set elbow down flag if necessary */
    //   if (rtapi_fabs(joint[2]*PM_PI/180 - rtapi_atan2(PUMA_A3, PUMA_D4) +
    //       rtapi_atan2(k, -rtapi_sqrt(PUMA_A3 * PUMA_A3 +
    //       PUMA_D4 * PUMA_D4 - k * k))) < FLAG_FUZZ)
    //   {
    //      *iflags |= PUMA_ELBOW_DOWN;
    //   }

//    /* set singular flag if necessary */
//    t1 = -hom.rot.z.x * s1 + hom.rot.z.y * c1;
//    t2 = -hom.rot.z.x * c1 * c23 - hom.rot.z.y * s1 * c23 +
//            hom.rot.z.z * s23;
//    if (rtapi_fabs(t1) < SINGULAR_FUZZ && rtapi_fabs(t2) < SINGULAR_FUZZ)
//    {
//        *iflags |= PUMA_SINGULAR;
//    }
//
//    /* if not singular set wrist flip flag if necessary */
//    else{
//        if (! (rtapi_fabs(joint[3]*PM_PI/180 - rtapi_atan2(t1, t2)) < FLAG_FUZZ))
//        {
//            *iflags |= PUMA_WRIST_FLIP;
//        }
//    }

    /* convert hom.rot to world->quat */

//    hom.rot.x.x = rtapi_rint(hom.rot.x.x * 1000000.0) * 0.000001;
//    hom.rot.x.y = rtapi_rint(hom.rot.x.y * 1000000.0) * 0.000001;
//    hom.rot.x.z = rtapi_rint(hom.rot.x.x * 1000000.0) * 0.000001;
//    hom.rot.y.x = rtapi_rint(hom.rot.y.x * 1000000.0) * 0.000001;
//    hom.rot.y.y = rtapi_rint(hom.rot.y.y * 1000000.0) * 0.000001;
//    hom.rot.y.z = rtapi_rint(hom.rot.y.x * 1000000.0) * 0.000001;
//    hom.rot.z.x = rtapi_rint(hom.rot.z.x * 1000000.0) * 0.000001;
//    hom.rot.z.y = rtapi_rint(hom.rot.z.y * 1000000.0) * 0.000001;
//    hom.rot.z.z = rtapi_rint(hom.rot.z.x * 1000000.0) * 0.000001;

    pmHomPoseConvert(&hom, &worldPose);
    pmQuatRpyConvert(&worldPose.rot,&rpy);
    world->tran = worldPose.tran;
    world->a = rpy.r * 180.0/PM_PI;
    world->b = rpy.p * 180.0/PM_PI;
    world->c = rpy.y * 180.0/PM_PI;

    /* return 0 and exit */
    return 0;
}

static int compute_jfwd(go_link * link_params,
                        int link_number,
                        go_matrix * Jfwd,
                        go_pose * T_L_0)
{
    GO_MATRIX_DECLARE(Jv, Jvstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jw, Jwstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(R_i_ip1, R_i_ip1stg, 3, 3);
    GO_MATRIX_DECLARE(scratch, scratchstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(R_inv, R_invstg, 3, 3);
    go_pose pose;
    go_quat quat;
    go_vector P_ip1_i[3];
    int row, col;

    /* init matrices to possibly smaller size */
    go_matrix_init(Jv, Jvstg, 3, link_number);
    go_matrix_init(Jw, Jwstg, 3, link_number);
    go_matrix_init(R_i_ip1, R_i_ip1stg, 3, 3);
    go_matrix_init(scratch, scratchstg, 3, link_number);
    go_matrix_init(R_inv, R_invstg, 3, 3);

    Jv.el[0][0] = 0, Jv.el[1][0] = 0, Jv.el[2][0] = (GO_QUANTITY_LENGTH == link_params[0].quantity ? 1 : 0);
    Jw.el[0][0] = 0, Jw.el[1][0] = 0, Jw.el[2][0] = (GO_QUANTITY_ANGLE == link_params[0].quantity ? 1 : 0);

    /* initialize inverse rotational transform */
    if (GO_LINK_DH == link_params[0].type) {
        go_dh_pose_convert(&link_params[0].u.dh, &pose);
    } else if (GO_LINK_PP == link_params[0].type) {
        pose = link_params[0].u.pp.pose;
    } else {
        return GO_RESULT_IMPL_ERROR;
    }

    *T_L_0 = pose;

    for (col = 1; col < link_number; col++) {
        /* T_ip1_i */
        if (GO_LINK_DH == link_params[col].type) {
            go_dh_pose_convert(&link_params[col].u.dh, &pose);
        } else if (GO_LINK_PP == link_params[col].type) {
            pose = link_params[col].u.pp.pose;
        } else {
            return GO_RESULT_IMPL_ERROR;
        }

        go_cart_vector_convert(&pose.tran, P_ip1_i);
        go_quat_inv(&pose.rot, &quat);
        go_quat_matrix_convert(&quat, &R_i_ip1);

        /* Jv */
        go_matrix_vector_cross(&Jw, P_ip1_i, &scratch);
        go_matrix_matrix_add(&Jv, &scratch, &scratch);
        go_matrix_matrix_mult(&R_i_ip1, &scratch, &Jv);
        Jv.el[0][col] = 0, Jv.el[1][col] = 0, Jv.el[2][col] = (GO_QUANTITY_LENGTH == link_params[col].quantity ? 1 : 0);
        /* Jw */
        go_matrix_matrix_mult(&R_i_ip1, &Jw, &Jw);
        Jw.el[0][col] = 0, Jw.el[1][col] = 0, Jw.el[2][col] = (GO_QUANTITY_ANGLE == link_params[col].quantity ? 1 : 0);
        if (GO_LINK_DH == link_params[col].type) {
            go_dh_pose_convert(&link_params[col].u.dh, &pose);
        } else if (GO_LINK_PP == link_params[col].type) {
            pose = link_params[col].u.pp.pose;
        } else {
            return GO_RESULT_IMPL_ERROR;
        }
        go_pose_pose_mult(T_L_0, &pose, T_L_0);
    }

    /* rotate back into {0} frame */
    go_quat_matrix_convert(&T_L_0->rot, &R_inv);
    go_matrix_matrix_mult(&R_inv, &Jv, &Jv);
    go_matrix_matrix_mult(&R_inv, &Jw, &Jw);

    /* put Jv atop Jw in J */
    for (row = 0; row < 6; row++) {
        for (col = 0; col < link_number; col++) {
            if (row < 3) {
                Jfwd->el[row][col] = Jv.el[row][col];
            } else {
                Jfwd->el[row][col] = Jw.el[row - 3][col];
            }
        }
    }

    return GO_RESULT_OK;
}

/* compute the inverse of the jacobian matrix */
static int compute_jinv(go_matrix * Jfwd, go_matrix * Jinv)
{
    int retval;
    GO_MATRIX_DECLARE(JT, JTstg, GENSER_MAX_JOINTS, 6);

    /* compute inverse, or pseudo-inverse */
    if (Jfwd->rows == Jfwd->cols) {
        retval = go_matrix_inv(Jfwd, Jinv);
        if (GO_RESULT_OK != retval)
            return retval;
    } else if (Jfwd->rows < Jfwd->cols) {
        /* underdetermined, optimize on smallest sum of square of speeds */
        /* JT(JJT)inv */
        GO_MATRIX_DECLARE(JJT, JJTstg, 6, 6);

        go_matrix_init(JT, JTstg, Jfwd->cols, Jfwd->rows);
        go_matrix_init(JJT, JJTstg, Jfwd->rows, Jfwd->rows);
        go_matrix_transpose(Jfwd, &JT);
        go_matrix_matrix_mult(Jfwd, &JT, &JJT);
        retval = go_matrix_inv(&JJT, &JJT);
        if (GO_RESULT_OK != retval)
            return retval;
        go_matrix_matrix_mult(&JT, &JJT, Jinv);
    } else {
        /* overdetermined, do least-squares best fit */
        /* (JTJ)invJT */
        GO_MATRIX_DECLARE(JTJ, JTJstg, GENSER_MAX_JOINTS, GENSER_MAX_JOINTS);

        go_matrix_init(JT, JTstg, Jfwd->cols, Jfwd->rows);
        go_matrix_init(JTJ, JTJstg, Jfwd->cols, Jfwd->cols);
        go_matrix_transpose(Jfwd, &JT);
        go_matrix_matrix_mult(&JT, Jfwd, &JTJ);
        retval = go_matrix_inv(&JTJ, &JTJ);
        if (GO_RESULT_OK != retval)
            return retval;
        go_matrix_matrix_mult(&JTJ, &JT, Jinv);
    }

    return GO_RESULT_OK;
}

int genser_kin_init(void) {
    genser_struct *genser = KINS_PTR;
    int t;

    /* init them all and make them revolute joints */
    /* FIXME: should allow LINEAR joints based on HAL param too */
    for (t = 0; t < GENSER_MAX_JOINTS; t++) {
        genser->links[t].u.dh.a = A(t);
        genser->links[t].u.dh.alpha = ALPHA(t);
        genser->links[t].u.dh.d = D(t);
        genser->links[t].u.dh.theta = 0;
        genser->links[t].type = GO_LINK_DH;
        genser->links[t].quantity = GO_QUANTITY_ANGLE;
    }

    /* set a select few to make it PUMA-like */
    // FIXME-AJ: make a hal pin, also set number of joints based on it
    genser->link_num = 6;

    return GO_RESULT_OK;
}

int genser_kin_fwd(void *kins, const go_real * joints, go_pose * pos)
{
    genser_struct *genser = kins;
    go_link linkout[GENSER_MAX_JOINTS];

    int link;
    int retval;

    genser_kin_init();

    for (link = 0; link < genser->link_num; link++) {
        retval = go_link_joint_set(&genser->links[link], joints[link], &linkout[link]);
        if (GO_RESULT_OK != retval)
            return retval;
    }

    retval = go_link_pose_build(linkout, genser->link_num, pos);
    if (GO_RESULT_OK != retval)
        return retval;

    return GO_RESULT_OK;
}

int kinematicsInverse(const EmcPose * world,
                      double *joints,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
    genser_struct *genser = KINS_PTR;
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jinv, Jinv_stg, GENSER_MAX_JOINTS, 6);
    go_pose T_L_0;
    go_real dvw[6];
    go_real jest[GENSER_MAX_JOINTS];
    go_real dj[GENSER_MAX_JOINTS];
    go_pose pest, pestinv, Tdelta;      // pos = converted pose from EmcPose
    go_rpy rpy;
    go_rvec rvec;
    go_cart cart;
    go_link linkout[GENSER_MAX_JOINTS];
    int link;
    int smalls;
    int retval;

    //    rtapi_print("kineInverse(joints: %f %f %f %f %f %f)\n", joints[0],joints[1],joints[2],joints[3],joints[4],joints[5]);
    //    rtapi_print("kineInverse(world: %f %f %f %f %f %f)\n", world->tran.x, world->tran.y, world->tran.z, world->a, world->b, world->c);

    //    genser_kin_init();

    // FIXME-AJ: rpy or zyx ?
    rpy.y = world->c * PM_PI / 180;
    rpy.p = world->b * PM_PI / 180;
    rpy.r = world->a * PM_PI / 180;

    go_rpy_quat_convert(&rpy, &haldata->pos->rot);
    haldata->pos->tran.x = world->tran.x;
    haldata->pos->tran.y = world->tran.y;
    haldata->pos->tran.z = world->tran.z;

    go_matrix_init(Jfwd, Jfwd_stg, 6, genser->link_num);
    go_matrix_init(Jinv, Jinv_stg, genser->link_num, 6);

    /* jest[] is a copy of joints[], which is the joint estimate */
    for (link = 0; link < genser->link_num; link++) {
        // jest, and the rest of joint related calcs are in radians
        jest[link] = joints[link] * (PM_PI / 180);
    }

    for (genser->iterations = 0; genser->iterations < genser->max_iterations; genser->iterations++) {
        /* update the Jacobians */
        for (link = 0; link < genser->link_num; link++) {
            go_link_joint_set(&genser->links[link], jest[link], &linkout[link]);
        }
        retval = compute_jfwd(linkout, genser->link_num, &Jfwd, &T_L_0);
        if (GO_RESULT_OK != retval) {
            rtapi_print("ERR kI - compute_jfwd (joints: %f %f %f %f %f %f), (iterations=%d)\n", joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], genser->iterations);
            return retval;
        }
        retval = compute_jinv(&Jfwd, &Jinv);
        if (GO_RESULT_OK != retval) {
            rtapi_print("ERR kI - compute_jinv (joints: %f %f %f %f %f %f), (iterations=%d)\n", joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], genser->iterations);
            return retval;
        }

        /* pest is the resulting pose estimate given joint estimate */
        genser_kin_fwd(KINS_PTR, jest, &pest);
        //      printf("jest: %f %f %f %f %f %f\n",jest[0],jest[1],jest[2],jest[3],jest[4],jest[5]);
        /* pestinv is its inverse */
        go_pose_inv(&pest, &pestinv);
        /*
            Tdelta is the incremental pose from pest to pos, such that

            0        L         0
            . pest *  Tdelta =  pos, or
            L        L         L

            L         L          0
            .Tdelta =  pestinv *  pos
            L         0          L
         */
        go_pose_pose_mult(&pestinv, haldata->pos, &Tdelta);

        /*
            We need Tdelta in 0 frame, not pest frame, so rotate it
            back. Since it's effectively a velocity, we just rotate it, and
            don't translate it.
         */

        /* first rotate the translation differential */
        go_quat_cart_mult(&pest.rot, &Tdelta.tran, &cart);
        dvw[0] = cart.x;
        dvw[1] = cart.y;
        dvw[2] = cart.z;

        /* to rotate the rotation differential, convert it to a
            velocity screw and rotate that */
        go_quat_rvec_convert(&Tdelta.rot, &rvec);
        cart.x = rvec.x;
        cart.y = rvec.y;
        cart.z = rvec.z;
        go_quat_cart_mult(&pest.rot, &cart, &cart);
        dvw[3] = cart.x;
        dvw[4] = cart.y;
        dvw[5] = cart.z;

        /* push the Cartesian velocity vector through the inverse Jacobian */
        go_matrix_vector_mult(&Jinv, dvw, dj);

        /* check for small joint increments, if so we're done */
        for (link = 0, smalls = 0; link < genser->link_num; link++) {
            if (GO_QUANTITY_LENGTH == linkout[link].quantity) {
                if (GO_TRAN_SMALL(dj[link]))
                    smalls++;
            } else {
                if (GO_ROT_SMALL(dj[link]))
                    smalls++;
            }
        }
        if (smalls == genser->link_num) {
            /* converged, copy jest[] out */
            for (link = 0; link < genser->link_num; link++) {
                // convert from radians back to angles
                joints[link] = jest[link] * 180 / PM_PI;
            }
            //          rtapi_print("DONEkineInverse(joints: %f %f %f %f %f %f), (iterations=%d)\n", joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], genser->iterations);
            return GO_RESULT_OK;
        }
        /* else keep iterating */
        for (link = 0; link < genser->link_num; link++) {
            jest[link] += dj[link]; //still in radians
        }
    }                           /* for (iterations) */

    rtapi_print("ERRkineInverse(joints: %f %f %f %f %f %f), (iterations=%d)\n", joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], genser->iterations);
    return GO_RESULT_ERROR;
}

int kinematicsHome(EmcPose * world,
        double * joint,
        KINEMATICS_FORWARD_FLAGS * fflags,
        KINEMATICS_INVERSE_FLAGS * iflags)
{
    /* use joints, set world */
    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType(void)
{
    //  return KINEMATICS_FORWARD_ONLY;
    return KINEMATICS_BOTH;
}

#ifdef RTAPI

static vtkins_t vtk = {
        .kinematicsForward = kinematicsForward,
        .kinematicsInverse  = kinematicsInverse,
        // .kinematicsHome = kinematicsHome,
        .kinematicsType = kinematicsType
};

static int comp_id, vtable_id;
static const char *name = "ra605kins";

int rtapi_app_main(void) {
    int res=0;
    int i;

    comp_id = hal_init(name);
    if (comp_id < 0) return comp_id;

    vtable_id = hal_export_vtable(name, VTVERSION, &vtk, comp_id);
    if (vtable_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "%s: ERROR: hal_export_vtable(%s,%d,%p) failed: %d\n",
                name, name, VTVERSION, &vtk, vtable_id );
        return -ENOENT;
    }

    haldata = hal_malloc(sizeof(struct haldata));
    if (!haldata) goto error;

//    if((res = hal_pin_float_new("ra605kins.A1", HAL_IO, &(haldata->a1), comp_id)) < 0) goto error;
//    if((res = hal_pin_float_new("ra605kins.A2", HAL_IO, &(haldata->a2), comp_id)) < 0) goto error;
//    if((res = hal_pin_float_new("ra605kins.A3", HAL_IO, &(haldata->a3), comp_id)) < 0) goto error;
//    if((res = hal_pin_float_new("ra605kins.D4", HAL_IO, &(haldata->d4), comp_id)) < 0) goto error;
//    if((res = hal_pin_float_new("ra605kins.D6", HAL_IO, &(haldata->d6), comp_id)) < 0) goto error;
    for (i = 0; i < GENSER_MAX_JOINTS; i++) {
        if ((res =
                hal_pin_float_newf(HAL_IO, &(haldata->a[i]), comp_id,
                    "ra605kins.A-%d", i)) < 0)
            goto error;
        *(haldata->a[i])=0;
        if ((res =
                hal_pin_float_newf(HAL_IO, &(haldata->alpha[i]), comp_id,
                    "ra605kins.ALPHA-%d", i)) < 0)
            goto error;
        *(haldata->alpha[i])=0;
        if ((res =
                hal_pin_float_newf(HAL_IO, &(haldata->d[i]), comp_id,
                    "ra605kins.D-%d", i)) < 0)
            goto error;
        *(haldata->d[i])=0;
    }
//    printf ("TODO:... ra605kins.c\n");
//    PUMA_A1 = DEFAULT_PUMA560_A2; // TODO...
//    PUMA_A2 = DEFAULT_PUMA560_A2;
//    PUMA_A3 = DEFAULT_PUMA560_A3;
//    PUMA_D4 = DEFAULT_PUMA560_D4;
//    PUMA_D6 = DEFAULT_PUMA560_D3;

    hal_ready(comp_id);
    return 0;

    error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void)
{
    hal_remove_vtable(vtable_id);
    hal_exit(comp_id);
}

#endif
