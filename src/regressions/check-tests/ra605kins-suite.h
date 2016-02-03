#ifndef _TEST_RA605KINS_SUITE
#define _TEST_RA605KINS_SUITE

#undef ULAPI
#include "ra605kins.c"
#include "report_kins.h"

// for HIWIN RA605
#define RA605_A0 0
#define RA605_ALPHA0 0
#define RA605_D1 0

#define RA605_A1 30
#define RA605_ALPHA1 -PI_2
#define RA605_D2 0

#define RA605_A2 340
#define RA605_ALPHA2 0
#define RA605_D3 0

#define RA605_A3 40
#define RA605_ALPHA3 -PI_2
#define RA605_D4 338

#define RA605_A4 0
#define RA605_ALPHA4 PI_2
#define RA605_D5 0

#define RA605_A5 0
#define RA605_ALPHA5 -PI_2
#define RA605_D6 86

int comp_id;

START_TEST(test_hello_ra605kins)
{
    int i, res;

    comp_id = hal_init("check_ra605kins");
    if (comp_id < 0) {
        fprintf(stderr, "ERROR: comp_id(%d)\n", comp_id);
        exit(1);
    }

    haldata = hal_malloc(sizeof(struct haldata));
    if (!haldata) {
        fprintf(stderr, "ERROR: could not connect to HAL\n");
        exit(1);
    }

    for (i = 0; i < GENSER_MAX_JOINTS; i++) {
        if ((res =
                hal_pin_float_newf(HAL_IO, &(haldata->a[i]), comp_id,
                    "check_ra605kins.A-%d", i)) < 0)
            goto error;
        *(haldata->a[i])=0;
        if ((res =
                hal_pin_float_newf(HAL_IO, &(haldata->alpha[i]), comp_id,
                    "check_ra605kins.ALPHA-%d", i)) < 0)
            goto error;
        *(haldata->alpha[i])=0;
        if ((res =
                hal_pin_float_newf(HAL_IO, &(haldata->d[i]), comp_id,
                    "check_ra605kins.D-%d", i)) < 0)
            goto error;
        *(haldata->d[i])=0;
    }

    KINS_PTR = hal_malloc(sizeof(genser_struct));
    haldata->pos = (go_pose *) hal_malloc(sizeof(go_pose));
    if (KINS_PTR == NULL) {
        hal_exit(comp_id);
        exit(1);
    }
    if (haldata->pos == NULL) {
        hal_exit(comp_id);
        exit(1);
    }
    if ((res=
        hal_param_s32_newf(HAL_RO, &(KINS_PTR->iterations), comp_id, "check_ra605kins.last-iterations")) < 0)
        goto error;
    if ((res=
        hal_param_s32_newf(HAL_RW, &(KINS_PTR->max_iterations), comp_id, "check_ra605kins.max-iterations")) < 0)
        goto error;
    KINS_PTR->max_iterations = GENSER_DEFAULT_MAX_ITERATIONS;

    A(0) = RA605_A0;
    A(1) = RA605_A1;
    A(2) = RA605_A2;
    A(3) = RA605_A3;
    A(4) = RA605_A4;
    A(5) = RA605_A5;
    ALPHA(0) = RA605_ALPHA0;
    ALPHA(1) = RA605_ALPHA1;
    ALPHA(2) = RA605_ALPHA2;
    ALPHA(3) = RA605_ALPHA3;
    ALPHA(4) = RA605_ALPHA4;
    ALPHA(5) = RA605_ALPHA5;
    D(0) = RA605_D1;
    D(1) = RA605_D2;
    D(2) = RA605_D3;
    D(3) = RA605_D4;
    D(4) = RA605_D5;
    D(5) = RA605_D6;

    hal_ready(comp_id);
    return;

error:
    hal_exit(comp_id);
    exit(1);
}
END_TEST



START_TEST(test_ra605kins)
{
    double joint_pos[6] = {0,};
    EmcPose carte_pos_fb;   /* actual Cartesian position */
    KINEMATICS_FORWARD_FLAGS fflags = 0;
    KINEMATICS_INVERSE_FLAGS iflags = 0;
    int ret;

    ck_assert_int_eq(0, 0);

    joint_pos[5] = 0;
    joint_pos[1] = -90;
    ret = kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DREPORT_FWD(ret, joint_pos, carte_pos_fb);

    joint_pos[5] = 10;
    joint_pos[1] = -90;
    ret = kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DREPORT_FWD(ret, joint_pos, carte_pos_fb);

    joint_pos[5] = 89;
    joint_pos[1] = -90;
    ret = kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DREPORT_FWD(ret, joint_pos, carte_pos_fb);

    joint_pos[5] = 90;
    joint_pos[1] = -90;
    ret = kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DREPORT_FWD(ret, joint_pos, carte_pos_fb);

    joint_pos[5] = 91;
    joint_pos[1] = -90;
    ret = kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DREPORT_FWD(ret, joint_pos, carte_pos_fb);

    joint_pos[5] = 179;
    joint_pos[1] = -90;
    ret = kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DREPORT_FWD(ret, joint_pos, carte_pos_fb);

    joint_pos[5] = 180;
    joint_pos[1] = -90;
    ret = kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DREPORT_FWD(ret, joint_pos, carte_pos_fb);
    
    //TODO: this is a singular point:
    ret = kinematicsInverse(&carte_pos_fb, joint_pos, &fflags, &iflags);
    DP ("inv: ret(%d), j0(%.2f) j1(%.2f) j2(%.2f) j3(%.2f) j4(%.2f) j5(%.2f)\n",
                      ret,
                      joint_pos[0],
                      joint_pos[1],
                      joint_pos[2],
                      joint_pos[3],
                      joint_pos[4],
                      joint_pos[5]);

    joint_pos[0] = 0;
    joint_pos[1] = -12.525;
    joint_pos[2] = 0;
    joint_pos[3] = 0;
    joint_pos[4] = 0;
    joint_pos[5] = 0;
    kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DP ("fwd: j0(%.2f) j1(%.2f) j2(%.2f) j3(%.2f) j4(%.2f) j5(%.2f)\n",
                      joint_pos[0],
                      joint_pos[1],
                      joint_pos[2],
                      joint_pos[3],
                      joint_pos[4],
                      joint_pos[5]);
    DP ("     x(%.2f) y(%.2f) z(%.2f) a(%.2f) b(%.2f) c(%.2f)\n",
                      carte_pos_fb.tran.x,
                      carte_pos_fb.tran.y,
                      carte_pos_fb.tran.z,
                      carte_pos_fb.a,
                      carte_pos_fb.b,
                      carte_pos_fb.c);

    joint_pos[0] = 0;
    joint_pos[1] = -90;
    joint_pos[2] = 0;
    joint_pos[3] = 0;
    joint_pos[4] = 0;
    joint_pos[5] = 0;
    kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DP ("fwd: j0(%.2f) j1(%.2f) j2(%.2f) j3(%.2f) j4(%.2f) j5(%.2f)\n",
                      joint_pos[0],
                      joint_pos[1],
                      joint_pos[2],
                      joint_pos[3],
                      joint_pos[4],
                      joint_pos[5]);
    DP ("     x(%.2f) y(%.2f) z(%.2f) a(%.2f) b(%.2f) c(%.2f)\n",
                      carte_pos_fb.tran.x,
                      carte_pos_fb.tran.y,
                      carte_pos_fb.tran.z,
                      carte_pos_fb.a,
                      carte_pos_fb.b,
                      carte_pos_fb.c);
    
    ret = kinematicsInverse(&carte_pos_fb, joint_pos, &fflags, &iflags);
    DP ("inv: ret(%d), j0(%.2f) j1(%.2f) j2(%.2f) j3(%.2f) j4(%.2f) j5(%.2f)\n",
                      ret,
                      joint_pos[0],
                      joint_pos[1],
                      joint_pos[2],
                      joint_pos[3],
                      joint_pos[4],
                      joint_pos[5]);

    joint_pos[0] = 0;
    joint_pos[1] = 90;
    kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DP ("fwd: j0(%.2f) j1(%.2f) j2(%.2f) j3(%.2f) j4(%.2f) j5(%.2f)\n",
                      joint_pos[0],
                      joint_pos[1],
                      joint_pos[2],
                      joint_pos[3],
                      joint_pos[4],
                      joint_pos[5]);
    DP ("     x(%.2f) y(%.2f) z(%.2f) a(%.2f) b(%.2f) c(%.2f)\n",
                      carte_pos_fb.tran.x,
                      carte_pos_fb.tran.y,
                      carte_pos_fb.tran.z,
                      carte_pos_fb.a,
                      carte_pos_fb.b,
                      carte_pos_fb.c);
}
END_TEST

START_TEST(test_goodbye_ra605kins)
{
    ck_assert_int_eq(0, 0);
    if (comp_id > 0) {
        hal_exit(comp_id);
        comp_id = 0;
    }
}
END_TEST

Suite * ra605kins_suite(void)
{
    Suite *s;
    TCase *tc_core;
    s = suite_create("ra605Kins");

    tc_core = tcase_create("Core");
    tcase_add_test(tc_core, test_hello_ra605kins);
    tcase_add_test(tc_core, test_ra605kins);
    tcase_add_test(tc_core, test_goodbye_ra605kins);
    suite_add_tcase(s, tc_core);

    return s;
}


#endif /* _TEST_RA605KINS_SUITE */
