#ifndef _TEST_GENSERKINS_SUITE
#define _TEST_GENSERKINS_SUITE

#undef ULAPI
#include "genserkins.c"

// for HIWIN RA605
#define RA605_A1 0
#define RA605_ALPHA1 0
#define RA605_D1 0

#define RA605_A2 30
#define RA605_ALPHA2 -PI_2
#define RA605_D2 0

#define RA605_A3 340
#define RA605_ALPHA3 0
#define RA605_D3 0

#define RA605_A4 40
#define RA605_ALPHA4 -PI_2
#define RA605_D4 338

#define RA605_A5 0
#define RA605_ALPHA5 PI_2
#define RA605_D5 0

#define RA605_A6 0
#define RA605_ALPHA6 -PI_2
#define RA605_D6 86

int comp_id;

START_TEST(test_hello_genserkins)
{
    int i, res;

    comp_id = hal_init("check_genserkins");
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
                    "check_genserkins.A-%d", i)) < 0)
            goto error;
        *(haldata->a[i])=0;
        if ((res =
                hal_pin_float_newf(HAL_IO, &(haldata->alpha[i]), comp_id,
                    "check_genserkins.ALPHA-%d", i)) < 0)
            goto error;
        *(haldata->alpha[i])=0;
        if ((res =
                hal_pin_float_newf(HAL_IO, &(haldata->d[i]), comp_id,
                    "check_genserkins.D-%d", i)) < 0)
            goto error;
        *(haldata->d[i])=0;
        if ((res =
                hal_param_s32_newf(HAL_RW, &(haldata->unrotate[i]), comp_id,
                    "check_genserkins.unrotate-%d", i)) < 0)
            goto error;
        haldata->unrotate[i]=0;
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
        hal_param_s32_newf(HAL_RO, &(KINS_PTR->iterations), comp_id, "check_genserkins.last-iterations")) < 0)
        goto error;
    if ((res=
        hal_param_s32_newf(HAL_RW, &(KINS_PTR->max_iterations), comp_id, "check_genserkins.max-iterations")) < 0)
        goto error;
    KINS_PTR->max_iterations = GENSER_DEFAULT_MAX_ITERATIONS;

    A(0) = RA605_A1;
    A(1) = RA605_A2;
    A(2) = RA605_A3;
    A(3) = RA605_A4;
    A(4) = RA605_A5;
    A(5) = RA605_A6;
    ALPHA(0) = RA605_ALPHA1;
    ALPHA(1) = RA605_ALPHA2;
    ALPHA(2) = RA605_ALPHA3;
    ALPHA(3) = RA605_ALPHA4;
    ALPHA(4) = RA605_ALPHA5;
    ALPHA(5) = RA605_ALPHA6;
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

START_TEST(test_genserkins)
{
    double joint_pos[6] = {0,};
    EmcPose carte_pos_fb;   /* actual Cartesian position */
    KINEMATICS_FORWARD_FLAGS fflags = 0;
    KINEMATICS_INVERSE_FLAGS iflags = 0;
    int ret;

    ck_assert_int_eq(0, 0);

    ret = kinematicsForward(joint_pos, &carte_pos_fb, &fflags, &iflags);
    DP ("kfwd: carte_pos: x(%.2f) y(%.2f) z(%.2f) a(%.2f) b(%.2f) c(%.2f)\n",
                      carte_pos_fb.tran.x,
                      carte_pos_fb.tran.y,
                      carte_pos_fb.tran.z,
                      carte_pos_fb.a,
                      carte_pos_fb.b,
                      carte_pos_fb.c);
    
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

START_TEST(test_goodbye_genserkins)
{
    ck_assert_int_eq(0, 0);
    if (comp_id > 0) {
        hal_exit(comp_id);
        comp_id = 0;
    }
}
END_TEST

Suite * genserkins_suite(void)
{
    Suite *s;
    TCase *tc_core;
    s = suite_create("genSerKins");

    tc_core = tcase_create("Core");
    tcase_add_test(tc_core, test_hello_genserkins);
    tcase_add_test(tc_core, test_genserkins);
    tcase_add_test(tc_core, test_goodbye_genserkins);
    suite_add_tcase(s, tc_core);

    return s;
}


#endif /* _TEST_GENSERKINS_SUITE */
