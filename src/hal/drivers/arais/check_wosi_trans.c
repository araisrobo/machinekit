#include <check.h>
#include "../../src/hal/drivers/arais/wosi_trans.h"

START_TEST (test_wosi_trans_init)
{
    int ret;
    ret = wosi_trans_init();
    ck_assert_int_eq (ret, 0);
}
END_TEST

//START_TEST (test_wosi_trans_run)
//{
//    int ret;
//    ret = wosi_trans_run();
//    ck_assert_int_eq (ret, 0);
//}
//END_TEST

START_TEST (test_wosi_trans_exit)
{
    int ret;
    ret = wosi_trans_exit();
    ck_assert_int_eq (ret, 0);
}
END_TEST

Suite *
wosi_suite (void)
{
    Suite *s = suite_create ("WOSI_TRANS");

    /* Core test case */
    TCase *tc_core = tcase_create ("Core");
    tcase_add_test (tc_core, test_wosi_trans_init);
//    tcase_add_test (tc_core, test_wosi_trans_run);
    tcase_add_test (tc_core, test_wosi_trans_exit);
//    tcase_add_test (tc_core, test_wosi_connect);
//    tcase_add_test (tc_core, test_tx_timeout);
//    tcase_add_test (tc_core, test_wosi_recv);
//    tcase_add_test (tc_core, test_wosi_prog_risc);
//    tcase_add_test (tc_core, test_clock_buf_full);
//    tcase_add_test (tc_core, test_wosi_close);
    suite_add_tcase (s, tc_core);

    return s;
}

int
main (void)
{
    int number_failed;

    Suite *s = wosi_suite ();
    SRunner *sr = srunner_create (s);
    // set CK_NOFORK to share static variables between tests
    srunner_set_fork_status(sr, CK_NOFORK);
    srunner_run_all (sr, CK_NORMAL);
    number_failed = srunner_ntests_failed (sr);
    srunner_free (sr);

    if (number_failed == 0)
    {   /* start wosi-transceiver after passing checks */
        int ret;
        ret = wosi_trans_init();
        if (ret != 0) {
            printf ("ERROR: wosi_trans_init() ret(%d)\n", ret);
            ret = wosi_trans_exit();
            exit(ret);
        }
        ret = wosi_trans_run();
        if (ret != 0) {
            printf ("ERROR: wosi_trans_run() ret(%d)\n", ret);
            ret = wosi_trans_exit();
            exit(ret);
        }
        ret = wosi_trans_exit();
    }

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;

}
