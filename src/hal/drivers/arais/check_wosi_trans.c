#include <check.h>
#include <getopt.h>
#include "wosi_trans.h"

static char *option_string = "h:r:I:";
static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"ring", required_argument, 0, 'r'},    // ring name
        {"ini", required_argument, 0, 'I'},     // default: getenv(INI_FILE_NAME)
        {0,0,0,0}
};
static char *inifile;
static char *ring;      //<! ring buffer name

START_TEST (test_wosi_trans_init)
{
    int ret;
    ret = wosi_trans_init(ring, inifile);
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

//    tcase_add_test (tc_core, test_wosi_trans_exit);
//    tcase_add_test (tc_core, test_wosi_connect);
//    tcase_add_test (tc_core, test_tx_timeout);
//    tcase_add_test (tc_core, test_wosi_recv);
//    tcase_add_test (tc_core, test_wosi_prog_risc);
//    tcase_add_test (tc_core, test_clock_buf_full);
//    tcase_add_test (tc_core, test_wosi_close);
    suite_add_tcase (s, tc_core);

    return s;
}


static void usage(int argc, char **argv) {
    printf("Usage:  %s [options]\n", argv[0]);
    printf("This is a userspace HAL program, typically loaded using the halcmd \"loadusr\" command:\n"
            "    loadusr wosi_trans [options]\n"
            "Options are:\n"
            "-I or --ini <inifile>\n"
            "    Use <inifile> (default: take ini filename from environment variable INI_FILE_NAME)\n"
            );
}


int main(int argc, char **argv)
{
    int number_failed;
    int opt;

    inifile = getenv("INI_FILE_NAME");
    ring = "ring_0";

    while ((opt = getopt_long(argc, argv, option_string, long_options, NULL)) != -1) {
        switch(opt) {
        case 'I':
            inifile = optarg;
            break;
        case 'r':
            ring = optarg;
            break;
        case 'h':
        default:
            usage(argc, argv);
            exit(0);
        }
    }

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
