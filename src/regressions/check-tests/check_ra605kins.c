// all tests which require a running RT instance.
// does not fork.

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <check.h>
#include "timers.h"
#include "check-util.h"
#include "hal.h"
#include "hal_priv.h"
#include "rtapi.h"
#include "ring.h"

int verbose, timing, debug, delta, hop;
static struct affinity a;

#include "ra605kins-suite.h"

static struct option long_options[] = {
    {"help",  no_argument,          0, 'h'},
    {"delta",  required_argument,   0, 'D'},
    {"verbose",  no_argument,   0, 'v'},
    {"timing",  no_argument,   0, 't'},
    {"hop",  required_argument,   0, 'H'},
    {0, 0, 0, 0}
};
int main(int argc, char **argv)
{
    int c;

    while (1) {
	int option_index = 0;
	c = getopt_long (argc, argv, "hD:vt",
			 long_options, &option_index);
	if (c == -1)
	    break;

	switch (c)	{
	case 'D':
	    delta = a.delta = atoi(optarg);
	    break;

	case 'd':
	    debug++;
	    break;
	case 'v':
	    verbose++;
	    break;
	case 't':
	    timing++;
	    break;
	case 'H':
	    hop = atoi(optarg);
	    break;
	}
    }

    int number_failed;
    Suite *s;
    SRunner *sr;
//    hal_setup();

    s = ra605kins_suite();
    sr = srunner_create(s);
    srunner_set_fork_status (sr, CK_NOFORK);
    {
	WITH_WALL_TIME("all tests", RES_MS);
	srunner_run_all(sr, CK_NORMAL);
    }
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);
//    hal_teardown();
    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
