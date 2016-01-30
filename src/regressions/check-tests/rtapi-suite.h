#ifndef _TEST_RTAPI_SUITE
#define _TEST_RTAPI_SUITE

#include "rtapi.h"
#include "rtapi_app.h"
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

extern int comp_id;

#define SAMPLES 10000


START_TEST(test_rtapi_timing)
{
    // determine the rtapi_get_clocks/rtapi_get_time scaling factor if any
    long long int t1,t2,c1,c2;
    t1 = rtapi_get_time();
    c1 = rtapi_get_clocks();
    sleep(1);
    t2 = rtapi_get_time();
    c2 = rtapi_get_clocks();
    double scale = (double)(c2 - c1)/(double)(t2-t1);

    // on VM's rtapi_get_clocks() comes out pretty random
    fprintf(stderr, "rtapi_get_clocks:rtapi_get_time scale: %f\n", scale);

    volatile long long int s;
    {
	int i;
    	WITH_PROCESS_CPUTIME_N("rtapi_get_clocks ns/op", SAMPLES, RES_NS);
	for (i = 0; i < SAMPLES; i++)
	    s = rtapi_get_clocks();
    }
    {
	int i;
    	WITH_PROCESS_CPUTIME_N("rtapi_get_time ns/op", SAMPLES, RES_NS);
	for (i = 0; i < SAMPLES; i++)
	    s = rtapi_get_time();
    }
}
END_TEST


Suite * rtapi_suite(void)
{
    Suite *s;
    TCase *tc_core;
    s = suite_create("RTAPI");

    tc_core = tcase_create("rtapi");

    tcase_add_test(tc_core, test_rtapi_timing);
    suite_add_tcase(s, tc_core);

    return s;
}


#endif /* _TEST_HAL_SUITE */
