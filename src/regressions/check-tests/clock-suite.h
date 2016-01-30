#ifndef _TEST_CLOCK_SUITE
#define _TEST_CLOCK_SUITE


#define CLOCK_SAMPLES 10000

// https://github.com/btorpey/clocks

#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

// http://c-faq.com/stdio/commaprint.html
#include <locale.h>
char *commaprint(unsigned long n)
{
	static int comma = '\0';
	static char retbuf[30];
	char *p = &retbuf[sizeof(retbuf)-1];
	int i = 0;

	if(comma == '\0') {
		struct lconv *lcp = localeconv();
		if(lcp != NULL) {
			if(lcp->thousands_sep != NULL &&
				*lcp->thousands_sep != '\0')
				comma = *lcp->thousands_sep;
			else	comma = ',';
		}
	}

	*p = '\0';

	do {
		if(i%3 == 0 && i != 0)
			*--p = comma;
		*--p = '0' + n % 10;
		n /= 10;
		i++;
	} while(n != 0);

	return p;
}

// macro to call clock_gettime w/different values for CLOCK
#define do_clock(CLOCK) \
do { \
   struct timespec x; \
   clock_getres(CLOCK, &x); \
   printf("%25s\t%15s", #CLOCK, commaprint(x.tv_nsec)); \
   clock_gettime(CLOCK, &x); \
   printf("\t%15s", commaprint(x.tv_sec)); \
   printf("\t%15s\n", commaprint(x.tv_nsec)); \
} while(0)


START_TEST(test_clock_resolution)
{
   printf("%25s\t%15s\t%15s\t%15s\n", "clock", "res (ns)", "secs", "nsecs");

   struct timeval tv;
   gettimeofday(&tv, NULL);
   printf("%25s\t%15s\t%15s\t", "gettimeofday","1,000", commaprint(tv.tv_sec));
   printf("%15s\n", commaprint(tv.tv_usec*1000));
   
#if _POSIX_TIMERS > 0   

   #ifdef CLOCK_REALTIME
   do_clock(CLOCK_REALTIME);
   #endif

   #ifdef CLOCK_REALTIME_COARSE
   do_clock(CLOCK_REALTIME_COARSE);
   #endif

   #ifdef CLOCK_REALTIME_HR
   do_clock(CLOCK_REALTIME_HR);
   #endif

   #ifdef CLOCK_MONOTONIC
   do_clock(CLOCK_MONOTONIC);
   #endif

   #ifdef CLOCK_MONOTONIC_RAW
   do_clock(CLOCK_MONOTONIC_RAW);
   #endif

   #ifdef CLOCK_MONOTONIC_COARSE
   do_clock(CLOCK_MONOTONIC_COARSE);
   #endif
   
#endif
}
END_TEST

static volatile int sec;
#define do_clock_cost(CLOCK)						\
    do {								\
    int i;								\
    struct timespec x;							\
    WITH_PROCESS_CPUTIME_N(#CLOCK " ns/op", CLOCK_SAMPLES, RES_NS);	\
    for (i = 0; i < CLOCK_SAMPLES; i++)	{				\
	clock_gettime(CLOCK, &x);					\
	sec = x.tv_sec;							\
    }									\
    } while (0)

START_TEST(test_clock_cost)
{

    {
	int i;
	WITH_PROCESS_CPUTIME_N("gettimeofday ns/op", CLOCK_SAMPLES, RES_NS);
	for (i = 0; i < CLOCK_SAMPLES; i++)	{
	    struct timeval tv;
	    gettimeofday(&tv, 0);
	}
    }

#if _POSIX_TIMERS > 0   

#ifdef CLOCK_REALTIME
    do_clock_cost(CLOCK_REALTIME);
#endif

#ifdef CLOCK_REALTIME_COARSE
    do_clock_cost(CLOCK_REALTIME_COARSE);
#endif

#ifdef CLOCK_REALTIME_HR
    do_clock_cost(CLOCK_REALTIME_HR);
#endif

#ifdef CLOCK_MONOTONIC
    do_clock_cost(CLOCK_MONOTONIC);
#endif

#ifdef CLOCK_MONOTONIC_RAW
    do_clock_cost(CLOCK_MONOTONIC_RAW);
#endif

#ifdef CLOCK_MONOTONIC_COARSE
    do_clock_cost(CLOCK_MONOTONIC_COARSE);
#endif
   
#endif
}
END_TEST

Suite * clock_suite(void)
{
    Suite *s;
    TCase *tc_core;
    s = suite_create("posix clock resolutions");

    tc_core = tcase_create("rtapi");

    tcase_add_test(tc_core, test_clock_resolution);
    tcase_add_test(tc_core, test_clock_cost);
    suite_add_tcase(s, tc_core);

    return s;
}
#endif
