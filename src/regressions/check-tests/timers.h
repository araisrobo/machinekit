#include <time.h>                // for gettimeofday()
#include <stdint.h>

enum res_t {
    RES_S = 0,
    RES_MS,
    RES_US,
    RES_NS
};
struct scoped_timer {
    struct timespec start;
    unsigned divisor;
    int tmode, scale;
    char *text, *type;
};
void _autorelease_timer(struct scoped_timer *sc);

#ifndef __PASTE
#define __PASTE(a,b)    a##b
#endif
#define _WITH_TIMER(unique, mode, typ, txt, div, sc)			\
    struct scoped_timer  __PASTE(__scope_protector_, unique)            \
         __attribute__((cleanup(_autorelease_timer))) = {		\
	.tmode = mode,							\
        .text = txt,							\
	.type = typ,							\
	.divisor = div,							\
	.scale = sc,							\
    };									\
    if (clock_gettime(mode,						\
		      &__PASTE(__scope_protector_,unique).start))	\
	perror("start clock_gettime");

#define WITH_PROCESS_CPUTIME_N(text, div, scale)				\
    _WITH_TIMER(__LINE__,CLOCK_PROCESS_CPUTIME_ID,"process time", text, div, scale)
#define WITH_PROCESS_CPUTIME(text, scale)				\
    _WITH_TIMER(__LINE__,CLOCK_PROCESS_CPUTIME_ID,"process time", text, 1, scale)

#define WITH_THREAD_CPUTIME_N(text, div, scale)					\
    _WITH_TIMER(__LINE__,CLOCK_THREAD_CPUTIME_ID, "thread time", text, div, scale)
#define WITH_THREAD_CPUTIME(text, scale)				\
    _WITH_TIMER(__LINE__,CLOCK_THREAD_CPUTIME_ID, "thread time", text, 1, scale)

#define WITH_WALL_TIME_N(text, div, scale)				\
    _WITH_TIMER(__LINE__,CLOCK_REALTIME,"wall time", text, div, scale)

#define WITH_WALL_TIME(text, scale)					\
    _WITH_TIMER(__LINE__,CLOCK_REALTIME,"wall time", text, 1, scale)
