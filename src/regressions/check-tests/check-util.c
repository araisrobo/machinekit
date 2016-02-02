#ifndef __USE_GNU
#define __USE_GNU
#endif
#define _GNU_SOURCE
#include <sched.h>

#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include "check-util.h"


static unsigned seed;

int mkrandom(void)
{
    return rand_r(&seed);
}

#ifndef gettid
pid_t
gettid(void)
{
    return syscall(SYS_gettid);
}
#endif /* gettid */

int cores;

int num_cores(void)
{
    cpu_set_t cs;
    CPU_ZERO(&cs);
    sched_getaffinity(0, sizeof(cs), &cs);
    return CPU_COUNT(&cs);
}

__attribute__((constructor)) void _initcores (void)
{
    cores = num_cores();
    seed = getpid();
}
