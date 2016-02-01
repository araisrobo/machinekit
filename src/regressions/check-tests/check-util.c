#ifndef __USE_GNU
#define __USE_GNU
#endif
#define _GNU_SOURCE
#include <sched.h>

#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>

#include <ck_pr.h>

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

int
aff_iterate(struct affinity *acb)
{
	cpu_set_t s;
	unsigned int c;

	c = ck_pr_faa_uint(&acb->request, acb->delta);
	CPU_ZERO(&s);
	CPU_SET(c % cores, &s);

	int ret = sched_setaffinity(gettid(), sizeof(s), &s);
	if (ret < 0) {
	    fprintf(stderr, "sched_setaffinity: %s\n", strerror(errno));
	}
	return ret;
}

int
aff_iterate_core(struct affinity *acb, unsigned int *core)
{
	cpu_set_t s;

	*core = ck_pr_faa_uint(&acb->request, acb->delta);
	CPU_ZERO(&s);
	CPU_SET((*core) % cores, &s);

	int ret = sched_setaffinity(gettid(), sizeof(s), &s);
	if (ret < 0) {
	    fprintf(stderr, "sched_setaffinity: %s\n", strerror(errno));
	}
	return ret;
}
