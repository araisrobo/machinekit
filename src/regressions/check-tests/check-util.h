#ifndef _CHECK_UTIL_H
#define _CHECK_UTIL_H

struct affinity {
	unsigned int delta;
	unsigned int request;
};

#define DP(fmt, args...)                                                \
    do {                                                                \
        printf("%s: (%s:%d) ",                                          \
                         __FILE__, __FUNCTION__, __LINE__ );            \
        printf(fmt, ##args);                                            \
    } while (0)

#define DPS(fmt, args...)                                               \
    do {                                                                \
        printf(fmt, ##args);                                            \
    } while (0)

extern int cores;

int mkrandom(void);
pid_t gettid(void);
int num_cores(void);
int aff_iterate(struct affinity *acb);
int aff_iterate_core(struct affinity *acb, unsigned int *core);

#endif
