#include <stdio.h>

#include "timers.h"

static struct res {
    double scale;
    char *unit;
} scale[] = {
    {1.0E-9, "sec" },
    {1.0E-6, "ms" },
    {1.0E-3, "us" },
    {1.0,    "ns" },
    {1.0E-9, "sec" }
};

void _autorelease_timer(struct scoped_timer *sc)
{
    struct timespec now;
    if (clock_gettime(sc->tmode, &now))
	perror("end clock_gettime");

    double nsec = (now.tv_sec - sc->start.tv_sec) *1E9 +
	(now.tv_nsec - sc->start.tv_nsec);
    if (sc->divisor > 1)
	nsec /= (double) sc->divisor;
    printf("%s - %s: %f %s", sc->type, sc->text,
	   nsec*scale[sc->scale].scale, scale[sc->scale].unit);
    if (sc->divisor > 1)
	printf("   (count = %u)\n", sc->divisor);
    else
	printf("\n");
}
