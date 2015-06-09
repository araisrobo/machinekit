#ifndef __WOSI_TRANS_H__
#define __WOSI_TRANS_H__

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>

#include "rtapi_math.h"
#include "rtapi.h"
#include "hal.h"

#include "hal_priv.h"
#include "hal_ring.h"           /* ringbuffer declarations */

#include "tick_jcmd.h"

// configuration and execution state
typedef struct params {
    char *modname;
    char *ring_name;
    int debug;
    int hal_comp_id;
} params_type, *param_pointer;

int wosi_trans_init();
int wosi_trans_run();
int wosi_trans_exit();

int wosi_driver_init(int hal_comp_id, char *inifile);
void wosi_transceive(tick_jcmd_t *tick_jcmd);

#endif
