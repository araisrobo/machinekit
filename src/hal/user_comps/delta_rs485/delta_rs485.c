/*
  vfdb_vfd.c

  userspace HAL program to control a Delta VFD-B VFD

  Yishin Li, adapted from Michael Haberler's vfs11_vfd/.

  Copyright (C) 2007, 2008 Stephen Wille Padnos, Thoth Systems, Inc.
  Copyright (C) 2009 John Thornton
  Copyright (C) 2009,2010,2011,2012 Michael Haberler
  Copyright (C) 2013 Yishin Li
  Copyright (C) 2013 Sebastian Kuzminsky

  Based on a work (test-modbus program, part of libmodbus) which is
  Copyright (C) 2001-2005 Stéphane Raimbault <stephane.raimbault@free.fr>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, version 2.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA.

  see 'man vfdb_vfd' and the VFD-B section in the Drivers manual.

 */


#ifndef ULAPI
#error This is intended as a userspace component only.
#endif

#ifdef DEBUG
#define DBG(fmt, ...)					\
        do {						\
            if (param.debug) printf(fmt,  ## __VA_ARGS__);	\
        } while(0)
#else
#define DBG(fmt, ...)
#endif

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>

#include "rtapi.h"
#include "hal.h"
#include <modbus.h>
#include <modbus-tcp.h>
#include "inifile.h"


// status registers for DELTA DVP32ES2
#define SR_ERROR_CODE           0x0C01                  //
#define SR_X0                   0x0400                   
//#define SR_X1                   0x0401
//#define SR_X2                   0x0402
//#define SR_X3                   0x0403
//#define SR_X4                   0x0404
//#define SR_X5                   0x0405
//#define SR_X6                   0x0406
//#define SR_X7                   0x0407
//#define SR_X8                   0x0408
//#define SR_X9                   0x0409
//#define SR_XA                   0x040A
//#define SR_XB                   0x040B
//#define SR_XC                   0x040C
//#define SR_XD                   0x040D
//#define SR_XE                   0x040E
//#define SR_XF                   0x040F

#define SR_Y0                   0x0500                   
//#define SR_Y1                   0x0501
//#define SR_Y2                   0x0502
//#define SR_Y3                   0x0503
//#define SR_Y4                   0x0504
//#define SR_Y5                   0x0505
//#define SR_Y6                   0x0506
//#define SR_Y7                   0x0507
//#define SR_Y8                   0x0508
//#define SR_Y9                   0x0509
//#define SR_YA                   0x050A
//#define SR_YB                   0x050B
//#define SR_YC                   0x050C
//#define SR_YD                   0x050D
//#define SR_YE                   0x050E
//#define SR_YF                   0x050F


/* There's an assumption in the gs2_vfd code, namely that the interesting registers
 * are contiguous and all of them can be read with a single read_holding_registers()
 * operation.
 *
 * However, the interesting VFD-B registers are not contiguous, and must be read
 * one-by-one, because the Toshiba Modbus implementation only supports single-value
 * modbus_read_registers() queries, slowing things down considerably. It seems that
 * other VFD's have similar restrictions.
 *
 * Then, not all registers are equally important. We would like to read the
 * VFD status and actual frequency on every Modbus turnaround, but there is no need to
 * the read CPU version and inverter model more than once at startup, and the load factor etc 
 * every so often. 
 */
#define POLLCYCLES 	10      // read less important parameters only on every 10th transaction
#define MODBUS_MIN_OK	10      // assert the modbus-ok pin after 10 successful modbus transactions
#define MAX_DIO         19

/* HAL data struct */
typedef struct {
    hal_s32_t   *error_code;
    hal_s32_t 	*status;
    hal_bit_t   *X[MAX_DIO];    // actual output frequency
    hal_bit_t   *Y[MAX_DIO];    // actual output frequency

    hal_bit_t	*modbus_ok;	// the last MODBUS_OK transactions returned successfully
    hal_s32_t	*errorcount;    // number of failed Modbus transactions - hints at logical errors

    hal_float_t	looptime;
} haldata_t;

// configuration and execution state
typedef struct params {
    char *modname;
    int modbus_debug;
    int debug;
    int slave;
    int pollcycles; 
    char *device;
    int baud;
    int bits;
    char parity;
    int stopbits;
    char *progname;
    char *section;
    FILE *fp;
    char *inifile;
    int reconnect_delay;
    modbus_t *ctx;
    haldata_t *haldata;
    int hal_comp_id;
    int read_initial_done;
    // int old_err_reset;
    uint16_t old_cmd1_reg;		// copy of last write to FA00 */
    int modbus_ok;
    uint16_t failed_reg;		// remember register for failed modbus transaction for debugging
    int	last_errno;
    int report_device;
    int motor_hz;  // rated frequency of the motor
    int motor_rpm;  // rated speed of the motor
} params_type, *param_pointer;

// default options; read from inifile or command line
static params_type param = {
        .modname = NULL,
        .modbus_debug = 0,
        .debug = 0,
        .slave = 1,
        .pollcycles = POLLCYCLES,
        .device = "/dev/ttyS0",
        .baud = 19200,
        .bits = 8,
        .parity = 'E',
        .stopbits = 1,
        .progname = "vfdb_vfd",
        .section = "VFD-B",
        .fp = NULL,
        .inifile = NULL,
        .reconnect_delay = 1,
        .ctx = NULL,
        .haldata = NULL,
        .hal_comp_id = -1,
        .read_initial_done = 0,
        // .old_err_reset = 0,
        .old_cmd1_reg = 0,
        .modbus_ok = 0,    // set modbus-ok bit if last MODBUS_OK transactions went well
        .failed_reg =0,
        .last_errno = 0,
        .report_device = 0,
        .motor_hz = 50,     // 50 is common in Europe, 60 is common in the US
        .motor_rpm = 1410,  // 1410 is common in Europe, 1730 is common in the US
};


static int connection_state;
enum connstate {NOT_CONNECTED, OPENING, CONNECTING, CONNECTED, RECOVER, DONE};

static char *option_string = "dhrmn:S:I:";
static struct option long_options[] = {
        {"debug", no_argument, 0, 'd'},
        {"help", no_argument, 0, 'h'},
        {"modbus-debug", no_argument, 0, 'm'},
        {"report-device", no_argument, 0, 'r'},
        {"ini", required_argument, 0, 'I'},     // default: getenv(INI_FILE_NAME)
        {"section", required_argument, 0, 'S'}, // default section = LIBMODBUS
        {"name", required_argument, 0, 'n'},    // vfd-b
        {0,0,0,0}
};


void  windup(param_pointer p) 
{
    if (p->haldata && *(p->haldata->errorcount)) {
        fprintf(stderr,"%s: %d modbus errors\n",p->progname, *(p->haldata->errorcount));
        fprintf(stderr,"%s: last command register: 0x%.4x\n",p->progname, p->failed_reg);
        fprintf(stderr,"%s: last error: %s\n",p->progname, modbus_strerror(p->last_errno));
    }
    if (p->hal_comp_id >= 0)
        hal_exit(p->hal_comp_id);
    if (p->ctx)
        modbus_close(p->ctx);
}

static void toggle_modbus_debug(int sig)
{
    param.modbus_debug = !param.modbus_debug;
    modbus_set_debug(param.ctx, param.modbus_debug);
}

static void toggle_debug(int sig)
{
    param.debug = !param.debug;
}

static void quit(int sig) 
{
    if (param.debug)
        fprintf(stderr,"quit(connection_state=%d)\n",connection_state);

    switch (connection_state) {

    case CONNECTING:  
        // modbus_tcp_accept() or TCP modbus_connect()  were interrupted
        // these wont return to the main loop, so exit here
        windup(&param);
        exit(0);
        break;

    default:
        connection_state = DONE;
        break;
    }
}

enum kwdresult {NAME_NOT_FOUND, KEYWORD_INVALID, KEYWORD_FOUND};
#define MAX_KWD 10

int findkwd(param_pointer p, const char *name, int *result, const char *keyword, int value, ...)
{
    const char *word;
    va_list ap;
    const char *kwds[MAX_KWD], **s;
    int nargs = 0;

    if ((word = iniFind(p->fp, name, p->section)) == NULL)
        return NAME_NOT_FOUND;

    kwds[nargs++] = keyword;
    va_start(ap, value);

    while (keyword != NULL) {
        if (!strcasecmp(word, keyword)) {
            *result = value;
            va_end(ap);
            return KEYWORD_FOUND;
        }
        keyword = va_arg(ap, const char *);
        kwds[nargs++] = keyword;
        if (keyword)
            value = va_arg(ap, int);
    }  
    fprintf(stderr, "%s: %s:[%s]%s: found '%s' - not one of: ", 
            p->progname, p->inifile, p->section, name, word);
    for (s = kwds; *s; s++) 
        fprintf(stderr, "%s ", *s);
    fprintf(stderr, "\n");
    va_end(ap);
    return KEYWORD_INVALID;
}

int read_ini(param_pointer p)
{
    const char *s;
    int value;

    if ((p->fp = fopen(p->inifile,"r")) != NULL) {
        if (!p->debug)
            iniFindInt(p->fp, "DEBUG", p->section, &p->debug);
        if (!p->modbus_debug)
            iniFindInt(p->fp, "MODBUS_DEBUG", p->section, &p->modbus_debug);
        iniFindInt(p->fp, "BITS", p->section, &p->bits);
        iniFindInt(p->fp, "BAUD", p->section, &p->baud);
        iniFindInt(p->fp, "STOPBITS", p->section, &p->stopbits);
        iniFindInt(p->fp, "TARGET", p->section, &p->slave);
        iniFindInt(p->fp, "POLLCYCLES", p->section, &p->pollcycles);
        iniFindInt(p->fp, "RECONNECT_DELAY", p->section, &p->reconnect_delay);

        iniFindInt(p->fp, "MOTOR_HZ", p->section, &p->motor_hz);
        iniFindInt(p->fp, "MOTOR_RPM", p->section, &p->motor_rpm);

        if ((s = iniFind(p->fp, "DEVICE", p->section))) {
            p->device = strdup(s);
        }
        value = p->parity;
        if (findkwd(p, "PARITY", &value,
                "even",'E',
                "odd", 'O',
                "none", 'N',
                NULL) == KEYWORD_INVALID)
            return -1;
        printf("%d\n", value);
        p->parity = value;
    } else {
        fprintf(stderr, "%s:cant open inifile '%s'\n",
                p->progname, p->inifile);
        return -1;
    }
    return 0;
}

void usage(int argc, char **argv) {
    printf("Usage:  %s [options]\n", argv[0]);
    printf("This is a userspace HAL program, typically loaded using the halcmd \"loadusr\" command:\n"
            "    loadusr vfdb_vfd [options]\n"
            "Options are:\n"
            "-I or --ini <inifile>\n"
            "    Use <inifile> (default: take ini filename from environment variable INI_FILE_NAME)\n"
            "-S or --section <section-name> (default 8)\n"
            "    Read parameters from <section_name> (default 'VFD-B')\n"
            "-d or --debug\n"
            "    Turn on debugging messages. Toggled by USR1 signal.\n"
            "-m or --modbus-debug\n"
            "    Turn on modbus debugging.  This will cause all modbus messages\n"
            "    to be printed in hex on the terminal. Toggled by USR2 signal.\n"
            "-r or --report-device\n"
            "    Report device properties on console at startup\n");
}

#define GETREG(reg,into)					\
        do {							\
            curr_reg = reg;						\
            if (modbus_read_registers(ctx, reg, 1, into) != 1)	\
            goto failed;					\
        } while (0)

#define GETBIT(reg,into)                                        \
        do {                                                    \
            curr_reg = reg;                                             \
            if (modbus_read_input_bits(ctx, reg, 1, into) != 1)  \
            goto failed;                                        \
        } while (0)

int read_data(modbus_t *ctx, haldata_t *haldata, param_pointer p)
{
    int retval, n;
    uint8_t val;
    uint16_t curr_reg;


    for (n = 0; n < MAX_DIO; n++) {
        GETBIT(SR_X0+n, &val);
        *(haldata->X[n]) = val;
        GETBIT(SR_Y0+n, &val);
        *(haldata->Y[n]) = val;
    }

    p->last_errno = retval = 0;
    return 0;

    failed:
    p->failed_reg = curr_reg;
    p->last_errno = errno;
    (*haldata->errorcount)++;
    if (p->debug)
        fprintf(stderr, "%s: read_data: modbus_read_registers(0x%4.4x): %s\n",
                p->progname, curr_reg, modbus_strerror(errno));
    return p->last_errno;
}

#undef GETREG

#define PIN(x)					\
        do {						\
            status = (x);					\
            if ((status) != 0)				\
            return status;				\
        } while (0)

int hal_setup(int id, haldata_t *h, const char *name)
{
    int status, n;

    for (n = 0; n < MAX_DIO; n++) {
        PIN(hal_pin_bit_newf(HAL_OUT, &(h->X[n]), id, "%s.X.%02d", name, n));
        PIN(hal_pin_bit_newf(HAL_OUT, &(h->Y[n]), id, "%s.Y.%02d", name, n));
    }
    PIN(hal_pin_bit_newf(HAL_OUT, &(h->modbus_ok), id, "%s.modbus-ok", name)); // JET
    PIN(hal_pin_s32_newf(HAL_OUT, &(h->error_code), id, "%s.error-code", name));
    PIN(hal_pin_s32_newf(HAL_OUT, &(h->status), id, "%s.status", name));
    PIN(hal_pin_s32_newf(HAL_OUT, &(h->errorcount), id, "%s.error-count", name));
    PIN(hal_param_float_newf(HAL_RW, &(h->looptime), id, "%s.loop-time", name));
    return 0;
}
#undef PIN

int set_defaults(param_pointer p)
{
    int n;

    haldata_t *h = p->haldata;
    *(h->status) = 0;
    for (n = 0; n < MAX_DIO; n++) {
        *(h->X[n]) = 0;
        *(h->Y[n]) = 0;
    }
    *(h->modbus_ok) = 0;
    *(h->errorcount) = 0;
    h->looptime = 0.1;
    p->failed_reg = 0;
    return 0;
}

int main(int argc, char **argv)
{
    struct timespec loop_timespec;
    int opt;
    param_pointer p = &param;
    int retval = -1;

    p->progname = argv[0];
    connection_state = NOT_CONNECTED;
    p->inifile = getenv("INI_FILE_NAME");

    while ((opt = getopt_long(argc, argv, option_string, long_options, NULL)) != -1) {
        switch(opt) {
        case 'n':
            p->modname = strdup(optarg);
            break;
        case 'm':
            p->modbus_debug = 1;
            break;
        case 'd':
            p->debug = 1;
            break;
        case 'S':
            p->section = optarg;
            break;
        case 'I':
            p->inifile = optarg;
            break;
        case 'r':
            p->report_device = 1;
            break;
        case 'h':
        default:
            usage(argc, argv);
            exit(0);
        }
    }

    if (p->inifile) {
        if (read_ini(p))
            goto finish;
        if (!p->modname)
            p->modname = "vfdb_vfd";
    } else {
        fprintf(stderr, "%s: ERROR: no inifile - either use '--ini inifile' or set INI_FILE_NAME environment variable\n", p->progname);
        goto finish;
    }

    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    signal(SIGUSR1, toggle_debug);
    signal(SIGUSR2, toggle_modbus_debug);

    // create HAL component 
    p->hal_comp_id = hal_init(p->modname);
    if ((p->hal_comp_id < 0) || (connection_state == DONE)) {
        fprintf(stderr, "%s: ERROR: hal_init(%s) failed: HAL error code=%d\n",
                p->progname, p->modname, p->hal_comp_id);
        retval = p->hal_comp_id;
        goto finish;
    }

    // grab some shmem to store the HAL data in
    p->haldata = (haldata_t *)hal_malloc(sizeof(haldata_t));
    if ((p->haldata == 0) || (connection_state == DONE)) {
        fprintf(stderr, "%s: ERROR: unable to allocate shared memory\n", p->modname);
        retval = -1;
        goto finish;
    }
    if (hal_setup(p->hal_comp_id,p->haldata, p->modname))
        goto finish;

    set_defaults(p);
    hal_ready(p->hal_comp_id);

    DBG("using libmodbus version %s\n", LIBMODBUS_VERSION_STRING);

    connection_state = OPENING;
    if ((p->ctx = modbus_new_rtu(p->device, p->baud, p->parity, p->bits, p->stopbits)) == NULL) {
        fprintf(stderr, "%s: ERROR: modbus_new_rtu(%s): %s\n",
                p->progname, p->device, modbus_strerror(errno));
        goto finish;
    }
    DBG("device(%s) baud(%d) parity(%s) bits(%d) stopbits(%d)\n", p->device, p->baud, &(p->parity), p->bits, p->stopbits);
    if (modbus_set_slave(p->ctx, p->slave) < 0) {
        fprintf(stderr, "%s: ERROR: invalid slave number: %d\n", p->modname, p->slave);
        goto finish;
    }
    if ((retval = modbus_connect(p->ctx)) != 0) {
        fprintf(stderr, "%s: ERROR: couldn't open serial device: %s\n", p->modname, modbus_strerror(errno));
        goto finish;
    }
    DBG("%s: serial port %s connected\n", p->progname, p->device);

    modbus_set_debug(p->ctx, p->modbus_debug);
    if (modbus_set_slave(p->ctx, p->slave) < 0) {
        fprintf(stderr, "%s: ERROR: invalid slave number: %d\n", p->modname, p->slave);
        goto finish;
    }

    connection_state = CONNECTED;
    while (connection_state != DONE) {

        while (connection_state == CONNECTED) {
            if ((retval = read_data(p->ctx, p->haldata, p))) {
                p->modbus_ok = 0;
//                connection_state = RECOVER;

            } else {
                p->modbus_ok++;
            }
            if (p->modbus_ok > MODBUS_MIN_OK) {
                *(p->haldata->modbus_ok) = 1;
            } else {
                *(p->haldata->modbus_ok) = 0;
            }
//            if ((retval = write_data(p->ctx, p->haldata, p))) {
//                p->modbus_ok = 0;
//                if ((retval == EBADF || retval == ECONNRESET || retval == EPIPE)) {
//                    connection_state = RECOVER;
//                }
//            } else {
//                p->modbus_ok++;
//            }
//            if (p->modbus_ok > MODBUS_MIN_OK) {
//                *(p->haldata->modbus_ok) = 1;
//            } else {
//                *(p->haldata->modbus_ok) = 0;
//            }
            /* don't want to scan too fast, and shouldn't delay more than a few seconds */
            if (p->haldata->looptime < 0.001) p->haldata->looptime = 0.003;
            if (p->haldata->looptime > 2.0) p->haldata->looptime = 0.003;
            loop_timespec.tv_sec = (time_t)(p->haldata->looptime);
            loop_timespec.tv_nsec = (long)((p->haldata->looptime - loop_timespec.tv_sec) * 1000000000l);
//            if (!p->haldata->max_speed)
//                nanosleep(&loop_timespec, &remaining);
        }

        switch (connection_state) {
        case DONE:
            // cleanup actions before exiting.
            modbus_flush(p->ctx);
            // clear the command register (control and frequency override) so panel operation gets reactivated
//            if ((retval = modbus_write_register(p->ctx, REG_COMMAND1, 0)) != 1) {
//                // not much we can do about it here if it goes wrong, so complain
//                fprintf(stderr, "%s: failed to release VFD from bus control (write to register 0x%x): %s\n",
//                        p->progname, REG_COMMAND1, modbus_strerror(errno));
//            } else {
//                DBG("%s: VFD released from bus control.\n", p->progname);
//            }
            break;

        case RECOVER:
            DBG("recover\n");
            set_defaults(p);
            p->read_initial_done = 0;
            // reestablish connection to slave

            modbus_flush(p->ctx);
            modbus_close(p->ctx);
            while ((connection_state != CONNECTED) &&
                    (connection_state != DONE)) {
                sleep(p->reconnect_delay);
                if (!modbus_connect(p->ctx)) {
                    connection_state = CONNECTED;
                    DBG("rtu/tcpclient reconnect\n");
                } else {
                    fprintf(stderr, "%s: recovery: modbus_connect(): %s\n",
                            p->progname, modbus_strerror(errno));
                }
            }

            break;
            default: ;
        }
    }
    retval = 0;	

    finish:
    windup(p);
    return retval;
}

