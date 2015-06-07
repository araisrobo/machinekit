/*
 Yishin Li

 Copyright (C) 2015 Yishin Li

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
 */

#ifndef ULAPI
#error This is intended as a userspace component only.
#endif

#include "wosi_trans.h"

static ringbuffer_t rb;
static ringiter_t ri;

// default options; read from inifile or command line
static params_type param = {
        .modname = "wosi_trans", // hal module param.modname
        .ring_name = "ring_0",
        .debug = 0,
        .hal_comp_id = -1,
};

static void quit(int sig)
{
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: quit(), signal(%d)\n",
            param.modname, param.hal_comp_id);
    wosi_trans_exit();
    exit(0);
}
/**
 * wosi_trans(): WOSI Transceiver
 **/
int wosi_trans_init()
{
    int retval;

    param.hal_comp_id = hal_init(param.modname);
    if (param.hal_comp_id < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed: %d\n",
                param.modname, param.hal_comp_id);
        return param.hal_comp_id;
    }

    retval = hal_ring_attach(param.ring_name, &rb, NULL );
    if (retval)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "%s: ERROR: hal_ring_attach(%s) failed: %d\n", param.modname,
                param.ring_name, retval);
        return retval;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: attached ring '%s' size=%zu type=%d"
            " rmutex=%d wmutex=%d reader=%d writer=%d scratchpad=%zu\n",
            param.modname, param.ring_name, rb.header->size, rb.header->type,
            ring_use_rmutex(&rb), ring_use_wmutex(&rb), rb.header->reader,
            rb.header->writer, ring_scratchpad_size(&rb));

    rb.header->reader = param.hal_comp_id;
    rb.header->reader_instance = rtapi_instance;

    retval = record_iter_init(&rb, &ri);
    if (retval)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "%s: ERROR: record_iter_init() failed: %d\n", param.modname,
                retval);
        return retval;
    }

    retval = wosi_driver_init(param.hal_comp_id);
    if (retval)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "%s: ERROR: wosi_driver_init() failed: %d\n", param.modname,
                retval);
        return retval;
    }



    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    return retval;
}

int wosi_trans_exit()
{
    int retval = 0;

    if (param.hal_comp_id >= 0)
    {
        retval = hal_exit(param.hal_comp_id);
        if (retval)
        {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "%s: ERROR: hal_exit(%d) failed: %d\n", param.modname,
                    param.hal_comp_id, retval);
            return retval;
        }
    }

    return retval;
}

int wosi_trans_run()
{
    ring_size_t rsize;
    char *name;
    char *ring;
    uint32_t underrun = 0; // "number of failed read attempts";
    uint32_t received = 0; // "number of successful read attempts";
    int retval;
    tick_jcmd_t *tick_jcmd;

    name = param.modname;
    ring = param.ring_name;

    if (rb.header->type != RINGTYPE_RECORD)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s(%s): type(%d) should be RINGTYPE_RECORD(0)",
                name, ring, rb.header->type);
        return -1;
    }

    retval = hal_ready(param.hal_comp_id);
    if (retval)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_ready(%d) failed: %d\n",
                param.modname, param.hal_comp_id, retval);
        return retval;
    }

    /* RINGTYPE_RECORD, queue mode */
    while (1)
    {
        rsize = record_next_size(&rb);
        if (rsize <= 0)
        {
            // ring empty
            underrun++;
//            rtapi_print_msg(RTAPI_MSG_INFO,
//                    "%s(%s): wosi_trans_run() record size(%d) underrun(%d)\n", name, ring, rsize, underrun);
            usleep(50);
            continue;
        }

        // point tick_jcmd to ringBuffer
        tick_jcmd = (tick_jcmd_t *) record_next(&rb);

//        rtapi_print_msg(RTAPI_MSG_INFO,
//                "%s(%s): record-len=%d, writer=%d tick(%d)\n", name, ring, rsize,
//                rb.header->writer, tick_jcmd->_tick);
//        rtapi_print_msg(RTAPI_MSG_INFO,
//                "j0_pos_cmd(%f) j1_pos_cmd(%f) j2_pos_cmd(%f) j3_pos_cmd(%f) j4_pos_cmd(%f) j5_pos_cmd(%f)\n",
//                tick_jcmd->pos_cmd[0], tick_jcmd->pos_cmd[1], tick_jcmd->pos_cmd[2],
//                tick_jcmd->pos_cmd[3], tick_jcmd->pos_cmd[4], tick_jcmd->pos_cmd[5]);

        // issue wosi_transceive transaction as receiving servo-tick
        wosi_transceive(tick_jcmd);

        // consume record
        record_shift(&rb);
        received++;

    };

    return 0;
}
