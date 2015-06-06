/********************************************************************
* Description: tc_debug.h
*
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2013 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef TP_DEBUG_H
#define TP_DEBUG_H

#include "rtapi.h"  /* printing functions */

/** TP debug stuff */
#ifdef TP_DEBUG
//Kludge because I didn't know any better at the time
//FIXME replace these with better names?
#define tp_debug_print(fmt, args...)                                    \
    do {                                                                \
        /*rtapi_print("%s: (%s:%d) ", __FILE__, __FUNCTION__, __LINE__);*/ \
        rtapi_print(fmt, ##args);                                       \
    } while (0)
#else
#define tp_debug_print(...) 
#endif

/** Use for profiling to make static function names visible */
#ifdef TP_PROFILE
#define STATIC
#else
#define STATIC static
#endif

/** "TC" debug info for inspecting trajectory planner output at each timestep */
#ifdef TC_DEBUG
#define tc_debug_print(fmt, args...)                                    \
    do {                                                                \
        /* rtapi_print("%s: (%s:%d) ", __FILE__, __FUNCTION__, __LINE__ );*/ \
        rtapi_print(fmt, ##args);                                       \
    } while (0)

#else
#define tc_debug_print(...) 
#endif

/** TP position data output to debug acceleration spikes */
#ifdef TP_POSEMATH_DEBUG
#define tp_posemath_debug(...) rtapi_print(__VA_ARGS__)
#else
#define tp_posemath_debug(...)
#endif

/** TP misc data logging */
#ifdef TP_INFO_LOGGING
#define tp_info_print(...) rtapi_print(__VA_ARGS__)
#else
#define tp_info_print(...)
#endif

#if (TRACE)
#include <stdio.h>
#define DP(fmt, args...)                                                \
    do {                                                                \
        fprintf(dptrace, "%s: (%s:%d) ",                                \
                         __FILE__, __FUNCTION__, __LINE__ );            \
        fprintf(dptrace, fmt, ##args);                                  \
        fflush(dptrace);                                                \
    } while (0)
#define DPS(fmt, args...)                                               \
    do {                                                                \
        fprintf(dptrace, fmt, ##args);                                  \
        fflush(dptrace);                                                \
    } while (0)
#else 
// TRACE == 0
#define DP(fmt, args...)     do {} while(0)
#define DPS(fmt, args...)    do {} while(0)
#endif

#define ERRP(fmt, args...)                                              \
    do {                                                                \
        fprintf(stderr, "%s: (%s:%d) ERROR: ",                          \
                         __FILE__, __FUNCTION__, __LINE__ );            \
        fprintf(stderr, fmt, ##args);                                   \
        fflush(stderr);                                                 \
    } while (0)
#define ERRPS(fmt, args...)                                             \
    do {                                                                \
        fprintf(stderr, fmt, ##args);                                   \
        fflush(stderr);                                                 \
    } while (0)

int gdb_fake_catch(int condition);
int gdb_fake_assert(int condition);
#endif
