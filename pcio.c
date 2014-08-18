/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2009-2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 *            Evan Seguin
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file pcio.c
 * \author Neil Dantam
 * \author Evan Seguin
 */

#include <stdint.h>
#include <amino.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <ntcan.h>
#include <math.h>
#include <ntcanopen.h>
#include <sched.h>
#include <unistd.h>
#include <somatic/util.h>
#include "pcio.h"



#define ERROR_MSG "Error"
#define INFO_MSG "Info"

#define CHECK_RETURN(call) {                            \
        int _pcio_check_return = call;                  \
        if( _pcio_check_return != NTCAN_SUCCESS )       \
            return _pcio_check_return;                  \
    }

#define CHECK_RETURN_MSG(call, MSG) {                   \
        int _pcio_check_return = call;                  \
        if( _pcio_check_return != NTCAN_SUCCESS ) {     \
            fprintf(stderr, MSG);                       \
            return _pcio_check_return; }}               \

/** Get a parameter value */
static int pcio_group_get( pcio_group_t *g, int parm_id,
                           void *vals, int bits, size_t n_vals );

static int pcio_group_msg_recv( pcio_group_t *g, int cmd_id, int mot_parm_id,
                                void *data, int bits, uint8_t *state, size_t cnt );

static int pcio_group_msg_send( pcio_group_t *g, int continue_on_error );

static void pcio_group_msg_setid( pcio_group_t *g, int idmask,
                                  int cmdid, int motion_param_id );
static void pcio_group_msg_set32( pcio_group_t *g, const void *data );

typedef struct {
    const char *flag;
    const char *type;
    uint32_t value;
} state_msg_t;

// An array of the possible states a module can be in.
// Both informational and error messages.
static const state_msg_t states[] = {
    {.flag = "STATE_HOME_OK", .type = INFO_MSG, .value = PCIO_STATE_HOME_OK},
    {.flag = "STATE_HALTED", .type = INFO_MSG, .value = PCIO_STATE_HALTED},
    {.flag = "STATE_SWR", .type = INFO_MSG, .value = PCIO_STATE_SWR},
    {.flag = "STATE_SW1", .type = INFO_MSG, .value = PCIO_STATE_SW1},
    {.flag = "STATE_SW2", .type = INFO_MSG, .value = PCIO_STATE_SW2},
    {.flag = "STATE_BRAKEACTIVE", .type = INFO_MSG, .value = PCIO_STATE_BRAKEACTIVE},
    {.flag = "STATE_CURLIMIT", .type = INFO_MSG, .value = PCIO_STATE_CURLIMIT},
    {.flag = "STATE_MOTION", .type = INFO_MSG, .value = PCIO_STATE_MOTION},
    {.flag = "STATE_RAMP_ACC", .type = INFO_MSG, .value = PCIO_STATE_RAMP_ACC},
    {.flag = "STATE_RAMP_STEADY", .type = INFO_MSG, .value = PCIO_STATE_RAMP_STEADY},
    {.flag = "STATE_RAMP_DEC", .type = INFO_MSG, .value = PCIO_STATE_RAMP_DEC},
    {.flag = "STATE_RAMP_END", .type = INFO_MSG, .value = PCIO_STATE_RAMP_END},
    {.flag = "STATE_INPROGRESS", .type = INFO_MSG, .value = PCIO_STATE_INPROGRESS},
    {.flag = "STATE_FULLBUFFER", .type = INFO_MSG, .value = PCIO_STATE_FULLBUFFER},
    {.flag = "STATE_ERROR", .type = ERROR_MSG, .value = PCIO_STATE_ERROR},
    {.flag = "STATE_POWERFAULT", .type = ERROR_MSG, .value = PCIO_STATE_POWERFAULT},
    {.flag = "STATE_TOW_ERROR", .type = ERROR_MSG, .value = PCIO_STATE_TOW_ERROR},
    {.flag = "STATE_COMM_ERROR", .type = ERROR_MSG, .value = PCIO_STATE_COMM_ERROR},
    {.flag = "STATE_POW_VOLT_ERR", .type = ERROR_MSG, .value = PCIO_STATE_POW_VOLT_ERR},
    {.flag = "STATE_POW_FET_TEMP", .type = ERROR_MSG, .value = PCIO_STATE_POW_FET_TEMP},
    {.flag = "STATE_POW_WDG_TEMP", .type = ERROR_MSG, .value = PCIO_STATE_POW_WDG_TEMP},
    {.flag = "STATE_POW_SHORTCUR", .type = ERROR_MSG, .value = PCIO_STATE_POW_SHORTCUR},
    {.flag = "STATE_POW_HALLERR", .type = ERROR_MSG, .value = PCIO_STATE_POW_HALLERR},
    {.flag = "STATE_POW_INTEGRAL_ERR", .type = ERROR_MSG,
     .value = PCIO_STATE_POW_INTEGRAL_ERR},
    {.flag = "STATE_CPU_OVERLOAD", .type = ERROR_MSG, .value = PCIO_STATE_CPU_OVERLOAD},
    {.flag = "STATE_BEYOND_HARD", .type = ERROR_MSG, .value = PCIO_STATE_BEYOND_HARD},
    {.flag = "STATE_BEYOND_SOFT", .type = ERROR_MSG, .value = PCIO_STATE_BEYOND_SOFT},
    {.flag = "STATE_LOGIC_VOLT", .type = ERROR_MSG, .value = PCIO_STATE_LOGIC_VOLT}
};



typedef struct {
    const char *flag;
    const char *remark;
    uint32_t value;
} config_msg_t;

static const config_msg_t configs[] = {
    {.value = 0x00000008L, .flag="CONFIGID_MOD_BRAKE_PRESENT",
     .remark="1 = Brake is present"},
    {.value = 0x00000010L, .flag="CONFIGID_MOD_BRAKE_AT_POWERON",
     .remark="0 = Brake is released on power on"},
    {.value = 0x00000020L, .flag="CONFIGID_MOD_SWR_WITH_ENCODERZERO",
     .remark="1 = Encoderindex signal is used in homing procedure"},
    {.value = 0x00000040L, .flag="CONFIGID_MOD_SWR_AT_FALLING_EDGE",
     .remark="1 = Homing on falling edge of limit switch"},
    {.value = 0x00000080L, .flag="CONFIGID_MOD_CHANGE_SWR_TO_LIMIT",
     .remark="1 = Home switch is limit switch (except during Homing)"},
    {.value = 0x00000100L, .flag="CONFIGID_MOD_SWR_ENABLED",
     .remark="1 = Home switch is enabled"},
    {.value = 0x00000200L, .flag="CONFIGID_MOD_SWR_LOW_ACTIVE",
     .remark="1 = Home switch is low active"},
    {.value = 0x00000400L, .flag="CONFIGID_MOD_SWR_USE_EXTERNAL",
     .remark="1 = External home switch is used"},
    {.value = 0x00000800L, .flag="CONFIGID_MOD_SW1_ENABLED",
     .remark="1 = Limit switch 1 is enabled"},
    {.value = 0x00001000L, .flag="CONFIGID_MOD_SW1_LOW_ACTIVE",
     .remark="1 = Limit switch 1 is low active"},
    {.value = 0x00002000L, .flag="CONFIGID_MOD_SW1_USE_EXTERNAL",
     .remark="1 = External limit switch 1 is used"},
    {.value = 0x00004000L, .flag="CONFIGID_MOD_SW2_ENABLED",
     .remark="1 = Limit switch 2 is enabled"},
    {.value = 0x00008000L, .flag="CONFIGID_MOD_SW2_LOW_ACTIVE",
     .remark="1 = Limit switch 2 is low active"},
    {.value = 0x00010000L, .flag="CONFIGID_MOD_SW2_USE_EXTERNAL",
     .remark="1 = External Limit switch 2 is used"},
    {.value = 0x00020000L, .flag="CONFIGID_MOD_LINEAR",
     .remark="1 = Module is a linear type"},
    {.value = 0x00080000L, .flag="CONFIGID_MOD_ALLOW_FULL_CUR",
     .remark="0 = Commanded current (PCube_moveCur) is limited to nominal current."},
    {.value = 0x00100000L, .flag="CONFIGID_MOD_M3_COMPATIBLE",
     .remark="1 = Module is M3 compatible. "
     "This concerns CAN communication and behaviour at PCube_moveStep. "
     "Module does not accept motion commands unless successfully homed."},
    {.value = 0x00200000L, .flag="CONFIGID_MOD_LINEAR_SCREW",
     .remark="1 = Module is linear, driven by ball screw."},
    {.value = 0x00800000L, .flag="CONFIGID_MOD_DISABLE_ON_HALT",
     .remark="1 = Motor power disabled In case of error"},
    {.value = 0x01000000L, .flag="CONFIGID_MOD_WATCHDOG_ENABLE",
     .remark="1 = Watchdog is enabled. "
     "Activated automatically by the first 'life sign/ of the host control."},
    {.value = 0x02000000L, .flag="CONFIGID_MOD_ZERO_MOVE_AFTER_HOK",
     .remark="1 = After Homing the module moves to zero"},
    {.value = 0x04000000L, .flag="CONFIGID_MOD_DISABLE_ACK",
     .remark="1 = All acknowledge messages are disabled. "
     "All Get commands will still be acknowledged. Valid only for CAN-Bus."},
    {.value = 0x08000000L, .flag="CONFIGID_MOD_SYNC_MOTION",
     .remark="1 = Sychronized motion commands enabled. "
     "After sending a motion command the drive expects the "
     "broadcast PCube_startMotionAll to start motion. Valid only for CAN-Bus."}
};

/*------------------------*/
/* Module-Level Functions */
/*------------------------*/

/* Set the value (current or velocity) of a single module. */
/*
  int pcio_module_set_value(NTCAN_HANDLE h, pcio_module_t *mod,
  pcio_motion_id motion_id, float value);
  int pcio_module_set_current(NTCAN_HANDLE h, pcio_module_t *mod, float amps);
  int pcio_module_set_velocity(NTCAN_HANDLE h, pcio_module_t *mod, float vel);


  int pcio_module_set_value(NTCAN_HANDLE h, pcio_module_t *mod,
  pcio_motion_id motion_id, float value) {
  int r = 0;
  CMSG msg;
  memset( &msg, 0, sizeof(msg) );
  printf("setting value (%d) -- 0x%x: %f\n", h, mod->id, value );
  //sleep(1);
  // make message
  msg.id = PCIO_CANID_CMDPUT( mod->id );
  msg.data[0] = PCIO_SET_MOTION;
  msg.data[1] = motion_id;
  endconv_st_le_s( & (msg.data[2]), value );
  msg.len =  6 ;
  // send message
  int n = 1;
  //usleep(1);
  r = canWrite( h, &msg, &n, NULL );
  //sched_yield();
  //usleep(1);
  return r;
  }

  int pcio_module_set_current(NTCAN_HANDLE h, pcio_module_t *mod, float amps) {
  return pcio_module_set_value(h, mod, PCIO_FCUR_ACK, amps);
  }

  int pcio_module_set_velocity(NTCAN_HANDLE h, pcio_module_t *mod, float vel) {
  return pcio_module_set_value(h, mod, PCIO_FVEL_ACK, vel);
  }

  int pcio_module_set_position(NTCAN_HANDLE h, pcio_module_t *mod, float pos) {
  return pcio_module_set_value(h, mod, PCIO_FRAMP_ACK, pos);
  }
*/


/*------------------*/
/* Group Management */
/*------------------*/


int pcio_group_dump_config( pcio_group_t *g ) {
    size_t n = pcio_group_size(g);
    uint32_t config[n];
    memset(config, 0, sizeof(config));
    CHECK_RETURN( pcio_group_getu32(g, PCIO_PARAM_CONFIG, config, n) );
    size_t k = 0;
    for(size_t i = 0; i < g->bus_cnt; i++) {
        for(size_t j = 0; j < g->bus[i].module_cnt; j++) {
            somatic_verbprintf(0, "Config bus %d, id %d (%d): 0x%x\n",
                               g->bus[i].net, g->bus[i].module[j].id,
                               k, config[k]);
            pcio_resolve_module_config_word(0, config[k]);
            k++;
        }
    }
    assert(k == n);
    return 0;
}

int pcio_group_init( pcio_group_t *g ) {

    // create message structs
    {
        g->msg = AA_NEW0_AR( CMSG*, g->bus_cnt );

        //build messages
        for( size_t i = 0; i < g->bus_cnt; i++ ) {
            // allocate messages for this bus
            g->msg[i] = AA_NEW0_AR( CMSG,  g->bus[i].module_cnt );
            // fill in module ids
            for( size_t j = 0; j < g->bus[i].module_cnt; j ++ ) {
                g->msg[i][j].id = PCIO_CANID_CMDPUT( g->bus[i].module[j].id );
            }
        }
    }


    // open can handles and set baudrate
    for( size_t i = 0; i < g->bus_cnt; i++ ) {
        CHECK_RETURN_MSG( canOpen( g->bus[i].net ,   //net
                                   0,   //flags
                                   (int32_t)(4*g->bus[i].module_cnt),  //txqueue
                                   (int32_t)(4*g->bus[i].module_cnt),  //rxqueue
                                   100, //txtimeout
                                   100, //rxtimeout
                                   &g->bus[i].handle // handle
                              ),
                          "Couldn't open net\n" );
        CHECK_RETURN_MSG( canSetBaudrate(g->bus[i].handle, NTCAN_BAUD_1000),
                          "Couldn't set baud\n" );
    }


    // bind CAN-IDS
    for( size_t i = 0; i < g->bus_cnt; i++ ) {
        for( size_t j = 0; j < g->bus[i].module_cnt; j++ ) {
            CHECK_RETURN_MSG( canIdAdd(g->bus[i].handle,
                                       PCIO_CANID_CMDACK( g->bus[i].module[j].id ) ),
                              "Couldn't bind CAN id\n");
        }
    }


    // reset and release limits
    CHECK_RETURN( pcio_group_reset( g ) );
    {
        size_t n = pcio_group_size(g);

        // check errors
        {
            uint32_t error[n];
            CHECK_RETURN_MSG( pcio_group_getu32(g, PCIO_PARAM_ERROR, error, n),
                              "Couldn't read error word\n");
            size_t k = 0;
            int r = 0;
            for( size_t i = 0; i < g->bus_cnt; i++ ) {
                for( size_t j = 0; j < g->bus[i].module_cnt; j++ ) {
                    assert( k < n );
                    somatic_verbprintf(1, "STATE OF BUS %d ID %d:  0x%x\n",
                                       g->bus[i].net, g->bus[i].module[j].id, error[k] );
                    pcio_resolve_module_state_word(2, error[k]);
/*
                    // check to make sure we get home_oks
                    if(!(error[k] & PCIO_STATE_HOME_OK)) {
	                    fprintf(stderr, "Did not get HOME_OK from BUS %d ID %d\n",
	                            g->bus[i].net, g->bus[i].module[j].id);
	                    r = -1;
                    }
*/
                    // check for power fault
                    if( (error[k] & PCIO_STATE_POWERFAULT) &&
                        (error[k] & PCIO_STATE_POW_VOLT_ERR) ){
                        fprintf(stderr, "Power Fault on bus %d ID %d\n",
                                g->bus[i].net, g->bus[i].module[j].id );
                        fprintf(stderr, "You may have a blown fuse.\n");
                        r = -1;
                    }
                    k++;
                }
            }
            if (r) return r;
        }

        // initialize joint limit info
        {
            double min_pos[n]; AA_ZERO_AR(min_pos);
            double act_pos[n]; AA_ZERO_AR(act_pos);
            double max_pos[n]; AA_ZERO_AR(max_pos);
            double max_acc[n]; AA_ZERO_AR(max_acc);

            CHECK_RETURN_MSG( pcio_group_getd( g, PCIO_PARAM_MIN_FPOS, min_pos, n ),
                                "Couldn't read minimum position limits\n");
            CHECK_RETURN_MSG( pcio_group_getd( g, PCIO_ACT_FPOS, act_pos, n ),
                                "Couldn't read actual initial position\n");
            CHECK_RETURN_MSG( pcio_group_getd( g, PCIO_PARAM_MAX_FPOS, max_pos, n ),
                                "Couldn't read maximum position limits\n");
            CHECK_RETURN_MSG( pcio_group_getd( g, PCIO_PARAM_MAX_ACC, max_acc, n ),
                                "Couldn't read maximum acceleration limits\n" );

            static double const SAFE_LIMIT_RATIO = 0.98; // 2% buffer on hard-limits
            size_t i_val = 0;
            for( size_t i_bus = 0; i_bus < g->bus_cnt; ++i_bus ) {
                pcio_bus_t * bus = &g->bus[i_bus];
                for( size_t i_bus_mod = 0; i_bus_mod < bus->module_cnt; ++i_bus_mod, ++i_val ) {
                    pcio_module_t * mod = &bus->module[i_bus_mod];
                    mod->min_pos = min_pos[i_val] * SAFE_LIMIT_RATIO;
                    mod->last_pos = act_pos[i_val];
                    mod->max_pos = max_pos[i_val] * SAFE_LIMIT_RATIO;
                    mod->max_acc = max_acc[i_val] * SAFE_LIMIT_RATIO;
//                    fprintf( stderr, "Limits[b=%d,m=%d]:  min=%f  max=%f    max_acc=%f\n",
//                             bus->net, mod->id,
//                             mod->min_pos, mod->max_pos, mod->max_acc );
                }
            }
        }
    }

    return NTCAN_SUCCESS; // this is 0
}

int pcio_group_set_fullcur( pcio_group_t *g, int enable ) {
    size_t n = pcio_group_size(g);

    {
        // release limits
        uint32_t config[n];
        CHECK_RETURN( pcio_group_getu32(g, PCIO_PARAM_CONFIG, config, n) );
        size_t k = 0;
        for( size_t i = 0; i < g->bus_cnt; i++ ) {
            for( size_t j = 0; j < g->bus[i].module_cnt; j++ ) {
                assert( k < n );
                somatic_verbprintf(1, "CONFIG OF BUS %d ID %d: 0x%x\n",
                                   g->bus[i].net, g->bus[i].module[j].id, config[k]);
                pcio_resolve_module_config_word(2, config[k]);
                if( 1 == enable ) {
                    // release current limit, clear undocumented flags
                    config[k] = (config[i] | 0x00080000) & 0xFFFFFFF ;
                } else {
                    // set current limit, clear undocumented flags
                    config[k] = (config[i] & 0xFFF7FFFF) & 0xFFFFFFF ;

                }

                k++;
            }
        }
        CHECK_RETURN( pcio_group_setu32(g, PCIO_PARAM_CONFIG, config, n) );
    }


    //CHECK_RETURN( pcio_group_reset( g ) );
    return 0;
}


/// return number of modules in the group
size_t pcio_group_size( pcio_group_t *g ) {
    size_t s = 0;
    for( size_t i = 0; i < g->bus_cnt; i ++ )
        s += g->bus[i].module_cnt;
    return s;
}

int pcio_group_destroy ( pcio_group_t *g ) {

    // stop the modules
    pcio_group_halt( g );

    // close handles and free()
    for( size_t i = 0; i < g->bus_cnt; i++ ) {
        canClose( g->bus[i].handle );
        free( g->msg[i] );
    }
    free( g->msg );
    return 0;
}


/*--------------------------------*/
/* Message Construction/Send/Recv */
/*--------------------------------*/

/** Set the CAN and amtec proto ids for group
 */
static void pcio_group_msg_setid( pcio_group_t *g, int idmask,
                                  int cmdid, int motionid ) {
    for( size_t i = 0; i < g->bus_cnt; i++ ) {
        assert( g->msg[i] );
        for( size_t j = 0; j < g->bus[i].module_cnt; j ++ ) {
            assert( g->bus[i].module[j].id );
            g->msg[i][j].id = idmask + g->bus[i].module[j].id ;
            assert( PCIO_CANID_MODID( g->msg[i][j].id ) == g->bus[i].module[j].id );
            g->msg[i][j].data[0] = (uint8_t)cmdid;
            if( motionid > 0 ) { // normal commands
                g->msg[i][j].data[1] = (uint8_t)motionid;
                g->msg[i][j].len = 2;
            } else { //special commands (reset, home...)
                g->msg[i][j].len = 1;
            }
        }
    }
}


/** Sets bytes 2-5 of each message with little-endian 32-bit values in data array. */
static void pcio_group_msg_set32( pcio_group_t *g, const void *data ) {
    size_t ia = 0;
    uint32_t *data32 = (uint32_t*)data;
    for( size_t i = 0; i < g->bus_cnt; i++ ) {
        assert( g->msg[i] );
        for( size_t j = 0; j < g->bus[i].module_cnt; j ++ ) {
            g->msg[i][j].len =  6 ;
            aa_endconv_st_le_u32( & (g->msg[i][j].data[2]), data32[ia++] );
        }
    }
}


/** Sends all messages in g->msg */
static int pcio_group_msg_send( pcio_group_t *g, int continue_on_error ) {
    // send messages
    int status = NTCAN_SUCCESS; // bug, only gets the last error
    for( size_t i = 0; i < g->bus_cnt; i++ ) {
        // the wrong way, should send multiple messages with each canWrite or interleave buses
        for( size_t j = 0; j < g->bus[i].module_cnt; j ++ ) {
            assert( PCIO_CANID_MODID( g->msg[i][j].id ) == g->bus[i].module[j].id );
            assert( g->msg[i][j].id > g->bus[i].module[j].id );
            int n = 1;
            int r = canWrite( g->bus[i].handle, &g->msg[i][j], &n, NULL );
            //int is_error =  pcio_state_word_contains_errors(r); // probably wrong -ntd
            int is_error = r;
            if( is_error ) {
                fprintf(stderr, "CAN error sending message: %d -- %s\n",
                        r, canResultString(r) );
                if( ! continue_on_error ) {
                    return r;
                } else {
                    status = r;
                }
            }
        }
    }
    return status;
}

static int pcio_group_msg_recv( pcio_group_t *g,
                                int cmd_id, int mot_parm_id,
                                void *data, int bits, uint8_t *state, size_t cnt ) {
    assert( 32 == bits || 16 == bits || 8 == bits || 0 == bits );
    assert( (pcio_group_size( g ) == cnt && NULL != data) ||
            (0 == cnt && NULL == data ) );

    uint8_t  *data8 = (uint8_t*)data;
    uint16_t *data16 = (uint16_t*)data;
    uint32_t *data32 = (uint32_t*)data;
    size_t ia = 0;
    for( size_t i = 0; i < g->bus_cnt; i ++ ) { // loop through buses
        for( size_t j = 0; j < g->bus[i].module_cnt; ) { // count through modules
            // Must init CMSG to zero, the esd library will not!
            CMSG msg;
            memset( &msg, 0, sizeof(msg) );
            int n = 1;
            int r = canRead( g->bus[i].handle, &msg, &n, NULL );
            if( NTCAN_SUCCESS != r ) { 
						printf("bus index: %lu, module id: %lu, canstring: %s\n", i, j, canResultString(r) ); return r;}
            assert( msg.len <= 8 );
            int mod_id = PCIO_CANID_MODID( msg.id );
            int msg_cmd_id = msg.data[0];
            int msg_mot_parm_id = msg.data[1];
            if(( msg.len >= 2 &&  // maybe get a parameter
                 msg_cmd_id == cmd_id &&
                 msg_mot_parm_id == mot_parm_id ) ||
               ( 1 == msg.len &&  // special no param id commands
                 msg_cmd_id == cmd_id &&
                 mot_parm_id < 0 ) ) {
                assert( msg.len - 2 >= bits/8 ||
                        ( 1 == msg.len && mot_parm_id < 0 ) ); // ensures msg has enough bytes for requested data bits in data
                for( size_t k = 0; k < g->bus[i].module_cnt; k++ ) { // find right module
                    if(g->bus[i].module[k].id == mod_id ) {
                        if( NULL != data ) {
                            if( 8 == bits )
                                data8[ia + k] = msg.data[2];
                            else if( 16 == bits )
                                data16[ia + k] = aa_endconv_ld_le_u16( &msg.data[2] );
                            else if (32 == bits)
                                data32[ia + k] = aa_endconv_ld_le_u32( &msg.data[2] );
                            else assert(0);
                        } // data
                        if( NULL != state && msg.len >= 7) {
                            state[ia + k] = msg.data[6];
                        }
                        j++;
                        break;
                    } // mod_id
                } // for g->bus
            } // msg
        } // for( size_t j...)
        ia += g->bus[i].module_cnt;
    } // for( size_t i...)
    return NTCAN_SUCCESS;
}

/*-------------------------*/
/* Command/Param Send/Recv */
/*-------------------------*/

int pcio_group_all( pcio_group_t *g,
                    int cmd_id, int parm_id ) {

    for( size_t i = 0; i < g->bus_cnt; i++ ) {
        CMSG msg;
        memset( &msg, 0, sizeof(msg) );
        msg.id = PCIO_CANID_CMDALL;
        msg.data[0] = (uint8_t) cmd_id;
        if( parm_id >= 0 ) {
            msg.data[1] = (uint8_t)parm_id;
            msg.len = 2;
        } else {
            msg.len = 1;
        }

        int n = 1;
        int r = canWrite( g->bus[i].handle, &msg, &n, NULL );

        if( NTCAN_SUCCESS != r ) return r;
    }
    return NTCAN_SUCCESS;
}

int pcio_group_do( pcio_group_t *g, int id_mask,
                   int cmd_id, int parm_id,
                   const void *txvals, int tx_bits, size_t n_tx,
                   void *rxvals, int rx_bits, uint8_t *state, size_t n_rx,
                   int continue_on_error ) {
    assert( 32 == tx_bits || 0 == tx_bits );
    assert( 32 == rx_bits || 16 == rx_bits || 8 == rx_bits || 0 == rx_bits );
    assert( ( pcio_group_size(g) == n_tx && NULL != txvals ) ||
            ( 0 == n_tx && NULL == txvals ) );
    assert( ( pcio_group_size(g) == n_rx && NULL != rxvals ) ||
            ( 0 == n_rx && NULL == rxvals ) );

    // build messages
    pcio_group_msg_setid( g, id_mask, cmd_id, parm_id );
    if( txvals )
        pcio_group_msg_set32( g, txvals );

    // send messages
    {
        int r = pcio_group_msg_send( g, continue_on_error );
        if( NTCAN_SUCCESS != r ) {
            return r;
        }
    }

    // collect response
    {
        int r = pcio_group_msg_recv( g, cmd_id, parm_id,
                                     rxvals, rx_bits, state, n_rx );
        if( NTCAN_SUCCESS != r ) return r;
    }

    return NTCAN_SUCCESS;
}


int pcio_group_cmd_ack( pcio_group_t *g, double *ack, size_t cnt,
                        int motion_id, const double *cmd ) {
    somatic_verbprintf(2, "pcio_group_cmd_ack(0x%x)\n", motion_id);
    assert( PCIO_FVEL_ACK == motion_id || /* velocity command */
            PCIO_FCUR_ACK == motion_id || /* current command */
            PCIO_FRAMP == motion_id); /* position command */

    assert( pcio_group_size(g) == cnt );

    float tx[cnt];
    float rx[cnt];
    uint8_t state[cnt];
    memset(state,0,sizeof(state));
    memset(tx,0,sizeof(tx));
    memset(rx,0,sizeof(rx));
    somatic_d2s( tx, cmd, cnt );

    for( size_t i = 0; i < g->bus_cnt; i ++ ) { // loop through buses
        for( size_t j = 0; j < g->bus[i].module_cnt; j++) { // count through modules
            if(g->bus[i].module[j].state % 2 == 1) {
                fprintf(stderr, "Found error in bus, %d, id %d\n",
                        g->bus[i].net, g->bus[i].module[j].id);
                pcio_group_halt( g );
                return 1;
            }
        }
    }

    int r = pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                           PCIO_SET_MOTION, motion_id,
                           tx, 32, (NULL != tx) ? cnt : 0,
                           rx, 32, state, cnt,
                           0 );
    if( NTCAN_SUCCESS != r ) {
        fprintf(stderr, "Couldn't send message, halting group\n");
        pcio_group_halt( g );
        return r;
    }

    size_t k = 0;
    int halt = 0;
    for( size_t i = 0; i < g->bus_cnt; i ++ ) { // loop through buses
        for( size_t j = 0; j < g->bus[i].module_cnt; j++) { // count through modules
            uint8_t s = state[k++];
            g->bus[i].module[j].state = s;
            if(s & PCIO_SHORT_NOT_OK) {
                halt = 1;
                fprintf(stderr, "Error in bus net %d, module id %d: "
                        "short state 0x%x\n",
                        g->bus[i].net, g->bus[i].module[j].id, s );
            }
            //printf("%x ", state[i]);
        }
    }
    //printf("\n");

    if(halt) {
        fprintf(stderr, "Found error, halting group\n");
        pcio_group_halt( g );
        return PCIO_ERR_MODULE; // error
    }

    if( ack ) {
        somatic_s2d( ack, rx, cnt );
        pcio_group_set_last_position( g, ack, cnt );
    } else {
        double pos[cnt];
        somatic_s2d( pos, rx, cnt );
        pcio_group_set_last_position( g, pos, cnt );
    }
    return r;
}

int pcio_group_dump_error (pcio_group_t *g ) {
    size_t n = pcio_group_size(g);
    uint32_t error[n];
    memset(error, 0, sizeof(error));
    pcio_group_getu32(g, PCIO_PARAM_ERROR, error, n);
    size_t k = 0;
    for( size_t i = 0; i < g->bus_cnt; i++ ) {
        for( size_t j = 0; j < g->bus[i].module_cnt; j++ ) {
            assert( k < n );
            somatic_verbprintf(1, "STATE OF BUS %d ID %d:  0x%x\n",
                               g->bus[i].net, g->bus[i].module[j].id, error[k] );
            pcio_resolve_module_state_word(0, error[k]);
            // check for power fault
            if( (error[k] & PCIO_STATE_POWERFAULT) &&
                (error[k] & PCIO_STATE_POW_VOLT_ERR) ){
                fprintf(stderr, "Power Fault on bus %d ID %d\n",
                        g->bus[i].net, g->bus[i].module[j].id );
                fprintf(stderr, "You may have a blown fuse.\n");
            }

            k++;
        }
    }
    return 0;
}

int pcio_group_get( pcio_group_t *g, int parm_id,
                    void *vals, int bits, size_t n_vals ) {

    return  pcio_group_do( g, PCIO_IDMASK_CMDGET,
                           PCIO_GET_PARAM, parm_id,
                           NULL, 32, 0,
                           vals, bits, NULL, n_vals,
                           0 );
}


int pcio_group_setd( pcio_group_t *g, int parm_id,
                     const double *vals, size_t n_vals ) {
    float svals[n_vals];
    somatic_d2s( svals, vals, n_vals );
    uint8_t ret[n_vals];
    //FIXME: should check status in ret
    return  pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                           PCIO_SET_PARAM, parm_id,
                           svals, 32, n_vals,
                           ret, 8, NULL, n_vals,
                           0 );
}

int pcio_group_set32( pcio_group_t *g, int parm_id,
                      const int32_t *vals, size_t n_vals ) {
    uint8_t ret[n_vals];
    //FIXME: should check status in ret
    return  pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                           PCIO_SET_PARAM, parm_id,
                           vals, 32, n_vals,
                           ret, 8, NULL, n_vals,
                           0 );
}

int pcio_group_setu32( pcio_group_t *g, int parm_id,
                       const uint32_t *vals, size_t n_vals ) {
    uint8_t ret[n_vals];
    //FIXME: should check status in ret
    return  pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                           PCIO_SET_PARAM, parm_id,
                           vals, 32, n_vals,
                           ret, 8, NULL, n_vals,
                           0 );
}


int pcio_group_getd( pcio_group_t *g, int parm_id,
                     double *vals, size_t n_vals ) {
    float svals[n_vals];
    int r = pcio_group_get( g, parm_id, svals, 32, n_vals );
    somatic_s2d( vals, svals, n_vals );
    if( (NTCAN_SUCCESS == r) && (PCIO_ACT_FPOS == parm_id) ) {
        pcio_group_set_last_position( g, vals, n_vals );
    }
    return r;
}

int pcio_group_get16( pcio_group_t *g, int parm_id,
                      int16_t *vals, size_t n_vals ) {
    return  pcio_group_get( g, parm_id, vals, 16, n_vals );
}

int pcio_group_get32( pcio_group_t *g, int parm_id,
                      int32_t *vals, size_t n_vals ) {
    return  pcio_group_get( g, parm_id, vals, 32, n_vals );
}

int pcio_group_getu32( pcio_group_t *g, int parm_id,
                       uint32_t *vals, size_t n_vals ) {
    return  pcio_group_get( g, parm_id, vals, 32, n_vals );
}


/*-------------------*/
/* Specific Commands */
/*-------------------*/

int pcio_group_reset( pcio_group_t *g ) {
    somatic_verbprintf(2, "pcio_group_reset()\n");
    for( size_t i = 0; i < g->bus_cnt; i ++ ) { // loop through buses
        for( size_t j = 0; j < g->bus[i].module_cnt; j++ ) { // count through modules
            g->bus[i].module[j].state = 0;
        }
    }
    return pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                          PCIO_RESET, -1,
                          NULL, 0, 0,
                          NULL, 0, NULL, 0,
                          0 );
}

int pcio_group_halt( pcio_group_t *g ) {
    return pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                          PCIO_HALT, -1,
                          NULL, 0, 0,
                          NULL, 0, NULL, 0,
                          0 );
}

int pcio_group_home( pcio_group_t *g ) {
    return pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                          PCIO_HOME, -1,
                          NULL, 0, 0,
                          NULL, 0, NULL, 0,
                          0 );
}


int pcio_group_at_home( pcio_group_t *g ) {
    // set the new home position be the the current position plus the
    // previous home position.

    size_t n = pcio_group_size(g);
    int r = 0;
    double oldpos[n];
    double oldhome[n];
    double newpos[n];
    double newhome[n];

    void dump(const char *s, double *x) {
        if( somatic_opt_verbosity >= 1 ) {
            fprintf(stderr, "%s: [ ", s);
            for( size_t i = 0; i < n; i ++ ) {
                fprintf(stderr, "%.2lf  ", x[i]*180.0/M_PI );
            }
            fprintf(stderr, "]\n");
        }
    }

    // get old state
    r = pcio_group_getd( g, PCIO_HOME_OFFSET, oldpos, n );
    if( NTCAN_SUCCESS != r ) return r;
    dump( "OLD POS", oldpos );

    r = pcio_group_getd( g, PCIO_ACT_FPOS, oldhome, n );
    if( NTCAN_SUCCESS != r ) return r;
    dump( "OLD HOME", oldhome );

    double zero[n];
    aa_fzero( zero, n );
    for( size_t i = 0; i < n; i ++ )
        zero[i] = oldpos[i] + oldhome[i];

    // set stuff
    r = pcio_group_setd( g, PCIO_HOME_OFFSET, zero, n );
    if( NTCAN_SUCCESS != r ) return r;

    pcio_group_reset(g);

    // get new state
    r = pcio_group_getd( g, PCIO_ACT_FPOS, newpos, n );
    if( NTCAN_SUCCESS != r ) return r;
    dump( "NEW ACT_POS", newpos );

    r = pcio_group_getd( g, PCIO_HOME_OFFSET, newhome, n );
    if( NTCAN_SUCCESS != r ) return r;
    dump( "NEW HOME", newhome );

    return r;
}

int pcio_group_at_pos( pcio_group_t *g, const double *pos ) {
		// adjust the home offset so that the current position is changed to the one supplied
		// in the argument pos
	
    size_t n = pcio_group_size(g);
    int r = 0;
    double oldpos[n];
    double oldhome[n];
		double newpos[n];
    double newhome[n];

    void dump(const char *s, double *x) {
        if( somatic_opt_verbosity >= 1 ) {
            fprintf(stderr, "%s: [ ", s);
            for( size_t i = 0; i < n; i ++ ) {
                fprintf(stderr, "%.2lf  ", x[i]*180.0/M_PI );
            }
            fprintf(stderr, "]\n");
        }
    }

    // get old state
    r = pcio_group_getd( g, PCIO_HOME_OFFSET, oldhome, n );
    if( NTCAN_SUCCESS != r ) return r;
    dump( "OLD POS", oldpos );

    r = pcio_group_getd( g, PCIO_ACT_FPOS, oldpos, n );
    if( NTCAN_SUCCESS != r ) return r;
    dump( "OLD HOME", oldhome );

    for( size_t i = 0; i < n; i ++ )
        newhome[i] = oldpos[i] - pos[i] + oldhome[i];

    // set stuff
    r = pcio_group_setd( g, PCIO_HOME_OFFSET, newhome, n );
    if( NTCAN_SUCCESS != r ) return r;

    pcio_group_reset(g);

    // get new state
    r = pcio_group_getd( g, PCIO_ACT_FPOS, newpos, n );
    if( NTCAN_SUCCESS != r ) return r;
    dump( "NEW ACT_POS", newpos );

    r = pcio_group_getd( g, PCIO_HOME_OFFSET, newhome, n );
    if( NTCAN_SUCCESS != r ) return r;
    dump( "NEW HOME", newhome );

    return r;
}

void pcio_group_set_last_position( pcio_group_t *g, const double *pos, size_t cnt ) {
    assert( cnt = pcio_group_size(g) );

    size_t i_val = 0;
    for( size_t i_bus = 0; i_bus < g->bus_cnt; ++i_bus ) {
        pcio_bus_t * bus = &g->bus[i_bus];
        for( size_t i_bus_mod = 0; i_bus_mod < bus->module_cnt; ++i_bus_mod, ++i_val ) {
            pcio_module_t * mod = &bus->module[i_bus_mod];
//            if( mod->last_pos != pos[i_val] ) {
                mod->last_pos = pos[i_val];
//                fprintf( stderr, "%u,%u: position change: %f\n",
//                         bus->net, mod->id, pos[i_val] );
//            }
        }
    }
}

void pcio_group_limit_current( pcio_group_t *g, double * cur, size_t cnt ) {
    assert( pcio_group_size(g) == cnt );

    fprintf(stderr, "Limiting current...\n");

    //// std. motion equation with x_0 = 0
    //const double delta_pos = vel * delta_t + acc * delta_t * delta_t / 2;

    size_t i_val = 0;
    for( size_t i_bus = 0; i_bus < g->bus_cnt; ++i_bus ) {
        pcio_bus_t * bus = &g->bus[i_bus];
        for( size_t i_bus_mod = 0; i_bus_mod < bus->module_cnt; ++i_bus_mod, ++i_val ) {
            pcio_module_t * mod = &bus->module[i_bus_mod];

            fprintf( stderr, "%u,%u:\t%f <= %f <= %f\t\t%f\n",
                     bus->net, mod->id, mod->min_pos, mod->last_pos,
                     mod->max_pos, cur[i_val] );
            if( mod->last_pos < mod->min_pos ) {
                if( cur[i_val] > 0.0 ) {
                    fprintf( stderr, "%u,%u caused MIN limit with: %f   at: %f\n",
                             bus->net, mod->id, cur[i_val], mod->last_pos );
                    cur[i_val] = 0.0;
                }
            }
            else if( mod->last_pos > mod->max_pos ) {
                if( cur[i_val] < 0.0 ) {
                    fprintf( stderr, "%u,%u caused MAX limit with: %f   at: %f\n",
                             bus->net, mod->id, cur[i_val], mod->last_pos );
                    cur[i_val] = 0.0;
                }
            }
        }
    }
}

void pcio_group_limit_velocity( pcio_group_t *g, double *vel, size_t cnt,
                                double delta_t ) {
    assert( pcio_group_size(g) == cnt );

//    fprintf( stderr, "Min:");
//    for( size_t i_bus = 0; i_bus < g->bus_cnt; ++i_bus ) {
//        pcio_bus_t * bus = &g->bus[i_bus];
//        for( size_t i_bus_mod = 0; i_bus_mod < bus->module_cnt; ++i_bus_mod )
//            fprintf( stderr, " %f", bus->module[i_bus_mod].min_pos );
//    }
//    fprintf( stderr, "\n" );
//
//    fprintf( stderr, "Act:");
//    for( size_t i_bus = 0; i_bus < g->bus_cnt; ++i_bus ) {
//        pcio_bus_t * bus = &g->bus[i_bus];
//        for( size_t i_bus_mod = 0; i_bus_mod < bus->module_cnt; ++i_bus_mod )
//            fprintf( stderr, " %f", bus->module[i_bus_mod].last_pos );
//    }
//    fprintf( stderr, "\n" );
//
//    fprintf( stderr, "Max:");
//    for( size_t i_bus = 0; i_bus < g->bus_cnt; ++i_bus ) {
//        pcio_bus_t * bus = &g->bus[i_bus];
//        for( size_t i_bus_mod = 0; i_bus_mod < bus->module_cnt; ++i_bus_mod )
//            fprintf( stderr, " %f", bus->module[i_bus_mod].max_pos );
//    }
//    fprintf( stderr, "\n" );
//
//    fprintf( stderr, "Vel:");
//    for( size_t i = 0; i < cnt; ++i )
//        fprintf( stderr, " %f", vel[i] );
//    fprintf( stderr, "\n" );

    size_t i_val = 0;
    for( size_t i_bus = 0; i_bus < g->bus_cnt; ++i_bus ) {
        pcio_bus_t * bus = &g->bus[i_bus];
        for( size_t i_bus_mod = 0; i_bus_mod < bus->module_cnt; ++i_bus_mod, ++i_val ) {
            pcio_module_t * mod = &bus->module[i_bus_mod];

//            // The distance to the limit
//            const double dist = ( vel[i_val] < 0.0 )
//                    ? (mod->last_pos - mod->min_pos)
//                    : (mod->max_pos - mod->last_pos);
//
//            // The time it would take to get from the limit to here,
//            // from a full stop at max acceleration
//            const double time_to_pos = (dist / mod->max_acc / mod->max_acc);
//
//            // acceleration time excluding this step
//            const double slack_time = (time_to_pos < delta_t) ? 0.0 : (time_to_pos - delta_t);
//
//            // Max speed given max acceleration at the end of this step
//            const double max_speed = slack_time * mod->max_acc;
//
//            if( fabs(vel[i_val]) > max_speed ) {
//                fprintf( stderr, "%u,%u caused velocity limit: %f\t%f\n",
//                        bus->net, mod->id, vel[i_val], max_speed );
//                if( vel[i_val] < 0 )
//                    vel[i_val] = -max_speed;
//                else
//                    vel[i_val] = max_speed;
//            }
            const double new_pos = mod->last_pos + vel[i_val] * delta_t;
            if(new_pos < mod->min_pos) {
                vel[i_val] = (mod->min_pos - mod->last_pos) / delta_t;
            } else if( new_pos > mod->max_pos ) {
                vel[i_val] = (mod->max_pos - mod->last_pos) / delta_t;
            }
        }
    }
}

void pcio_group_limit_position( pcio_group_t *g, double *pos, size_t cnt )
{
    assert( pcio_group_size(g) == cnt );

//    fprintf(stderr, "Limiting position...\n");

//    // std. motion equation with x_0 = 0
//    const double delta_pos = vel * delta_t + acc * delta_t * delta_t / 2;

    size_t i_val = 0;
    for( size_t i_bus = 0; i_bus < g->bus_cnt; ++i_bus ) {
        pcio_bus_t * bus = &g->bus[i_bus];
        for( size_t i_bus_mod = 0; i_bus_mod < bus->module_cnt; ++i_bus_mod, ++i_val ) {
            pcio_module_t * mod = &bus->module[i_bus_mod];
            if( pos[i_val] < mod->min_pos ) {
//                fprintf( stderr, "%u,%u caused MIN limit with: %f\n",
//                         bus->net, mod->id, pos[i_val] );
                pos[i_val] = mod->min_pos;
            }
            else if( pos[i_val] > mod->max_pos ) {
//                fprintf( stderr, "%u,%u caused MIN limit with: %f\n",
//                                         bus->net, mod->id, pos[i_val] );
                pos[i_val] = mod->max_pos;
            }
        }
    }
}

int pcio_group_setpos( pcio_group_t *g,
                       const double *pos, size_t n_pos,
                       double acc, double vel ) {
    // send velocity
    {
        double v_ar[n_pos];
        aa_fset( v_ar, vel, n_pos );
        pcio_group_setd( g, PCIO_TARGET_VEL, v_ar, n_pos );
    }

    // send accel
    {
        double a_ar[n_pos];
        aa_fset( a_ar, acc, n_pos );
        pcio_group_setd( g, PCIO_TARGET_ACC, a_ar, n_pos );
    }

    // send motion
    {
        float tx[n_pos];
        uint8_t rx[n_pos];
        somatic_d2s( tx, pos, n_pos );
        int r = pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                               PCIO_SET_MOTION, PCIO_FRAMP,
                               tx, 32, n_pos,
                               rx, 8, NULL, n_pos,
                               0 );
        if( NTCAN_SUCCESS != r )
            return r;
    }

    return NTCAN_SUCCESS ;
}

/**
 *
 */
int pcio_group_setpos_ack( pcio_group_t *g,
                           const double *pos, size_t n_pos,
                           double acc, double vel, double *ack )
{
    // send velocity
    {
        double v_ar[n_pos];
        aa_fset( v_ar, vel, n_pos );
        pcio_group_setd( g, PCIO_TARGET_VEL, v_ar, n_pos );
    }

    // send accel
    {
        double a_ar[n_pos];
        aa_fset( a_ar, acc, n_pos );
        pcio_group_setd( g, PCIO_TARGET_ACC, a_ar, n_pos );
    }

    // send motion
    {
        float tx[n_pos];
        float rx[n_pos];
        somatic_d2s( tx, pos, n_pos );
        int r = pcio_group_do( g, PCIO_IDMASK_CMDPUT,
                               PCIO_SET_MOTION, PCIO_FRAMP,
                               tx, 32, n_pos,
                               rx, 32, NULL, n_pos,
                               0 );
        if( NTCAN_SUCCESS != r )
            return r;

        if( ack )
            somatic_s2d( ack, rx, n_pos );
    }

    return NTCAN_SUCCESS ;
}

/*----------------*/
/* Error Handling */
/*----------------*/
void pcio_state_error_to_string( char *buf, size_t n, uint32_t msg ) {
    size_t i;
    buf[0] = '\0';
    n--;
    for ( i = 0; i < sizeof(states) / sizeof(state_msg_t); i++) {
        if ((msg & states[i].value) == states[i].value) {
            strncat(buf, states[i].type, n - strlen(buf));
            strncat(buf, ": ", n - strlen(buf));
            strncat(buf, states[i].flag, n - strlen(buf));
            strncat(buf, "  ", n - strlen(buf));
        }
    }
}

void pcio_resolve_module_state_word(int level, uint32_t msg) {
    size_t i;
    for ( i = 0; i < sizeof(states) / sizeof(state_msg_t); i++) {
        if ((msg & states[i].value) == states[i].value) {
            somatic_verbprintf(level, "  %s: %s (0x%x)\n",
                               states[i].type, states[i].flag, states[i].value);
        }
    }
}

int pcio_state_word_contains_errors( uint32_t msg ) {
    size_t i;
    for ( i = 0; i < sizeof(states) / sizeof(state_msg_t); i++) {
        if ( (msg & states[i].value) == states[i].value
             && 0 == strcmp(states[i].type, ERROR_MSG) ) {
            return 1;
        }
    }
    return 0;
}


void pcio_resolve_module_config_word(int level, uint32_t msg) {
    size_t i;
    for ( i = 0; i < sizeof(configs) / sizeof(config_msg_t); i++) {
        if ((msg & configs[i].value) == configs[i].value) {
            somatic_verbprintf(level, "  %s (0x%x)\n",
                               configs[i].flag, configs[i].value);
        }
    }
}


pcio_group_t *pcio_group_alloc( size_t bus_cnt,
                                int *bus_nets,
                                size_t *module_cnts,
                                int *module_ids ) {
    // group
    pcio_group_t *g = AA_NEW0(pcio_group_t);
    g->bus_cnt = bus_cnt;

    // busses
    g->bus = AA_NEW0_AR( pcio_bus_t, g->bus_cnt );

    size_t k = 0;
    size_t c = 0;
    for( size_t i = 0;  i < g->bus_cnt; i++ ) {
        // set bus
        g->bus[i].module_cnt =  module_cnts[i];
        g->bus[i].module = AA_NEW0_AR( pcio_module_t, g->bus[i].module_cnt );
        g->bus[i].net = bus_nets[i];
        c += g->bus[i].module_cnt;
        for( size_t j = 0; j < g->bus[i].module_cnt; j++ ) {
            g->bus[i].module[j].id = module_ids[k++];
        }
        assert( c == k );
    }
    assert( pcio_group_size(g) == k );

    return g;
}

void pcio_group_free( pcio_group_t *g ) {
    for( size_t i = 0;  i < g->bus_cnt; i++ ) {
        free(g->bus[i].module);
    }
    free(g->bus);
    free(g);
}
