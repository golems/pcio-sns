/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
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

#ifndef PCIO_H
#define PCIO_H
#include <ntcan.h>
/** \file pcio.h
 *
 *  Library to Interact with Schunk PowerCubes using the Amtec CAN
 *  protocol.
 *
 *  These data structures are NOT THREAD SAFE! Since the Amtec
 *  protocol isn't really thread safe either, this doesn't seem like a
 *  problem.
 *
 *  enum's based off of documentation:
 *  Data Exchange with PowerCube, rev. 1.3, 2007-07-15
 *  http://collab.cc.gatech.edu/humanoids/sites/edu.humanoids/files/Powercube-data-exchange.pdf
 *
 *  \author Neil Dantam
 *  \author Evan Seguin
 *  \author Tobias Kunz
 */

#ifdef __cplusplus
extern "C" {
#endif


/// CAN ID masks for cmd ack message (A0)
#define PCIO_IDMASK_CMDACK (0x5<<5)
/// CAN ID masks for cmd get message (C0)
#define PCIO_IDMASK_CMDGET (0x6<<5)
/// CAN ID masks for cmd put message (E0)
#define PCIO_IDMASK_CMDPUT (0x7<<5)

/// CAN ID to send command to all modules
#define PCIO_CANID_CMDALL (0x1<<8)

/// CAN ID for module acknowledgements
#define PCIO_CANID_CMDACK( mod_adr ) ( PCIO_IDMASK_CMDACK + (mod_adr) )
/// CAN ID for host put commands
#define PCIO_CANID_CMDGET( mod_adr ) ( PCIO_IDMASK_CMDGET + (mod_adr) )
/// CAN ID for host get commands
#define PCIO_CANID_CMDPUT( mod_adr ) ( PCIO_IDMASK_CMDPUT + (mod_adr) )

#define PCIO_CANID_MODID( canid ) ( canid & 0x1f )

#define PCIO_ERRNO_BASE (NTCAN_ERRNO_BASE + 4096)
#define PCIO_ERR_MODULE (PCIO_ERRNO_BASE + 1)

    typedef enum {
        // Bitmask for flags in long state DWORD
        PCIO_STATE_HOME_OK = 0x2,			// Set after successful homing procedure.
        PCIO_STATE_HALTED = 0x4,			// Cube is halted
        PCIO_STATE_SWR = 0x40,				// Home switch is active
        PCIO_STATE_SW1 = 0x80,				// Limit switch 1 is active
        PCIO_STATE_SW2 = 0x100,				// Limit switch 2 is active
        PCIO_STATE_BRAKEACTIVE = 0x200,		// Brake is active and servo loop open
        PCIO_STATE_CURLIMIT = 0x400,		// Warning: servo at max current output
        PCIO_STATE_MOTION = 0x800,			// Drive is in motion
        PCIO_STATE_RAMP_ACC = 0x1000,		// Drive in acceleration for ramp motion
        PCIO_STATE_RAMP_STEADY = 0x2000,	// Drive at constant velocity for ramp motion
        PCIO_STATE_RAMP_DEC = 0x4000,		// Drive in deceleration for ramp motion
        PCIO_STATE_RAMP_END = 0x8000,		// End of ramp motion profile
        PCIO_STATE_INPROGRESS = 0x10000,	// Step motion command in progress
        PCIO_STATE_FULLBUFFER = 0x20000,	// command deferred in step motion stack (queue?)
        PCIO_STATE_ERROR = 0x1,				// Error! Check error flags. Must reset.
        PCIO_STATE_POWERFAULT = 0x8,		// Error: servo amplifier. Check flags 18-23. Must reboot.
        PCIO_STATE_TOW_ERROR = 0x10,		// Error: tow limit, couldn't follow target position exactly
        PCIO_STATE_COMM_ERROR = 0x20,		// Error: watchdog expired, emergency stop
        PCIO_STATE_POW_VOLT_ERR = 0x40000,	// Error: voltage drop/overvoltage, check pwr supply
        PCIO_STATE_POW_FET_TEMP = 0x80000,	// Error: power transistors overheated, must reboot
        PCIO_STATE_POW_WDG_TEMP = 0x100000,	// Error: motor overheated, must reboot
        PCIO_STATE_POW_SHORTCUR = 0x200000,	// Error: Short curcuit, module overloaded, must reboot
        PCIO_STATE_POW_HALLERR = 0x400000,	// Error: reading hall effect sensors, motor overheated, must reboot
        PCIO_STATE_POW_INTEGRAL_ERR = 0x800000, // Error: drive overloaded, must reboot
        PCIO_STATE_CPU_OVERLOAD = 0x1000000,// Error: Comm fail with CPU. WTF. Must reboot.
        PCIO_STATE_BEYOND_HARD = 0x2000000,	// Error: e-stop on Hard limit. See manual.
        PCIO_STATE_BEYOND_SOFT = 0x4000000,	// Error: e-stop on Soft limit. Must Reset.
        PCIO_STATE_LOGIC_VOLT = 0x8000000	// Error: logic voltage dropped/over-voltage.
    } pcio_state_t;


    /// Amtec proto cmd id's
    typedef enum {
        PCIO_RESET = 0x00,
        PCIO_HOME = 0x01,
        PCIO_HALT = 0x02,
        PCIO_SET_PARAM = 0x08,
        PCIO_GET_PARAM = 0x0a,
        PCIO_SET_MOTION = 0x0b,
        PCIO_SAVE_POS = 0x0e
    } pcio_cmd_id;

    /// Amtec proto motion id's
    typedef enum {
        PCIO_FRAMP = 14,
        PCIO_FVEL_ACK = 17,
        PCIO_FCUR_ACK = 18
    } pcio_motion_id;

    /// Amtec proto param id's
    typedef enum {
        PCIO_DEF_HOME_OFFSET = 0x00,
        PCIO_ACT_FPOS = 0x3c,
        PCIO_ACT_FVEL = 0x41,
        PCIO_TARGET_VEL = 0x4f,
        PCIO_TARGET_ACC = 0x50,
        PCIO_DEF_CUBE_VERSION = 0x1d,
        PCIO_POS_COUNT = 0x24,
        PCIO_REF_POS_COUNT = 0x25,
        PCIO_PARAM_MAX_DELTA_POS = 0x40,
        PCIO_PARAM_MIN_FPOS = 0x45,
        PCIO_PARAM_MAX_FPOS = 0x46,
        PCIO_PARAM_MAX_VEL = 0x48,
        PCIO_PARAM_MAX_ACC = 0x4a,
        PCIO_PARAM_MAX_CUR = 0x4c,
        PCIO_HOME_OFFSET = 0x89,
        // From Martin (comments by cerdogan regarding using current values for feedback):
        PCIO_PARAM_ERROR=0x27,
        PCIO_PARAM_CONFIG =     0x39,
        PCIO_PARAM_BUSCURRENT = 0x70,                       // float, 0'ish values, not meaningful
        PCIO_PARAM_BUSVOLTAGE = 0x71,                       // float, useless, constant
        PCIO_ACT_FPSEUDOCURRENT = 0x4d,                     // float, can detect impact,
                                                            // may need to break to reset
        PCIO_PARAM_RAWCUR = 0x35,                           // int16, kinda useful...
        PCIO_PARAM_NOMINAL_CURRENT = 0x78, //not in manual
        PCIO_PARAM_MAX_CURRENT = 0x79,  //not in manual
        PCIO_PARAM_OVERSHOOT_TIME_NOMIAL_CURRENT = 0x7e, //not in manual
        PCIO_PARAM_OVERSHOOT_TIME_MAX_CURRENT = 0x7f,   //not in manual
    } pcio_param_id;

    typedef enum {
        // Bitmask for short state word, corresponding long state
        PCIO_SHORT_NOT_OK = 0x01,    // PCIO_STATE_ERROR || !PCIO_STATE_HOME_OK || PCIO_STATE_HALTED
        PCIO_SHORT_SWR = 0x02,       // PCIO_STATE_SWR
        PCIO_SHORT_SW1 = 0x04,       // PCIO_STATE_SW1
        PCIO_SHORT_SW2 = 0x08,       // PCIO_STATE_SW2
        PCIO_SHORT_MOTION = 0x10,    // PCIO_STATE_MOTION
        PCIO_SHORT_RAMP_END = 0x20,      // PCIO_STATE_RAMP_END
        PCIO_SHORT_INPROGRESS = 0x40,// PCIO_STATE_INPROGRESS
        PCIO_SHORT_FULLBUFFER = 0x80,// PCIO_STATE_FULLBUFFER
    } pcio_short_state_id;


    /// Struct for representing params, states, and configuration codes
    typedef struct {
        const char *name;
        const char *desc;
        const char *errtype;
        uint32_t flag;
        int type;
    } pcio_code_t;

    extern pcio_code_t pcio_param_codes[];
    extern pcio_code_t pcio_config_codes[];
    extern pcio_code_t pcio_state_codes[];

    /// Structure representing a single powercube module
    typedef struct {
        int id; //< module ID
        uint8_t state;
        double min_pos;
        double max_pos;
        double max_acc;
        double last_pos;
    } pcio_module_t;

    /// Structure representing a several powercubes on a single CAN bus
    typedef struct {
        int net; //< NTCAN net number
        NTCAN_HANDLE handle; //< handle to open can descriptor
        size_t module_cnt; //< number of modules on the bus
        pcio_module_t *module; //< array of the modules
    } pcio_bus_t;

    /// Structure representing a several powercubes on multiple CAN busses
    typedef struct {
        size_t bus_cnt; //< number of busses
        pcio_bus_t *bus; //< array of busses
        CMSG **msg; //< ragged 2-D array of messages, one per module
    } pcio_group_t;


    /** Initialize a powercube group.

        \pre g->bus and g->bus[i]->module are allocated, counts are set
        correctly, g->bus[i]->net is set correctly.

        \post CAN handles are opened, CAN ids properly bound, g->msg is allocated correctly
    */
    int pcio_group_init( pcio_group_t *g );


    /** Destroy previously initialized powercube group
        \post CAN handles are closed, g->msg is free'ed
    */
    int pcio_group_destroy ( pcio_group_t *g );

    /** returns number of modules in group. */
    size_t pcio_group_size( pcio_group_t *g );

    /** Sends a motion command and receives a position acknowledgment. */
    int pcio_group_cmd_ack( pcio_group_t *g, double *ack, size_t cnt,
                            int motion_id, const double *cmd );

    /** Get a floating point (double) parameter from all modules. */
    int pcio_group_getd( pcio_group_t *g, int parm_id,
                         double *vals, size_t n_vals );

    /** Get a int16 parameter from all modules. */
    int pcio_group_get16( pcio_group_t *g, int parm_id,
                          int16_t *vals, size_t n_vals );

    /** Get a int32 parameter from all modules. */
    int pcio_group_get32( pcio_group_t *g, int parm_id,
                          int32_t *vals, size_t n_vals );

    /** Get a uint32 parameter from all modules. */
    int pcio_group_getu32( pcio_group_t *g, int parm_id,
                           uint32_t *vals, size_t n_vals );

    /** Set a floating point (double) parameter to all modules. */
    int pcio_group_setd( pcio_group_t *g, int parm_id,
                         const double *vals, size_t n_vals );

    /** Set a int32 parameter from all modules. */
    int pcio_group_set32( pcio_group_t *g, int parm_id,
                          const int32_t *vals, size_t n_vals );

    /** Set a uint32 parameter from all modules. */
    int pcio_group_setu32( pcio_group_t *g, int parm_id,
                           const uint32_t *vals, size_t n_vals );

    /** Sends powercubes to pos. */
    int pcio_group_setpos( pcio_group_t *g,
                           const double *pos, size_t n_pos,
                           double acc, double vel);

    /** Sends powercubes to pos, and save acks */
    int pcio_group_setpos_ack( pcio_group_t *g,
                               const double *pos, size_t n_pos,
                               double acc, double vel, double *ack);


    /** Enables or disables current limit */
    int pcio_group_set_fullcur( pcio_group_t *g, int enable );

    /** Sends reset command to group */
    int pcio_group_reset( pcio_group_t *g );

    /** Send halt command to group */
    int pcio_group_halt( pcio_group_t *g );

    /** Send home command to group */
    int pcio_group_home( pcio_group_t *g );

    int pcio_group_dump_config( pcio_group_t *g );

    int pcio_group_dump_error(pcio_group_t *g);

    int pcio_group_at_home( pcio_group_t *g );

    void pcio_group_set_last_position( pcio_group_t *g, const double *pos,
                                       size_t cnt );

    void pcio_group_limit_current( pcio_group_t *g, double * cur, size_t cnt );

    void pcio_group_limit_velocity( pcio_group_t *g, double *vel, size_t cnt,
                                    double delta_t );

    void pcio_group_limit_position( pcio_group_t *g, double * pos, size_t cnt );

    void pcio_resolve_module_state_word( int level, uint32_t msg );
    void pcio_resolve_module_config_word( int level, uint32_t msg );
    void pcio_state_error_to_string( char *buf, size_t n, uint32_t msg );
    int pcio_state_word_contains_errors( uint32_t msg );

    pcio_group_t *pcio_group_alloc( size_t bus_cnt,
                                    int *bus_nets,
                                    size_t *module_cnts,
                                    int *module_ids );

    void pcio_group_free( pcio_group_t *group );

    void pcio_code_dump( const pcio_code_t *code );
    int pcio_code_lookup( const pcio_code_t *code, const char *name, uint32_t *x, int* );

#ifdef __cplusplus
}
#endif


#endif
