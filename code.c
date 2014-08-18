/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2009-2010, Georgia Tech Research Corporation
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

#include <amino.h>
#include <stdint.h>

#include "pcio.h"

void pcio_code_dump( const pcio_code_t *code ) {
    for( size_t i = 0; code[i].name; i ++ ) {
        printf("- %s (0x%x)\n", code[i].name, code[i].flag);
        if( code[i].desc ) {
            printf("  ");
            for( size_t j = 0; '\0' != code[i].desc[j]; j++ ) {
                putc(code[i].desc[j],stdout);
                if( j &&  j % 78 == 0 ) printf("\n  ");
            }
            putc('\n',stdout);
        }
    }
}

int pcio_code_lookup( const pcio_code_t *code, const char *name,
                      uint32_t *v, int *t ) {
    for( size_t i = 0; code[i].name; i ++ ) {
        if( 0 == strcasecmp( name, code[i].name ) ) {
            *v = code[i].flag;
            if(t) *t = code[i].type;
            return 0;
        }
    }
    return 1;
}


pcio_code_t pcio_state_codes[] = {
    {.name = "STATE_HOME_OK", .errtype = "info", .flag = PCIO_STATE_HOME_OK},
    {.name = "STATE_HALTED", .errtype = "info", .flag = PCIO_STATE_HALTED},
    {.name = "STATE_SWR", .errtype = "info", .flag = PCIO_STATE_SWR},
    {.name = "STATE_SW1", .errtype = "info", .flag = PCIO_STATE_SW1},
    {.name = "STATE_SW2", .errtype = "info", .flag = PCIO_STATE_SW2},
    {.name = "STATE_BRAKEACTIVE", .errtype = "info", .flag = PCIO_STATE_BRAKEACTIVE},
    {.name = "STATE_CURLIMIT", .errtype = "info", .flag = PCIO_STATE_CURLIMIT},
    {.name = "STATE_MOTION", .errtype = "info", .flag = PCIO_STATE_MOTION},
    {.name = "STATE_RAMP_ACC", .errtype = "info", .flag = PCIO_STATE_RAMP_ACC},
    {.name = "STATE_RAMP_STEADY", .errtype = "info", .flag = PCIO_STATE_RAMP_STEADY},
    {.name = "STATE_RAMP_DEC", .errtype = "info", .flag = PCIO_STATE_RAMP_DEC},
    {.name = "STATE_RAMP_END", .errtype = "info", .flag = PCIO_STATE_RAMP_END},
    {.name = "STATE_INPROGRESS", .errtype = "info", .flag = PCIO_STATE_INPROGRESS},
    {.name = "STATE_FULLBUFFER", .errtype = "info", .flag = PCIO_STATE_FULLBUFFER},
    {.name = "STATE_ERROR", .errtype = "error", .flag = PCIO_STATE_ERROR},
    {.name = "STATE_POWERFAULT", .errtype = "error", .flag = PCIO_STATE_POWERFAULT},
    {.name = "STATE_TOW_ERROR", .errtype = "error", .flag = PCIO_STATE_TOW_ERROR},
    {.name = "STATE_COMM_ERROR", .errtype = "error", .flag = PCIO_STATE_COMM_ERROR},
    {.name = "STATE_POW_VOLT_ERR", .errtype = "error", .flag = PCIO_STATE_POW_VOLT_ERR},
    {.name = "STATE_POW_FET_TEMP", .errtype = "error", .flag = PCIO_STATE_POW_FET_TEMP},
    {.name = "STATE_POW_WDG_TEMP", .errtype = "error", .flag = PCIO_STATE_POW_WDG_TEMP},
    {.name = "STATE_POW_SHORTCUR", .errtype = "error", .flag = PCIO_STATE_POW_SHORTCUR},
    {.name = "STATE_POW_HALLERR", .errtype = "error", .flag = PCIO_STATE_POW_HALLERR},
    {.name = "STATE_POW_INTEGRAL_ERR", .errtype = "error",
     .flag = PCIO_STATE_POW_INTEGRAL_ERR},
    {.name = "STATE_CPU_OVERLOAD", .errtype = "error", .flag = PCIO_STATE_CPU_OVERLOAD},
    {.name = "STATE_BEYOND_HARD", .errtype = "error", .flag = PCIO_STATE_BEYOND_HARD},
    {.name = "STATE_BEYOND_SOFT", .errtype = "error", .flag = PCIO_STATE_BEYOND_SOFT},
    {.name = "STATE_LOGIC_VOLT", .errtype = "error", .flag = PCIO_STATE_LOGIC_VOLT},
    {.name = NULL}
};


pcio_code_t pcio_config_codes[] = {
    {.flag = 0x00000008L, .name="CONFIGID_MOD_BRAKE_PRESENT",
     .desc="1 = Brake is present"},
    {.flag = 0x00000010L, .name="CONFIGID_MOD_BRAKE_AT_POWERON",
     .desc="0 = Brake is released on power on"},
    {.flag = 0x00000020L, .name="CONFIGID_MOD_SWR_WITH_ENCODERZERO",
     .desc="1 = Encoderindex signal is used in homing procedure"},
    {.flag = 0x00000040L, .name="CONFIGID_MOD_SWR_AT_FALLING_EDGE",
     .desc="1 = Homing on falling edge of limit switch"},
    {.flag = 0x00000080L, .name="CONFIGID_MOD_CHANGE_SWR_TO_LIMIT",
     .desc="1 = Home switch is limit switch (except during Homing)"},
    {.flag = 0x00000100L, .name="CONFIGID_MOD_SWR_ENABLED",
     .desc="1 = Home switch is enabled"},
    {.flag = 0x00000200L, .name="CONFIGID_MOD_SWR_LOW_ACTIVE",
     .desc="1 = Home switch is low active"},
    {.flag = 0x00000400L, .name="CONFIGID_MOD_SWR_USE_EXTERNAL",
     .desc="1 = External home switch is used"},
    {.flag = 0x00000800L, .name="CONFIGID_MOD_SW1_ENABLED",
     .desc="1 = Limit switch 1 is enabled"},
    {.flag = 0x00001000L, .name="CONFIGID_MOD_SW1_LOW_ACTIVE",
     .desc="1 = Limit switch 1 is low active"},
    {.flag = 0x00002000L, .name="CONFIGID_MOD_SW1_USE_EXTERNAL",
     .desc="1 = External limit switch 1 is used"},
    {.flag = 0x00004000L, .name="CONFIGID_MOD_SW2_ENABLED",
     .desc="1 = Limit switch 2 is enabled"},
    {.flag = 0x00008000L, .name="CONFIGID_MOD_SW2_LOW_ACTIVE",
     .desc="1 = Limit switch 2 is low active"},
    {.flag = 0x00010000L, .name="CONFIGID_MOD_SW2_USE_EXTERNAL",
     .desc="1 = External Limit switch 2 is used"},
    {.flag = 0x00020000L, .name="CONFIGID_MOD_LINEAR",
     .desc="1 = Module is a linear type"},
    {.flag = 0x00080000L, .name="CONFIGID_MOD_ALLOW_FULL_CUR",
     .desc="0 = Commanded current (PCube_moveCur) is limited to nominal current."},
    {.flag = 0x00100000L, .name="CONFIGID_MOD_M3_COMPATIBLE",
     .desc="1 = Module is M3 compatible. "
     "This concerns CAN communication and behaviour at PCube_moveStep. "
     "Module does not accept motion commands unless successfully homed."},
    {.flag = 0x00200000L, .name="CONFIGID_MOD_LINEAR_SCREW",
     .desc="1 = Module is linear, driven by ball screw."},
    {.flag = 0x00800000L, .name="CONFIGID_MOD_DISABLE_ON_HALT",
     .desc="1 = Motor power disabled In case of error"},
    {.flag = 0x01000000L, .name="CONFIGID_MOD_WATCHDOG_ENABLE",
     .desc="1 = Watchdog is enabled. "
     "Activated automatically by the first 'life sign/ of the host control."},
    {.flag = 0x02000000L, .name="CONFIGID_MOD_ZERO_MOVE_AFTER_HOK",
     .desc="1 = After Homing the module moves to zero"},
    {.flag = 0x04000000L, .name="CONFIGID_MOD_DISABLE_ACK",
     .desc="1 = All acknowledge messages are disabled. "
     "All Get commands will still be acknowledged. Valid only for CAN-Bus."},
    {.flag = 0x08000000L, .name="CONFIGID_MOD_SYNC_MOTION",
     .desc="1 = Sychronized motion commands enabled. "
     "After sending a motion command the drive expects the "
     "broadcast PCube_startMotionAll to start motion. Valid only for CAN-Bus."},
    {.name = NULL}
};




pcio_code_t pcio_param_codes[] = {
    {.name = "PARAM_DEF_HOME_OFFSET", .flag=PCIO_DEF_HOME_OFFSET, .type=AA_TYPE_DOUBLE},
    {.name = "PARAM_ACT_POS", .flag= PCIO_ACT_FPOS, .type=AA_TYPE_DOUBLE},
    {.name = "PARAM_ACT_FVEL", .flag=PCIO_ACT_FVEL},
    {.name = "PARAM_TARGET_VEL", .flag=PCIO_TARGET_VEL},
    {.name = "PARAM_TARGET_ACC", .flag=PCIO_TARGET_ACC},
    {.name = "PARAM_DEF_CUBE_VERSION", .flag=PCIO_DEF_CUBE_VERSION},
    {.name = "PARAM_POS_COUNT", .flag = PCIO_POS_COUNT},
    {.name = "PARAM_REF_POS_COUNT", .flag=PCIO_REF_POS_COUNT},
    {.name = "PARAM_MIN_FPOS", .flag=PCIO_PARAM_MIN_FPOS, .type=AA_TYPE_DOUBLE},
    {.name = "PARAM_MAX_FPOS", .flag=PCIO_PARAM_MAX_FPOS, .type=AA_TYPE_DOUBLE},
    {.name = "PARAM_MAX_VEL", .flag = PCIO_PARAM_MAX_VEL,
     .type = AA_TYPE_DOUBLE},
    {.name = "PARAM_MAX_ACC", .flag = PCIO_PARAM_MAX_ACC,
     .type = AA_TYPE_DOUBLE},
    {.name = "PARAM_MAX_CUR", .flag = PCIO_PARAM_MAX_CUR,
     .type = AA_TYPE_DOUBLE},
    {.name = "PARAM_MAX_DELTA_POS", .flag = PCIO_PARAM_MAX_DELTA_POS},
    {.name = "PARAM_HOME_OFFSET", .flag = PCIO_HOME_OFFSET, .type = AA_TYPE_DOUBLE},
    {.name = "PARAM_ERROR", .flag = PCIO_PARAM_ERROR},
    {.name = "PARAM_CONFIG", .flag = PCIO_PARAM_CONFIG},
    {.name = "PARAM_BUSCURRENT", .flag = PCIO_PARAM_BUSCURRENT},
    {.name = "PARAM_BUSVOLTAGE", .flag=PCIO_PARAM_BUSVOLTAGE},
    {.name = "PARAM_ACT_PSEUDOCURRENT", .flag=PCIO_ACT_FPSEUDOCURRENT},
    {.name = "PARAM_RAWCUR", .flag=PCIO_PARAM_RAWCUR},
    {.name = "PARAM_NOMINAL_CURRENT", .flag = PCIO_PARAM_NOMINAL_CURRENT,
     .type = AA_TYPE_DOUBLE},
    {.name = "PARAM_MAX_CURRENT", .flag = PCIO_PARAM_MAX_CURRENT,
     .type = AA_TYPE_DOUBLE},
    {.name = "PARAM_NOMINAL_OVERSHOOT_TIME",
     .flag = PCIO_PARAM_OVERSHOOT_TIME_NOMIAL_CURRENT, .type = AA_TYPE_UINT32},
    {.name = "PARAM_MAX_OVERSHOOT_TIME", .flag = PCIO_PARAM_OVERSHOOT_TIME_MAX_CURRENT,
     .type=AA_TYPE_UINT32},
    {.name = NULL}
};
