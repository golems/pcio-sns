/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2009-2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jon Scholz <jkscholz@gatech.edu>
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

/*
 * pciod.h
 *
 *  Created on: Apr 4, 2010
 *      Author: jscholz
 */

#ifndef PCIOD_H_
#define PCIOD_H_

/// Default command channel name
#define PCIOD_CMD_CHANNEL_NAME "pciod-cmd"

/// Default state channel name
#define PCIOD_STATE_CHANNEL_NAME "pciod-state"

/// Pre-calculated size of command channel
#define PCIOD_CMD_CHANNEL_SIZE 67

/// Pre-calculated size of state channel
#define PCIOD_STATE_CHANNEL_SIZE 130 // 65 when only publishing position

#include <pcio.h>

typedef struct {
    somatic_d_t d;
    somatic_d_opts_t d_opts;
    size_t n; // module count
    ach_channel_t cmd_chan;
    ach_channel_t state_chan;
    pcio_group_t group;
    Somatic__MotorState state_msg;
    struct {
        Somatic__Vector position;
        Somatic__Vector velocity;
        Somatic__Vector current;
    } state_msg_fields;
} pciod_t;

#endif /* PCIOD_H_ */
