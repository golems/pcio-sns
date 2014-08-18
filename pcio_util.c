/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
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

/** \file pcio_util.c
 *
 *  Shell tool used just like pciod, but for hacking random pcio
 *  messages to the CAN network, rather than running as a motor
 *  daemon.
 *
 *  \author Jon Scholz
 */

#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include <somatic/util.h>

#include "include/pcio.h"

#define MAX_BUSSES 4

/**
 *  A single mapping of a module ID and associated CAN bus
 *  This is a linked list, which we need for processing
 *  command args sequentially
 */
struct pcio_modinfo_t {
    int module_id;
    int bus_num;
    int index;
    struct pcio_modinfo_t *head;
    struct pcio_modinfo_t *next;
};
typedef struct pcio_modinfo_t pcio_module_list_t; ///< A nice typedef for our module info linked-list

/* ---------- */
/* ARGP Junk  */
/* ---------- */

/* Option Vars */
static int opt_verbosity = 0;
static int opt_homeset = 0;
static size_t n_busses = 0;
static size_t n_modules = 0;
static size_t bus_count[MAX_BUSSES];    // number of modules per bus
static int bus_ids[MAX_BUSSES]; // Array of bus ids (allows busses to be specified in any order)
// A list mapping motor indices to module information (needed to build the pcio_group_t)
static pcio_module_list_t *pcio_mod_list;

/* Options Struct */
static struct argp_option options[] = {
    {
        .name = "verbose",
        .key = 'v',
        .arg = NULL,
        .flags = 0,
        .doc = "Causes verbose output"
    },
    {
        .name = "homeset",
        .key = 'h',
        .arg = NULL,
        .flags = 0,
        .doc = "Defines the current joint values as the home position"
    },
    {
        .name = "index",
        .key = 'i',
        .arg = "index",
        .flags = 0,
        .doc = "Define a motor index (by default increments with every new module id"
    },
    {
        .name = "module",
        .key = 'm',
        .arg = "module_id",
        .flags = 0,
        .doc = "Define a module ID for a motor index"
    },
    {
        .name = "bus",
        .key = 'b',
        .arg = "CAN_bus",
        .flags = 0,
        .doc = "define a CAN bus for a module"
    },
    {
        .name = NULL,
        .key = 0,
        .arg = NULL,
        .flags = 0,
        .doc = NULL
    }
};


/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "pcio_util v0.0.1";
/// argp program arguments documention
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "A shell utility for passing CAN commands using PCIO\n";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


static int parse_opt( int key, char *arg, struct argp_state *state) {
    (void) state; // ignore unused parameter

    static int current_index = 0; // current motor index to be used with 'm' and 'b' args
    static int current_bus_id = 0;              // The actual bus id to set in pcio_group
    static int current_bus_index = 0;   // The ordinality of the current bus

    switch(key) {
    case 'v':
        opt_verbosity++;
        break;
    case 'h':
        opt_homeset = 1;
        break;
    case 'i':
        current_index = atoi(arg);
        break;
    case 'm':
        /* Add a new module to the list, and set its CAN bus number and command index */
    {
        pcio_mod_list->module_id = atoi( arg );
        pcio_mod_list->bus_num = current_bus_id;
        pcio_mod_list->index = current_index;

        /* List overhead: */
        pcio_mod_list->next = (pcio_module_list_t*) malloc(sizeof(pcio_module_list_t));
        pcio_mod_list->next->head = pcio_mod_list->head;
        pcio_mod_list = pcio_mod_list->next;
        pcio_mod_list->next = NULL;

        /* Update the size of current bus */
        bus_count[current_bus_index]++;
        current_index++; // increment idx, in case user isn't specifying manually
        n_modules++;    // increment n_modules to, since it might differ from current_index
    }
    break;
    case 'b':
        /* set the bus for subsequent modules */
        current_bus_id = atoi(arg);
        // Find index of this bus id
        int i;
        current_bus_index = -1;
        for (i=0;i<MAX_BUSSES;++i)
            if (bus_ids[i] == current_bus_id)
                current_bus_index = i;
        if (current_bus_index == -1) {
            current_bus_index = (int)n_busses;
            bus_ids[current_bus_index] = current_bus_id;
            n_busses++;
        }
        //printf("%d %d %d %d\n\n",bus_ids[0],bus_ids[1],bus_ids[2],bus_ids[3]);
        break;
    case 0:
        break;
    }
    return 0;
}

/* ------- */
/* GLOBALS */
/* ------- */


/* --------------------- */
/* FUNCTION DECLARATIONS */
/* --------------------- */
int build_pcio_group(pcio_group_t *group, pcio_module_list_t *module_list);

/* ---- */
/* MAIN */
/* ---- */
int main(int argc, char **argv) {

    /// Initialize pcio_mod_list
    pcio_mod_list = (pcio_module_list_t*) malloc(sizeof(pcio_module_list_t));
    pcio_mod_list->head = pcio_mod_list;

    // Initialize bus_ids
    memset(&bus_ids, -1, sizeof(int)*MAX_BUSSES);

    /// Parse command line args using argp
    argp_parse(&argp, argc, argv, 0, NULL, NULL);

    /// Declare a pcio_group_t to hold the information on all the modules and busses.
    pcio_group_t pcio_group;

    /// Use pcio_mod_list to build a pcio_group_t
    pcio_mod_list = pcio_mod_list->head;
    build_pcio_group(&pcio_group, pcio_mod_list);

    /// Initialize the CAN network
    int r = pcio_group_init( &pcio_group );
    somatic_hard_assert(r == NTCAN_SUCCESS, "pcio group initialization failed\n");

    if (opt_verbosity) {
        fprintf(stderr, "\n* pcio_util *\n");
        fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
        fprintf(stderr, "-------\n");
    }

    // Do whatever you want with the pcio group here:


    if (opt_homeset) {

        double pos_vals[n_modules];
        r = pcio_group_getd( &pcio_group, PCIO_ACT_FPOS, pos_vals, n_modules );

        pos_vals[1] = 1.2;

        printf("setting home offset to what is currently: \n");
        //somatic_realprint(pos_vals, n_modules);

        r = pcio_group_setd( &pcio_group, PCIO_HOME_OFFSET, pos_vals, n_modules );
        if( NTCAN_SUCCESS != r ) return r;

        //pcio_group_do( pcio_group, PCIO_IDMASK_CMDPUT, PCIO_SET_PARAM, parm_id, svals, 32, n_vals, ret, 8, n_vals,0 );

    }

    return 0;
}

/**
 * Fills out a pcio_group_t using information from arg parsing stored
 * in n_busses, bus_count, and pcio_id_map
 *
 * (changed from array to list to allow groups to be sized dynamically)
 */
int build_pcio_group(pcio_group_t *group, pcio_module_list_t *module_list)
{
    // 1) use n_busses to create an array of busses of the appropriate size
    group->bus = (pcio_bus_t*)malloc(sizeof(pcio_bus_t) * n_busses);
    group->bus_cnt = n_busses;

    // 2) loop through this array, and for each bus create module_t using bus_counts
    size_t i;
    for (i = 0; i < n_busses; ++i) {
        group->bus[i].module = (pcio_module_t*)malloc(sizeof(pcio_module_t) * bus_count[i]);
        group->bus[i].module_cnt = bus_count[i];
        group->bus[i].net = bus_ids[i];
    }

    // 3) walk down pcio_id_map, set bus nets, and add modules to appropriate bus
    int modules_added[n_busses]; // counters for index of where to add a module in each bus
    memset(&modules_added, 0, sizeof(int)*n_busses);

    do {
        int bus = module_list->bus_num;
        int id = module_list->module_id;
        int m; int bus_idx = -1;
        // Look for index of bus with this id
        for (m = 0; (size_t)m < group->bus_cnt; ++m)
            if (group->bus[m].net == bus)
                bus_idx = m;
        somatic_hard_assert(bus_idx != -1,"No busses in group.  Please specify with the \"-b\" flag\n");

        group->bus[bus_idx].module[modules_added[bus_idx]].id = id;
        modules_added[bus_idx]++;

        module_list = module_list->next;
    } while (module_list->next != NULL);

    if (opt_verbosity) {
        size_t j;
        for (i = 0; i<group->bus_cnt; ++i)
            for (j = 0; j<group->bus[i].module_cnt; ++j)
                printf("bus: %d, module: %d \n", group->bus[i].net, group->bus[i].module[j].id);        // Initialize the group.
    }

    return(0);
}
