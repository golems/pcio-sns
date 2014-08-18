/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Can Erdogan (cerdogan3@gatech.edu) Ana Huaman Quispe (ahuaman3@gatech.edu)
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
 * @file query.c
 * @author Can Erdogan, Ana Huaman Quispe
 * @date August 18, 2014
 * @brief The program to query Schunk module 4 on channel 0, and print its state.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include <amino.h>
#include <ntcan.h>
#include <ntcanopen.h>
#include <ach.h>
#include <sns.h>

#include "pcio.h"

/* ******************************************************************************************** */
// Helpful data structures

/// The structure for the module
typedef struct pciod_arg_mod {
	uint32_t id;
	union {
		uint32_t u32;
		double d;
	};
	struct pciod_arg_mod *next;
} pciod_arg_mod_t;

/// The structure for the bus
typedef struct pciod_arg_bus {
	uint32_t net;
	struct pciod_arg_bus *next;
	pciod_arg_mod_t *mod;
} pciod_arg_bus_t;

/* ******************************************************************************************** */
// The input variables

static double opt_frequency = 30; // refresh at 30 hz
static int opt_full_cur = 0;
static pciod_arg_bus_t *opt_bus = NULL;
static const char *opt_query = NULL;
static const char *opt_set = NULL;
static double opt_period_sec = 0.0; // derived from opt_frequency in init
static uint32_t opt_param = 0;
static int opt_param_type = 0;
static int opt_reset = 0;
static int opt_list = 0;
static int opt_home = 0;

static const char* opt_config_enable = NULL;
static const char* opt_config_disable = NULL;
uint32_t opt_flag;

#define ARG_KEY_DISABLE_FULL_CUR 302
#define ARG_KEY_ENABLE_FULL_CUR 303
#define ARG_KEY_PARAM_MAX_DELTA_POS 304

pcio_group_t group;

/* ******************************************************************************************** */
int build_pcio_group(pcio_group_t *group);

/* ******************************************************************************************** */
/// Builds a pcio group and initializes it
static void init_group() {
	build_pcio_group(&group);
	int r = pcio_group_init(&group);
	aa_hard_assert(r == NTCAN_SUCCESS, "pcio group init failed: %s,%i\n", canResultString(r), r);
}

/* ******************************************************************************************** */
static void query( ) {

	// Dump the config or the state based on the query type
	SNS_LOG(LOG_INFO, "Querying:\n");
	pcio_group_dump_error(&group);
	return;

	// Initialize the memory for the value
	pcio_group_t *g = &group;
	size_t n = pcio_group_size(g);
	double Xd[n];
	uint32_t Xu32[n];

	// Get the parameter value
	if( AA_TYPE_DOUBLE == opt_param_type ) {
		pcio_group_getd( g, (int)opt_param, Xd, n);
	} else if( AA_TYPE_UINT32 == opt_param_type ) {
		pcio_group_getu32( g, (int)opt_param, Xu32, n);
	} else {
		printf("ERROR: Unknown type: %d\n", opt_param_type);
		exit(EXIT_FAILURE);
	}

	// Print the values for each module on each specified bus
	size_t k = 0;
	fprintf(stderr, "--------------------------------------------------\n");
	for(size_t i = 0; i < g->bus_cnt; i++) {
		for(size_t j = 0; j < g->bus[i].module_cnt; j++) {
			fprintf(stderr, "[%d] bus %d, module %d:\t", k, g->bus[i].net, g->bus[i].module[j].id);
			if(AA_TYPE_DOUBLE == opt_param_type) fprintf(stderr, "%f\n", Xd[k]);
			else if( AA_TYPE_UINT32 == opt_param_type) fprintf(stderr, "%u\n", Xu32[k]);
		  else assert(0);
			k++;
		}
	}

	// Sanity check: the # of prints should be the # of modules in the group
	assert(k == n);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char *argv[]) {

	sns_init();
	sns_cx.verbosity++;

	// Setup the bus
	pciod_arg_bus_t *bus = AA_NEW0( pciod_arg_bus_t );
	bus->net = 0;
	bus->next = opt_bus;
	opt_bus = bus;

	// Setup the module
	pciod_arg_mod_t *mod = AA_NEW0(pciod_arg_mod_t);
	mod->id = 4;
	mod->next = opt_bus->mod;
	opt_bus->mod = mod;

	// Prepare the code for looking up the state information
	opt_query = "state";
// 	int r = pcio_code_lookup( pcio_param_codes, "state", &opt_param, &opt_param_type );
// 	printf("param: 0x%x, type %d\n", opt_param, opt_param_type);

	// Create the group
	init_group();

	// Output the results of the query
	query();
}

/* ******************************************************************************************** */
/// Fills out a pcio_group_t using information from arg parsing stored in n_busses, bus_count, 
/// and pcio_id_map
int build_pcio_group(pcio_group_t *g) {

	// Create the busses
	g->bus_cnt=0;
	for(pciod_arg_bus_t *bus = opt_bus; bus; bus=bus->next) g->bus_cnt++;
	g->bus = AA_NEW0_AR( pcio_bus_t, g->bus_cnt );

	// Allocate space for modules on each bus, while incrementing the module count for each bus
	size_t i = g->bus_cnt - 1;
	for(pciod_arg_bus_t *bus = opt_bus; bus; bus=bus->next) {

		// Count the number of modules on this bus
		g->bus[i].module_cnt = 0;
		for(pciod_arg_mod_t *mod = bus->mod; NULL != mod; mod=mod->next) g->bus[i].module_cnt++;

		// Allocate the space
		g->bus[i].module = AA_NEW0_AR(pcio_module_t, g->bus[i].module_cnt);
		i--;
	}

	// Set the module information on the allocated spaces
	i = g->bus_cnt - 1;
	for(pciod_arg_bus_t *bus = opt_bus; bus; bus=bus->next ){

		// Set the network address of the bus in the group and print if requested
		g->bus[i].net = (int)bus->net;
		// somatic_verbprintf(2, "Filling bus %d, net %d\n", i, g->bus[i].net);

		// Set the bus ids of the modules (in this bus) in the group
		size_t j = g->bus[i].module_cnt - 1;
		for( pciod_arg_mod_t *mod = bus->mod; NULL != mod; mod=mod->next ) {
			g->bus[i].module[j].id = (int)mod->id;
			// somatic_verbprintf(2, "Filling module %d, id %d\n", j, g->bus[i].module[j].id);
			j--;
		}
		i--;
	}

	return 0;
}
