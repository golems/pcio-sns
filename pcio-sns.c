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

/** 
 * @file pciod.c
 * @author Jon Scholz, Neil Dantam, Evin Seguin, Can Erdogan 
 * @date June 26, 2013
 * @brief The daemon to control Schunk motors.
 */

#include <argp.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>
//#include <stdarg.h>
//#include <stdint.h>
//
//#include <amino.h>
#include <ntcan.h>
#include <ntcanopen.h>
//
#include <sns.h>
#include <ach.h>
//#include <sns/msg.h>

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
/// Default command channel name
#define PCIOD_CMD_CHANNEL_NAME "pciod-cmd"

/// Default state channel name
#define PCIOD_STATE_CHANNEL_NAME "pciod-state"

typedef struct {
    // somatic_d_t d;
    // somatic_d_opts_t d_opts;
    size_t n; // module count
    ach_channel_t cmd_chan;
    ach_channel_t state_chan;
    pcio_group_t group;
		struct sns_msg_motor_state* state_msg;
		struct sns_msg_motor_ref* ref_msg;
    // Somatic__MotorState state_msg;
    // struct {
    //     Somatic__Vector position;
    //     Somatic__Vector velocity;
    //     Somatic__Vector current;
    // } state_msg_fields;
} pciod_t;

/* ******************************************************************************************** */
// The input variables

static const char *opt_cmd_chan = PCIOD_CMD_CHANNEL_NAME;
static const char *opt_state_chan = PCIOD_STATE_CHANNEL_NAME;
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

/* ******************************************************************************************** */
/* Options Struct */
static struct argp_option options[] = {
  {"verbose", 'v', NULL, 0, "Causes verbose output"},
  {"set", 'S', "param", 0, "Sets module parameter"},
  {"value", 'x', "value", 0, "Parameter value"},
  {"query", 'Q', "'param'|'state'|'config'", 0, "Queries module parameter"},
  {"reset", 'R', NULL, 0, "Resets all modules"},
  {"config-enable", '1', "config-id", 0, "Sets config bit true"},
  {"config-disable", '0', "config-id", 0, "Sets config bit false"},
  {"list", 'L', NULL, 0, "Display listing of all control codes"},
  {"home", 'H', NULL, 0, "Home modules"},
  {"disable-full-current", ARG_KEY_DISABLE_FULL_CUR, NULL, 0, "Full current"},
  {"enable-full-current", ARG_KEY_ENABLE_FULL_CUR, NULL, 0, "Full current"},
  {"frequency", 'f', "freq", 0, "refresh rate on state channel when no commands are received"},
  {"module", 'm', "module_id", 0, "Define a module ID for a motor index"},
	{"bus", 'b', "CAN_bus", 0, "Define a CAN bus for a module"},
	{"cmd-chan", 'c', "pcio_cmd_channel", 0, "ach channel to send powercube commands to"},
  {"state-chan", 's', "pcio_state_channel", 0, "ach channel to listen for commands on"},
	{"daemonize", 'd', NULL, 0, "fork off daemon process"},
	{"ident", 'I', "IDENT", 0, "identifier for this daemon"},
	{NULL, 0, NULL, 0, NULL}
};

/* ******************************************************************************************** */
/// argp parsing function
static int parse_opt(int key, char *arg, struct argp_state *state) {
	pciod_t *cx = (pciod_t*)state->input;
	double parsef() {
		char *endptr = NULL;
		double f = strtod( arg, &endptr );
		if( NULL == endptr ){
			fprintf(stderr, "Error parsing float value %s", arg);
			exit(EXIT_FAILURE);
		}
		return f;
	}
	uint32_t parseu() {
		char *endptr = NULL;
		uint32_t u = strtoul( arg, &endptr, 10 );
		if( NULL == endptr ){
			fprintf(stderr, "Error parsing float value %s", arg);
			exit(EXIT_FAILURE);
		}
		return u;
	}
	(void) state; // ignore unused parameter

	int r;
	switch (key) {

		// Add a new module to the list, and set its CAN bus number and command index 
		case 'm': {
			aa_hard_assert(NULL != opt_bus, "Must specify bus before module\n");
			pciod_arg_mod_t *mod = AA_NEW0(pciod_arg_mod_t);
			mod->id = parseu();
			mod->next = opt_bus->mod;
			opt_bus->mod = mod;
			SNS_LOG(LOG_INFO, "Arg mod: %d on bus %d\n", mod->id, opt_bus->net);
		} break;

		// Parse the bus number
		case 'b': {
			pciod_arg_bus_t *bus = AA_NEW0( pciod_arg_bus_t );
			bus->net = parseu();
			SNS_LOG(LOG_INFO, "Arg bus: %d\n", bus->net);
			bus->next = opt_bus;
			opt_bus = bus;
		}	break;

		// Set the requested parameter value if it is not config or state (?)
		case 'Q': {
			SNS_REQUIRE( NULL == opt_set, "Can't query and set\n");
			opt_query = strdup(arg);
			if((strcasecmp(opt_query, "config" ) != 0) && 
					(strcasecmp( opt_query, "state" ) != 0)) {
				r = pcio_code_lookup( pcio_param_codes, arg, &opt_param, &opt_param_type );
				SNS_REQUIRE( 0 == r, "Unknown Parameter: %s\n", opt_query);
				SNS_LOG(LOG_INFO, "param: 0x%x, type %d\n", opt_param, opt_param_type );
			}
		}	break;

		// Get the parameter that the user wants to set 
		case 'S': {
			SNS_REQUIRE( NULL == opt_query, "Can't query and set\n");
			opt_set = strdup(arg);
			r = pcio_code_lookup( pcio_param_codes, arg, &opt_param, &opt_param_type );
			SNS_REQUIRE( 0 == r, "Unknown Parameter: %s\n", opt_set);
		}	break;

		// Set the given config id true
		case '1': {
			opt_config_enable = strdup(arg);
			r = pcio_code_lookup( pcio_config_codes, arg, &opt_flag, NULL );
			SNS_REQUIRE( 0 == r, "Unknown configid: %s\n", arg);
		} break;

		// Set the given config id false
		case '0': {
			opt_config_disable = strdup(arg);
			r = pcio_code_lookup( pcio_config_codes, arg, &opt_flag, NULL );
			SNS_REQUIRE( 0 == r, "Unknown configid: %s\n", arg);
		} break;

		// Get the value for the given specified parameter before with 'S' and check its type
		case 'x':	{
			SNS_REQUIRE( NULL != opt_set, "value only valid for -S\n");
			if(AA_TYPE_DOUBLE == opt_param_type) {								
				opt_bus->mod->d = parsef();
				SNS_LOG(LOG_INFO, "param %d.%d: %f\n", opt_bus->net, opt_bus->mod->id, opt_bus->mod->d);
			} 
			else if (AA_TYPE_UINT32 == opt_param_type) {
				opt_bus->mod->u32 = parseu();
				SNS_LOG(LOG_INFO, "param %d.%d: %u\n",opt_bus->net, opt_bus->mod->id, opt_bus->mod->u32);
			} 
			else SNS_REQUIRE( 0, "Unknown type: %d\n", opt_param_type );
		} break;

		// Set the rest of the flags to true or increment them
		case 'c': opt_cmd_chan = strdup(arg); break;
		case 's': opt_state_chan = strdup(arg); break;
		case 'R': opt_reset = 1; break;
		case 'L': opt_list = 1; break;
		case 'H': opt_home = 1; break;
		case 'v': sns_cx.verbosity ++; break;
		case 'f': opt_frequency = parsef(); break;
		case ARG_KEY_DISABLE_FULL_CUR: opt_full_cur = 0; break;
		case ARG_KEY_ENABLE_FULL_CUR: opt_full_cur = 1; break;
		case 0:
			break;
	}

	// somatic_d_argp_parse(key, arg, &cx->d_opts);
	return 0;
}

/// argp program doc line
static char doc[] = "reads somatic messages and sends pcio motor commands";

/// argp object
static struct argp argp = {options, parse_opt, NULL, doc, NULL, NULL, NULL };

/* ******************************************************************************************** */
int build_pcio_group(pcio_group_t *group);
int execute_and_update_state(pciod_t *cx);
static void update_state(pciod_t *cx, double *pos_acks);

/* ******************************************************************************************** */
/// Builds a pcio group and initializes it
static void init_group( pciod_t *cx ) {
	build_pcio_group(&cx->group);
	int r = pcio_group_init( &cx->group );
	aa_hard_assert(r == NTCAN_SUCCESS, "pcio group init failed: %s,%i\n", canResultString(r), r);
}

/* ******************************************************************************************** */
/// Sets up the message we will be sending to the motor group
void setupMessage (pciod_t* cx) {
	cx->state_msg = sns_msg_motor_state_heap_alloc(cx->n);
  cx->ref_msg = sns_msg_motor_ref_heap_alloc (cx->n);
}

/* ******************************************************************************************** */
/// Initializes the daemon, channels and sets up the messages
static void init( pciod_t *cx ) {

	// Initialize the daemon
	sns_init();

	// Initialize the group and home it if necessary
	init_group(cx);
	if (opt_home) pcio_group_home(&cx->group);

	/// Initialize the state and command ach channels 
	sns_chan_open (&cx->cmd_chan, opt_cmd_chan, NULL);
	sns_chan_open (&cx->state_chan, opt_state_chan, NULL);

	// Get the group size
	cx->n = pcio_group_size(&cx->group);

	/// Set up the message we will be sending to motors with position, velocity and current values
	setupMessage(cx);
	
	// Set the frequency
	opt_period_sec = 1.0 / opt_frequency;

	// Set the current mode
	SNS_LOG(LOG_INFO, "Full Current: %s\n", opt_full_cur ? "yes" : "no" );
	int r = pcio_group_set_fullcur(&cx->group, opt_full_cur);
	aa_hard_assert(r == NTCAN_SUCCESS, "Failed to set full current\n");
}


/**
 * @function 
 * @brief Set zero velocity command when message is expired
 */
void zero_vel( pciod_t *_cx ) {
   size_t n = _cx->n;
   double ack_vals[n];
   double u[n]; for( size_t i = 0; i < n; ++i ) { u[i] = 0; }
   int got_ack = 0;
   int r;
   pcio_group_t *g = &_cx->group;

   
   // If velocity is already in zero, leave it, otherwise send the command
   // (no need to constantly send messages if you can avoid it)
   int doHalt = 0;
   for( size_t i = 0; i < n; ++i ) {
     if( _cx->state_msg->X[i].vel != 0 ) { doHalt = 1; break; }
   }

   // Set velocity to zero
   if( doHalt == 1 ) {
  
  // printf(" Actually halting - n: %d \n", n);
   r = pcio_group_cmd_ack( g, ack_vals, n, PCIO_FVEL_ACK, u );
    got_ack = 1;
    if( sns_cx.verbosity >= 3 ) { fprintf( stdout, "Setting motor to halt (vel zero) :["); }
    if( r != NTCAN_SUCCESS  ) { pcio_group_dump_error(g); }
  }
}


/* ******************************************************************************************** */
/// READ FROM COMMAND CHANNEL AND CALL EXECUTE IF MESSAGE EXISTS AND IS WELL-FORMED
static void update( pciod_t *cx ) {

	// Compute the absolute time when receive should give up waiting. Note that we need to use 
	// CLOCK_MONOTONIC because ach uses it and amino does not.
	struct timespec currTime;
	clock_gettime( CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(opt_period_sec), currTime);

	// Read current reference from command channel
  const size_t expected_size = sns_msg_motor_ref_size_n(cx->n);
  size_t frame_size = 0;
  ach_status_t r = ach_get(&cx->cmd_chan, cx->ref_msg, expected_size,
                              &frame_size, &abstime, ACH_O_WAIT | ACH_O_LAST);

	// Check message reception
	int good = (((ACH_OK == r || ACH_MISSED_FRAME == r) && cx->ref_msg) 
						 || (ACH_TIMEOUT == r));
	SNS_CHECK(good, LOG_WARNING, 0, "pciod-update: ach result: %s", canResultString(r));

	// If the message has timed out, request an update
	if (r == ACH_TIMEOUT) {

     update_state(cx, NULL);

  //*******************************
  // Check if message is expired in TIMEOUT CASE (Similar to how it is done in can402)
    if( sns_msg_is_expired(&cx->ref_msg->header, &abstime ) ) {

       // Only if previous message was velocity (i.e. not to interrupt position)
       if( cx->ref_msg != NULL ) {
         if( cx->ref_msg->mode == SNS_MOTOR_MODE_VEL ) {
       		zero_vel(cx);
         }
      }
    }
  }
  //*******************************

	// Validate, execute and update the message
	else if((ACH_OK == r || ACH_MISSED_FRAME == r) && cx->ref_msg) {

		// Check if the message has one of the expected parameters
		int goodParam = (SNS_MOTOR_MODE_POS == cx->ref_msg->mode ||
				 SNS_MOTOR_MODE_VEL == cx->ref_msg->mode ||
				 SNS_MOTOR_MODE_CUR == cx->ref_msg->mode ||
				 SNS_MOTOR_MODE_HALT == cx->ref_msg->mode ||
				 SNS_MOTOR_MODE_RESET == cx->ref_msg->mode);
		
		// Check if the command has the right number of parameters if pos, vel or current
		int goodValues = ((cx->ref_msg->u && cx->ref_msg->header.n == cx->n) ||
				SNS_MOTOR_MODE_HALT == cx->ref_msg->mode ||
				SNS_MOTOR_MODE_RESET == cx->ref_msg->mode);

		// Use somatic interface to combine the finalize the checks in case there is an error
		SNS_REQUIRE(goodParam, "invalid motor param, val: %d", cx->ref_msg->mode);
		SNS_REQUIRE(goodValues, "wrong motor count: %d, wanted %d", cx->ref_msg->header.n, cx->n);

		// Execute the command
		execute_and_update_state(cx);
	}
}

/* ******************************************************************************************** */
static void destroy( pciod_t *cx) {
	pcio_group_destroy(&cx->group); 
	ach_close(&cx->cmd_chan);
	ach_close(&cx->state_chan);
	sns_end();
}

/* ******************************************************************************************** */
static void query( pciod_t *cx) {

	// Dump the config or the state based on the query type
	SNS_LOG(LOG_INFO, "Querying:\n");
	if( 0 == strcasecmp("config", opt_query) ){
		pcio_group_dump_config( &cx->group );
		return;
	} else if( 0 == strcasecmp("state", opt_query) ){
		pcio_group_dump_error(&cx->group);
		return;
	}

	// Initialize the memory for the value
	pcio_group_t *g = &cx->group;
	size_t n = pcio_group_size(g);
	double Xd[n];
	uint32_t Xu32[n];

	// Get the parameter value
	if( AA_TYPE_DOUBLE == opt_param_type ) {
		pcio_group_getd( g, (int)opt_param, Xd, n);
	} else if( AA_TYPE_UINT32 == opt_param_type ) {
		pcio_group_getu32( g, (int)opt_param, Xu32, n);
	} else {
		SNS_LOG(LOG_INFO, "ERROR: Unknown type: %d\n", opt_param_type);
		exit(EXIT_FAILURE);
	}

	// Print the values for each module on each specified bus
	size_t k = 0;
	fprintf(stderr, "QUERYING %s\n", opt_query);
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
/// Sets the given parameter value with '-x' for parameter type chosen with '-S'
static void set( pciod_t *cx) {

	// Set the memory
	pcio_group_t *g = &cx->group;
	size_t n = pcio_group_size(g);
	double Xd[n];
	uint32_t Xu32[n];

	// Fill the array backwards based on the parameter type: double or uint32
	size_t k = n-1;
	for(pciod_arg_bus_t *bus = opt_bus; bus; bus=bus->next) {
		for(pciod_arg_mod_t *mod = bus->mod; mod; mod=mod->next) {
			assert(k < n);
			if(AA_TYPE_DOUBLE == opt_param_type) {
				Xd[k] = mod->d;
				SNS_LOG(LOG_INFO, "param %d: %f\n", k, Xd[k]);
			} else if(AA_TYPE_UINT32 == opt_param_type) {
				Xu32[k] = mod->u32;
				SNS_LOG(LOG_INFO, "param %d: %u\n", k, Xu32[k]);
			} else assert(0);
			k--;
		}
	}

	// Set the values
	if(AA_TYPE_DOUBLE == opt_param_type) pcio_group_setd( g, (int)opt_param, Xd, n);
	else if(AA_TYPE_UINT32 == opt_param_type) pcio_group_setu32( g, (int)opt_param, Xu32, n);
	else assert(0);

	// If in verbose mode, query it to show that the set action was successful
	if( sns_cx.verbosity ) {
		opt_query = opt_set;
		query(cx);
	}
}


/* ******************************************************************************************** */
/// Sets the config value
static void set_config( pciod_t *cx) {

	// Get the current config
	pcio_group_t *g = &cx->group;
	size_t n = pcio_group_size(g);
	uint32_t config[n];
	int r = pcio_group_getu32(g, PCIO_PARAM_CONFIG, config, n) ;
	if(0 != r ) {
		fprintf(stderr, "Error retreiving config: %s (%d)\n", canResultString(r), r);
		exit(EXIT_FAILURE);
	}

	// Mask the read config values based on whether we want to enable or disable them
	size_t k = 0;
	for(size_t i = 0; i < g->bus_cnt; i++) {
		for(size_t j = 0; j < g->bus[i].module_cnt; j++) {

			// Sanity check and print for debugging
			assert( k < n );
			SNS_LOG(LOG_INFO, "Initial config bus %d id %d: 0x%x\n",
							   g->bus[i].net, g->bus[i].module[j].id, config[k]);

			// Set the config; either enable/disable
			if(opt_config_enable) config[k] = (config[i] | opt_flag) & 0xFFFFFFF ;
			else if (opt_config_disable) config[k] = (config[i] & ~opt_flag) & 0xFFFFFFF ;
			else assert(0);
			k++;
		}
	}

	// Set the new config value
	r = pcio_group_setu32(g, PCIO_PARAM_CONFIG, config, n) ;
	if( 0 != r ) {
		fprintf(stderr, "Error setting config: %s (%d)\n", canResultString(r), r);
		exit(EXIT_FAILURE);
	}
}

/* ******************************************************************************************** */
/// Listens on the motor command channel, and issue a pcio command for  each incoming message. 
/// When an acknowledgment is received from the module group, post it on the state channel.
/// TODO: In addition, set a blocking timeout, and when it expires issue a state update request to 
/// the modules and update the state channel
static void run( pciod_t *cx) {

	// Send a "running" notice on the event channel
	sns_start();

	// Keep updating
	while (!sns_cx.shutdown) {
		update(cx);
		aa_mem_region_local_release();
	}
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char *argv[]) {

	// ========================================================================
	// Initial setup

	// Set the options for the daemon
	static pciod_t cx;
	memset(&cx,0,sizeof(cx));

	// Parse the arguments
	argp_parse(&argp, argc, argv, 0, NULL, &cx);

	// ========================================================================
	// If any short actions requested, carry them out

	// Output the results of the query
	if (opt_query ) {
		SNS_LOG(LOG_INFO, "Querying %s\n", opt_query);
		init_group(&cx);
		query(&cx);
	}

	// Set the requested value
	else if (opt_set) {
		SNS_LOG(LOG_INFO, "Setting %s\n", opt_set);
		init_group(&cx);
		set(&cx);
	}

	// Reset all the modules
	else if (opt_reset) {
		SNS_LOG(LOG_INFO, "Resetting modules\n");
		init_group(&cx);
		int r = pcio_group_reset(&cx.group);
		SNS_LOG(LOG_INFO, "Resetting status: %s (%d)\n",
						   canResultString(r), r);

	// List the state, config and parameter values
	}else if ( opt_list ) {
		printf("STATE CODES\n-----------\n");
		pcio_code_dump( pcio_state_codes );
		printf("\nCONFIG CODES\n-----------\n");
		pcio_code_dump( pcio_config_codes );
		printf("\nPARAM CODES\n-----------\n");
		pcio_code_dump( pcio_param_codes );
	} 

	// Enable or disable the configuration
	else if (opt_config_enable || opt_config_disable) {
		SNS_LOG(LOG_INFO, "Setting configuration\n");
		init_group(&cx);
		set_config(&cx);
	}

	// ========================================================================
	// Otherwise go into the daemon mode and keep processing commands

	else {

		// Initialize the daemon resources
		init(&cx);

		// Print the frequency, state and command channels
		SNS_LOG(LOG_INFO, "frequency: %f\n", opt_frequency);
		SNS_LOG(LOG_INFO, "state chan: %s\n", opt_state_chan);
		SNS_LOG(LOG_INFO, "cmd chan: %s\n", opt_cmd_chan);

		// Continuously update to get command message and output state 
		run(&cx);

		// Destroy the resources
		destroy(&cx);
	}

	return 0;
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
		SNS_LOG(LOG_INFO, "Filling bus %d, net %d\n", i, g->bus[i].net);

		// Set the bus ids of the modules (in this bus) in the group
		size_t j = g->bus[i].module_cnt - 1;
		for( pciod_arg_mod_t *mod = bus->mod; NULL != mod; mod=mod->next ) {
			g->bus[i].module[j].id = (int)mod->id;
			SNS_LOG(LOG_INFO, "Filling module %d, id %d\n", j, g->bus[i].module[j].id);
			j--;
		}
		i--;
	}

	return 0;
}

/* ******************************************************************************************** */
// Generate the pcio calls requested by the specified motor command message, and update the state 
// with the module acknowledgments. Note: msg size should match group size.
// HANDLE REF, SEND TO HARDWARE
int execute_and_update_state(pciod_t *cx) {

	// Switch on the command flag type and execute the command
	double ack_vals[cx->ref_msg->header.n];
	int got_ack = 0;
	int r;
	pcio_group_t *g = &cx->group;
	switch (cx->ref_msg->mode) {

		// Set the current values
		case SNS_MOTOR_MODE_CUR: {
			double data [cx->n];
			r = pcio_group_cmd_ack(g, ack_vals, cx->ref_msg->header.n, PCIO_FCUR_ACK, cx->ref_msg->u);
			if(sns_cx.verbosity >= 3) fprintf(stdout, "Setting motor currents: [");
			got_ack = 1;
		} break;

		// Set the velocity values after limiting them using the expected time period (?)
		case SNS_MOTOR_MODE_VEL: {
			pcio_group_limit_velocity(g, cx->ref_msg->u, cx->ref_msg->header.n, opt_period_sec);
			r = pcio_group_cmd_ack(g, ack_vals, cx->ref_msg->header.n, PCIO_FVEL_ACK, cx->ref_msg->u);
			got_ack = 1;
			if (sns_cx.verbosity >= 3) fprintf(stdout, "Setting motor velocities: [");
		} break;

		// Set the motor positions after limiting them
		case SNS_MOTOR_MODE_POS: {
			pcio_group_limit_position( g, cx->ref_msg->u, cx->ref_msg->header.n);
			r = pcio_group_setpos_ack( g, cx->ref_msg->u, cx->ref_msg->header.n, 0.5, 4.0, ack_vals);
			got_ack = 1;
			if (sns_cx.verbosity >= 3) fprintf(stdout, "Setting motor positions: [");
		} break;

		// Send a halt message
		case SNS_MOTOR_MODE_HALT: {
			r = pcio_group_halt(g);
			if (sns_cx.verbosity >= 3) fprintf(stdout, "Halting motor: [");
		} break;

		// Send a reset message
		case SNS_MOTOR_MODE_RESET: {
			r = pcio_group_reset(g);
			if (sns_cx.verbosity >= 3) fprintf(stdout, "Resetting motor: [");
		} break;

		// Should not reach here - if does, set success to attempt to continue
		default: {
			SNS_REQUIRE(0, "invalid param: %d", cx->ref_msg->mode);
			if (sns_cx.verbosity >= 3) fprintf(stdout, "default: [");
			r = NTCAN_SUCCESS; 
		} break;
	}

	// If there were not any errors, update the state; otherwise, give an error statement.
	// NOTE: We reuse the position acknowledgement to save some work in updating
	SNS_CHECK(r == NTCAN_SUCCESS, LOG_WARNING, 0, "execute_and_update_state: ntcan result: %s", 
		canResultString(r));
	if(r == NTCAN_SUCCESS) update_state(cx, got_ack ? ack_vals : NULL);
	else pcio_group_dump_error(g);

	// Print the message contents
	if (sns_cx.verbosity >= 3) {
		size_t i;
		for (i = 0; i < cx->ref_msg->header.n; ++i) {
			if (i < cx->ref_msg->header.n-1 ) fprintf(stdout, "%lf::", cx->ref_msg->u[i]);
			else fprintf(stdout, "%lf]\n", cx->ref_msg->u[i]);
		}
	}

	return r;
}

/* ******************************************************************************************** */
/// Issues a request for position and velocity of the modules, and posts on state channel. 
/// If pos_acks is NULL, update_state will query both position and velocity from the network  If 
/// an array is provided, it will pull the position values from this array to save an extra call.
/// SENDS STATE_MSG TO THE ACK 
static void update_state(pciod_t *cx, double *pos_acks) {

	// Set the status of the message
	struct sns_msg_motor_state* msg = cx->state_msg;

	// Set positions into local variable if they are not already provided and place in state_msg
	int r;
	double pos_vals[cx->n];
	if (pos_acks == NULL) {

		// Get the positions
		r = pcio_group_getd( &cx->group, PCIO_ACT_FPOS, pos_vals, cx->n );
		SNS_CHECK(r == NTCAN_SUCCESS, LOG_WARNING, 0, "update_state-pos: ntcan result: %s", 
			canResultString(r));
		if(r == NTCAN_SUCCESS) 
			for(size_t i = 0; i < cx->n; i++) msg->X[i].pos = pos_vals[i];
	} else {
		for(size_t i = 0; i < cx->n; i++) msg->X[i].pos = pos_acks[i];
	}

	// Set velocities into the msg; if failed set the data to zero and update status
	double vel_vals[cx->n];
	r = pcio_group_getd( &cx->group, PCIO_ACT_FVEL, vel_vals, cx->n );
	SNS_CHECK(r == NTCAN_SUCCESS, LOG_WARNING, 0, "update_state-vel: ntcan result: %s", 
		canResultString(r));
	if(r == NTCAN_SUCCESS) 
		for(size_t i = 0; i < cx->n; i++) msg->X[i].vel = vel_vals[i];


	// Set sequence number and time
  cx->state_msg->header.seq++;
	// sns_msg_set_time( &cx->msg_state->header, &cx->now, (int64_t)(opt_timeout_sec*1e9*2) );

	// Set currents into the msg; if failed set the data to zero and update status
//	double cur_vals[cx->n];
//	r = pcio_group_getd( &cx->group, PCIO_ACT_FPSEUDOCURRENT, cur_vals, cx->n );
//	SNS_CHECK(r == NTCAN_SUCCESS, LOG_WARNING, "update_state-cur: ntcan result: %s", 
//		canResultString(r));
//	if(r == NTCAN_SUCCESS)
//	  msg->current->data = cur_vals;
//	} else {
//	  msg->current->data = NULL;
//	  msg->status |= SOMATIC__MOTOR_STATUS__MOTOR_FAIL | SOMATIC__MOTOR_STATUS__MOTOR_COMM_FAIL;
//	}

	// Get status words too and check if any of them have an error. TODO: If failed to get, error.
	//uint32_t status_vals[cx->n];
	//msg->has_status = 1;
	//msg->status = SOMATIC__MOTOR_STATUS__MOTOR_OK;
	//r = pcio_group_getu32( &cx->group, PCIO_PARAM_ERROR, status_vals, cx->n);
	//SNS_CHECK(r == NTCAN_SUCCESS, LOG_WARNING, "update_state-status: ntcan result: %s", 
	//	canResultString(r));
	//if(r == NTCAN_SUCCESS) {
	//	for(size_t j=0; j < cx->n; j++) {
	//		if(status_vals[j] & PCIO_STATE_ERROR) {
	//			msg->status |= SOMATIC__MOTOR_STATUS__MOTOR_FAIL| SOMATIC__MOTOR_STATUS__MOTOR_HW_FAIL;
	//			break;
	//		}
	//	}
	//}

	// Print the message contents
	if (SNS_LOG_PRIORITY(LOG_DEBUG)) {
		size_t i;
		printf("\nseq %d: \n", cx->state_msg->header.seq);
		for (i = 0; i < cx->state_msg->header.n; ++i) {
			printf("%lf, %lf\n", cx->state_msg->X[i].pos, cx->state_msg->X[i].vel); 
		}
	}

	// Package a state message for the ack returned, and send to state channel
	// r = SOMATIC_PACK_SEND( &cx->state_chan, somatic__motor_state, msg );
  r = ach_put( &cx->state_chan, cx->state_msg, sns_msg_motor_state_size(cx->state_msg));

	/// check message transmission
	SNS_CHECK(r == NTCAN_SUCCESS, LOG_WARNING, 0, "update_state: ntcan result: %s", 
		canResultString(r));
}
/* ******************************************************************************************** */
