/********************************************************************
* Description:  cns_comp.c
*               
*
* Author: Sam Marcom
* License: GPL Version 2
*
*
* Copyright (c) 2023	All rights reserved.
*
* Last change:
********************************************************************/

#include "rtapi_math.h"
#include "rtapi.h"			/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"			/* HAL public API decls */

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cns_defs.h"
//#include "cns_defs.c"

#define MODNAME "cnscomp"
#define PREFIX "cnscomp"


MODULE_AUTHOR("Sam Marcom");
MODULE_DESCRIPTION("cns component");
MODULE_LICENSE("GPL v2");


//Global Variable Definitions
static int 			comp_id;	
static const char *modname = MODNAME;
static const char *prefix = PREFIX;
const char * resourcefile = "~/resources/lookuptable.csv";


lookup_table_t g_lookup_table; 
module_functions_t g_function_pointers;
module_methods_t g_current_state;
lookup_table_parameters_t g_parameters;
static hal_data_t *data;


lookup_table_t create_lookup_table(const char *  filename){
   lookup_table_entry_values_t entry[NUM_METHODS];
   FILE *file = fopen(filename,"r");
  
    char line[256];
    int row = 0;
    while(fgets(line,sizeof(line),file)){

        char *token ;
        token = strtok(line, ",");
        entry[row].entry_method = (module_methods_t)row;

        token = strtok(NULL, ",");
        entry[row].override = (module_methods_t)row;

        // Parse required_matches
        token = strtok(NULL, ",");
        entry[row].required_matches = atoi(token);

        // Parse values
        for (int i = 0; i < PARAMETER_COUNT; i++) {
            for (int j = 0; j < 2; j++) {
                token = strtok(NULL, ",");
                entry[row].values[i][j] = atoi(token);
            }
        }

        row++;
    }

    lookup_table_t lookup_table;
    for (int i = 0; i < NUM_METHODS; i++) {
        lookup_table.entries[i] = entry[i];
    

    }
    return lookup_table;
}

int rtapi_app_main(void)
{
   char name[HAL_NAME_LEN + 1];
   int n, retval;
    
   comp_id = hal_init(modname);
    if (comp_id < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
		return -1;
    }

    data = hal_malloc(sizeof(hal_data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: hal_malloc() failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

    retval = hal_pin_bit_newf(HAL_IN, &(data->cut_recovery),
			comp_id, "%s.cut_recovery", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->external_estop),
			comp_id, "%s.external_estop", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->machine_is_on),
			comp_id, "%s.machine_is_on", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->program_is_idle),
			comp_id, "%s.program_is_idle", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->program_is_paused),
			comp_id, "%s.program_is_paused", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->program_is_running),
			comp_id, "%s.program_is_running", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->resume_cut),
			comp_id, "%s.resume_cut", prefix);
	if (retval != 0) goto error;


    retval = hal_pin_bit_newf(HAL_IN, &(data->motion_type),
			comp_id, "%s.motion_type", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->spindle_0_is_on),
			comp_id, "%s.spindle_0_is_on", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->spindle_0_stopped),
			comp_id, "%s.spindle_0_stopped", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->spindle_0_at_speed),
			comp_id, "%s.spindle_0_at_speed", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->resume),
			comp_id, "%s.resume", prefix);
	if (retval != 0) goto error;

    

    ///In Float Pins

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_x_max_limit),
			comp_id, "%s.axis_x_max_limit", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_x_min_limit),
			comp_id, "%s.axis_x_min_limit", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_x_position),
			comp_id, "%s.axis_x_position", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_y_max_limit),
			comp_id, "%s.axis_y_max_limit", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_y_min_limit),
			comp_id, "%s.axis_y_min_limit", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_y_position),
			comp_id, "%s.axis_y_position", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_z_max_limit),
			comp_id, "%s.axis_z_max_limit", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_z_min_limit),
			comp_id, "%s.axis_z_min_limit", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->axis_z_position),
			comp_id, "%s.axis_z_position", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_u32_newf(HAL_IN, &(data->module_type),
			comp_id, "%s.module_type", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_u32_newf(HAL_IN, &(data->cut_type),
			comp_id, "%s.cut_type", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_u32_newf(HAL_IN, &(data->state_override),
			comp_id, "%s.state_override", prefix);
	if (retval != 0) goto error;

    //Out Pins

    retval = hal_pin_bit_newf(HAL_OUT, &(data->probe_good),
			comp_id, "%s.probe_good", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->recovering),
			comp_id, "%s.recovering", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->active_x_y_offsets),
			comp_id, "%s.active_x_y_offsets", prefix);
	if (retval != 0) goto error;

    error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: pin export failed with err=%i\n",
		        modname, retval);
		hal_exit(comp_id);
		return -1;
	}


    rtapi_snprintf(name, sizeof(name), "%s.run_state_machine", prefix);
	retval = hal_export_funct(name, run_state_machine, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: state machine function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
    return 0;

    g_lookup_table = create_lookup_table(resourcefile); 
    g_parameters.override = data->state_override;
    g_parameters.values[0] = data -> module_type;
    g_parameters.values[1] = data -> cut_type;
    g_parameters.values[2] = data -> motion_type;
    g_parameters.values[3] = data -> program_is_idle;
    g_parameters.values[4] = data -> program_is_paused;
    g_parameters.values[5] = data -> program_is_running;
    g_parameters.values[6] = data -> machine_is_on;
    g_parameters.values[7] = data -> spindle_0_is_on;
    g_parameters.values[8] = data -> spindle_0_stopped;
    g_parameters.values[9] = data -> spindle_0_at_speed;
    g_parameters.values[10] = data -> probe_good;
    g_parameters.values[11] = data -> recovering;
    g_parameters.values[12] = data -> active_x_y_offsets;
    g_parameters.values[13] = data -> resume;

    g_function_pointers.function_ptrs[SEMI_AUTO_GC][E_STOP] = semi_auto_gc_estop;
}


void rtapi_app_exit(void)
{
    //hal_exit("cnscomp");
}


void run_state_machine(void){
module_methods_t g_current_state; 
g_current_state = determine_state(&g_parameters, &g_lookup_table );

while (g_current_state != NUM_METHODS){
    g_current_state = g_function_pointers.function_ptrs[0][g_current_state]();
}



}

module_methods_t determine_state(lookup_table_parameters_t *params_l,lookup_table_t *lookup_table_l ){

    for (int i = 0; i < NUM_METHODS; i++) {
        lookup_table_entry_values_t *entry = &lookup_table_l->entries[i];
        int match_count = 0;

        if (params_l->override == (module_methods_t *)entry->override){
            return entry->entry_method;
        }

        for (int j = 0; j < PARAMETER_COUNT; j++) {
            int value = *(int *)params_l->values[j];
            if (value >= entry->values[j][0] && value <= entry->values[j][1]) {
                match_count++;
            }
        
        }
    if (match_count>= (PARAMETER_COUNT- (entry->required_matches) ) )
        return entry-> entry_method;

    }
    return E_STOP;

}

module_methods_t semi_auto_gc_estop(void){}