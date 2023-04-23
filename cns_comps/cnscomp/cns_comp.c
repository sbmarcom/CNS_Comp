/********************************************************************
* Description:  cns_comp.c       
*
* Author: Sam Marcom
* License: GPL Version 2
*
* Copyright (c) 2023	All rights reserved.
*
* Last change:
********************************************************************/

#include "rtapi_math.h"
#include "rtapi.h"			/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"
#include <sys/stat.h>			/* HAL public API decls */

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cns_defs.h"
#include "cns_defs.c"

#define MODNAME "cns_comp"
#define PREFIX "cns_comp"


MODULE_AUTHOR("Sam Marcom");
MODULE_DESCRIPTION("cns component");
MODULE_LICENSE("GPL v2");


//Global Variable Definitions
static int 			comp_id;	
static const char *modname = MODNAME;
static const char *prefix = PREFIX;
const char * resourcefile = "/home/pi/resources/lookuptable.csv";


lookup_table_t g_lookup_table; 
module_functions_t g_function_pointers;
module_methods_t g_current_state;
lookup_table_parameters_t g_parameters;
static hal_data_t *data;
contour_map g_contour;
struct positions g_axes_positions;
process_recovery_t g_process_recovery;


float g_pierce_timer;
float g_preheat_timer;

int l = 0;
int ticker = 0; 

int rtapi_app_main(void)

{
    rtapi_set_msg_level(3);
    rtapi_print_msg(RTAPI_MSG_INFO, "%s teststr \n", modname);
   
   char name[HAL_NAME_LEN + 1];
   int n, retval;
    
   comp_id = hal_init(modname);
    if (comp_id < 0)
	{
		rtapi_print_msg(RTAPI_MSG_INFO, "%s ERROR: hal_init() failed \n", modname);
		return -1;
    }

    data = hal_malloc(sizeof(hal_data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: hal_malloc() failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

    retval = hal_pin_float_newf(HAL_IN, &(data->temp),
			comp_id, "plasmac.cut-feed-rate", prefix);
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
   

    retval = hal_pin_bit_newf(HAL_IN, &(data->spindle_0_is_on),
			comp_id, "%s.spindle_0_is_on", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->spindle_0_stopped),
			comp_id, "%s.spindle_0_stopped", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->spindle_0_at_speed),
			comp_id, "%s.spindle_0_at_speed", prefix);
	if (retval != 0) goto error;


    retval = hal_pin_bit_newf(HAL_IN, &(data->probe_trigger),
			comp_id, "%s.probe_trigger", prefix);
	if (retval != 0) goto error;
   
    retval = hal_pin_bit_newf(HAL_IN, &(data->probe_status),
			comp_id, "%s.probe_status", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->clear_probe_data),
			comp_id, "%s.clear_probe_data", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->pierce_command),
			comp_id, "%s.pierce_command", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data-> all_homed),
			comp_id, "%s.all_homed", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->process_recovery_enable),
			comp_id, "%s.process_recovery_enable", prefix);
	if (retval != 0) goto error;


    retval = hal_pin_bit_newf(HAL_IN, &(data->process_recovery_x_motion),
			comp_id, "%s.process_recovery_x_motion", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->process_recovery_y_motion),
			comp_id, "%s.process_recovery_y_motion", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->process_recovery_resume),
                comp_id, "%s.process_recovery_resume", prefix);
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

    retval = hal_pin_float_newf(HAL_IN, &(data->x_offset_current),
			comp_id, "%s.x_offset_current", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->y_offset_current),
			comp_id, "%s.y_offset_current", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->z_offset_current),
			comp_id, "%s.z_offset_current", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->probe_start_height),
			comp_id, "%s.probe_start_height", prefix);
	if (retval != 0) goto error;    

    retval = hal_pin_float_newf(HAL_IN, &(data->cut_height),
			comp_id, "%s.cut_height", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->cut_feed_rate),
			comp_id, "%s.cut_feed_rate", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->kerf_width),
			comp_id, "%s.kerf_width", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->pierce_height),
			comp_id, "%s.pierce_height", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->probe_feed_rate),
			comp_id, "%s.probe_feed_rate", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->setup_velocity),
			comp_id, "%s.setup_velocity", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->safe_height),
			comp_id, "%s.safe_height", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->pierce_delay),
			comp_id, "%s.pierce_delay", prefix);
	if (retval != 0) goto error;
    
    retval = hal_pin_float_newf(HAL_IN, &(data->new_probe_point_x),
			comp_id, "%s.new_probe_point_x", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->new_probe_point_y),
			comp_id, "%s.new_probe_point_y", prefix);
	if (retval != 0) goto error;

    

    // In u32 pins

    retval = hal_pin_s32_newf(HAL_IN, &(data->module_type),
			comp_id, "%s.module_type", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_IN, &(data->cut_type),
			comp_id, "%s.cut_type", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_u32_newf(HAL_IN, &(data->state_override),
			comp_id, "%s.state_override", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_u32_newf(HAL_IN, &(data->probe_point_count),
			comp_id, "%s.probe_point_count", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_IN, &(data->user_z_motion_command),
			comp_id, "%s.user_z_motion_command", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_IN, &(data->motion_type),
			comp_id, "%s.motion_type", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_IN, &(data->probe_type),
			comp_id, "%s.probe_type", prefix);
	if (retval != 0) goto error;


    //Out Bit Pins

    retval = hal_pin_bit_newf(HAL_OUT, &(data->probe_good),
			comp_id, "%s.probe_good", prefix);
	if (retval != 0) goto error;

   

    retval = hal_pin_bit_newf(HAL_OUT, &(data->active_x_y_offsets),
			comp_id, "%s.active_x_y_offsets", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->clear_eoffsets),
			comp_id, "%s.clear_eoffsets", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->enable_eoffsets),
			comp_id, "%s.enable_eoffsets", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->probe_enable),
			comp_id, "%s.probe_enable", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->feed_hold),
			comp_id, "%s.feed_hold", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->torch_on),
			comp_id, "%s.torch_on", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->process_started),
			comp_id, "%s.process_started", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->program_started),
			comp_id, "%s.program_started", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->process_recovering),
			comp_id, "%s.process_recovering", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->ticker),
			comp_id, "%s.ticker", prefix);
	if (retval != 0) goto error;

    //Out S32 Pins

    retval = hal_pin_s32_newf(HAL_OUT, &(data->x_offset_counts),
			comp_id, "%s.x_offset_counts", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_OUT, &(data->y_offset_counts),
			comp_id, "%s.y_offset_counts", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_OUT, &(data->z_offset_counts),
			comp_id, "%s.z_offset_counts", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_OUT, &(data->state_out),
			comp_id, "%s.state_out", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_OUT, &(data->method_out),
			comp_id, "%s.method_out", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_OUT, &(data->pierce_count),
			comp_id, "%s.pierce_count", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_s32_newf(HAL_OUT, &(data->recovery_path_points),
			comp_id, "%s.recovery_path_points", prefix);
	if (retval != 0) goto error;

    //Out Float Pins

    retval = hal_pin_float_newf(HAL_OUT, &(data->x_offset_scale),
			comp_id, "%s.x_offset_scale", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_OUT, &(data->y_offset_scale),
			comp_id, "%s.y_offset_scale", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_OUT, &(data->z_offset_scale),
			comp_id, "%s.z_offset_scale", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_OUT, &(data->user_in_process_z_offset),
			comp_id, "%s.user_in_process_z_offset", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_OUT, &(data->z_abs_out),
			comp_id, "%s.z_abs_out", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_OUT, &(data->height_from_stock),
			comp_id, "%s.height_from_stock", prefix);
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

    
    
    hal_ready(comp_id);
    //Adding parameters to the parameter table in the correct order
    
    g_lookup_table = create_lookup_table(resourcefile); 
 
    
    rtapi_print_msg(RTAPI_MSG_INFO, " g_parameters pointer  %i \n", &g_parameters);
    rtapi_print_msg(RTAPI_MSG_INFO, " machine on halpin pointer  %i \n", (data -> machine_is_on));

   
    
    // Adding function pointers to the array
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][E_STOP] =                   semi_auto_gc_estop;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][PROGRAM_PAUSED]=            semi_auto_gc_program_paused;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][MACHINE_DISABLED]=          semi_auto_gc_machine_disabled;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][PROBE]=                     semi_auto_gc_probe;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][BEGIN_PROCESS]=             semi_auto_gc_begin_process;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][ACTIVATE_IMPLEMENT]=        semi_auto_gc_torch_on;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][DEACTIVATE_IMPLEMENT]=      semi_auto_gc_torch_off;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][IN_PROCESS_MOTION]=         semi_auto_gc_in_process_motion;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][LINK_PROCESS_STEPS]=        semi_auto_gc_link_process_steps;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][PROCESS_RECOVERY_MOTION]=   semi_auto_gc_cut_recovery_motion;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][PROCESS_RECOVERY_RESUME]=   semi_auto_gc_cut_recovery_resume;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][END_PROCESS]=               semi_auto_gc_end_process;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][INITIALIZE_PROBE]=          semi_auto_gc_initialize_probe;
 

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	
    return 0;
}
void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
void run_state_machine(){
//rtapi_print_msg(RTAPI_MSG_INFO, "entering cns_comp\n");

    if (l==0){
        g_process_recovery.path_points_idx = 0;
        g_contour.points_probed = 0;
        *(data ->probe_good) = 0;
        
        *(data ->active_x_y_offsets) = 0;
        *(data ->clear_eoffsets) = 0;
        *(data ->enable_eoffsets) = 0;
        *(data ->probe_enable) = 0;
        *(data ->feed_hold) = 0;
        *(data ->z_offset_scale ) = .01;
        *(data ->state_out) = 0;
        *(data ->clear_probe_data ) = 0;
        *(data ->process_recovering) = 0;
        *(data ->process_recovery_enable) = 1;
        *(data ->process_started ) = 0;
        *(data ->program_started ) = 0;

        l++;
    }

    
    g_parameters.override =  data -> state_override;
    g_parameters.values[0] = (volatile int *)data -> module_type;
    g_parameters.values[1] = (volatile int *)data -> cut_type;
    g_parameters.values[2] = (volatile int *)data -> external_estop;
    g_parameters.values[3] = (volatile int *)data -> machine_is_on;
    g_parameters.values[4] = (volatile int *)data -> program_is_paused;
    g_parameters.values[5] = (volatile int *)data -> program_is_running;
    g_parameters.values[6] = (volatile int *)data -> process_started;
    g_parameters.values[7] = (volatile int *)data -> program_started;    
    g_parameters.values[8] = (volatile int *)data -> spindle_0_is_on;
    g_parameters.values[9] = (volatile int *)data -> spindle_0_at_speed;
    g_parameters.values[10] = (volatile int *)data -> probe_good;


if (ticker == 0){
    ticker = 1;
}
else{
    ticker =0;
}
*(data->ticker) = ticker;


compute_datum();
compute_positions();
module_methods_t current_method = 0; 
current_method = determine_method(&g_parameters, &g_lookup_table );

//rtapi_print_msg(RTAPI_MSG_INFO, "function call: %i\n", (int) current_method);
while (current_method != NUM_METHODS){
    *(data->method_out) = (int) current_method;
    current_method = g_function_pointers.function_ptrs[0][current_method]();
    //rtapi_print_msg(RTAPI_MSG_INFO, "function call: %i\n", (int) current_method);}

}

}
module_methods_t determine_method(lookup_table_parameters_t *params_l,lookup_table_t *lookup_table_l ){
    int value;
    //For each method, check and see whether the parameters match the entry
    for (int i = 0; i < NUM_METHODS; i++) {
        //Pull out the appropriate entry
        lookup_table_entry_values_t * entry = &(lookup_table_l->entries[i]);
        //Start a count of how many parameters match
        int match_count = 0;
       
        //If the override matches then stop and return the method
        if (*(params_l->override) == (module_methods_t)entry->override){
            rtapi_print_msg(RTAPI_MSG_INFO, " overriding to  %i \n", entry-> entry_method);
            return (module_methods_t)entry->entry_method;
        }     
        //For each parameter go through and see if it is in range, if so increment the match counter
        for (int j = 0; j < PARAMETER_COUNT; j++) {
            value = *(int *)params_l->values[j];
            if ((value >= (entry->values[j][0])) && (value <= (entry->values[j][1]))) {
                match_count++;
            }
            
        }
        if (match_count >= PARAMETER_COUNT ){
            return (module_methods_t)(entry-> entry_method);

        }
    
    }
    //rtapi_print_msg(RTAPI_MSG_INFO, " defaulting to return e_stop \n");
    return NUM_METHODS;
}

module_methods_t semi_auto_gc_estop(void){
    return NUM_METHODS;
}

module_methods_t semi_auto_gc_machine_disabled(void){
    //rtapi_print_msg(RTAPI_MSG_INFO, "In Disabled Function\n");
    *(data->enable_eoffsets) = FALSE;
    *(data->x_offset_counts) = 0;
    *(data->y_offset_counts) = 0;
    *(data->z_offset_counts) = 0;
    *(data->probe_enable) = 0;
    *(data -> state_out) = OFF;
    return NUM_METHODS;
}
module_methods_t semi_auto_gc_probe(void){
    
    switch (*(data -> probe_type)){

        case KEEP_POINTS:
            if (*(data -> probe_point_count)){
                *data->probe_good = TRUE;
                return NUM_METHODS;
            }
            else{
                rtapi_print_msg(RTAPI_MSG_ERR, "NEED PROBE DATA TO KEEP POINTS \n");
                return NUM_METHODS;
            }
        case ON_FIRST_PIERCE:
            if(g_contour.points_probed >= 1){
                *data->probe_good = TRUE;
                return NUM_METHODS;
            }

            else{
                g_contour.probe_points_requested[g_contour.points_probed][0] = g_axes_positions.x_absolute;
                g_contour.probe_points_requested[g_contour.points_probed][1] = g_axes_positions.y_absolute;
                break;
            }
            
        case ON_EACH_PIERCE:
            if(g_contour.points_probed > *data->pierce_count){
                *data->probe_good = TRUE;
                return NUM_METHODS;
            }

            else{
                g_contour.probe_points_requested[g_contour.points_probed][0] = g_axes_positions.x_absolute;
                g_contour.probe_points_requested[g_contour.points_probed][1] = g_axes_positions.y_absolute;
                break;
            }

        case PROBE_HERE:
            //This has the same action here as probe_contours, so no need to duplicate code
            rtapi_print_msg(RTAPI_MSG_ERR, "GOING TO CASE PROBE_CONTOURS\n");
        case PROBE_CONTOURS:
            if( g_contour.points_probed >= g_contour.probe_points_requested_qty){
                break;
            }

            if (*data -> active_x_y_offsets){
                //We'll only end up here if there are enough points but the offsets need to be cleared
                offset_move(ABSOLUTE,COUNTS, 'x', (float)2,(float) 0 );
                offset_move(ABSOLUTE,COUNTS, 'y', (float)2,(float) 0 );
                return NUM_METHODS;
            }
            else{
                *data->probe_good = TRUE;
                return NUM_METHODS;
            }
        case CLEAR_CONTOUR_MAP:
            rtapi_print_msg(RTAPI_MSG_ERR, "INVALID PROBE TYPE FOR RUNNING PROGRAM\n");
            return NUM_METHODS;

    }
    if (!*(data -> enable_eoffsets)){
         *(data -> enable_eoffsets) = TRUE;
         *(data ->z_offset_scale ) = .0001;
         *(data ->x_offset_scale ) = .0001;
         *(data ->y_offset_scale ) = .0001;
    }

    if (*(data -> state_out) == PROBE_SETUP){

        if(g_axes_positions.z_absolute <= (*(data->probe_start_height) + EOFFSET_ERROR_MARGIN) &&
           g_axes_positions.z_absolute >= (*(data->probe_start_height) - EOFFSET_ERROR_MARGIN) &&
           
           g_axes_positions.x_absolute <= (g_contour.probe_points_requested[g_contour.points_probed][0] + EOFFSET_ERROR_MARGIN) &&
           g_axes_positions.x_absolute >= (g_contour.probe_points_requested[g_contour.points_probed][0] - EOFFSET_ERROR_MARGIN) &&

           g_axes_positions.y_absolute <= (g_contour.probe_points_requested[g_contour.points_probed][1] + EOFFSET_ERROR_MARGIN) &&
           g_axes_positions.y_absolute >= (g_contour.probe_points_requested[g_contour.points_probed][1] - EOFFSET_ERROR_MARGIN)
           ) {

            *(data -> state_out) = PROBING;
            *(data -> probe_enable)= TRUE; 
            return NUM_METHODS;
        }
        else{
            offset_move(ABSOLUTE,IN,'z', *(data->probe_feed_rate), *(data->probe_start_height) );
            offset_move(ABSOLUTE,IN,'x', *(data->probe_feed_rate), g_contour.probe_points_requested[g_contour.points_probed][0] );
            offset_move(ABSOLUTE,IN,'y', *(data->probe_feed_rate), g_contour.probe_points_requested[g_contour.points_probed][1] );
            return NUM_METHODS;
        }
    }
    if (*(data -> state_out) == PROBING){
         
        //rtapi_print_msg(RTAPI_MSG_INFO, "in case PROBING\n");
        if( *(data -> probe_trigger)){
            *(data -> probe_enable)= FALSE;
            *(data -> state_out) = PROBE_SETUP;
            add_to_contour_map();
            return NUM_METHODS;
        }

        if (*(data->probe_status)){
            float probe_bottom_height = 0.05;
            int a = offset_move(ABSOLUTE,IN,'z', *(data->probe_feed_rate), probe_bottom_height);
            return NUM_METHODS;
        }
        
        else{
            *(data -> probe_enable)= FALSE;
            
            rtapi_print_msg(RTAPI_MSG_INFO, "TURNING OFF PROBE\n");
            *(data -> state_out) = PROBE_SETUP;
            
            return NUM_METHODS;
        }

    }

    //Write probe location info to struct

    
    *(data -> state_out) = PROBE_SETUP;
    return NUM_METHODS;

    

    }

module_methods_t semi_auto_gc_begin_process(void){
    *(data -> feed_hold)=TRUE;
    if (*(data -> spindle_0_at_speed)){
        if (g_pierce_timer <= 0 ){

            float in_process_height = *(data -> user_in_process_z_offset) + *(data -> cut_height) ;
            if (g_axes_positions.height_from_stock-EOFFSET_ERROR_MARGIN >in_process_height ||
                g_axes_positions.height_from_stock+EOFFSET_ERROR_MARGIN <in_process_height){
                    return IN_PROCESS_MOTION;
                 }
            *(data -> feed_hold)=FALSE;
            *(data -> state_out) = RUNNING;
            *(data -> process_started) = TRUE;
            *(data -> pierce_count)++;
            return NUM_METHODS;
        }
        else{
            g_pierce_timer -= fperiod;
            return NUM_METHODS;            
        } 
    }
    else{
        *(data -> state_out) = STARTING_PROCESS;
        if (g_axes_positions.height_from_stock >= (*(data -> pierce_height)+EOFFSET_ERROR_MARGIN) 
            || g_axes_positions.height_from_stock <= (*(data -> pierce_height)-EOFFSET_ERROR_MARGIN)){
            
            offset_move(FROM_DATUM,IN, 'z',*(data -> setup_velocity), *(data -> pierce_height));
            g_preheat_timer = MAX_PREHEAT_TIME;
            return NUM_METHODS;
        }
        if (*(data-> pierce_command) || g_preheat_timer <= 0){
            g_pierce_timer = *(data -> pierce_delay);
            return ACTIVATE_IMPLEMENT;
        }
        else{
            g_preheat_timer -= fperiod;
            return NUM_METHODS;
        }
            
        }
}
module_methods_t semi_auto_gc_torch_on(void){
    *(data -> torch_on )= TRUE;
    return NUM_METHODS;

}
module_methods_t semi_auto_gc_torch_off(void){
    *(data -> torch_on) = FALSE;
    return NUM_METHODS;
}
module_methods_t semi_auto_gc_in_process_motion(void){
    *(data -> user_in_process_z_offset) += *(data -> user_z_motion_command)*.0001;
    float in_process_height = *(data -> user_in_process_z_offset) + *(data -> cut_height) ;
    offset_move(FROM_DATUM,IN,'z',*(data -> setup_velocity), in_process_height);
    return NUM_METHODS;
    
}
module_methods_t semi_auto_gc_link_process_steps(void){
    if (!*(data -> enable_eoffsets)){
         *(data -> enable_eoffsets) = TRUE;
         *(data ->z_offset_scale ) = .0001;
         *(data ->x_offset_scale ) = .0003;
         *(data ->y_offset_scale ) = .0003;
    }
    
    *(data -> state_out ) = LINKING;
    *(data -> program_started)= TRUE;
    *(data -> process_started)= FALSE;
    *(data -> probe_good)= FALSE;
    if (g_axes_positions.z_absolute+EOFFSET_ERROR_MARGIN <= *(data -> safe_height) ||
        g_axes_positions.z_absolute-EOFFSET_ERROR_MARGIN >= *(data -> safe_height)){
        *(data -> feed_hold) = TRUE;
    }
    else{ 
        *(data -> feed_hold) = FALSE;
    }
    offset_move(ABSOLUTE,IN,'z', *(data -> setup_velocity), *(data -> safe_height));

    if(*data-> torch_on){
        return DEACTIVATE_IMPLEMENT;
    }
    return NUM_METHODS;

}

module_methods_t semi_auto_gc_program_paused(void){
    if (!*data-> process_recovering){
        *data -> probe_good = FALSE;
        *data-> state_out = PAUSED;
        if (*data-> torch_on){
            return DEACTIVATE_IMPLEMENT;
        }
        if (g_axes_positions.z_absolute+EOFFSET_ERROR_MARGIN <= *(data -> safe_height) ){
            offset_move(ABSOLUTE,IN,'z', *(data -> setup_velocity), *(data -> safe_height));
            return NUM_METHODS;
        }
    }

    if (*data-> process_recovery_enable){
        if (*data->process_recovery_resume){
            return PROCESS_RECOVERY_RESUME;
        }
        else{
            return PROCESS_RECOVERY_MOTION;
        }

    }

    return NUM_METHODS;
}
module_methods_t semi_auto_gc_cut_recovery_motion(void){
    *data->state_out = RECOVERING;
    *data->process_recovering = TRUE;
    if (!*data->enable_eoffsets){
        *data->enable_eoffsets= TRUE;
        *data ->x_offset_scale = .0001;
        *data ->y_offset_scale = .0001;
    }

    if (*data->process_recovery_x_motion ||*data->process_recovery_y_motion  ){
        int x_counts = g_process_recovery.path_points[g_process_recovery.path_points_idx][0];
        int y_counts = g_process_recovery.path_points[g_process_recovery.path_points_idx][1];
        g_process_recovery.path_points_idx ++;
        *data->recovery_path_points = g_process_recovery.path_points_idx;
        g_process_recovery.path_points[g_process_recovery.path_points_idx][0] = x_counts + *data->process_recovery_x_motion;
        g_process_recovery.path_points[g_process_recovery.path_points_idx][1] = y_counts + *data->process_recovery_y_motion;
    }

    offset_move(ABSOLUTE,COUNTS,'x',(float)2, (float)g_process_recovery.path_points[g_process_recovery.path_points_idx][0]);
    offset_move(ABSOLUTE,COUNTS,'y',(float)2, (float)g_process_recovery.path_points[g_process_recovery.path_points_idx][1]);

    return NUM_METHODS;
}
module_methods_t semi_auto_gc_cut_recovery_resume(void){
    /*
    if (!*data -> probe_good){
        return PROBE;
    }
    if (!*data->torch_on || *data->feed_hold){
        return BEGIN_PROCESS;
    }
    */

    if (*data->x_offset_counts == g_process_recovery.path_points[g_process_recovery.path_points_idx][0] &&
        *data->y_offset_counts == g_process_recovery.path_points[g_process_recovery.path_points_idx][1]){

        if (g_process_recovery.path_points_idx ==0){
            *data->process_recovering= FALSE;  
            return IN_PROCESS_MOTION;
        } 
        g_process_recovery.path_points_idx --;
        *data->recovery_path_points = g_process_recovery.path_points_idx;

        }

    offset_move(ABSOLUTE,COUNTS,'x',(float)2, (float)g_process_recovery.path_points[g_process_recovery.path_points_idx][0]);
    offset_move(ABSOLUTE,COUNTS,'y',(float)2, (float)g_process_recovery.path_points[g_process_recovery.path_points_idx][1]);

    return IN_PROCESS_MOTION;
}

module_methods_t semi_auto_gc_end_process(void){
    *(data -> state_out)= ENDING_CUT;
    *(data -> process_started)= FALSE;
    if (g_axes_positions.height_from_stock-EOFFSET_ERROR_MARGIN <= *(data -> safe_height) && 
        g_axes_positions.height_from_stock+EOFFSET_ERROR_MARGIN >= *(data -> safe_height)){
            *(data -> state_out)= IDLE;
            *(data -> program_started)= FALSE;
            return NUM_METHODS;
    }
    offset_move(FROM_DATUM,IN,'z', *(data -> setup_velocity), *(data -> safe_height));
    if(*data-> torch_on){
        return DEACTIVATE_IMPLEMENT;
    }
    return NUM_METHODS;

    
    
    offset_move(FROM_DATUM,IN,'z', *(data -> setup_velocity), *(data -> safe_height));
    return NUM_METHODS;
}


module_methods_t semi_auto_gc_initialize_probe(void){
    switch (*(data -> probe_type)){
        case KEEP_POINTS:
            return NUM_METHODS;
        case ON_FIRST_PIERCE:
            clear_contour_map();
            return NUM_METHODS;
            
        case ON_EACH_PIERCE:
            clear_contour_map();
            return NUM_METHODS;
            
        case PROBE_HERE:
            for (int i = 0; i < g_contour.probe_points_requested_qty; i++) {
                if (fabs(*(data ->new_probe_point_x)- g_contour.probe_points_requested[i][0]) <= EOFFSET_ERROR_MARGIN &&
                    fabs(*(data ->new_probe_point_y) - g_contour.probe_points_requested[i][1]) <= EOFFSET_ERROR_MARGIN) {
                        rtapi_print_msg(RTAPI_MSG_INFO, "THIS POINT ALREADY IN BUFFER\n");
                        return NUM_METHODS;
                    }      
            }

            g_contour.probe_points_requested[g_contour.probe_points_requested_qty][0] = *(data ->new_probe_point_x);
            g_contour.probe_points_requested[g_contour.probe_points_requested_qty][1] = *(data ->new_probe_point_y);

            g_contour.probe_points_requested_qty++;
            return NUM_METHODS;


        case PROBE_CONTOURS:

            for (int i = 0; i < g_contour.probe_points_requested_qty; i++) {
                if (fabs(*(data ->new_probe_point_x)- g_contour.probe_points_requested[i][0]) <= EOFFSET_ERROR_MARGIN &&
                    fabs(*(data ->new_probe_point_y) - g_contour.probe_points_requested[i][1]) <= EOFFSET_ERROR_MARGIN) {
                        rtapi_print_msg(RTAPI_MSG_INFO, "THIS POINT ALREADY IN BUFFER\n");
                        return NUM_METHODS;
                    }      
            }

            g_contour.probe_points_requested[g_contour.probe_points_requested_qty][0] = *(data ->new_probe_point_x);
            g_contour.probe_points_requested[g_contour.probe_points_requested_qty][1] = *(data ->new_probe_point_y);

            g_contour.probe_points_requested_qty++;
            return NUM_METHODS;
        case CLEAR_CONTOUR_MAP: 
            clear_contour_map();
            return NUM_METHODS;
    }

    return NUM_METHODS;


}
void add_to_contour_map(void){
    g_contour.point_data[(g_contour.points_probed)][0] = g_axes_positions.x_absolute;
    g_contour.point_data[(g_contour.points_probed)][1] = g_axes_positions.y_absolute;
    g_contour.point_data[(g_contour.points_probed)][2] = g_axes_positions.z_absolute;
    g_contour.points_probed +=1;
}

void clear_contour_map(void){
    g_contour.points_probed =0;
    g_contour.probe_points_requested_qty =0;
}

int offset_move(motion_types motion_type, units_t units,  char axis,float velocity, float target){
    float position=0;
    int counts_per_cycle=0;
    float error_margin=0;
    float motion_per_cycle=0; 
    switch (axis){
     case 'x':
        switch (units){
            case MM:
                counts_per_cycle = (int) round((velocity *fperiod)/(*(data-> x_offset_scale)));
                position = g_axes_positions.x_absolute;
                error_margin = EOFFSET_ERROR_MARGIN *25.4;
                motion_per_cycle = velocity*fperiod;
                break;

        
            case IN:
                counts_per_cycle = (int) round((velocity *fperiod)/(*(data-> x_offset_scale)));
                position = g_axes_positions.x_absolute;
                error_margin = EOFFSET_ERROR_MARGIN;
                motion_per_cycle = velocity*fperiod;
                break;

            case COUNTS:
                counts_per_cycle = velocity;
                position = *data -> x_offset_counts;
                error_margin = 0;
                break;
        }

        switch (motion_type){

            case INCREMENTAL:
                *(data-> x_offset_counts)+= counts_per_cycle;
                return 0;

            case ABSOLUTE:
                if (((position-error_margin)) >= target){
                    *(data -> x_offset_counts) -= counts_per_cycle;
                    return 0;
                }
                if ((position+error_margin) <= target){
                    *(data -> x_offset_counts) += counts_per_cycle;         
                return 0;
                }
                
 
        
        }
        return 0;
    
    case 'y':
        switch(units){
            case MM:
                counts_per_cycle = (int) round((velocity *fperiod)/(*(data-> y_offset_scale)));
                position = g_axes_positions.y_absolute;
                error_margin = EOFFSET_ERROR_MARGIN *25.4;
                motion_per_cycle = velocity*fperiod;
                break;
        
            case IN:
                counts_per_cycle = (int) round((velocity *fperiod)/(*(data-> y_offset_scale)));
                position = g_axes_positions.y_absolute;
                error_margin = EOFFSET_ERROR_MARGIN;
                motion_per_cycle = velocity*fperiod;
                break;

            case COUNTS:
                counts_per_cycle = velocity;
                position = *data -> y_offset_counts;
                error_margin = 0;
                motion_per_cycle = counts_per_cycle;
                break;
        }
        switch (motion_type){
            case INCREMENTAL:
                *(data-> y_offset_counts)+= counts_per_cycle;
                return 0;

            case ABSOLUTE:

                if (position-error_margin >= target){
                    *(data -> y_offset_counts) -= counts_per_cycle;
                    return 0;
                }
                if (position+error_margin <= target){
                    *(data -> y_offset_counts) += counts_per_cycle;
                    return 0;
                }
                
        }
        return 0;
    
    case 'z':
        switch(units){
            case MM:
                counts_per_cycle = (int) round((velocity *fperiod)/(*(data-> z_offset_scale)));
                position = g_axes_positions.z_absolute;
                error_margin = EOFFSET_ERROR_MARGIN *25.4;
                motion_per_cycle = velocity*fperiod;
                break;

            case IN:
                counts_per_cycle = 2;//(int) round((velocity *fperiod)/(*(data-> z_offset_scale)));
                position = g_axes_positions.z_absolute;
                *(data -> z_abs_out) = position;
                error_margin = EOFFSET_ERROR_MARGIN;
                motion_per_cycle = velocity*fperiod;
                break;
                

            case COUNTS:
                counts_per_cycle = velocity;
                position = *data -> z_offset_counts;
                error_margin = 0;
                break;
        }

        //rtapi_print_msg(RTAPI_MSG_INFO, "counts_per_cycle %i\n",counts_per_cycle)
        
        switch (motion_type){

            case INCREMENTAL:
                *(data-> z_offset_counts)+= counts_per_cycle;
                return 0;

            case ABSOLUTE:
                
                if (position-error_margin >= target){
                    *(data -> z_offset_counts) -= counts_per_cycle; 
                    return 0 ; 
                }
                if (position+error_margin <= target){
                    *(data -> z_offset_counts) += counts_per_cycle;
                }
                return 0;
               
                
            case FROM_DATUM:
                position = g_axes_positions.height_from_stock;
                if (position-motion_per_cycle >= target){
                    *(data -> z_offset_counts) -= counts_per_cycle;
                }
                if (position+motion_per_cycle <= target){
                    *(data -> z_offset_counts) += counts_per_cycle;
                }
                return 0 ;
            
        }  
    }
}

void compute_datum(void){
    //rtapi_print_msg(RTAPI_MSG_INFO, "entering compute_datum %i \n", *(data -> state_out));
    if (g_contour.points_probed != 0){
        //will add contour mapping math here
            g_axes_positions.stock_height = g_contour.point_data[0][2];
    }
    else{
        g_axes_positions.stock_height = 0;
    }
    //rtapi_print_msg(RTAPI_MSG_INFO, "exiting compute_datum %i \n", *(data -> state_out));
}

void compute_positions(void){
    //rtapi_print_msg(RTAPI_MSG_INFO, "entering compute_positions %i \n", *(data -> state_out));
    g_axes_positions.x_absolute = *(data-> axis_x_position) + *(data-> x_offset_current);
    //rtapi_print_msg(RTAPI_MSG_INFO, "absolute x %f \n", g_axes_positions.x_absolute);
    g_axes_positions.y_absolute = *(data-> axis_y_position) + *(data-> y_offset_current);
    //rtapi_print_msg(RTAPI_MSG_INFO, "absolute y %f \n", g_axes_positions.y_absolute);
    g_axes_positions.z_absolute = *(data-> axis_z_position) + *(data-> z_offset_current);
    //*(data -> z_abs_out) = g_axes_positions.z_absolute;
    g_axes_positions.height_from_stock = *(data-> axis_z_position) + *(data-> z_offset_current)-g_axes_positions.stock_height;
    *(data -> height_from_stock) = g_axes_positions.height_from_stock;

    if (*data-> x_offset_counts || *data-> y_offset_counts){
        *data->active_x_y_offsets = TRUE;
    }
    else{
        *data->active_x_y_offsets = FALSE;
    }

    
    //rtapi_print_msg(RTAPI_MSG_INFO, "datum z %f \n", g_axes_positions.height_from_stock);
    
}

lookup_table_t create_lookup_table(const char *  filename){
   lookup_table_entry_values_t entry[NUM_METHODS];
   rtapi_print_msg(RTAPI_MSG_INFO, "made entry \n");

   FILE *file = fopen(filename,"r");
   if (file == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Error opening file\n");
   }
  
    char line[256];
    int row = 0;
    while(fgets(line,sizeof(line),file)){
        //rtapi_print_msg(RTAPI_MSG_INFO, "row %i %s \n", row,line );
        char *token ;
        token = strtok(line, ",");
        entry[row].entry_method = (module_methods_t)atoi(token);

        token = strtok(NULL, ",");
        entry[row].override = (module_methods_t)atoi(token);

        // Parse required_matches

        // Parse values
        for (int i = 0; i < PARAMETER_COUNT; i++) {
            token = strtok(NULL, ",");
            entry[row].values[i][0] = atoi(token);
            token = strtok(NULL, ",");
            entry[row].values[i][1] = atoi(token);
        }

        row++;
    }

    lookup_table_t lookup_table;
    for (int i = 0; i < NUM_METHODS; i++) {
        
        lookup_table.entries[i] = entry[i];

    }
    return lookup_table;
}

