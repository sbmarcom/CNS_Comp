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
#include "hal.h"			/* HAL public API decls */

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
const char * resourcefile = "~/resources/lookuptable.csv";


lookup_table_t g_lookup_table; 
module_functions_t g_function_pointers;
module_methods_t g_current_state;
lookup_table_parameters_t g_parameters;
static hal_data_t *data;
machine_state g_state = WAITING;
contour_map g_contour;
struct positions g_axes_positions;

float g_pierce_timer;
float g_preheat_timer;



lookup_table_t create_lookup_table(const char *  filename){
   lookup_table_entry_values_t entry[NUM_METHODS];
   FILE *file = fopen(filename,"r");
  
    char line[256];
    int row = 0;
    while(fgets(line,sizeof(line),file)){
        rtapi_print_msg(RTAPI_MSG_INFO, "row %i %s \n", row,line );
        char *token ;
        token = strtok(line, ",");
        entry[row].entry_method = (module_methods_t)token;
        
        token = strtok(NULL, ",");
        entry[row].override = (module_methods_t)token;

        // Parse required_matches
        token = strtok(NULL, ",");
        entry[row].required_matches = atoi(token);

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

    retval = hal_pin_bit_newf(HAL_IN, &(data->temp),
			comp_id, "plasmac.cut-feed-rate", prefix);
	if (retval != 0) goto error;

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

    retval = hal_pin_bit_newf(HAL_IN, &(data->probe_trigger),
			comp_id, "%s.probe_trigger", prefix);
	if (retval != 0) goto error;
    /*
    retval = hal_pin_bit_newf(HAL_IN, &(data->add_probe_point),
			comp_id, "%s.add_probe_point", prefix);
	if (retval != 0) goto error;
    */
    retval = hal_pin_bit_newf(HAL_IN, &(data->clear_probe_data),
			comp_id, "%s.probe_status", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_IN, &(data->pierce_command),
			comp_id, "%s.pierce_command", prefix);
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
    /*

    retval = hal_pin_float_newf(HAL_IN, &(data->new_probe_point_x),
			comp_id, "%s.new_probe_point_x", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_float_newf(HAL_IN, &(data->new_probe_point_y),
			comp_id, "%s.new_probe_point_y", prefix);
	if (retval != 0) goto error;

    */

    // In u32 pins

    retval = hal_pin_u32_newf(HAL_IN, &(data->module_type),
			comp_id, "%s.module_type", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_u32_newf(HAL_IN, &(data->cut_type),
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

    //Out Bit Pins

    retval = hal_pin_bit_newf(HAL_OUT, &(data->probe_good),
			comp_id, "%s.probe_good", prefix);
	if (retval != 0) goto error;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->recovering),
			comp_id, "%s.recovering", prefix);
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

    
    

    //Adding parameters to the parameter table in the correct order
    //g_lookup_table = create_lookup_table(resourcefile); 
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

    g_contour.num_points_requested    = data -> probe_point_count;
    g_contour.point_data = malloc(10 * sizeof(float *)); // Allocating memory for 10 pointers to float arrays  

    rtapi_print_msg(RTAPI_MSG_INFO, "%s made it past assiging parameters \n", modname);

    if (!g_contour.num_points_requested || !g_contour.point_data) {
        fprintf(stderr, "Failed to allocate memory\n");
        hal_exit(comp_id);
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s made it past malloc\n", modname);
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
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][PROCESS_RECOVERY_RESET]=    semi_auto_gc_cut_recovery_reset;
    g_function_pointers.function_ptrs[SEMI_AUTO_GC][END_PROCESS]=               semi_auto_gc_end_process;
    //g_function_pointers.function_ptrs[SEMI_AUTO_GC][IDLE]=                      semi_auto_gc_idle;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
    return 0;
}


void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}


void run_state_machine(){


module_methods_t current_method; 
current_method = determine_method(&g_parameters, &g_lookup_table );

while (current_method != NUM_METHODS){
    current_method = g_function_pointers.function_ptrs[0][current_method]();
}



}


module_methods_t determine_method(lookup_table_parameters_t *params_l,lookup_table_t *lookup_table_l ){
    //For each method, check and see whether the parameters match the entry
    for (int i = 0; i < NUM_METHODS; i++) {
        //Pull out the appropriate entry
        lookup_table_entry_values_t *entry = &lookup_table_l->entries[i];
        //Start a count of how many parameters match
        int match_count = 0;

        //If the override matches then stop and return the method
        if (params_l->override == (module_methods_t *)entry->override){
            return entry->entry_method;
        }

        //For each parameter go through and see if it is in range, if so increment the match counter
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





module_methods_t semi_auto_gc_estop(void){
    return MACHINE_DISABLED;
}
module_methods_t semi_auto_gc_program_paused(void){

    return MACHINE_DISABLED;
}

module_methods_t semi_auto_gc_machine_disabled(void){
    *(data->enable_eoffsets) = FALSE;
    *(data->x_offset_counts) = FALSE;
    *(data->y_offset_counts) = FALSE;
    *(data->z_offset_counts) = FALSE;
    g_state = DISABLED;
    return NUM_METHODS;
}


module_methods_t semi_auto_gc_probe(void){
    if( *(data -> clear_probe_data)){
        clear_contour_map();
        return NUM_METHODS;
    }
    if( *(data -> probe_good)){
        return NUM_METHODS;
    }
    else{
    *(data -> enable_eoffsets) = TRUE;
    switch(g_state){

        case PROBE_SETUP:
            if(g_axes_positions.z_absolute <= (*(data->probe_start_height)+EOFFSET_ERROR_MARGIN) &&  g_axes_positions.z_absolute >= (*(data->probe_start_height)-EOFFSET_ERROR_MARGIN)){
                g_state = PROBING;
                *(data -> probe_enable)= TRUE; 
            }
            else{
                offset_move(ABSOLUTE,'z', *(data->probe_feed_rate), (float) 0 );
                return NUM_METHODS;
            }
            
        case PROBING:
            if( *(data -> probe_trigger)){
                *(data -> probe_enable)= FALSE;
                break;
            }
            if (*(data -> probe_status)){
                //CONTINE TO OFFSET MOVE DOWN
                return NUM_METHODS;
            }
            else{
                *(data -> probe_enable)= FALSE;
                //PRINT ERROR  
                return NUM_METHODS;
            }
            
    }

    add_to_contour_map();
    //Write probe location info to struct

    if(g_contour.points_probed != *(g_contour.num_points_requested) ){
        g_state = PROBE_SETUP;
        return NUM_METHODS;

    }
    
    else{
        *(data -> probe_good)= TRUE;
        return NUM_METHODS;

    }


    }

}

module_methods_t semi_auto_gc_begin_process(void){
    *(data -> feed_hold)=TRUE;

    if (*(data -> spindle_0_at_speed)){
        if (g_pierce_timer <=0){
            *(data -> feed_hold)=FALSE;
            g_state = RUNNING_PROCESS;
        }
        else{
            g_pierce_timer -= fperiod;
            
        }
        return NUM_METHODS;
    }
    else{
        if (g_axes_positions.height_from_stock >= (*(data -> pierce_height)+EOFFSET_ERROR_MARGIN) || g_axes_positions.height_from_stock <= (*(data -> pierce_height)-EOFFSET_ERROR_MARGIN)){
            offset_move(FROM_DATUM, 'z',*(data -> setup_velocity), *(data -> pierce_height) );
            g_preheat_timer = MAX_PREHEAT_TIME;
        }
        if (*data-> pierce_command || g_preheat_timer <= 0){
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
    return NUM_METHODS;

}
module_methods_t semi_auto_gc_torch_off(void){
    return NUM_METHODS;
}

module_methods_t semi_auto_gc_in_process_motion(void){
    *(data -> user_in_process_z_offset) += *(data -> user_z_motion_command);
    float in_process_height = *(data -> user_in_process_z_offset)/ *(data -> z_offset_scale) + *(data -> cut_height) ;
    offset_move(FROM_DATUM,'z',*(data -> setup_velocity), in_process_height);
    return NUM_METHODS;
    
}
module_methods_t semi_auto_gc_link_process_steps(void){
    if (g_axes_positions.height_from_stock+EOFFSET_ERROR_MARGIN <= *(data -> safe_height) ){
        *(data -> feed_hold) = TRUE;

    }
    else{ 
        *(data -> feed_hold) = FALSE;
    }
    offset_move(FROM_DATUM,'z', *(data -> setup_velocity), *(data -> safe_height));
    return NUM_METHODS;

}
module_methods_t semi_auto_gc_cut_recovery_motion(void){}
module_methods_t semi_auto_gc_cut_recovery_reset(void){}
module_methods_t semi_auto_gc_end_process(void){}

module_methods_t semi_auto_gc_idle(void){
    return NUM_METHODS;
}

void add_to_contour_map(void){
    g_contour.point_data[(g_contour.points_probed)] = malloc(2 * sizeof(float));
    g_contour.point_data[(g_contour.points_probed)][0] = *(data-> axis_x_position);
    g_contour.point_data[(g_contour.points_probed)][1] = *(data-> axis_y_position);
    g_contour.points_probed +=1;
}

void clear_contour_map(void){
    for (int i = 0; i < g_contour.points_probed; i++) {
        free(g_contour.point_data[i]);
        
    }
    g_contour.points_probed =0;


}

int offset_move(motion_types motion_type, char axis, float velocity, float target){
    float position;
    int counts_per_cycle;
    switch (axis){
     case 'x':
        position = g_axes_positions.x_absolute;
        counts_per_cycle = (int) round(velocity * (*(data-> x_offset_scale))*fperiod);
        if (motion_type == INCREMENTAL ){
            *(data-> x_offset_counts)+= counts_per_cycle;
        }

        if (motion_type == ABSOLUTE){
            if (position-velocity*fperiod >= target){
                *(data -> x_offset_counts) -= counts_per_cycle;
                return 0;
            }
            if (position+velocity*fperiod <= target){
                *(data -> x_offset_counts) += counts_per_cycle;
                return 0;
            }

            else{
                return 0;
            }
        }
    
    case 'y':
        position = g_axes_positions.y_absolute;
        counts_per_cycle = (int) round(velocity * (*(data-> y_offset_scale))*fperiod);
        if (motion_type == INCREMENTAL ){
            *(data-> y_offset_counts)+= counts_per_cycle;
        }

        if (motion_type == ABSOLUTE){
            if (position-velocity*fperiod > target){
                *(data -> y_offset_counts) -= counts_per_cycle;
                return 0;
            }
            if (position+velocity*fperiod < target){
                *(data -> y_offset_counts) += counts_per_cycle;
                return 0;
            }

            else{
                return 0;
            }
        }
        
    
    case 'z':
        position = g_axes_positions.z_absolute;
        counts_per_cycle = (int) round(velocity * (*(data-> z_offset_scale))*fperiod);
        if (motion_type == INCREMENTAL ){
            *(data-> z_offset_counts)+= counts_per_cycle;
        }

        if (motion_type == ABSOLUTE){
            if (position-velocity*fperiod >= target){
                *(data -> z_offset_counts) -= counts_per_cycle;
                return 0;
            }
            if (position+velocity*fperiod <= target){
                *(data -> z_offset_counts) += counts_per_cycle;
                return 0;
            }

            else{
                return 0;
            }
        }
        if (motion_type == FROM_DATUM ){
            position = g_axes_positions.height_from_stock;
            if (position-velocity*fperiod >= target){
                *(data -> z_offset_counts) -= counts_per_cycle;
                return 0;
            }
            if (position+velocity*fperiod <= target){
                *(data -> z_offset_counts) += counts_per_cycle;
                return 0;
            }

            else{
                return 0;
            }
        }
        
    
    }
}

void compute_datum(void){
    if (g_contour.points_probed){
            g_axes_positions.stock_height = (g_contour.point_data[0][2]);


    }


    else{
        g_axes_positions.stock_height = 0;
    }
}

void compute_positions(void){
    g_axes_positions.x_absolute = *(data-> axis_x_position) + *(data-> x_offset_current);
    g_axes_positions.y_absolute = *(data-> axis_y_position) + *(data-> y_offset_current);
    g_axes_positions.z_absolute = *(data-> axis_z_position) + *(data-> z_offset_current);
    g_axes_positions.height_from_stock = *(data-> axis_z_position) + *(data-> z_offset_current)+g_axes_positions.stock_height;
}