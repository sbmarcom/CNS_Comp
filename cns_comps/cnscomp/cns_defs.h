#ifndef CNS_DEFS_H
#define CNS_DEFS_H

#define fperiod .002
#define EOFFSET_ERROR_MARGIN .01
#define MAX_PREHEAT_TIME 100

#define PARAMETER_COUNT 15

enum tf{
    FALSE,
    TRUE
};

typedef enum {
    E_STOP,
    PROGRAM_PAUSED,
    MACHINE_DISABLED,
    PROBE,
    BEGIN_PROCESS,
    ACTIVATE_IMPLEMENT,
    DEACTIVATE_IMPLEMENT,
    IN_PROCESS_MOTION,
    LINK_PROCESS_STEPS,
    PROCESS_RECOVERY_MOTION,
    PROCESS_RECOVERY_RESET,
    END_PROCESS,
    NUM_METHODS
}module_methods_t ;

typedef enum {
    SEMI_AUTO_GC,
    PLASMA,
    NUM_MODULES
}module_names_t ;


typedef enum {
    WAITING,
    DISABLED,
    PROBE_SETUP,
    PROBING,
    PROCESS_SETUP,
    RUNNING_PROCESS,
    LINKING,
    NUM_STATES,
}machine_state;

typedef enum{
    ABSOLUTE,
    FROM_DATUM,
    INCREMENTAL
}  motion_types;

//Struct of pointers to the various parameters used in the lookup table. 
// The pointers are added to this list in an appropriate order so they match the lookup table
typedef struct{
    volatile module_methods_t * override;
    volatile void *values[PARAMETER_COUNT];
}lookup_table_parameters_t;


//nested list of pointers to module methods
//First index is the module number
//Second index is the method number 
typedef struct{
    //Semi_Auto_GC_Functions
    module_methods_t (*function_ptrs[NUM_MODULES][NUM_METHODS])(void);
    //Plasma Functions

}module_functions_t;

//This is the entry format for the lookup values for each method
typedef struct{
    module_methods_t entry_method;
    module_methods_t override; 
    int required_matches;
    int values [PARAMETER_COUNT][2];
}lookup_table_entry_values_t ;

//Number of entries in the struct is equal to the number of methods to choose from
//The overall lookup table struct containing all of the entries
typedef struct {
    lookup_table_entry_values_t entries [NUM_METHODS];
}lookup_table_t;

//All the available hal pins
typedef struct {

    //In Pins
    hal_bit_t     * cut_recovery;                //"recover from cut error";
    hal_bit_t     * external_estop;              //"external estop input";
    hal_bit_t     * machine_is_on;               //"machine is on signal";
    hal_bit_t     * program_is_idle;             //"program is idle, connect to halui.program.is-idle";
    hal_bit_t     * program_is_paused;           //"program is paused, connect to halui.program.is-paused";
    hal_bit_t     * program_is_running;          //"program is running, connect to halui.program.is-running";
    hal_bit_t     * resume_cut;                  //"True when user presses resume cut button in cut recovery UI";
   
    hal_bit_t     * spindle_0_is_on;             //Connect to spindle.0.on
    hal_bit_t     * spindle_0_stopped;            //Connect to spindle.0.stop
    hal_bit_t     * spindle_0_at_speed;          //Connect to spindle.0.at-speed
    hal_bit_t     * resume;                  //on when user resumes cut from cut recovery
    hal_bit_t     * probe_trigger;          //True when probe is triggered
    //hal_bit_t     * add_probe_point;          //True when probe is triggered
    hal_bit_t     * probe_status;          //True when probe is triggered
    hal_bit_t     * clear_probe_data;          //True when probe is triggered
    hal_bit_t     * pierce_command;           //User command to pierce
    hal_bit_t     * temp;

    hal_float_t   * axis_x_max_limit;            //"axis x maximum limit, connect to ini.x.max-limit";
    hal_float_t   * axis_x_min_limit;            //"axis x minimum limit, connect to ini.x.min-limit";
    hal_float_t   * axis_x_position;             //"current x axis position, connect to axis.x.pos-cmd";
    hal_float_t   * axis_y_max_limit;            //"axis y maximum limit, connect to ini.y.max-limit";
    hal_float_t   * axis_y_min_limit;            //"axis y minimum limit, connect to ini.y.min-limit";
    hal_float_t   * axis_y_position;             //"current y axis position, connect to axis.y.pos-cmd";
    hal_float_t   * axis_z_max_limit;            //"axis z maximum limit, connect to ini.z.max-limit";
    hal_float_t   * axis_z_min_limit;            //"axis z minimum limit, connect to ini.z.min-limit";
    hal_float_t   * axis_z_position;             //"current z axis position, connect to axis.x.pos-cmd";
    hal_float_t   * x_offset_current;             //"current x axis offset, connect to axis.x.eoffset";
    hal_float_t   * y_offset_current;             // "current y axis offset, connect to axis.y.eoffset";
    hal_float_t   * z_offset_current;             //"current z axis offset, connect to axis.z.eoffset";
    hal_float_t   * probe_start_height;           //"current z axis offset, connect to axis.z.eoffset";
    hal_float_t   * cut_height;                   //"current z axis offset, connect to axis.z.eoffset";
    hal_float_t   * cut_feed_rate;                //"current z axis offset, connect to axis.z.eoffset";
    hal_float_t   * kerf_width;                   //"current z axis offset, connect to axis.z.eoffset";
    hal_float_t   * pierce_height;                //"current z axis offset, connect to axis.z.eoffset";
    hal_float_t   * probe_feed_rate;              //"current z axis offset, connect to axis.z.eoffset";
    hal_float_t   * setup_velocity;               //velocity for setup moves like linking and moving to pierce height
    //hal_float_t   * new_probe_point_x;          // A new probe x point to add to the list of points to probe;
    hal_float_t   * new_probe_point_y;          // A new probe y point to add to the list of points to probe
    hal_float_t   * safe_height;               //height for linkign
    hal_float_t   * pierce_delay;               //pierce delay time

    hal_u32_t     * module_type;                  // The current module type
    hal_u32_t     * cut_type;                     // The current cut type
    hal_u32_t     * state_override;               //if the GUI does a state override for some reason
    hal_u32_t     * probe_point_count;             // Number of points to probe

    hal_s32_t     * user_z_motion_command;        // 0,1,or -1 depending on whether the user desires motion
    hal_s32_t     * motion_type;                 //Connect to motion.motion-type

    //Out Bit Pins
    hal_bit_t     * probe_good;                  //On when probe is good
    hal_bit_t     * recovering;                  //on when recovering
    hal_bit_t     * active_x_y_offsets;          // on when there are x and y external offsets active
    hal_bit_t     * clear_eoffsets;              // clear eoffsets tie to  axis.x.eoffset-clear,y,z
    hal_bit_t     * enable_eoffsets;             // enable eoffsets tie to  axis.x.eoffset-enable,y,z
    hal_bit_t     * probe_enable;                // turn on the probe
    hal_bit_t     * feed_hold;                   // "feed hold, connect to motion.feed-hold
    
    //Out S32 Pins
    hal_s32_t     * x_offset_counts;             //"current x axis offset, connect to axis.x.eoffset-counts";
    hal_s32_t     * y_offset_counts;             // "current y axis offset, connect to axis.y.eoffset-counts";
    hal_s32_t     * z_offset_counts;             //"current z axis offset, connect to axis.z.eoffset-counts";
     hal_s32_t    * state_out;             //"current z axis offset, connect to axis.z.eoffset-counts";
    //Out Float Pins

    hal_float_t   * x_offset_scale;             //"current x axis offset, connect to axis.x.eoffset-scale";
    hal_float_t   * y_offset_scale;             //"current y axis offset, connect to axis.y.eoffset-scale";
    hal_float_t   * z_offset_scale;             //"current z axis offset, connect to axis.z.eoffset-scale";
    hal_float_t   * user_in_process_z_offset;   //The user's requested z offset from the otherwise commanded height

}hal_data_t;

struct positions {
    float x_absolute;
    float y_absolute;
    float z_absolute;
    float height_from_stock;
    float stock_height;
};

typedef struct {
    volatile int * num_points_requested;
    float ** point_data;
    int points_probed;

}contour_map;

//Function Prototypes
lookup_table_t create_lookup_table(const char *  filename);
int offset_move(motion_types motion_type, char axis, float velocity, float target);
void offset_feedrates(void);
void run_state_machine();
void add_to_contour_map(void);
void clear_contour_map(void);
void compute_positions(void);
void compute_datum(void);
module_methods_t determine_method(lookup_table_parameters_t *params_l,lookup_table_t *lookup_table );

module_methods_t semi_auto_gc_estop(void);
module_methods_t semi_auto_gc_program_paused(void);
module_methods_t semi_auto_gc_machine_disabled(void);
module_methods_t semi_auto_gc_probe(void);
module_methods_t semi_auto_gc_begin_process(void);
module_methods_t semi_auto_gc_torch_on(void);
module_methods_t semi_auto_gc_torch_off(void);
module_methods_t semi_auto_gc_in_process_motion(void);
module_methods_t semi_auto_gc_link_process_steps(void);
module_methods_t semi_auto_gc_cut_recovery_motion(void);
module_methods_t semi_auto_gc_cut_recovery_reset(void);
module_methods_t semi_auto_gc_end_process(void);
module_methods_t semi_auto_gc_idle(void);

#endif