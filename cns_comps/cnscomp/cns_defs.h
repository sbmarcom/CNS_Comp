#ifndef CNS_DEFS_H
#define CNS_DEFS_H


#define PARAMETER_COUNT 8


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


typedef struct{
    volatile module_methods_t * override;
    volatile void *values[PARAMETER_COUNT];
}lookup_table_parameters_t;



typedef struct{
    //Semi_Auto_GC_Functions
    module_methods_t (*function_ptrs[NUM_MODULES][NUM_METHODS])(void);
    //Plasma Functions

}module_functions_t;


typedef struct{
    //The method that will be returned if this entry matches the parameters
    module_methods_t entry_method;
    // Parameter value required to override the lookup table
    module_methods_t override; 
    //The required number of parameters that must match the values
    int required_matches;
    int values [PARAMETER_COUNT][2];

}lookup_table_entry_values_t ;


typedef struct {
    //Number of entries in the struct is equal to the number of methods to choose from
    lookup_table_entry_values_t entries [NUM_METHODS];
}lookup_table_t;


typedef struct {

    //In Pins
    hal_bit_t     * cut_recovery;                //"recover from cut error";
    hal_bit_t     * external_estop;              //"external estop input";
    hal_bit_t     * machine_is_on;               //"machine is on signal";
    hal_bit_t     * program_is_idle;             //"program is idle, connect to halui.program.is-idle";
    hal_bit_t     * program_is_paused;           //"program is paused, connect to halui.program.is-paused";
    hal_bit_t     * program_is_running;          //"program is running, connect to halui.program.is-running";
    hal_bit_t     * resume_cut;                  //"True when user presses resume cut button in cut recovery UI";
    hal_bit_t     * motion_type;                 //Connect to motion.motion-type
    hal_bit_t     * spindle_0_is_on;             //Connect to spindle.0.on
    hal_bit_t     * spindle_0_stopped;            //Connect to spindle.0.stop
    hal_bit_t     * spindle_0_at_speed;          //Connect to spindle.0.at-speed
    hal_bit_t     * resume;                  //on when user resumes cut from cut recovery
    

    hal_float_t   * axis_x_max_limit;            //"axis x maximum limit, connect to ini.x.max-limit";
    hal_float_t   * axis_x_min_limit;            //"axis x minimum limit, connect to ini.x.min-limit";
    hal_float_t   * axis_x_position;             //"current x axis position, connect to axis.x.pos-cmd";
    hal_float_t   * axis_y_max_limit;            //"axis y maximum limit, connect to ini.y.max-limit";
    hal_float_t   * axis_y_min_limit;            //"axis y minimum limit, connect to ini.y.min-limit";
    hal_float_t   * axis_y_position;             //"current y axis position, connect to axis.y.pos-cmd";
    hal_float_t   * axis_z_max_limit;            //"axis z maximum limit, connect to ini.z.max-limit";
    hal_float_t   * axis_z_min_limit;            //"axis z minimum limit, connect to ini.z.min-limit";
    hal_float_t   * axis_z_position;             //"current z axis position, connect to joint.N.pos-fb";

    hal_u32_t     * module_type;                  // The current module type
    hal_u32_t     * cut_type;                     // The current cut type
    hal_u32_t     * state_override;               //if the GUI does a state override for some reason
    //Out Pins

    hal_bit_t     * probe_good;                  //On when probe is good
    hal_bit_t     * recovering;                  //on when recovering
    hal_bit_t     * active_x_y_offsets;          // on when there are x and y external offsets active
    

}hal_data_t;

//Function Prototypes
lookup_table_t create_lookup_table(const char *  filename);
int offset_move(int offset, int vel, int target);
void offset_feedrates(void);
void run_state_machine();
module_methods_t determine_state(lookup_table_parameters_t *params_l,lookup_table_t *lookup_table );

module_methods_t semi_auto_gc_estop(void);


#endif