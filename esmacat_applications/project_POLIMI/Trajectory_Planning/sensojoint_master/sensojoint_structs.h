#ifndef SENSOJOINT_STRUCTS_H
#define SENSOJOINT_STRUCTS_H

#include <iostream>
#include <map>
#include "math.h"
#include "vector"
using namespace std;


#define CYCLIC_SYNC_POSITION 8
#define CYCLIC_SYNC_VELOCITY 9
#define CYCLIC_SYNC_TORQUE   10

#define CYCLIC_SYNC_POSITION_OUTPUT -108
#define CYCLIC_SYNC_VELOCITY_OUTPUT -109
#define CYCLIC_SYNC_TORQUE_OUTPUT   -110

#define STATE_NOT_READY_SWITCH_ON   1000
#define STATE_SWITCH_ON_DISABLED    1001
#define STATE_READY_SWITCH_ON       1002
#define STATE_SWITCHED_ON           1003
#define STATE_OPERATION_ENABLED     1004
#define STATE_QUICK_STOP_ACTIVE     1005
#define STATE_FAULT_REACTION        1006
#define STATE_FAULT                 1007


double const Nm_to_mNm = 1000.0;
double const mNm_to_Nm = 1/1000.0;
double const deg_to_rad = M_PI/180;
double const rad_to_deg = 180/M_PI;
double const rpm_to_rad_s = 0.10472;


// NOTE: Maybe not needed
/* define pointer structure */
typedef struct PACKED
{
    uint16_t statusword;
    int8_t	modes_of_operation_display;
    int32_t	position_actual_value;
    int32_t	velocity_actual_value;
    int16_t	torque_actual_value;
    int32_t	output_torque_actual_value;
    int32_t	secondary_position_value;
    int32_t	secondary_velocity_value;
    uint32_t    timestamp;
    //  int8_t    safety_status_1;		//<< STO status 1
    //  int8_t    safety_status_2;		//<< STO status 2
} in_sensojoint_PDO;

typedef struct PACKED1
{
    uint16_t    controlword;
    int8_t	modes_of_operation;
    int16_t	target_torque;
    int32_t	target_position;
    int32_t	target_velocity;
    int32_t	target_output_torque;
    int32_t	target_output_position;
    int32_t	target_output_velocity;
    int16_t	target_torque_offset;
} out_sensojoint_PDO;


/**
 * @brief The CoE_state_t enum
 * CiA 402 finite state machine
 */
enum CoE_state_t{
    not_ready_to_switch_on  = 1000,
    switch_on_disabled      = 1001,
    ready_to_switch_on      = 1002,
    switched_on             = 1003,
    operation_enabled       = 1004,
    quick_stop_active       = 1005,
    fault_reaction          = 1006,
    fault                   = 1007,
};

/**
 * @brief String mapping for CoE_state_t
 */
static map< CoE_state_t, const char * > CoE_state_labels = {
    {not_ready_to_switch_on,        "STATE_NOT_READY_SWITCH_ON"},
    {switch_on_disabled,            "STATE_SWITCH_ON_DISABLED"},
    {ready_to_switch_on,            "STATE_READY_SWITCH_ON"},
    {switched_on,                   "STATE_SWITCHED_ON"},
    {operation_enabled,             "STATE_OPERATION_ENABLED"},
    {quick_stop_active,             "STATE_QUICK_STOP_ACTIVE"},
    {fault_reaction,                "STATE_FAULT_REACTION"},
    {fault,                         "STATE_FAULT"},
};

/**
 * @brief The CoE_modes enum
 * CiA 402 modes of operatioin
 */
enum CoE_mode_t{
    cyclic_synchronous_position  = 8,
    cyclic_synchronous_velocity  = 9,
    cyclic_synchronous_torque    = 10,
    cyclic_synchronous_position_output  = -108,
    cyclic_synchronous_velocity_output  = -109,
    cyclic_synchronous_torque_output    = -110,
    none                         = 100,
};

/**
 * @brief String mapping for CoE_state_t
 */
static map< CoE_mode_t, const char * > CoE_mode_labels = {
    {cyclic_synchronous_position,               "CYCLIC_SYNC_POSITION"},
    {cyclic_synchronous_velocity,               "CYCLIC_SYNC_VELOCITY"},
    {cyclic_synchronous_torque,                 "CYCLIC_SYNC_TORQUE"},
    {cyclic_synchronous_position_output,        "CYCLIC_SYNC_POSITION_OUTPUT"},
    {cyclic_synchronous_velocity_output,        "CYCLIC_SYNC_VELOCITY_OUTPUT"},
    {cyclic_synchronous_torque_output,          "CYCLIC_SYNC_TORQUE_OUTPUT"},
    {none,                                      "DEFAULT_MODE"},
};

/** enum for control mode of the robot */

enum robot_control_mode{
    quit                    = 200,
    stop                    = 201,
    standby                 = 202,
    zerotorque_control      = 203,      //test senza compensazione tutti i tipi di compensazione
    freeze_control          = 204,
    antig_control           = 205,
    transparent_control     = 206,
    impedance_control_beta  = 207,
    impedance_control_sigma = 208,
    antig_calibration       = 209,
    adaptive_control_fwd    =211,
};


///**
// * @brief Labels for control mode of the robot
// * not used up to now
// */
//const string robot_modes_labels[] = {
//    "QUIT"
//    "STOP"
//    "STANDBY"
//    "NULLTORQUE"//    "FREEZE"
//    "ANTIG"
//    "TRANSPARENT"
//    "IMPEDANCE BETA"
//    "IMPEDANCE SIGMA"
//    "ANTI-G CALIBRATION"


//};

static map< robot_control_mode, const char * > robot_control_labels = {
    {quit,                  "QUIT"          },
    {stop,                  "STOP"          },
    {standby,               "STAND-BY"      },
    {zerotorque_control,    "ZERO-TORQUE"   },
    {freeze_control,        "FREEZE"        },
    {antig_control,         "ANTI_G_CONTROL"},
    {transparent_control,   "TRANSPARENT"   },
    {impedance_control_beta,"IMPEDANCE_BETA"},
    {impedance_control_sigma,"IMPEDANCE_SIGMA"},
    {antig_calibration,     "G-CALIBRATION" },
    {adaptive_control_fwd,     "ADP--CONTROL" },
};

//******************************
// sensojoint_control structs:
//******************************
struct motor_values{
    double statusword=0;
    double modes_of_operation_display=0;
    double position_actual_value=0;
    double velocity_actual_value=0;
    double acceleration_value_filt = 0;
    double torque_actual_value=0;
    double output_torque_actual_value=0;
    double secondary_position_value=0;
    double secondary_velocity_value=0;
    double timestamp=0;
};

struct controller_configuration{
    // error check to enable operationwithout error
    bool joint_enabled = true;

    // joint modality of operatio;
    CoE_mode_t motor_mode_of_operation;
    robot_control_mode control_mode;
    };

struct  torque_control_terms
{
    //FEEDBACK TORQUE:
    double control_output;
    
    float max_velocity_threshold = 1;               // [rad*sec]max value allowed as motor velocity
    float max_load_stop_threshold = 15000;          // [mNm]max value allowed as load cell reading

    //FEEDFAWARD TORQUE:
    //Weight compensation
    double weight_compensation_torque;
    double weight_compensation_torque_before_scaling; //cambiare nome in scaling factor
    double gravity_robot_compensation_torque;
    double weight_assistance;
    double compensation_factor;

    //Inertia
    double inertia_torque ;
    double inertia_correction;




};

struct impedance_control_terms{
    float max_torque_control_input;    // max value allowed as torque input
    //Impedance:
    double K_gain;       // stiffness contribution
    double D_gain;       // Damping contribution

    float impedance_max_error_rad = 0.5;        // max value allowed as error in impedence control, it limit the effect of the control

    // Position setpoint for impedance control in radians
    float impedance_control_setpoint = 0.0;     //[Rad]

    //Soft Stop:

    double soft_stop_torque = 0.0; //soft stop torque in mNm
    double T_stops_max_torque = 10000.0;    //[mNm] max allowed torque of the soft stop contribution
    double Lim_low;     //[RAD] lower ROM limit
    double Lim_up;      //[RAd] upper rom limit
    double Delta_ss;    // Amplitude soft stops | deg to rad
    double Target_pos_reset;
    bool out_of_boundaries = false;


    float impedance_control_torque_mNm = 0.0;   // Output torque of the PD impedance control in milli-Nm
    float tot_gravity_torque_mNm = 0.0;             // Torque feedfoward applied due to gravity in milli-Nm

    float torque_feedfoward =0.0;        //feedfaward torque input mNm
    float torque_feedback = 0.0;        //total torque feedback mNm
    float torque_tot = 0.0;              //total torque feedback + feedfoward


};

struct control_input_command_t{
    robot_control_mode control_mode_input;
    double Lim_up_input;
    double Lim_down_input;
    double amplitude_soft_stops_input;
    double K_gain_input;
    double D_gain_input;
    double gravity_scailing_factor_input;       // robot weight correction
    double weight_scailing_factor_input;        // human user weight correction
    double inertia_scailing_factor_input;       // Inertia correction

};


struct arm_weight_compensation_config_t{    //copy from onedof_rt

    float human_weight_kg;    // Human weight Kg
    float weight_assistance;    // Percentage of weight assistance for human arm

    // Human height
    float human_height_m;

    /** arm length m */
    float arm_length_m;
    /** forearm length */
    float forearm_length_m;

    /** Initializes all the parameters to 0 */
    arm_weight_compensation_config_t()
    {
        human_weight_kg = 47;
        human_height_m = 1.60;
        weight_assistance = 1.0;
        arm_length_m = 0;
        forearm_length_m = 0;
    }
};


struct leg_weight_compensation_config_t{    //copy from onedof_rt

    float human_weight_kg;    // Human weight Kg
    float weight_assistance;    // Percentage of weight assistance for human arm
    float human_height_m;    // Human height


    /** arm length m */
    float leg_length_m;
    /** forearm length */
    float forearm_length_m;

    /** Initializes all the parameters to 0 */
    leg_weight_compensation_config_t()
    {
        human_weight_kg = 72;
        human_height_m = 1.75;
        weight_assistance = 1.0;
        leg_length_m = 0;
        forearm_length_m = 0;
    }
};


struct adaptive_traj_offline_t{
    bool traj_adaptation=false;
    bool traj_is_adapted=false;
//   int counter=0;
//   int time_exercise=0;

    //BETA
    //PARAMETER
    float traj_time_scaling=1;    // Time scaling paramater computed by traj_generation;
    float traj_amplitude_scaling=1; // Amplitude scaling parameter computed by traj_generation;
    float simm_scaling=1;          // Parameter that shift the beta-function towards the left side or right w.r.t to its max amplitude point
  };
struct adaptive_traj_online_forward {
    //Parameter
    float sigma_param;
    float gamma_param;
    float delta_param;
    float forgot_fct;

    //Time varying variables
    array<float,500> gauss_vector;
    array<float,500> centroids;
    array<float,500>fwd_virt_variable;
    array<float,500>prev_fwd_virt_variable;
    float norm_gauss_vector;
    float fwd_virt_variable_norm;
    float upg_term;
    float adapt_torque;
    float adapt_torque_err_part;
    float adapt_torque_decay_term;
};


#endif // SENSOJOINT_STRUCTS_H
