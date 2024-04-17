/** @file
 * @brief This file contains the declaration of the class associated with the user-defined
 * application for the EtherCAT Arduino Shield by Esmacat slave example project */

#ifndef sensojoint_manager_H
#define sensojoint_manager_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include <map>
#include "application.h"
using std::cout;
//Include the header file for the Esmacat slave you plan to use for e.g. Analog Input slave

#include "sensojoint.h"
#include <unistd.h>
#include "sensojoint_shared_memory.h"
#include "sensojoint_structs.h"
#include "sensojoint_control.h"

#define EXERCISE_DURATION 5000.0
#define EXERCISE_AMPLITUDE M_PI/3    // 70 deg
#define EXERCISE_START M_PI/18      // 10 deg
#define POS_OFFSET (M_PI*340)/180

#define SIGMA_EX_DURATION 4000.0
#define discretization_factor 100

#define APPLICATION_TIMEOUT_MS 60*1000 // = 1 min
/*****************************************************************************************
 *********************** CLASSES*********************************************************
 ****************************************************************************************/
/**
 * @brief Description of your custom application class
 *
 * Your custom application class sensojoint_manager inherits the class 'esmacat_application'
 * Write functions to override the parent functions of 'esmacat_application'
 * Declare an object of your slave class (e.g. ecat_ai)
 * Declare any other variables you might want to add
 * Define the constructor for initialization
 */
class sensojoint_manager : public esmacat_application
{
private:
    void assign_slave_sequence(); /** identify sequence of slaves and their types */
    void configure_slaves(); /** configure all slaves in communication chain */ 
    void init(); /** code to be executed in the first iteration of the loop */
    void loop(); /** control loop */
    void quit(); /** close sheared memory */
    void readsharedmemory();
    void writesharedmemory();
    void read_motor_value();
    void read_torque_value();
    void initialize_interface_parameters();

    sensojoint_control                      ecat_sensojoint;
    sensojoint_shared_memory_comm           sensojoint_shared_memory;

    motor_values                             motor_terms;
    controller_configuration                controller_config_interim;
    torque_control_terms                    torque_control_interim;
    impedance_control_terms                 impedance_terms_interim;
    control_input_command_t                 control_input_command;

    CoE_state_t                             sensojoint_state;

//    uint32_t                                curr_counter;


public:
    /** A constructor- sets initial values for class members */
    sensojoint_manager()
    {

        sensojoint_state = not_ready_to_switch_on;

    }



    double elapsed_time_ms_offset= 0;

    double position_offset_rad;
    bool power_up = false;
    double prev_mode = 0;
    double torque_output = 0.0;
    // ROM

    int32_t velocity_control;

    //Soft stops
    double target_freeze_pos=0;
    double ref_position = 0;   // set posistion zero link pointing the ground + one degree adjustment
    double elapsed_time_ms_offset_exercise = 0;
    int count = 0;
    int repetition_counter=0;

    double error_off;
    double target_pos_reset;

    double Ks_vel = 5;       // FIX unità di misura sbagliate
    double Kd_vel = 2.5;



    // INVERSE DYNAMICS
    // inertia
    double weight_compensation_torque;      //per ora variabile valida solo in sensojoint_manager
    double inverse_dynamics_torque;
    bool soft_stop_active = true;


    //freeze
    double Ks_freeze = 5.0*Nm_to_mNm;
    double Kd_freeze = 0.5*Nm_to_mNm;
    double impedance_control_error_freeze_rad;
    double target_impedance_control_torque_freeze_mNm;

    //INERTIA
    double w = 0.15; //weight factor
    double motor_inertia = 4.15*1E-5;
    double Inertia_torque_mNm;

    //BETA-FUNCTION
    bool start_trajectory = false;
    int total_rep_counter;

    // Trajectory generation
    double time_scaling_f;
    double P0 = EXERCISE_START;
    double P2 = 0.0;
    double P3 = 5.0; //5ht Order
    double P4 = P2+EXERCISE_DURATION;
    double P5 = 5.0; //5th Order
    double P1 = EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));
    double total_time_double_trajectory_sigma=2*EXERCISE_DURATION;
    double interim_time_exercise;
    double interim_setpoint_exercise;
    double error_traj;
    double target_k ;
    double target_d;
    double target_traj;
    double target_torq;


    //  Offline index parameter between repetition
    float Error_5 ;
    float Error_4 ;
    float Error_3 ;
    float Error_2 ;
    float Error_1 ;
    float Error_average;
    float Error_actual;
    float Error_squared_actual;
//   float velocity_error_vector[discretization_factor];
//    float average_error_velocity_vector[discretization_factor];
    float nominal_velocity;
    float average_velocity;
    int set_counter;
    int ind_good;
    //  Trajectory and impedance control update parameter;


    //anti-g calibration
    double target_calibration_position = 70*deg_to_rad; //[deg]
    //filtering input
    double derating_factor_input = 0.0;        // the elapsed_time_ms in input must be with the zero when the function start, not the absolute elapsed time
    double derating_param_time_to_arrive_75p_in_msec = 2500.0;   // Time in milliseconds to reach 75% of desired setpoint




    void print2screen();
    void initialize_write2file();
    void initialize_write2file_adaptive_traj_data();
    void read_file();
    void write2file_updateoffline();
    void write2file_adaptive_traj_data() ;
    void write2file();
    void closefile();
    ofstream CSVfile;
    ofstream CSVAdaptivetraj;


};

#endif // sensojoint_manager_H
