 /** @file
 * @brief Contains declarations required for the Series-Elastic Actuator (SEA) Driver
*/
#ifndef SENSOJOINT_CONTROL_H
#define SENSOJOINT_CONTROL_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "error_list.h"
#include "sensojoint_structs.h"
#include "file_handling/include/json_handling.h"
#include <iostream>
#include "sensojoint.h"

using namespace  std;
/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class sensojoint_control : public esmacat_sensojoint
{

private:
    controller_configuration            controller_config;
    torque_control_terms                torque_terms;
    impedance_control_terms             impedance_terms;
    leg_weight_compensation_config_t    leg_weight_comp;


    // Variables for control loops
    float prev_position_radians;/** Position in radians from the last cycle */
    float prev_filtered_velocity;   /** Velocity in radians/sec from the last cycle */

    float prev_torque_error_mNm;    /** Torque error in mNm from the last cycle */
    float filtered_deriv_torque_error;  /** Filtered rate of change of error used for the derivative control */
    float integ_torque_error;   /** Summation of the torque error used for Integral control */

    double derating_factor = 0.0;        // the elapsed_time_ms in input must be with the zero when the function start, not the absolute elapsed time
    double derating_param_time_to_arrive_75p_in_msec = 2500.0;   // Time in milliseconds to reach 75% of desired setpoint

    robot_control_mode control_mode;                // joint modality of operation;
    int32_t torque_sensor_OFFSET;               //[mNm]


    //*** IMPEDENCE CONTROL TERM***
    float impedance_max_error_rad;        // max value allowed as error in impedence control, it limit the effect of the control
    float max_velocity_threshold;               // [rad*sec]max value allowed as motor velocity
    float max_load_stop_threshold;              // [mNm]max value allowed as load cell reading
    float max_torque_control_input = 15000;     // max value allowed as torque input
    float torque_setpoint;                      // torque mNm of torque_control feeded to the joint at each cycle

    float kterm_mNm = 0;
    float dterm_mNm = 0;
    float controller_torque_mNm = 0;
    float error_radians = 0;

    //*** SOFT STOP
    double T_stops;       //[mNm] soft stops torque
    double Ks_ss;
    double Kd_ss;


    double impedance_soft_stops_error_rad = 0;
    bool out_of_boundaries = false;

    esmacat_err loadcell_reading_status;

    /** GRAVITY COMPENSATION TERM   */
    //Robot terms
    // Centers of mass properties
    float link1_com;
    float link2_com;
    float ankle_shell_com;
    float foot_holder_com;



    // Mass properties
    float link1;
    float link2;
    float ankle_shell;
    float leg;
    float foot;
    float foot_holder;

    // Length properties
    float forearm_length_m;
    float hand_length_m;
    float leg_length_m;
    float foot_length_m;

    //Center of mass properties
    double forearm_com_m;
    double hand_com_m;
    float leg_com_m;
    float foot_com_m;

    // Mass properties
    float forearm_mass_kg;
    float hand_mass_kg;
    float leg_mass_Kg;
    float foot_mass_Kg;


    //CYCLIC_SYNC_VELOCITY_OUTPUT Control terms

    double Ks_vel = 5*Nm_to_mNm;
    double Kd_vel = 0.5*Nm_to_mNm;
    double error_off;




public:

    // Controller configuration
    controller_configuration    get_controller_configuration();
    torque_control_terms        get_torque_control_terms();
    impedance_control_terms     get_impedance_control_terms();
    void    sensojoint_control_treshold_parameters();
    void    set_controller_config_mode(robot_control_mode input);
    void    set_torque_control_terms(torque_control_terms input);
    void    set_impedance_control_terms(impedance_control_terms input);


    void enable_joint(bool input);      // enable operation of the joint
    //VISUALIZATION FUNCTION
    void plog_debug_variables(double torque_setpoint);

    // IMPEDANCE CONTROL FUNCTION

    double impedance_control_offset_time_ms = 0.0; //off time
    esmacat_err impedance_control(float setpoint_radians,float gravity_torque_mNm, float elapsed_time_ms);
    esmacat_err torque_control(double torque_setpoint, float elapsed_time_ms);
    esmacat_err torque_control_with_soft_stop(double torque_setpoint, float elapsed_time_ms);

    esmacat_err sliding_mode_control();

    void set_motor_derating_factor (double factor);

    void set_impedance_control_setpoint(double input_torque);
    void set_impedance_control_Kgain(double K_input); //mNm*rad
    void set_impedance_control_Dgain(double D_input); //mNm per rad per sec
    void set_max_velocity_threshold(double input);
    void set_max_load_stop_threshold (double input);

    double get_impedance_control_Kgain();
    double get_impedance_control_Dgain();
    double get_control_output();
    bool   get_out_of_boundaries();

    //Soft Stop
    bool get_soft_stop_status();
    void set_soft_stop_parameters(double Lim_up_input, double Lim_down_input, double amplitude, bool Reset_pos);
    void set_soft_stop_gain(double ks, double kd);

    // Homing Mode
    esmacat_err homing_control(float elapsed_time_ms);      //procedura di misura del range of motion

    //weight compensation
    void set_user_parameters();
    void set_robot_parameters();
    double compute_inverse_dynamics(double position_rad);
    void set_inverse_dynamics_parameters();

    double get_inertia_torque_mNm();

//    float get_compensation_factor(){return controller_config.compensation_factor;};
//    float get_weight_assitance(){return controller_config.weight_assistance;};


    // Manage Control_mode
    void set_control_mode(robot_control_mode mode);
    robot_control_mode get_current_control_mode();

    //Display
    robot_control_mode get_control_mode_display();




};
#endif // SENSOJOINT_CONTROL_H
