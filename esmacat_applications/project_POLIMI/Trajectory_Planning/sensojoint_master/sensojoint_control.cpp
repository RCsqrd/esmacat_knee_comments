/** @file
 * @brief Contains definitions of functions used for the Series-Elastic Actuator (SEA) Driver
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "sensojoint_control.h"
#include "sensojoint.h"
/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/


void sensojoint_control::sensojoint_control_treshold_parameters() {
    // Controller LIMITS:
    torque_terms.max_velocity_threshold = 3.5;       // [rad*sec]max value allowed as motor velocity
    torque_terms.max_load_stop_threshold = 50000;   // [mNm]max value allowed as load cell reading

    impedance_terms.T_stops_max_torque = 15000;     //[mNm]
    impedance_terms.impedance_max_error_rad = 0.5;
    impedance_terms.max_torque_control_input = 50000;   // [mNm]

    Ks_vel = 5*Nm_to_mNm;
    Kd_vel = 0.5*Nm_to_mNm;
}
controller_configuration sensojoint_control::get_controller_configuration(){
    return controller_config;
}


torque_control_terms sensojoint_control::get_torque_control_terms(){
    return torque_terms;
}
impedance_control_terms sensojoint_control::get_impedance_control_terms(){
    return impedance_terms;
}

void sensojoint_control::set_controller_config_mode(robot_control_mode control_mode){
     controller_config.control_mode = control_mode;
}
void sensojoint_control::set_torque_control_terms(torque_control_terms input){
    torque_terms = input;
}
void sensojoint_control::set_impedance_control_terms(impedance_control_terms input){
    impedance_terms = input;
}

void sensojoint_control::enable_joint(bool input){
    controller_config.joint_enabled = input;
//    cout << " Joint Enabled "<<controller_config.joint_enabled<<endl;
}


//fare funzione per la compensazione della inerzia


void sensojoint_control::plog_debug_variables(double torque_setpoint)
{ PLOGW<<"info TorqueSetpoint"<<torque_setpoint<<endl;
 }


esmacat_err sensojoint_control::torque_control(double torque_setpoint, float elapsed_time_ms)
{
    esmacat_err error = NO_ERR;

//    if ( controller_config.control_mode != (robot_control_mode::stop) || controller_config.control_mode !=robot_control_mode::standby) // if wrong control mode is selected
//    {

//        enable_joint(false);
//        esmacat_sensojoint::set_target_output_torque(0.0);
//        return ERR_SEA_WRONG_CONTROL_MODE_SELECTED;
//    }
//    else{
//        enable_joint(true);
//    }

    if ( torque_setpoint > max_torque_control_input  ) // if the control input is larger than its allowed range
    {
        torque_setpoint = max_torque_control_input;
        error = ERR_CONTROLLER_MAX_TORQUE_SETPOINT_OUT_OF_RANGE;
        PLOGW<<" ERR_CONTROLLER_MAX_TORQUE_SETPOINT_OUT_OF_RANGE"<<endl;

    }
    if ( torque_setpoint < (-1)*max_torque_control_input  ) // if the control input is larger than its allowed range
    {
        torque_setpoint = (-1)*max_torque_control_input;
        error = ERR_CONTROLLER_MIN_TORQUE_SETPOINT_OUT_OF_RANGE;
        PLOGW<<" ERR_CONTROLLER_MAX_TORQUE_SETPOINT_OUT_OF_RANGE"<<endl;

    }


    // salvare torque_setpoint nelle variabili shared memory joint_values.torque_setpoint_mNm = torque_setpoint;

    // set derating factor
    if (derating_param_time_to_arrive_75p_in_msec < 1e-5){
        derating_factor = 1.0; }
    else{
        if ( ( elapsed_time_ms/derating_param_time_to_arrive_75p_in_msec )< 20.0) {
        derating_factor =0.5*(1+smooth_start_func_tanh(elapsed_time_ms/derating_param_time_to_arrive_75p_in_msec) );
        }
        else {
        derating_factor = 1.0;
        }
    set_motor_derating_factor(derating_factor);
    torque_setpoint = torque_setpoint * derating_factor;
    }
    torque_terms.control_output = torque_setpoint;
    //PLOGW<<"info TorqueSetpoint"<<torque_setpoint<<endl;
    torque_sensor_OFFSET = -50;

    // Run inner torque control
    esmacat_sensojoint::set_target_output_torque(static_cast<int32_t>(torque_setpoint) + torque_sensor_OFFSET);

    // Return error
    return error;

}


esmacat_err sensojoint_control::torque_control_with_soft_stop(double torque_setpoint, float elapsed_time_ms)
{
    esmacat_err error = NO_ERR;
    double ref_position = esmacat_sensojoint::get_position_actual_value_rad()- ((M_PI*345)/180);
    double Lim_up= impedance_terms.Lim_up;        //[Rad] lower ROM limit
    double Lim_low = impedance_terms.Lim_low;     //[Rad] upper rom limit
    double Delta_ss = impedance_terms.Delta_ss;   //[Rad] Amplitude soft stops
    double delta_up = Lim_up - Delta_ss;
    double delta_down = Lim_low + Delta_ss;

    if (ref_position >= Lim_low && ref_position <= Lim_up){
        out_of_boundaries = false;
    }


    // if motor position is inside the range of motion --> torque control
    if (out_of_boundaries == false) {
//        esmacat_sensojoint::set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);

        if (ref_position <= Lim_up && ref_position >= delta_up){  //Arm position in the high soft stop range
            impedance_soft_stops_error_rad = (delta_up) - ref_position ; //[rad]
            T_stops = Ks_ss* impedance_soft_stops_error_rad - Kd_ss*esmacat_sensojoint::get_velocity_actual_value_rad_s();
            PLOGE<< "T_stops"<<T_stops<< endl;
        }

        else if( ref_position >= Lim_low && ref_position <= delta_down){    // Arm position in the lower soft stop range

            impedance_soft_stops_error_rad = (delta_down) - ref_position; //[rad]
            T_stops = Ks_ss* impedance_soft_stops_error_rad - Kd_ss*esmacat_sensojoint::get_velocity_actual_value_rad_s();
            PLOGE<< "T_stops"<<T_stops<< endl;
        }
        else if ( ref_position > delta_down && ref_position <= delta_up){ // arm position between in allowed range
            T_stops = 0;
        }
        else if (ref_position <= Lim_low || ref_position >= Lim_up){
//            T_stops = 0; //when rom are exceed T_stops = max torque from the soft stop
           //rom exceede
//          esmacat_sensojoint::set_modes_of_operation(CoE_mode_t::cyclic_synchronous_velocity_output);
            out_of_boundaries = true;
            PLOGW <<"ROM EXCEEDED"<<endl;
        }

        if (T_stops >= impedance_terms.T_stops_max_torque){      //max torque allowed = 10Nm
            T_stops = impedance_terms.T_stops_max_torque;
            PLOGW <<"SOFT STOP MAX TORQUE REACHED"<<endl;
        }

    }
//************if rom are exceeded --> velocity control***************//

//    // if motor position is outside the range of motion --> velocity control
//    else if(out_of_boundaries == true){
//    error_off = controller_config.Target_pos_reset - ref_position;
//    double velocity_control = Ks_vel * error_off - Kd_vel*esmacat_sensojoint::get_velocity_actual_value_rad_s();
//    esmacat_sensojoint::set_target_output_velocity((int32)velocity_control);

//    if ( abs(error_off)< M_PI/18){
//        esmacat_sensojoint::set_modes_of_operation(CoE_mode_t::cyclic_synchronous_torque_output);
//        esmacat_sensojoint::set_target_output_torque(0);
//        out_of_boundaries = false;
//    }
//    }




    if(out_of_boundaries == true && ref_position <= Lim_low){
        impedance_soft_stops_error_rad = Delta_ss ; //[rad]
        T_stops = Ks_ss* impedance_soft_stops_error_rad - Kd_ss*esmacat_sensojoint::get_velocity_actual_value_rad_s();
        {PLOGW<<"Out of boundaries"<<out_of_boundaries<<endl;}
    }
    else if(out_of_boundaries == true && ref_position >= Lim_up){
        impedance_soft_stops_error_rad = -Delta_ss ; //[rad]
        T_stops = Ks_ss* impedance_soft_stops_error_rad - Kd_ss*esmacat_sensojoint::get_velocity_actual_value_rad_s();
        {PLOGW<<"Out of boundaries"<<out_of_boundaries<<endl; }

    }

    impedance_terms.out_of_boundaries = out_of_boundaries;
    impedance_terms.soft_stop_torque = T_stops;

    error = torque_control( torque_setpoint + T_stops, elapsed_time_ms );

//    PLOGW<<T_stops<<endl;

    // come sempre creare struct dove salvarsi tutti i contributi del controllo   joint_impedance_control_terms.soft_stop_torque_mNm = t_push_up-t_push_down;

     return error;
}

esmacat_err sensojoint_control::impedance_control(float setpoint_radians,float setpoint_vel ,float inverse_dynamics_torque_mNm, float elapsed_time_ms)
{
    esmacat_err error = NO_ERR;

    float impedance_control_k_gain = impedance_terms.K_gain;
    float impedance_control_d_gain = impedance_terms.D_gain;
    float impedance_max_error_rad = impedance_terms.impedance_max_error_rad;

    max_velocity_threshold = torque_terms.max_velocity_threshold;
    max_load_stop_threshold = torque_terms.max_load_stop_threshold;

  float soft_stop_torque_mNm = 0;



    //IMPEDANCE Controller

    double measured_position_radians = (esmacat_sensojoint::get_position_actual_value_rad() - ((345*M_PI)/180));
    error_radians = setpoint_radians - measured_position_radians;
    // limit the error measured to between -max to +max
    error_radians = std::min(std::max(error_radians,-impedance_max_error_rad), impedance_max_error_rad);

    kterm_mNm = impedance_control_k_gain * error_radians;
    dterm_mNm = -impedance_control_d_gain * esmacat_sensojoint::get_velocity_actual_value_rad_s();
    controller_torque_mNm = kterm_mNm+dterm_mNm;
    // Disable motor if velocity exceeds threshold
    if (std::abs(esmacat_sensojoint::get_velocity_actual_value_rad_s()) >= max_velocity_threshold)
    {
        enable_joint(false);
        error = ERR_MEASURED_VELOCITY_OUTSIDE_RANGE;
 //       {PLOGW<<"vel treshold"<<max_velocity_threshold<<endl;}
        std::cout<<"ERR_MEASURED_VELOCITY_OUTSIDE_RANGE. ESCON Disabled"<<std::endl;
        return error;
    }
    else
    {
        enable_joint(true);
    }


    // disable motor if load exceeds threshold
    esmacat_err loadcell_reading_status;
    float loadcell_reading_mNm = esmacat_sensojoint::get_output_torque_actual_value_mNm();
    if (loadcell_reading_status == NO_ERR)
    {
        if (std::abs(loadcell_reading_mNm) >= max_load_stop_threshold)
        {
            enable_joint(false);
            error = ERR_MEASURED_LOAD_OUTSIDE_RANGE;
            PLOGE << "ERR_MEASURED_LOAD_OUTSIDE_RANGE" << " Joint Disabled" <<loadcell_reading_mNm<<std::endl;
            return error;
        }
        else
        {
            enable_joint(true);
        }
    }
    else
    {
        error = loadcell_reading_status;
        return error;
    }

    //activate soft stop
    //computing the torque setpoint and running torque control
    torque_setpoint_mNm = inverse_dynamics_torque_mNm + controller_torque_mNm + soft_stop_torque_mNm;

    // Run inner torque control with soft-stop
//    if(soft_stop_enable)
//    {
        error = torque_control_with_soft_stop(torque_setpoint_mNm, elapsed_time_ms);
//    }
    // Run inner torque control without soft-stop
//    else
//    {
//        error = torque_control(torque_setpoint_mNm,elapsed_time_ms);
//    }
   //Saving useful variables

   impedance_terms.impedance_control_setpoint=setpoint_radians;
   impedance_terms.impedance_control_torque_mNm=controller_torque_mNm;
   impedance_terms.torque_feedback=kterm_mNm+dterm_mNm;
   impedance_terms.tot_gravity_torque_mNm=inverse_dynamics_torque_mNm;
   impedance_terms.torque_feedfoward=inverse_dynamics_torque_mNm;
   impedance_terms.torque_tot=torque_setpoint_mNm;

   return error;

}


void sensojoint_control::set_motor_derating_factor(double factor){
    derating_factor = factor;
}

void sensojoint_control::set_impedance_control_setpoint(double input_torque){
    impedance_terms.impedance_control_setpoint = input_torque;
}

void sensojoint_control::set_impedance_control_Kgain(double K_input){
    impedance_terms.K_gain = K_input;
}

void sensojoint_control::set_impedance_control_Dgain(double D_input){
    impedance_terms.D_gain = D_input;
}

void sensojoint_control::set_max_velocity_threshold(double input){
    max_load_stop_threshold = input;
}

void sensojoint_control::set_max_load_stop_threshold(double input){
    max_load_stop_threshold = input;
}


double sensojoint_control::get_impedance_control_Kgain(){
    return impedance_terms.K_gain;
}

double sensojoint_control::get_impedance_control_Dgain(){
    return impedance_terms.D_gain;
}
double sensojoint_control::get_control_output(){
    return torque_terms.control_output;
}
bool sensojoint_control::get_out_of_boundaries(){
    return impedance_terms.out_of_boundaries;
}


/*************************************/
/********* SOFT STOP FUCTIONs*********/
/*************************************/

void sensojoint_control::set_soft_stop_parameters(double Lim_up_input, double Lim_down_input, double amplitude, bool Reset_pos){
    impedance_terms.Lim_low = Lim_down_input * deg_to_rad;    //[RAD] lower ROM limit
    impedance_terms.Lim_up = Lim_up_input * deg_to_rad;     //[RAd] upper rom limit
    impedance_terms.Delta_ss = amplitude * deg_to_rad; // Amplitude soft stops | deg to rad

    if (Reset_pos == false){
        impedance_terms.Target_pos_reset = 0;
    }
    else{
        impedance_terms.Target_pos_reset = (Lim_up_input + Lim_down_input)*deg_to_rad/2;
    }
}

void sensojoint_control::set_soft_stop_gain(double ks, double kd){
    Ks_ss = ks * Nm_to_mNm;
    Kd_ss = kd * Nm_to_mNm;
}

bool sensojoint_control::get_soft_stop_status(){
    return controller_config.joint_enabled;
}
/********************************************************************
 ************************ Weight compensation************************
 ********************************************************************/
void sensojoint_control::set_user_parameters(){        // get value for the leg

    // Length properties
    forearm_length_m    = 0.146         *leg_weight_comp.human_height_m;
    hand_length_m       = 0.108         *leg_weight_comp.human_height_m;
    leg_length_m        = (0.285-0.039) *leg_weight_comp.human_height_m;
    foot_length_m       = 0.039         *leg_weight_comp.human_height_m;


    // Centers of mass properties
    forearm_com_m   = 0.43  *forearm_length_m;
    hand_com_m      = 0.506 *hand_length_m;
    leg_com_m       = 0.433 *leg_length_m;
    foot_com_m      = 0.5   *foot_length_m;

    // Mass properties
    forearm_mass_kg = 0.016     *leg_weight_comp.human_weight_kg;
    hand_mass_kg    = 0.006     *leg_weight_comp.human_weight_kg;
    leg_mass_Kg     = 0.0465    *leg_weight_comp.human_weight_kg;
    foot_mass_Kg    = 0.0145    *leg_weight_comp.human_weight_kg +0.3; // + shoes weight

    // altezza gamba 0.530*H
    // altezza stinco 0.285*H
    // altezza caviglia 0.039*H

}

void sensojoint_control::set_robot_parameters(){

    // Centers of mass properties from the center of rotation
    link1_com       = 0.13;     //[m]
    link2_com       = 0.350;    //[m]
    ankle_shell_com = 0.350;    //[m]
    foot_holder_com = 0.450;    //[m]

    // Mass properties
    link1       = 0.55;     //[Kg]
    link2       = 0.45;      //[Kg]
    ankle_shell = 0.1;      //[Kg]
    foot_holder = 0.1;      //[Kg]

}
//************************************//
//** GRAVITY COMPENSATION ALGORITHM **//
//************************************//

void sensojoint_control::set_inverse_dynamics_parameters(){
    torque_terms.weight_assistance = 1.05;
    torque_terms.compensation_factor = 0.99;
    //set torque_terms.compensation_factor=0.2 for test bench without ankle shell and foot holder and link 2
    torque_terms.inertia_correction = 0.15;
    }




double sensojoint_control::
compute_inverse_dynamics(double position_rad){

    double gravity_torque_robot;
    double weight_compensation;
    double Inertia_torque_mNm;
    double g = -9.81;
    float  gear_ratio = 121.0;
    float  motor_inertia = 4.15*1E-5;

    float w                     = torque_terms.inertia_correction;
    float weight_assistance     = torque_terms.weight_assistance;
    float gravity_weight_factor = torque_terms.compensation_factor;

    gravity_torque_robot          = link1*g*sin(position_rad)*link1_com*Nm_to_mNm +
                                + link2*g*sin(position_rad)*link2_com*Nm_to_mNm +
                                + ankle_shell*g*sin(position_rad)*ankle_shell_com*Nm_to_mNm +
                                + foot_holder*g*sin(position_rad)*foot_holder_com*Nm_to_mNm;

    weight_compensation     = leg_mass_Kg*g*sin(position_rad)*leg_com_m*Nm_to_mNm +
                                + foot_mass_Kg*g*sin(position_rad)*(leg_length_m+foot_com_m)*Nm_to_mNm;

    Inertia_torque_mNm      = w * motor_inertia*pow(gear_ratio,2)*esmacat_sensojoint::get_motor_acceleration_value_filt()*Nm_to_mNm;

    torque_terms.inertia_torque = Inertia_torque_mNm;
    torque_terms.weight_compensation_torque = weight_compensation;
    torque_terms.gravity_robot_compensation_torque= gravity_torque_robot;

  //return (-1)*gravity_weight_factor*gravity_torque_robot+ Inertia_torque_mNm;
  return (-1)*gravity_weight_factor*gravity_torque_robot+ Inertia_torque_mNm - weight_assistance*weight_compensation + Inertia_torque_mNm;
 }

double sensojoint_control::get_inertia_torque_mNm(){ //not used,  maybe useless
    return torque_terms.inertia_torque;
}


/*************************************************/
/***********Manage control_mode*******************/
/*************************************************/

void sensojoint_control::set_control_mode(robot_control_mode mode){
    controller_config.control_mode = (robot_control_mode) mode;
}
robot_control_mode sensojoint_control::get_current_control_mode(){
    return (robot_control_mode) controller_config.control_mode;
}

//DISPLAY

robot_control_mode sensojoint_control::get_control_mode_display(){
    robot_control_mode mode;
    mode = static_cast<robot_control_mode>(controller_config.control_mode);
    return mode;
}


