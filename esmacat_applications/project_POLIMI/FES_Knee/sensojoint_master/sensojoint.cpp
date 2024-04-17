#include "sensojoint.h"

esmacat_sensojoint::esmacat_sensojoint(){
	esmacat_slave_vendor_id = 0x0000063a;
	esmacat_slave_product_id = 0x005b9729;
    statusword=0;
    modes_of_operation_display=0;

    position_actual_value=0;
    velocity_actual_value=0;
    torque_actual_value=0;
    output_torque_actual_value=0;
    secondary_position_value=0;
    secondary_velocity_value=0;
    timestamp=0;
    controlword=0;
    modes_of_operation=0;
    target_torque=0;
    target_position=0;
    target_velocity=0;
    target_output_torque=0;
    target_output_position=0;
    target_output_velocity=0;
    target_torque_offset=0;

    motor_accel_actual_value_filt = 0;
    motor_velocity_actual_value_filt = 0;
    prev_motor_velocity_rad_s = 0;
}

uint16_t esmacat_sensojoint::get_statusword(){
    return statusword;
}

/**
 * @brief esmacat_sensojoint::get_CoE_state
 * @return CoE state
 * Refer to CoE CiA 402, SENSOJOINT Software Manual (pg 13-14).
 */
CoE_state_t esmacat_sensojoint::get_CoE_state(){

    // Not ready to switch on
    if ((statusword & 0b0000000001001111) == 0b0000000000000000){
        state = static_cast<CoE_state_t>(STATE_NOT_READY_SWITCH_ON);
    }
    // Switch on disabled
    else if ((statusword & 0b0000000001001111) == 0b0000000001000000){
        state = static_cast<CoE_state_t>(STATE_SWITCH_ON_DISABLED);
    }
    // Ready to switch on
    else if ((statusword & 0b0000000001101111) == 0b0000000000100001){
        state = static_cast<CoE_state_t>(STATE_READY_SWITCH_ON);
    }
    // Switched on
    else if ((statusword & 0b0000000001101111) == 0b0000000000100011){
        state = static_cast<CoE_state_t>(STATE_SWITCHED_ON);
    }
    // Operation enabled
    else if ((statusword & 0b0000000001101111) == 0b0000000000100111){
        state = static_cast<CoE_state_t>(STATE_OPERATION_ENABLED);
    }
    // Fault state
    else if ((statusword & 0b0000000001001111) == 0b0000000000001000){
        state = static_cast<CoE_state_t>(STATE_FAULT);
    }

    return state;
}


CoE_mode_t esmacat_sensojoint::get_modes_of_operation_display_CoE(){
    CoE_mode_t mode;
    mode  = static_cast<CoE_mode_t>(modes_of_operation_display);
    return mode;
}



int8_t esmacat_sensojoint::get_modes_of_operation_display(){

return modes_of_operation_display;
}

int32_t esmacat_sensojoint::get_position_actual_value(){
    return position_actual_value;
}

double esmacat_sensojoint::get_position_actual_value_rad(){
    double incremental_encoder_resolution_cpt = 524288.0;
    return get_position_actual_value() * (2*M_PI/incremental_encoder_resolution_cpt);

}

int32_t esmacat_sensojoint::get_velocity_actual_value(){
    return velocity_actual_value;
}

double esmacat_sensojoint::get_velocity_actual_value_rad_s(){
    // Filtered Velocity (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)

    double gear_ratio = 121.0;
#define MIN_VELOCITY_RAD_S 1E-3
    double velocity_rad_s;
    velocity_rad_s = static_cast<double>(get_velocity_actual_value())/gear_ratio*rpm_to_rad_s/1000.0;
    if (abs(velocity_rad_s)<MIN_VELOCITY_RAD_S) velocity_rad_s = 0.0;
    return velocity_rad_s;
}

/**
 * @brief esmacat_sensojoint::get_motor_acceleration_value_filt
 * @return
 */
double esmacat_sensojoint::get_motor_acceleration_value_filt(){ // (esmacat_app_one_cycle_time_ms/1000)
    // NOTE: not motor accel (gear_ratio is included)
    // Compute velocity
    double motor_velocity_rad_s;
    double gear_ratio = 121.0;
    motor_velocity_rad_s = static_cast<double>(get_velocity_actual_value())/gear_ratio*rpm_to_rad_s/1000.0;

    // Filtered Velocity (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    double lambda = 0.2;
    motor_velocity_actual_value_filt = lambda*motor_velocity_rad_s+(1-lambda)*motor_velocity_actual_value_filt;

    // Compute finite difference
    double diff_motor_accel_actual_value = (motor_velocity_actual_value_filt - prev_motor_velocity_rad_s) / 0.001; // 1 kHz

    // Filtered Acceleration (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    lambda = 0.5;
    motor_accel_actual_value_filt = lambda*diff_motor_accel_actual_value+(1-lambda)*motor_accel_actual_value_filt;

    // Save previous values
    prev_motor_velocity_rad_s = motor_velocity_actual_value_filt;

    return motor_accel_actual_value_filt;
}




int16_t esmacat_sensojoint::get_torque_actual_value(){
    return torque_actual_value;
}

double esmacat_sensojoint::get_torque_actual_value_mNm(){
    double rated_torque_mNm = 546.0;
    double gear_ratio = 121.0;
    return static_cast<double>(get_torque_actual_value())/1000.0*rated_torque_mNm*gear_ratio;
}

int32_t esmacat_sensojoint::get_output_torque_actual_value_mNm(){
    return output_torque_actual_value;
}
double esmacat_sensojoint::get_output_torque_actual_value_mNm_filt(){
    // Filtered Torque (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    double lambda = 0.30;
    torque_actual_value_filt = lambda*static_cast<double>(get_output_torque_actual_value_mNm())+(1-lambda)*torque_actual_value_filt;

    return torque_actual_value_filt;
}

int32_t esmacat_sensojoint::get_secondary_position_value(){
    return secondary_position_value;
}
double esmacat_sensojoint::get_secondary_position_value_rad(){
    double incremental_encoder_resolution_cpt = 524288.0;
    double gear_ratio = 121.0;
    return get_secondary_position_value() *  (M_PI/incremental_encoder_resolution_cpt)/gear_ratio;
}


int32_t esmacat_sensojoint::get_secondary_velocity_value_mrpm(){
    return secondary_velocity_value;
}

double esmacat_sensojoint::get_secondary_velocity_value_rad_s(){
    // Filtered Velocity (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
//    double lambda = 0.5;
//    velocity_actual_value_filt = lambda*static_cast<double>(get_velocity_actual_value())+(1-lambda)*static_cast<double>(velocity_actual_value_filt);
//    return velocity_actual_value_filt;
    double gear_ratio = 121.0;
    #define MIN_VELOCITY_RAD_S 1E-3
    double velocity_rad_s;
    velocity_rad_s = static_cast<double>(get_secondary_velocity_value_mrpm())*rpm_to_rad_s/1000.0;
    if (abs(velocity_rad_s)<MIN_VELOCITY_RAD_S) velocity_rad_s = 0.0;
    return velocity_rad_s;
}

uint32_t esmacat_sensojoint::get_timestamp(){
    return timestamp;
}
void esmacat_sensojoint::set_controlword(uint16_t value){
    controlword = value;
}

/**
 * @brief esmacat_sensojoint::set_modes_of_operation
 * @param mode of operation (8,9,10,-108,-109,-110 for cyclic modes)
 */
void esmacat_sensojoint::set_modes_of_operation(int8_t value){
    modes_of_operation =value;
}

void esmacat_sensojoint::set_modes_of_operation_display_CoE(CoE_mode_t value){
modes_of_operation_display_CoE= value;
}


void esmacat_sensojoint::set_target_torque(int16_t value){
    target_torque = value;
}

void esmacat_sensojoint::set_target_torque_mNm(double desired_torque_mNm){
    double rated_torque_mNm = 546.0;
    double gear_ratio = 121.0;
    set_target_torque(static_cast<int16_t>(desired_torque_mNm*1000.0/(gear_ratio*rated_torque_mNm)));
}


void esmacat_sensojoint::set_target_position(int32_t value){
    target_position = value;
}
void esmacat_sensojoint::set_target_velocity(int32_t value){
    target_velocity = value;
}
void esmacat_sensojoint::set_target_output_torque(int32_t value){
    target_output_torque = value;
}
void esmacat_sensojoint::set_target_output_position(int32_t value){
    target_output_position = value;
}
void esmacat_sensojoint::set_target_output_velocity(int32_t value){
    target_output_velocity = value;
}
void esmacat_sensojoint::set_torque_offset(int16_t value){
    target_torque_offset = value;
}
void esmacat_sensojoint::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){
	unsigned char input_variable[29];
	unsigned char output_variable[27];
	input_variable[0] = *(ec_slave_inputs+0);
	input_variable[1] = *(ec_slave_inputs+1);
	input_variable[2] = *(ec_slave_inputs+2);
	input_variable[3] = *(ec_slave_inputs+3);
	input_variable[4] = *(ec_slave_inputs+4);
	input_variable[5] = *(ec_slave_inputs+5);
	input_variable[6] = *(ec_slave_inputs+6);
	input_variable[7] = *(ec_slave_inputs+7);
	input_variable[8] = *(ec_slave_inputs+8);
	input_variable[9] = *(ec_slave_inputs+9);
	input_variable[10] = *(ec_slave_inputs+10);
	input_variable[11] = *(ec_slave_inputs+11);
	input_variable[12] = *(ec_slave_inputs+12);
	input_variable[13] = *(ec_slave_inputs+13);
    input_variable[14] = *(ec_slave_inputs+14);
	input_variable[15] = *(ec_slave_inputs+15);
	input_variable[16] = *(ec_slave_inputs+16);
	input_variable[17] = *(ec_slave_inputs+17);
	input_variable[18] = *(ec_slave_inputs+18);
	input_variable[19] = *(ec_slave_inputs+19);
	input_variable[20] = *(ec_slave_inputs+20);
	input_variable[21] = *(ec_slave_inputs+21);
	input_variable[22] = *(ec_slave_inputs+22);
	input_variable[23] = *(ec_slave_inputs+23);
	input_variable[24] = *(ec_slave_inputs+24);
	input_variable[25] = *(ec_slave_inputs+25);
	input_variable[26] = *(ec_slave_inputs+26);
	input_variable[27] = *(ec_slave_inputs+27);
	input_variable[28] = *(ec_slave_inputs+28);

    statusword = +(input_variable[0] << 0)+(input_variable[1] << 8);
    modes_of_operation_display = +(input_variable[2] << 0);
    position_actual_value = +(input_variable[3] << 0)+(input_variable[4] << 8)+(input_variable[5] << 16)+(input_variable[6] << 24);
    velocity_actual_value = +(input_variable[7] << 0)+(input_variable[8] << 8)+(input_variable[9] << 16)+(input_variable[10] << 24);
    torque_actual_value = +(input_variable[11] << 0)+(input_variable[12] << 8);
    output_torque_actual_value = +(input_variable[13] << 0)+(input_variable[14] << 8)+(input_variable[15] << 16)+(input_variable[16] << 24);
    secondary_position_value = +(input_variable[17] << 0)+(input_variable[18] << 8)+(input_variable[19] << 16)+(input_variable[20] << 24);
    secondary_velocity_value = +(input_variable[21] << 0)+(input_variable[22] << 8)+(input_variable[23] << 16)+(input_variable[24] << 24);
    timestamp = +(input_variable[25] << 0)+(input_variable[26] << 8)+(input_variable[27] << 16)+(input_variable[28] << 24);

	output_variable[0] = 0;
	output_variable[1] = 0;
	output_variable[2] = 0;
	output_variable[3] = 0;
	output_variable[4] = 0;
	output_variable[5] = 0;
	output_variable[6] = 0;
	output_variable[7] = 0;
	output_variable[8] = 0;
	output_variable[9] = 0;
	output_variable[10] = 0;
	output_variable[11] = 0;
	output_variable[12] = 0;
	output_variable[13] = 0;
	output_variable[14] = 0;
	output_variable[15] = 0;
	output_variable[16] = 0;
	output_variable[17] = 0;
	output_variable[18] = 0;
	output_variable[19] = 0;
	output_variable[20] = 0;
	output_variable[21] = 0;
	output_variable[22] = 0;
	output_variable[23] = 0;
	output_variable[24] = 0;
	output_variable[25] = 0;
	output_variable[26] = 0;

    output_variable[0] =  +((controlword >> 0) & 0x00ff);
    output_variable[1] =  +((controlword >> 8) & 0x00ff);

    output_variable[2] =  +((modes_of_operation >> 0) & 0x00ff);

    output_variable[3] =  +((target_torque >> 0) & 0x00ff);
    output_variable[4] =  +((target_torque >> 8) & 0x00ff);

    output_variable[5] =  +((target_position >> 0) & 0x00ff);
    output_variable[6] =  +((target_position >> 8) & 0x00ff);
    output_variable[7] =  +((target_position >> 16) & 0x00ff);
    output_variable[8] =  +((target_position >> 24) & 0x00ff);

    output_variable[9] =  +((target_velocity >> 0) & 0x00ff);
    output_variable[10] =  +((target_velocity >> 8) & 0x00ff);
    output_variable[11] =  +((target_velocity >> 16) & 0x00ff);
    output_variable[12] =  +((target_velocity >> 24) & 0x00ff);

    output_variable[13] =  +((target_output_torque >> 0) & 0x00ff);
    output_variable[14] =  +((target_output_torque >> 8) & 0x00ff);
    output_variable[15] =  +((target_output_torque >> 16) & 0x00ff);
    output_variable[16] =  +((target_output_torque >> 24) & 0x00ff);

    output_variable[17] =  +((target_output_position >> 0) & 0x00ff);
    output_variable[18] =  +((target_output_position >> 8) & 0x00ff);
    output_variable[19] =  +((target_output_position >> 16) & 0x00ff);
    output_variable[20] =  +((target_output_position >> 24) & 0x00ff);

    output_variable[21] =  +((target_output_velocity >> 0) & 0x00ff);
    output_variable[22] =  +((target_output_velocity >> 8) & 0x00ff);
    output_variable[23] =  +((target_output_velocity >> 16) & 0x00ff);
    output_variable[24] =  +((target_output_velocity >> 24) & 0x00ff);

    output_variable[25] =  +((target_torque_offset >> 0) & 0x00ff);
    output_variable[26] =  +((target_torque_offset >> 8) & 0x00ff);

	*(ec_slave_outputs+0) = output_variable[0];
	*(ec_slave_outputs+1) = output_variable[1];
	*(ec_slave_outputs+2) = output_variable[2];
	*(ec_slave_outputs+3) = output_variable[3];
	*(ec_slave_outputs+4) = output_variable[4];
	*(ec_slave_outputs+5) = output_variable[5];
	*(ec_slave_outputs+6) = output_variable[6];
	*(ec_slave_outputs+7) = output_variable[7];
	*(ec_slave_outputs+8) = output_variable[8];
	*(ec_slave_outputs+9) = output_variable[9];
	*(ec_slave_outputs+10) = output_variable[10];
	*(ec_slave_outputs+11) = output_variable[11];
	*(ec_slave_outputs+12) = output_variable[12];
	*(ec_slave_outputs+13) = output_variable[13];
	*(ec_slave_outputs+14) = output_variable[14];
	*(ec_slave_outputs+15) = output_variable[15];
	*(ec_slave_outputs+16) = output_variable[16];
	*(ec_slave_outputs+17) = output_variable[17];
	*(ec_slave_outputs+18) = output_variable[18];
	*(ec_slave_outputs+19) = output_variable[19];
	*(ec_slave_outputs+20) = output_variable[20];
	*(ec_slave_outputs+21) = output_variable[21];
	*(ec_slave_outputs+22) = output_variable[22];
	*(ec_slave_outputs+23) = output_variable[23];
	*(ec_slave_outputs+24) = output_variable[24];
	*(ec_slave_outputs+25) = output_variable[25];
	*(ec_slave_outputs+26) = output_variable[26];
}


// ////////////////////////////////////////////////////////////////
// ////////////////////// IMPEDANCE CONTROL ///////////////////////
/// \brief agree_motor_controller::set_impedance_control_rad
/// \param impedance_control_setpoint_rad
/// \param gravity_torque_mNm
/// \return
///
//esmacat_err esmacat_sensojoint::set_impedance_control_rad(float impedance_control_setpoint_rad,float gravity_torque_mNm)
//{

//    float controller_torque_k_mNm =0;
//    float controller_torque_d_mNm =0;
//    float impedance_control_torque_mNm =0;
//    float impedance_control_error_rad = 0;
//    float friction_comp_torque_mNm = 0;
//    float soft_stop_torque_mNm = 0;
//    float impedance_control_max_error_rad = M_PI/8;
//    float impedance_control_k_mNm_per_rad = 0;
//    float impedance_control_d_mNm_s_per_rad = 0;

//    double velocity_stop_threshold_rad_per_sec = M_PI*10; //joint_impedance_control_config.max_velocity_threshold_rad_per_sec;
//    double max_load_stop_threshold_mNm = 10000;

//    // Stiffness Control
//    impedance_control_error_rad = impedance_control_setpoint_rad - get_encoder_position_radians();
//    if( impedance_control_error_rad > impedance_control_max_error_rad) impedance_control_error_rad = impedance_control_max_error_rad;
//    else if (impedance_control_error_rad < -impedance_control_max_error_rad) impedance_control_error_rad = -impedance_control_max_error_rad;
//    controller_torque_k_mNm =  impedance_control_k_mNm_per_rad * impedance_control_error_rad;

//    // Damping Control
//    controller_torque_d_mNm = - impedance_control_d_mNm_s_per_rad * get_encoder_filt_speed_radians();

//    // Impedance control
//    impedance_control_torque_mNm = controller_torque_k_mNm + controller_torque_d_mNm;

//    // Soft End-stops to prevend collision with hard End-Stops
//    /** Positive soft end-stop in radians */
//    float soft_stop_max_rad = hard_stop_max_rad - soft_to_hard_stop_offset_rad;
//    /** Negative soft end-stop in radians */
//    float soft_stop_min_rad = hard_stop_min_rad + soft_to_hard_stop_offset_rad;

//    // If beyond positive soft end-stop
//    if ((position_rad >= soft_stop_max_rad))
//    {
//        float scaled_travel_beyond_soft_stop_pos = (position_rad-soft_stop_max_rad)/soft_to_hard_stop_offset_rad;
//        soft_stop_torque_mNm = + soft_stop_max_torque_mNm/2
//                               - soft_stop_max_torque_mNm * (sigmoid(-scaled_travel_beyond_soft_stop_pos))
//                               - soft_stop_damping * get_encoder_filt_speed_radians();
//    }
//    // If beyond positive soft end-stop
//    else if ((position_rad <= soft_stop_min_rad))
//    {
//        float scaled_travel_beyond_soft_stop_neg = (soft_stop_min_rad - position_rad )/soft_to_hard_stop_offset_rad;
//        soft_stop_torque_mNm =  - soft_stop_max_torque_mNm/2
//                                + soft_stop_max_torque_mNm * sigmoid(-scaled_travel_beyond_soft_stop_neg)
//                                - soft_stop_damping * get_encoder_filt_speed_radians();
//    }

//    // Max velocity stop
//    if (fabs(velocity_rad_per_s) >= velocity_stop_threshold_rad_per_sec){
//        stop_motor();
//        std::cout<<"ERR_MEASURED_VELOCITY_OUTSIDE_RANGE. EPOS Disabled"<<std::endl;
//    }else{

//    }

//    // Max load stop
//    if (fabs(loadcell_torque_mNm) >= max_load_stop_threshold_mNm){
//        stop_motor();
//        std::cout<<"ERR_MEASURED_LOAD_OUTSIDE_RANGE. EPOS Disabled"<<std::endl;
//    }


//    // Summing up Impedance Controller and Feedforward terms */

//    // Filtered Setpoint (Exponential Low-Pass Filter)
//    // y(k) = (1-a) * y(k-1) + a * x(k)
//    double lambda = 1;
//    filtered_setpoint_torque_mNm = (1-lambda)*filtered_setpoint_torque_mNm
//                                   + lambda  *(gravity_torque_mNm + impedance_control_torque_mNm + friction_comp_torque_mNm + soft_stop_torque_mNm);
//    // Torque loop
//    set_torque_control_mNm(filtered_setpoint_torque_mNm);

//    /** Save Impedance Controller Status */
//    joint_impedance_control_status.impedance_control_setpoint_rad    = impedance_control_setpoint_rad;
//    joint_impedance_control_status.impedance_control_error_rad       = impedance_control_error_rad;
//    joint_impedance_control_status.impedance_control_torque_mNm      = impedance_control_torque_mNm;
//    joint_impedance_control_status.impedance_control_k_mNm_per_rad   = impedance_control_k_mNm_per_rad;
//    joint_impedance_control_status.impedance_control_d_mNm_s_per_rad = impedance_control_d_mNm_s_per_rad;
//    joint_impedance_control_status.gravity_torque_mNm                = gravity_torque_mNm;
//    joint_impedance_control_status.filtered_setpoint_torque_mNm      = filtered_setpoint_torque_mNm;

//    return NO_ERR;
//}
