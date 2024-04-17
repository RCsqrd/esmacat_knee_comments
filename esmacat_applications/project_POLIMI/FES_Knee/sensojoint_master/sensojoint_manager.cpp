/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the EtherCAT Arduino Shield by Esmacat slave example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "sensojoint_manager.h"
#include <unistd.h>
#include "sensojoint.h"
#include "math.h"
#include "sensojoint_structs.h"
#include <fstream>
#include <iostream>


#define MAX_LOOPS 1*60*1000

//using std::cin;
using namespace std;

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 */
void sensojoint_manager::assign_slave_sequence(){
    // tell the master what type of slave is at which point in the chain
//    assign_esmacat_slave_index(&ecat_as,0);
     assign_esmacat_slave_index(&ecat_sensojoint,0);

}

/**
 * @brief Configure your Esmacat slave.
 * Link Esmacat slave object with the actual Esmacat slave in the EtherCAT communication chain.
 * Functions beginning with 'configure slave' must only be executed in this function
 */
void sensojoint_manager::configure_slaves(){

    // ecat_sensojoint.configure_control_parameters(1000);
    // add initialization code here
    // Functions starting with "configure_slave" work only in configure_slave() function
}


/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void sensojoint_manager::init()
{
    // Initializing the shared memory
    if (sensojoint_shared_memory.init())
    {
        PLOGI << "User Interface shared memory initialized with key " << hex << static_cast<int>(sensojoint_shared_memory.get_shared_memory_key()) << dec ;    // start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        sensojoint_shared_memory.detach_shared_memory();
    }

    // Initializes shared memory mode/command/status
    /**
    sensojoint_shared_memory.set_sensojoint_command(robot_control_mode::standby);
    sensojoint_shared_memory.set_sensojoint_status(robot_control_mode::standby); */

    sensojoint_manager::readsharedmemory();
    sensojoint_manager::initialize_write2file();
//    sensojoint_manager::read_file();

    //position_offset_rad = ecat_sensojoint.get_position_actual_value_rad();
    position_offset_rad = (M_PI*340)/180;

    ecat_sensojoint.enable_joint(true) ;    /** initialize joint_enable
                                                 ** if is TRUE the joint will enter in operational mode as soon as the program run
                                                 ** if is FALSE the joint wait input from shared memory to enter in operational mode*/
    // set up robot geometry and human weight
    ecat_sensojoint.set_inverse_dynamics_parameters();
    ecat_sensojoint.set_robot_parameters();
    ecat_sensojoint.set_user_parameters();

    // set up the controller max torque and max velocity and the error treshold for impedance control
    ecat_sensojoint.sensojoint_control_treshold_parameters();
    sensojoint_manager::read_motor_value();

    //set the values for the feedback part of the impedance control
    ecat_sensojoint.set_impedance_control_Kgain(70*Nm_to_mNm);
    ecat_sensojoint.set_impedance_control_Dgain(4*Nm_to_mNm);

    //MODIFY : use this function to update the soft stop from shared memory
    ecat_sensojoint.set_soft_stop_gain(35,3.5);   //(stiffness [Nm/rad], damping [Nm/(rad/sec)] )
    ecat_sensojoint.set_soft_stop_parameters(85.0, -12.0, 5, true); // [deg] lim up - lim low - amplitude soft stop - reset position
    total_rep_counter=15; //era 5
    //ref_position=(motor_terms.position_actual_value - (POS_OFFSET));  //necessario per avviare direttamente modalità controllo ad impedenza senza standby evitando riferimento senza offset
//  set up the initial desired control modality of operation
    controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_velocity_output;

    torque_control_interim = ecat_sensojoint.get_torque_control_terms();
    impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();


    sensojoint_manager::initialize_interface_parameters();

//    PLOGW << "impedance terms" << impedance_terms_interim.K_gain <<endl;
    sensojoint_manager::writesharedmemory();
//    PLOGW << "In Mod Operation"<<controller_config_interim.motor_mode_of_operation <<endl;
     PLOGW << "total time sigma double" << total_time_double_trajectory_sigma <<endl;
}

/**
 * @brief Executes functions at the defined loop rate
 */
void sensojoint_manager::loop(){
    // Read from shared-memory-segment

    sensojoint_manager::readsharedmemory();
    sensojoint_manager::read_motor_value();
    //update variable from sensojoint_control.cpp

    /*   if (loop_cnt <= 20) { PLOGW << "In Mod Operation"<<controller_config_interim.motor_mode_of_operation <<endl; }
//    controller_config_interim = ecat_sensojoint.get_controller_configuration();   */


    //Sensojoint Inizialization procedure
    // fault check end reset//update variable from sensojoin_interface.cpp

    if(ecat_sensojoint.get_CoE_state() == fault){
        PLOGE << "SensoJoint is in Fault State";
        ecat_sensojoint.set_controlword(128);
        PLOGI << "ERROR RESET";
        //continue;
    }
    //state machine steps:
    if (power_up == false){
        if (ecat_sensojoint.get_CoE_state() == not_ready_to_switch_on){    // not ready to switch on -> to switch on disable
            ecat_sensojoint.set_controlword(128);
            PLOGI << "OPERATING: "   << dec << CoE_state_labels[ecat_sensojoint.get_CoE_state()];
        }
        if (ecat_sensojoint.get_CoE_state() == switch_on_disabled || ecat_sensojoint.get_CoE_state() == not_ready_to_switch_on){    //switched on disable -> ready to switch on
            ecat_sensojoint.set_controlword(6);
            PLOGI << "OPERATING: "   << dec << CoE_state_labels[ecat_sensojoint.get_CoE_state()];
        }
        if (ecat_sensojoint.get_CoE_state() == ready_to_switch_on){  //ready to switch on ->switched on
            ecat_sensojoint.set_controlword(7);
            PLOGI << "OPERATING: "   << dec << CoE_state_labels[ecat_sensojoint.get_CoE_state()];
        }


        if (ecat_sensojoint.get_CoE_state() == switched_on){
            PLOGI << "Power up confirmed"<<endl;
            power_up = true;
        }
    }
    else if(power_up ==true) {


        ref_position = (motor_terms.position_actual_value - (POS_OFFSET)); //Correction on the absolute position wrt mounting angle
        motor_terms.position_actual_value = ref_position;

        error_off = target_pos_reset - ref_position;

        control_input_command.control_mode_input = ecat_sensojoint.get_current_control_mode();

//        controller_config_interim.motor_mode_of_operation=static_cast<CoE_mode_t>(cyclic_synchronous_torque_output);
//        controller_config_interim.motor_mode_of_operation = ecat_sensojoint.get_modes_of_operation_display_CoE();

        //update variable from sensojoin_interface.cpp
//       ecat_sensojoint.set_soft_stop_parameters(control_input_command.Lim_up_input, control_input_command.Lim_down_input, control_input_command.amplitude_soft_stops_input, true); // [deg] lim up - lim low - amplitude soft stop - reset position

        controller_config_interim.control_mode = control_input_command.control_mode_input;

        impedance_terms_interim.K_gain = control_input_command.K_gain_input;
        impedance_terms_interim.D_gain = control_input_command.D_gain_input;

//        // Change control mode
//        if(controller_config_interim.control_mode != prev_mode){
//            PLOGW << "New modality selected: " << controller_config_interim.control_mode << endl;
//            prev_mode = controller_config_interim.control_mode;
//            elapsed_time_ms_offset = elapsed_time_ms;
//            target_freeze_pos = ref_position;
//        }

        // if the joint is already enabled, disable th20e brake and activate the joint in zero torque mode
        if (controller_config_interim.joint_enabled == false){
            controller_config_interim.control_mode = robot_control_mode::standby; // settare che da commandpo da shared memory cambio modalità ed esco da qua
            if (loop_cnt%1000 == 0){PLOGI << " Joint disable --> stand-by "<<endl;}
        }
        else {

            ecat_sensojoint.set_modes_of_operation(static_cast<int8_t>(controller_config_interim.motor_mode_of_operation));
            //ecat_sensojoint.set_target_output_torque(0);
            ecat_sensojoint.set_controlword(15);
            if (loop_cnt%1000 == 0){PLOGI << "SWITCHED ON"<<endl;}   //It will automaticaly transition to ENABLE OPERATION
            if (ecat_sensojoint.get_CoE_state() == operation_enabled) {
            if (loop_cnt%1000 == 0){PLOGI << "Enabling joints"<<endl;}
                //ecat_sensojoint.set_control_mode(standby); // era freeze_control
                if(prev_mode==0.0) {
                    ecat_sensojoint.set_control_mode(standby);
                    prev_mode = controller_config_interim.control_mode;
                }
                else {
                    if(controller_config_interim.control_mode != static_cast<robot_control_mode>(prev_mode)) {
                                PLOGW << "New modality selected: " << controller_config_interim.control_mode << endl;
                                start_trajectory=false;
                                prev_mode = controller_config_interim.control_mode;
                                elapsed_time_ms_offset = elapsed_time_ms;
                                target_freeze_pos = ref_position;
                }
            }
            }
        }

        //********** Compute feedfaward torque term: Gravity compensation, inertia***********//
        //[Nmm]  weight compesation

         inverse_dynamics_torque = ecat_sensojoint.compute_inverse_dynamics(ref_position);
       if(loop_cnt%1000 == 0) {PLOGW<<" Mean_Err_Actual "<<inverse_dynamics_torque<<endl;}
        //update variable from sensojoint_control.cpp
        torque_control_interim = ecat_sensojoint.get_torque_control_terms();
        impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();
        //if(loop_cnt%1000 == 0) {PLOGW<<" Mean_Err_Actual "<<Error_actual<<endl;}
//      if (loop_cnt%1000 == 0){PLOGI << "Trial Value"<<trial_value<<endl;}
/*    if (loop_cnt%1000 == 0){//PLOGW<< "WeightCompensationTorque"<<torque_control_interim.weight_compensation_torque<<endl;}
        PLOGW<< "WeightAssist"<<torque_control_interim.weight_assistance<<endl;}
       PLOGW<< "Torque Control Output"<<torque_control_interim.control_output<<endl;}
        PLOGW<< "accel"<< fixed << setprecision(3)  << ecat_sensojoint.get_motor_acceleration_value_filt()<< endl;}
        PLOGW<< "CompFactor"<<torque_control_interim.compensation_factor<<endl;
         PLOGW<< "Inertia Correction"<<torque_control_interim.inertia_correction<<endl;}
        PLOGW<< "WeightCompensationTorqueBefScal"<<torque_control_interim.weight_compensation_torque_before_scaling<<endl;}
        PLOGW<< "Max_Velocity_Threshold"<<torque_control_interim.max_velocity_threshold<<endl;
        PLOGW<< "Inertia Torque"<<torque_control_interim.inertia_torque<<endl; }
        PLOGW<< "max_load_stop_threshold"<<torque_control_interim.max_load_stop_threshold<<endl; }  */

//    if (ecat_sensojoint.get_modes_of_operation_display_CoE() != CoE_mode_t::cyclic_synchronous_torque_output){
//        ecat_sensojoint.set_modes_of_operation(static_cast<int8_t>(cyclic_synchronous_torque_output));}



        //********************************//
        //*****Control mode Selection*****//
        //********************************//

        switch(static_cast<int>(controller_config_interim.control_mode)) {

        case robot_control_mode::quit:

            // Parte aggiunta da robot_control_mode::stop

            sensojoint_manager::quit();

            //
            break;

        case robot_control_mode::stop:
            PLOGW << "The Motor will arrest its motion: "<< endl;   //INSERIRE SMOOTH FUNCTION
             ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
             arrest_smooth_factor = smooth_start_func_tanh(4*(1-(elapsed_time_ms-elapsed_time_ms_offset)/arrest_sm_int_time));
             motor_terms.velocity_actual_value=ecat_sensojoint.get_velocity_actual_value_rad_s();
             ecat_sensojoint.set_target_output_velocity(arrest_smooth_factor*motor_terms.velocity_actual_value);
             if (abs(motor_terms.velocity_actual_value)< 0.005 ) {
            ecat_sensojoint.set_target_output_velocity(0);
             }
            break;

        case robot_control_mode::standby:

            if (controller_config_interim.joint_enabled == true  && ecat_sensojoint.get_CoE_state() == switched_on) {
                ecat_sensojoint.set_controlword(15);
                ecat_sensojoint.set_target_output_torque(0);
                if (loop_cnt%1000 == 0){PLOGI << "SWITCHED ON"<<endl;}   //It will automaticaly transition to ENABLE OPERATION
                if (ecat_sensojoint.get_CoE_state() == operation_enabled){
                    if (loop_cnt%1000 == 0){PLOGI << "Enabling joint"<<endl;}
                }
            }
            break;
        case robot_control_mode::zerotorque_control:
            if (ecat_sensojoint.get_modes_of_operation_display_CoE() != CoE_mode_t::cyclic_synchronous_torque_output){
                ecat_sensojoint.set_modes_of_operation(cyclic_synchronous_torque_output);
                controller_config_interim.motor_mode_of_operation=static_cast<CoE_mode_t>(cyclic_synchronous_torque_output);                              ;
                if (loop_cnt%1000 == 0){PLOGI << "Set modes of operation to CYCLIC_SYNC_TORQUE_OUTPUT"<<endl;}
            }
            else{
                ecat_sensojoint.torque_control(0, elapsed_time_ms-elapsed_time_ms_offset);
                torque_control_interim.control_output=0;
            }

            break;

        case robot_control_mode::freeze_control:
            //set the rigth sensojoint mode
            if (ecat_sensojoint.get_modes_of_operation_display_CoE() != CoE_mode_t::cyclic_synchronous_torque_output) {
                ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
                controller_config_interim.motor_mode_of_operation=cyclic_synchronous_torque_output;
                if (loop_cnt%1000 == 0) {PLOGI << "Set modes of operation to CYCLIC_SYNC_TORQUE_OUTPUT"<<endl;}
            }

            impedance_control_error_freeze_rad = target_freeze_pos - ref_position;
            target_impedance_control_torque_freeze_mNm = Ks_freeze*impedance_control_error_freeze_rad - Kd_freeze*motor_terms.velocity_actual_value;
            //weight compensation

            torque_output = target_impedance_control_torque_freeze_mNm+inverse_dynamics_torque;// + torque_control_interim.weight_assistance + torque_control_interim.inertia_torque;
            torque_control_interim.control_output=torque_output;
            //Motor torque input
            ecat_sensojoint.set_target_output_torque(torque_output);
            break;


        case robot_control_mode::antig_control:   case robot_control_mode::transparent_control:
            //set the rigth sensojoint mode: here CYCLIC_SYNC_TORQUE_OUTPUT
//            if (ecat_sensojoint.get_modes_of_operation_display_CoE() != CoE_mode_t::cyclic_synchronous_torque_output){
//                ecat_sensojoint.set_modes_of_operation(cyclic_synchronous_torque_output);

                if (ecat_sensojoint.get_modes_of_operation_display_CoE() != static_cast<CoE_mode_t>(cyclic_synchronous_torque_output )) {
                 ecat_sensojoint.set_modes_of_operation(static_cast<int8_t>(cyclic_synchronous_torque_output));
                 controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_torque_output;
                 PLOGW << "mode of operation"<<controller_config_interim.motor_mode_of_operation<<endl;

        //     ecat_sensojoint.set_modes_of_operation_display_CoE(cyclic_synchronous_torque_output);

              if (loop_cnt%1000 == 0){PLOGI << "Set modes of operation to CYCLIC_SYNC_TORQUE_OUTPUT"<<endl;}
            }

            if (controller_config_interim.control_mode == robot_control_mode::antig_control) {

                torque_output = (-1)*torque_control_interim.compensation_factor*torque_control_interim.gravity_robot_compensation_torque;

                //inserire calibrazione offset del sensore direttamente nel controllo
            }

            if (controller_config_interim.control_mode == robot_control_mode::transparent_control) {

                torque_output = torque_control_interim.inertia_torque;

//                Pm2_actual=0.001;
            }

           torque_control_interim.control_output=torque_output;
           ecat_sensojoint.torque_control_with_soft_stop(torque_output,elapsed_time_ms - elapsed_time_ms_offset);
//           ecat_sensojoint.torque_control(torque_output,elapsed_time_ms - elapsed_time_ms_offset);
           arrest_smooth_factor=1-smooth_start_func_tanh(4.0*(elapsed_time_ms-elapsed_time_ms_offset)/arrest_sm_int_time);
/*
           interim_time_exercise = (elapsed_time_ms-elapsed_time_ms_offset_exercise);
           //Calculate values for useful indexes
           Pm2_average=(Pm2_1+Pm2_2+Pm2_3+Pm2_4+Pm2_5)/5;
           time_scaling_f=70000;
           P1=P1 = EXERCISE_AMPLITUDE/pow((EXERCISE_DURATION-round(time_scaling_f*Pm2_average))/2,(P3+P5));
           P4=P4 = P2+EXERCISE_DURATION -round(time_scaling_f*Pm2_average);
           interim_setpoint_exercise = P0 - P1*pow((count-P2),P3) * pow((P4-count),P5);


           // Restart Beta-Function
           if(interim_time_exercise > (EXERCISE_DURATION-round(time_scaling_f*Pm2_average)) ) {
              PLOGW<<"Total trajectory time"<<interim_time_exercise<<endl;
//               PLOGW<<"Average error"<<Pm2_average<<endl;
               elapsed_time_ms_offset_exercise = elapsed_time_ms;
               start_trajectory = false;
                }
               target_traj  = interim_setpoint_exercise; // WARNING: Trajectory generated in the real-time code to avoid quantization errors.
               error_traj = interim_setpoint_exercise - (ecat_sensojoint.get_position_actual_value_rad() - (POS_OFFSET));
               Pm2_actual=Pm2_1;
               count++;
               if (count == static_cast<int>(EXERCISE_DURATION-time_scaling_f*Pm2_average) ) {
                   count = 0;
                   repetition_counter=repetition_counter+1;

               }
*/

       break;

        case robot_control_mode::impedance_control_beta:

            if (sensojoint_shared_memory.data->modality==802){
                ind = sensojoint_shared_memory.data->ind_act_pos;
                P0 = sensojoint_shared_memory.data->betafunction_sm[ind];
            }
            else {
                P0 = EXERCISE_START;
            }


            // reach trajectory starting point
            if (start_trajectory == false) {
                ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_VELOCITY_OUTPUT);
                controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_velocity_output;
                velocity_control = Ks_vel * (P0 - ref_position)*60*M_PI*10 + Kd_vel*motor_terms.velocity_actual_value;    //mrpm
                if (velocity_control > 4000.0) {
                    velocity_control= 4000.0;
                }
                if (loop_cnt%100 == 0) {/*: public esmacat_application*/
                   PLOGI<<"Velocity control target: " << velocity_control<<endl;
//                 PLOGI<<"stifness contribution target: " << Ks_vel * (go_to_traj_starting_pos-ref_position)<<endl;
//                 PLOGI<<"Damping contribution target: " <<Kd_vel*ecat_sensojoint.get_velocity_actual_value_rad_s()<<endl;
                }
                if(sensojoint_shared_memory.data->modality==802){
                    count = ind;
                }
                else {
                    count = 0;
                }
                //count=0;


                ecat_sensojoint.set_target_output_velocity(static_cast<int32>(velocity_control));
                if (abs(P0 - ref_position) < M_PI/180) {  // +/- 1°
                    ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
                    controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_torque_output;
//                    if(sensojoint_shared_memory.data->modality==802){ //802
////                        P0 = sensojoint_shared_memory.data->motor_terms_sm.position_actual_value;
//                        //Metti start=true se sono passati 5 secondi o se è arrivato il trigger
//                        if (elapsed_time_ms>=elapsed_time_ms_offset_exercise+5000 || sensojoint_shared_memory.data->ex_start == true){
//                            elapsed_time_ms_offset_exercise = elapsed_time_ms;
//                            start_trajectory = true;
//                            if  (repetition_counter == 0) {
//                                repetition_counter=1;
//                            }

//                        }
//                        else if(elapsed_time_ms<elapsed_time_ms_offset_exercise+5000 || sensojoint_shared_memory.data->ex_start == false) {
//                            elapsed_time_ms_offset_exercise = elapsed_time_ms;
//                            ecat_sensojoint.set_control_mode(freeze_control);
//                        }

//                    }
//                    else {
//                        elapsed_time_ms_offset_exercise = elapsed_time_ms;
//                        start_trajectory = true;
//                        if  (repetition_counter == 0) {
//                            repetition_counter=1;
//                        }
//                    }
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    start_trajectory = true;
                    if  (repetition_counter == 0) {
                        repetition_counter=1;
                    }


                }
            }
            // Compute time variable
//            interim_time_exercise = (elapsed_time_ms-elapsed_time_ms_offset_exercise);
            interim_time_exercise = (elapsed_time_ms-elapsed_time_ms_offset_exercise);


            //Calculate values for useful indexes
// it can be changed to a function of sensojoint control and then an update of a structure in struct that cointains all the indexes




            if (start_trajectory ==true) {
                if (loop_cnt%500== 0 ) {PLOGW<<"Repetition Number:"<<repetition_counter<<endl;}
                // Beta-Function trajectory generation

                if(sensojoint_shared_memory.data->modality==802){
                    interim_setpoint_exercise = sensojoint_shared_memory.data->betafunction_sm[count];
                }

                else  {
                    P1 = (sensojoint_shared_memory.data->rom)/pow(EXERCISE_DURATION/2,(P3+P5));
                    interim_setpoint_exercise = P0 + P1*pow((count-P2),P3) * pow((P4-count),P5);
                }



                //Beta-function velocity value(derivative of beta trajectory function)
                nominal_velocity=P1*pow(1000.0,8)*(P3*pow(((P4-count)/1000.0),P5)*pow(count/1000.0,P3-1)-P5*pow(((P4-count)/1000.0),P5-1)*pow(count/1000.0,P3));

                // Restart Beta-Function
                if(interim_time_exercise > (EXERCISE_DURATION) ) {
                    PLOGW<<"Total trajectory time"<<interim_time_exercise<<endl;
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    start_trajectory = false;
                    //        support_repetitions++;
                    //        PLOGW << "Repetition #" << support_repetitions;
                }

                // Compute trajectory and run outer impedance control
                //target_k     = 100.0*Nm_to_mNm;
                //target_d     = 10*Nm_to_mNm;
                target_traj  = interim_setpoint_exercise; // WARNING: Trajectory generated in the real-time code to avoid quantization errors.
                error_traj = interim_setpoint_exercise - (ecat_sensojoint.get_position_actual_value_rad() - (POS_OFFSET));
                //target_torq  = target_k * error_traj - target_d * motor_terms.velocity_actual_value;

                inverse_dynamics_torque = ecat_sensojoint.compute_inverse_dynamics(ref_position);
                weight_compensation_torque=(-1)*torque_control_interim.compensation_factor*torque_control_interim.gravity_robot_compensation_torque;//[Nmm]
                //target_torq = target_torq + weight_compensation_torque;
                //torque_control_interim.control_output=target_torq;

                ecat_sensojoint.impedance_control(target_traj ,inverse_dynamics_torque,interim_time_exercise);


                //Update the terms in the torque_interim and impedance_interim structures
                torque_control_interim = ecat_sensojoint.get_torque_control_terms();
                impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();

                //Calculate values for useful indexes
// it can be changed to a function of sensojoint control and then an update of a structure in struct that cointains all the indexes

                     Error_squared_actual=Error_squared_actual+(pow(error_traj,2)/(total_rep_counter*EXERCISE_DURATION));
//                   Pm2_actual=Pm2_actual+(error_traj^(2))/(total_rep_counter*(EXERCISE_DURATION-time_scaling_f*Pm2_average)));

             //      if( loop_cnt%1000 == 0){PLOGW<<" Torque Tot"<<impedance_terms_interim.torque_tot<<endl;}
             //      PLOGW<<" Gravity Torque"<<weight_compensation_torque<<endl;}

                        // Update Status
                count++;
                if (count == static_cast<int>(EXERCISE_DURATION) ){
                    count = 0;
                    repetition_counter=repetition_counter+1;
                }
            }
            sensojoint_shared_memory.data->start_traj = start_trajectory; //added
            sensojoint_shared_memory.data->inv_dyn_torque = inverse_dynamics_torque;
            break;

        case robot_control_mode::impedance_control_sigma:
            // reach trajectory starting point
            if (start_trajectory == false){
                ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_VELOCITY_OUTPUT);
                velocity_control = Ks_vel * (P0 - ref_position)*60*M_PI*10 + Kd_vel*motor_terms.velocity_actual_value;    //mrpm
                if (loop_cnt%100 == 0){/*: public esmacat_application*/
                    PLOGI<<"Velocity control target: " << velocity_control<<endl;
                    if (velocity_control > 4000.0) {
                        velocity_control= 4000.0;
                    }
//                    PLOGI<<"stifness contribution target: " << Ks_vel * (go_to_traj_starting_pos-ref_position)<<endl;
//                    PLOGI<<"Damping contribution target: " <<Kd_vel*ecat_sensojoint.get_velocity_actual_value_rad_s()<<endl;
                }
                ecat_sensojoint.set_target_output_velocity(static_cast<int32>(velocity_control));
                if (abs(P0 - ref_position)< M_PI/64){
                    ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
                    controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_torque_output;
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    start_trajectory = true;
                    if  (repetition_counter == 0) {
                        repetition_counter=1;
                     }
                }
            }
            // Compute time variable
            interim_time_exercise = (elapsed_time_ms-elapsed_time_ms_offset_exercise);




            if (start_trajectory ==true ) {
               if (loop_cnt%500== 0 ) {PLOGW<<"Repetition Number:"<<repetition_counter<<endl;}
            //Sigma function
            //Division of ms by 1000 to get to not have too big exponents to the exponential function
           if (interim_time_exercise <= EXERCISE_DURATION) {
                interim_setpoint_exercise = EXERCISE_AMPLITUDE*(-1)/( 1 + exp(-(count-EXERCISE_DURATION/2)/1000))+P0;
}
           else if (interim_time_exercise > EXERCISE_DURATION && interim_time_exercise <= total_time_double_trajectory_sigma ) {
                 interim_setpoint_exercise = EXERCISE_AMPLITUDE*(-1)/( 1 + exp(-(EXERCISE_DURATION/2)/1000))+P0+EXERCISE_AMPLITUDE/( 1 + exp(-(count-EXERCISE_DURATION/2)/1000));
}

                // Restart Sigma-Function
                 if(interim_time_exercise > total_time_double_trajectory_sigma) {
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    //        support_repetitions++;
                    //        PLOGW << "Repetition #" << support_repetitions;
                }

             // Compute trajectory and run outer impedance control
//                target_k     = 40.0*Nm_to_mNm;
//                target_d     = 5*Nm_to_mNm;
                  target_traj  = interim_setpoint_exercise;                       // WARNING: Trajectory generated in the real-time code to avoid quantization errors.
                  error_traj = interim_setpoint_exercise - (ecat_sensojoint.get_position_actual_value_rad() - (POS_OFFSET));
//                target_torq  = target_k* error_traj - target_d*motor_terms.velocity_actual_value;
                  inverse_dynamics_torque = ecat_sensojoint.compute_inverse_dynamics(ref_position);
                  weight_compensation_torque=(-1)*torque_control_interim.compensation_factor*torque_control_interim.gravity_robot_compensation_torque;

//                target_torq = target_torq + weight_compensation_torque;
//                torque_control_interim.control_output=target_torq;

                  ecat_sensojoint.impedance_control(target_traj ,weight_compensation_torque,interim_time_exercise);
//                  if(loop_cnt%500 == 0) { PLOGW<<"Ref traj"<<target_traj<<endl; };

              //Update the terms in the torque_interim and impedance_interim structures
                  torque_control_interim = ecat_sensojoint.get_torque_control_terms();
                  impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();
                   //Calculate values for useful indexes
    // it can be changed to a function of sensojoint control and then an update of a structure in struct that cointains all the indexes

                  Error_squared_actual=Error_squared_actual+(pow(error_traj,2)/(total_rep_counter*EXERCISE_DURATION));


               // Update Status
                 count++;
                 if (count == EXERCISE_DURATION){
                     count = 0;
                     repetition_counter=repetition_counter+1;
                 }
            }
            break;

        case robot_control_mode::adaptive_traj_control:

            // reach trajectory starting point
            if (start_trajectory == false) {
                ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_VELOCITY_OUTPUT);
                controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_velocity_output;
                velocity_control = Ks_vel * (P0 - ref_position)*60*M_PI*10 + Kd_vel*motor_terms.velocity_actual_value;    //mrpm
                if (loop_cnt%100 == 0) {/*: public esmacat_application*/
                   PLOGI<<"Velocity control target: " << velocity_control<<endl;
//                 PLOGI<<"stifness contribution target: " << Ks_vel * (go_to_traj_starting_pos-ref_position)<<endl;
//                 PLOGI<<"Damping contribution target: " <<Kd_vel*ecat_sensojoint.get_velocity_actual_value_rad_s()<<endl;
                }
                ecat_sensojoint.set_target_output_velocity(static_cast<int32>(velocity_control));
                if (abs(P0 - ref_position)< M_PI/64) {
                    ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
                    controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_torque_output;
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    start_trajectory = true;
                    if  (repetition_counter == 0) {
                        repetition_counter=1;
                     }

                }
            }
            // Compute time variable
            interim_time_exercise = (elapsed_time_ms-elapsed_time_ms_offset_exercise);
            //Calculate values for useful indexes
// it can be changed to a function of sensojoint control and then an update of a structure in struct that cointains all the indexes




            if (start_trajectory ==true) {
                if (loop_cnt%500== 0 ) {PLOGW<<"Repetition Number:"<<repetition_counter<<endl;}
                // Beta-Function trajectory generation
                Error_average=(Error_1+Error_2+Error_3+Error_4+Error_5)/5;
                time_scaling_f=30000;
                P1 = EXERCISE_AMPLITUDE/pow((EXERCISE_DURATION-round(time_scaling_f*Error_average))/2,(P3+P5));
                P4 = P2+EXERCISE_DURATION -round(time_scaling_f*Error_average);
                interim_setpoint_exercise = P0 + P1*pow((count-P2),P3) * pow((P4-count),P5);


                //Beta-function velocity value(derivative of beta trajectory function)
                nominal_velocity=(EXERCISE_AMPLITUDE/pow((EXERCISE_DURATION)/2,(P3+P5))*P3*pow((EXERCISE_DURATION-count),P3-1)* pow((count),P3-1)*(2*count-P4));

                // Restart Beta-Function
                if(interim_time_exercise > (EXERCISE_DURATION-round(time_scaling_f*Error_average)) ) {
                    PLOGW<<"Total trajectory time"<<interim_time_exercise<<endl;
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    start_trajectory = false;
                    //        support_repetitions++;
                    //        PLOGW << "Repetition #" << support_repetitions;
                }

                // Compute trajectory and run outer impedance control
                //target_k     = 100.0*Nm_to_mNm;
                //target_d     = 10*Nm_to_mNm;
                target_traj  = interim_setpoint_exercise; // WARNING: Trajectory generated in the real-time code to avoid quantization errors.
                error_traj = interim_setpoint_exercise - (ecat_sensojoint.get_position_actual_value_rad() - (POS_OFFSET));
                //target_torq  = target_k * error_traj - target_d * motor_terms.velocity_actual_value;

                inverse_dynamics_torque = ecat_sensojoint.compute_inverse_dynamics(ref_position);
                weight_compensation_torque=(-1)*torque_control_interim.compensation_factor*torque_control_interim.gravity_robot_compensation_torque;//[Nmm]
                //target_torq = target_torq + weight_compensation_torque;
                //torque_control_interim.control_output=target_torq;
                ecat_sensojoint.impedance_control(target_traj ,inverse_dynamics_torque,interim_time_exercise);


                //Update the terms in the torque_interim and impedance_interim structures
                torque_control_interim = ecat_sensojoint.get_torque_control_terms();
                impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();

                //Calculate values for useful indexes
// it can be changed to a function of sensojoint control and then an update of a structure in struct that cointains all the indexes

                     Error_actual=Error_actual+(error_traj)/(total_rep_counter*EXERCISE_DURATION-time_scaling_f*Error_average);
      //               Error_actual=Error_actual+(error_traj^(2))/(total_rep_counter*(EXERCISE_DURATION-time_scaling_f*Error_average));
                     average_velocity= average_velocity+abs(ecat_sensojoint.get_velocity_actual_value_rad_s()/(total_rep_counter*(EXERCISE_DURATION-time_scaling_f*Error_average)));
                     if( loop_cnt%500 == 0) {PLOGI<<"average velocity: " << average_velocity<<endl;}
             //      if( loop_cnt%1000 == 0){PLOGW<<" Torque Tot"<<impedance_terms_interim.torque_tot<<endl;}
             //      PLOGW<<" Gravity Torque"<<weight_compensation_torque<<endl;}
     //                if (loop_cnt%discretization_factor== 0 ) {
     //                    int n= static_cast<int>(loop_cnt/discretization_factor);
     //                velocity_error_vector[n] =nominal_velocity-ecat_sensojoint.get_velocity_actual_value_rad_s();
     //                }

                        // Update Status
                count++;
                if (count == static_cast<int>(EXERCISE_DURATION-time_scaling_f*Error_average) ){
                    count = 0;
                    repetition_counter=repetition_counter+1;
     //               int i=0;
     //                 for(i=0;i<(EXERCISE_DURATION-time_scaling_f*Pm2_average)/discretization_factor;i++) {
     //                 //Computation of velocity signed error
     //                 average_error_velocity_vector[i]=average_error_velocity_vector[i]+velocity_error_vector[i]/total_rep_counter;

 //                      }
                }
            }
            break;

       case robot_control_mode::target_position_control:
            ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_VELOCITY_OUTPUT);
            controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_velocity_output;
            velocity_control = Ks_pos * (target_pos - ref_position)*60*M_PI*10 + Kd_pos*motor_terms.velocity_actual_value;
            PLOGI<<"Velocity control target: " << target_pos<<endl; //mrpm
            if (loop_cnt%500 == 0){/*: public esmacat_application*/
                PLOGI<<"Velocity control target: " << velocity_control<<endl;
               }
                if (velocity_control > 5000.0) {
                    velocity_control= 5000.0;
                }
            ecat_sensojoint.set_target_output_velocity(static_cast<int32>(velocity_control));
           if  ( abs(target_pos - ref_position) < M_PI/180) {
                ecat_sensojoint.set_target_output_velocity(0);
                ecat_sensojoint.set_control_mode(freeze_control);
               }
               // ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
               // controller_config_interim.motor_mode_of_operation=cyclic_synchronous_torque_output;
               // if (loop_cnt%1000 == 0) {PLOGI << "Set modes of operation to CYCLIC_SYNC_TORQUE_OUTPUT"<<endl;}


            //impedance_control_error_freeze_rad = target_freeze_pos - ref_position;
            //target_impedance_control_torque_freeze_mNm = Ks_freeze*impedance_control_error_freeze_rad - Kd_freeze*motor_terms.velocity_actual_value;
            //weight compensation

            //torque_output = target_impedance_control_torque_freeze_mNm;// + torque_control_interim.weight_assistance + torque_control_interim.inertia_torque;
            //torque_control_interim.control_output=torque_output;
            //Motor torque input
            //ecat_sensojoint.set_target_output_torque(torque_output);
            break;

        case robot_control_mode::go_to_position:
            ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
            controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_torque_output;
            target_traj = P0;
           // weight_compensation_torque=(-1)*torque_control_interim.compensation_factor*torque_control_interim.gravity_robot_compensation_torque;//[Nmm]

            elapsed_time_ms_offset_exercise = elapsed_time_ms;
            interim_time_exercise = (elapsed_time_ms-elapsed_time_ms_offset_exercise);

            inverse_dynamics_torque = ecat_sensojoint.compute_inverse_dynamics(ref_position);

            support_traj = ref_position + ( target_traj - ref_position)/2.0*(tanh((elapsed_time_ms-elapsed_time_ms_offset)/1000.0-exp(1.0))+1.0);
            ecat_sensojoint.impedance_control(support_traj ,inverse_dynamics_torque,interim_time_exercise);

            break;




        case robot_control_mode::antig_calibration:
            if (ecat_sensojoint.get_modes_of_operation_display_CoE() != CoE_mode_t::cyclic_synchronous_position_output){
                ecat_sensojoint.set_modes_of_operation(CoE_mode_t::cyclic_synchronous_position_output);
                if (loop_cnt%1000 == 0){PLOGI << "Set modes of operation to CYCLIC_SYNC_POSITION_OUTPUT"<<endl;}
            }
            if (derating_param_time_to_arrive_75p_in_msec < 1e-5) derating_factor_input = 1.0;
            else derating_factor_input = smooth_start_func_tanh(elapsed_time_ms/derating_param_time_to_arrive_75p_in_msec);
            double input_position_error = target_calibration_position-ref_position *derating_factor_input;

            ecat_sensojoint.set_target_position(static_cast<int32>(input_position_error));
            double corretion_weigth_assistance = motor_terms.torque_actual_value/torque_control_interim.weight_compensation_torque_before_scaling; // da sistemare, così non credo funzioni
            if (loop_cnt%50 == 0){cout <<"\033[1;34m weigth assistance correction should be:\033[0m\n"<<fixed << corretion_weigth_assistance << endl;};

            break;

        }

    }


    // Print on screen for debug
   if (loop_cnt%1000 == 0) {

        print2screen();
  }

    if (loop_cnt%10 == 0) {
        sensojoint_manager::write2file();
    }
    if(repetition_counter > total_rep_counter ){
//    if(ecat_sensojoint.get_current_control_mode() == robot_control_mode::adaptive_traj_control ) {
       sensojoint_manager::write2file_updateoffline();
//     }
        ecat_sensojoint.set_control_mode(robot_control_mode::quit);
    }

    if(loop_cnt > APPLICATION_TIMEOUT_MS ){
//        if(ecat_sensojoint.get_current_control_mode() == robot_control_mode::adaptive_traj_control ) {
           sensojoint_manager::write2file_updateoffline();
//        }
        PLOGW<<"test"<<endl;
        ecat_sensojoint.set_control_mode(robot_control_mode::quit);
    }

    sensojoint_manager::writesharedmemory();

 //    PLOGW <<"Mode of operation"<<motor_terms.modes_of_operation_display<<endl;
}
//****************************//
//******** LOOP END **********//
//****************************//
void sensojoint_manager::read_motor_value(){
    motor_terms.statusword = ecat_sensojoint.get_statusword();
    motor_terms.modes_of_operation_display = ecat_sensojoint.get_modes_of_operation_display();
    motor_terms.position_actual_value = ecat_sensojoint.get_position_actual_value_rad();
    motor_terms.velocity_actual_value = ecat_sensojoint.get_velocity_actual_value_rad_s();
    motor_terms.acceleration_value_filt = ecat_sensojoint.get_motor_acceleration_value_filt();
    motor_terms.torque_actual_value = ecat_sensojoint.get_torque_actual_value_mNm();
    motor_terms.output_torque_actual_value = ecat_sensojoint.get_output_torque_actual_value_mNm(); //not filtered
    motor_terms.secondary_position_value = ecat_sensojoint.get_secondary_position_value_rad();
    motor_terms.secondary_velocity_value = ecat_sensojoint.get_secondary_velocity_value_rad_s();
    motor_terms.timestamp = ecat_sensojoint.get_timestamp();

}
void sensojoint_manager::initialize_interface_parameters(){
    control_input_command.control_mode_input = controller_config_interim.control_mode;
    control_input_command.Lim_up_input = impedance_terms_interim.Lim_up;
    control_input_command.Lim_down_input = impedance_terms_interim.Lim_low;
    control_input_command.amplitude_soft_stops_input = impedance_terms_interim.Delta_ss;
    control_input_command.K_gain_input = impedance_terms_interim.K_gain;
    control_input_command.D_gain_input = impedance_terms_interim.D_gain;
    control_input_command.gravity_scailing_factor_input = torque_control_interim.weight_assistance ; // human user weight correction
    control_input_command.weight_scailing_factor_input = torque_control_interim.compensation_factor;  // robot weight correction
    control_input_command.inertia_scailing_factor_input = torque_control_interim.inertia_correction;
}

void sensojoint_manager::readsharedmemory() {
    ecat_sensojoint.set_controller_config_mode(sensojoint_shared_memory.data -> controller_config.control_mode);
    controller_config_interim.motor_mode_of_operation=sensojoint_shared_memory.data -> controller_config.motor_mode_of_operation;
    ecat_sensojoint.enable_joint(sensojoint_shared_memory.data -> controller_config.joint_enabled);
    ecat_sensojoint.set_torque_control_terms(sensojoint_shared_memory.data -> torque_terms);
    ecat_sensojoint.set_impedance_control_terms(sensojoint_shared_memory.data -> impedance_terms);
    control_input_command = sensojoint_shared_memory.data -> control_input_command_sm;
    Error_actual = sensojoint_shared_memory.data ->Error_actual;
    average_velocity = sensojoint_shared_memory.data->average_velocity;
    //lettura da shared memory sovrascrive l'oggetto impedance_terms_interim cancellando i valori che arrivano da sensojoint_control.cpp
}

void sensojoint_manager::writesharedmemory(){
    sensojoint_shared_memory.data->elapsed_time_ms                  = elapsed_time_ms;
    sensojoint_shared_memory.data->loop_cnt                         = loop_cnt;
    sensojoint_shared_memory.data->controller_config.joint_enabled  = controller_config_interim.joint_enabled;

    sensojoint_shared_memory.data->motor_terms_sm                   = motor_terms;

    sensojoint_shared_memory.data->controller_config.motor_mode_of_operation = controller_config_interim.motor_mode_of_operation ;
    sensojoint_shared_memory.data->torque_terms                     = torque_control_interim;
    sensojoint_shared_memory.data->impedance_terms                  = impedance_terms_interim;

    sensojoint_shared_memory.data->controller_config.control_mode  = ecat_sensojoint.get_control_mode_display();

    sensojoint_shared_memory.data ->control_input_command_sm        = control_input_command;
    sensojoint_shared_memory.data ->Error_actual                      = Error_actual;
    sensojoint_shared_memory.data ->average_velocity                = average_velocity;

//    cout << "test"<<ecat_sensojoint.get_position_actual_value()<<endl;

//    cout << "test"<<sensojoint_shared_memory.data->controller_config.output_position<<endl;
}

void sensojoint_manager::quit () {

    // Parte aggiunta da robot_control_mode::stop
    ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_VELOCITY_OUTPUT);
    arrest_smooth_factor=1-smooth_start_func_tanh(4.0*(elapsed_time_ms-elapsed_time_ms_offset)/arrest_sm_int_time);
    motor_terms.velocity_actual_value=ecat_sensojoint.get_velocity_actual_value_rad_s();
    ecat_sensojoint.set_target_output_velocity(arrest_smooth_factor*motor_terms.velocity_actual_value);
    if (abs(motor_terms.velocity_actual_value)< 0.005 ) {
    PLOGW << "SHUT DOWN - BRAKE ENGAGED " << endl;
    ecat_sensojoint.set_target_output_velocity(0);
    CSVfile.close();
    ecat_sensojoint.set_controlword(5);
    // Parte aggiunta da robot_control_mode::stop
    sensojoint_shared_memory.~sensojoint_shared_memory_comm();
    stop();
    }

}

void sensojoint_manager::print2screen(){
    std::cout   <<"MOTOR DATA:"<<endl   
                << "OPERATING          :" << dec << CoE_state_labels[ecat_sensojoint.get_CoE_state()]                           << endl
                << "Torque     [mNm]   :" << fixed << setprecision(3)  << ecat_sensojoint.get_output_torque_actual_value_mNm()  << endl
                << "Position   [deg]   :" << fixed << setprecision(3)  << (ecat_sensojoint.get_position_actual_value_rad()-POS_OFFSET)*rad_to_deg<<endl                               << endl
                << "Velocity   [rad/s] :" << fixed << setprecision(3)  << ecat_sensojoint.get_velocity_actual_value_rad_s()     << endl
                
                <<"CONTROL DATA:"
                << "Control Mode       :" << dec << robot_control_labels[ecat_sensojoint.get_control_mode_display()]              << endl
                << "Mode of  Operation :" << dec << CoE_mode_labels[ecat_sensojoint.get_modes_of_operation_display_CoE()]       << endl
                   << "Mode (Non CoE) :" << fixed << setprecision(3)  <<  ecat_sensojoint.get_modes_of_operation_display()       << endl
                << "Control Torque     :" << fixed << setprecision(3)  << ecat_sensojoint.get_control_output()                  << endl
                << "Soft stop active   :" << fixed << ecat_sensojoint.get_out_of_boundaries()                                   <<endl
                << "Error Traj    :" << fixed << setprecision(3)  << error_traj                                  <<endl
//        fare funnzione get_soft_stop_torque        << "Soft Stop Torque    :"<< fixed << setprecision(3)  << ecat_sensojoint.impedance_terms.soft_stop_torque      << endl

                << endl;
}

void sensojoint_manager::initialize_write2file(){
    time_t t = time(nullptr);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];

    strftime (buffer,80,"SENSOjoint-Test-Eva_%Y-%m-%d-%H-%M-%S.csv",now);
    CSVfile.open (buffer);
//    CSVfile.open("../actualTestPredictions.csv");
    if(CSVfile.is_open())
    {
        PLOGI << "Log File Created Successfully";

        CSVfile << endl
                << "J_state" << ","

                << "J_mode_operation" << ","

                << "J_elapsed_time_ms" << ","

                << "J_time_ms" << ","

                << "J_position_rad" <<","

                << "J_reference_pos"<<","

                << "J_velocity_rad_s" << ","

                << "Position Error" << ","

                << "J_acceleration" << ","

                << "T_torque_loadcell_Nm" << ","

                << "T_torque_des" << ","
                << "T_torque_virtual_damper" << ","
                << "Nom_velocity"<<","

                <<"Position Value" << "," //valentina
                <<"Threshold" << "," //valentina

                <<"Stimulation Phase" <<"," //valentina

                <<"Current" << "," //valentina

                <<"Mean Torque" << "," //valentina

                <<"Max Torque" ; //valentina

    }
    else {
        PLOGE << "Log File Error";
    }
}

///
////// \brief agree_manager::write2file
/// This functions appends sensors signals, control variables and setpoints to the .csv log file at each timestamp (i.e. 1kHz)
///
///
void sensojoint_manager::write2file() {

    CSVfile << endl

            << ecat_sensojoint.get_CoE_state() << ","

            << ecat_sensojoint.get_modes_of_operation_display_CoE() << ","

            << elapsed_time_ms << ","

            << interim_time_exercise<<","

            << ecat_sensojoint.get_position_actual_value_rad() - (POS_OFFSET) << ","

            << interim_setpoint_exercise<<","

            << ecat_sensojoint.get_velocity_actual_value_rad_s() << ","

            << error_traj  << ","

            << ecat_sensojoint.motor_accel_actual_value_filt << ","

            << ecat_sensojoint.get_output_torque_actual_value_mNm_filt()*mNm_to_Nm << ","

            << ecat_sensojoint.get_control_output() << ","

            << impedance_terms_interim.torque_feedback << "," //torque feeedback

            <<  nominal_velocity <<"," //nominal velocity

            << (sensojoint_shared_memory.data->motor_terms_sm.position_actual_value) * rad_to_deg<<","  //valentina

            << sensojoint_shared_memory.data->threshold<<","  //valentina

            << sensojoint_shared_memory.data->stimulation_phase<<","  //valentina

            << sensojoint_shared_memory.data->current<<"," //valentina

            << sensojoint_shared_memory.data->meanTorque<<"," //valentina

            << sensojoint_shared_memory.data->maxTorque; //valentina

            //FIX salva i dati solo nella modality traiettoria

}

void sensojoint_manager::write2shmem(){ //valentina
    sensojoint_shared_memory.data->position_value = (ecat_sensojoint.get_position_actual_value_rad()-POS_OFFSET)*rad_to_deg;
}

void sensojoint_manager::write2file_updateoffline() {
//    PLOGI<<"Average velocity"<<average_velocity<<endl;
     ofstream Paramfile ("Offline_indexes_parameter.csv");
 //     if(Paramfile.is_open())  {
           Paramfile << endl
          << Error_4 << ","
          << Error_3 << ","
          << Error_2 << ","
          << Error_1 << ","
          << Error_actual << ","
          << total_rep_counter << ","
          << set_counter-1     << ","
          << average_velocity;
 //     }
}


/*void sensojoint_manager::read_file(){

  ifstream inputFile;
  string line;
  string t="65.7";
  string Error_5s,Error_4s,Error_3s,Error_2s,Error_1s,rep_counter_s,set_counter_s,average_velocity_s;
  inputFile.open("Offline_indexes_parameter.csv",ios::in);
  while(!inputFile.eof()) {
      getline(inputFile,Error_5s,',');
      getline(inputFile,Error_4s,',');
      getline(inputFile,Error_3s,',');
      getline(inputFile,Error_2s,',');
      getline(inputFile,Error_1s,',');
      getline(inputFile,rep_counter_s,',');
      getline(inputFile,set_counter_s,',');
      getline(inputFile,average_velocity_s,',');
  char *ending;
  //conversion of string into float
   Error_5 = std::stof(Error_5s);
   Error_4 = std::stof(Error_4s);
   Error_3 = std::stof(Error_3s);
   Error_2 = std::stof(Error_2s);
   Error_1= std::stof(Error_1s);
   total_rep_counter=std::stoi(rep_counter_s);
   set_counter = std::stoi(set_counter_s);
//   average_velocity = std::stof(average_velocity_s);
  }
 inputFile.close();
}*/

void sensojoint_manager::closefile() {
    CSVfile.close();
    PLOGI<<"LOG File close correctly"<<endl;
}




