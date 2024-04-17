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
// #include "../trajectory_generation.h"

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


    //position_offset_rad = ecat_sensojoint.get_position_actual_value_rad();
    position_offset_rad = M_PI;

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
    ecat_sensojoint.set_impedance_control_Kgain(75*Nm_to_mNm);
    ecat_sensojoint.set_impedance_control_Dgain(3.5*Nm_to_mNm);

    //MODIFY : use this function to update the soft stop from shared memory
    ecat_sensojoint.set_soft_stop_gain(20,2);   //(stiffness [Nm/rad], damping [Nm/(rad/sec)] )
    ecat_sensojoint.set_soft_stop_parameters(95.0,-12,4,true); // [deg] lim up - lim low - amplitude soft stop - reset position
    adaptive_rt_control.set_adaptive_parameter(0.025,0.1,10,0.5,60);
    adaptive_rt_control.initialize_adaptive();
    PLOGW<<"virt variable"<<adaptive_control_fwd_terms.prev_fwd_virt_variable.size()<<endl;
    total_rep_counter=20;
    repetition_counter=0;
//  set up the initial desired control modality of operation
    controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_velocity_output;

    torque_control_interim = ecat_sensojoint.get_torque_control_terms();
    impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();


    sensojoint_manager::initialize_interface_parameters();
    count=0;
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
        if (control_input_command.K_gain_input != impedance_terms_interim.K_gain || control_input_command.D_gain_input != impedance_terms_interim.D_gain  ) {
            ecat_sensojoint.set_impedance_control_Kgain(control_input_command.K_gain_input);
            ecat_sensojoint.set_impedance_control_Dgain(control_input_command.D_gain_input);
        }
        impedance_terms_interim.K_gain = control_input_command.K_gain_input;
        impedance_terms_interim.D_gain = control_input_command.D_gain_input;


        // if the joint is already enabled, disable th20e brake and activate the joint in zero torque mode
        if (controller_config_interim.joint_enabled == false){
            controller_config_interim.control_mode = robot_control_mode::standby;     // settare che da commandpo da shared memory cambio modalità ed esco da qua
            if (loop_cnt%1000 == 0){PLOGI << " Joint disable --> stand-by "<<endl;}
        }
        else {

            ecat_sensojoint.set_modes_of_operation(static_cast<int8_t>(controller_config_interim.motor_mode_of_operation));
            ecat_sensojoint.set_controlword(15);
            if (loop_cnt%1000 == 0){PLOGI << "SWITCHED ON"<<endl;}   //It will automaticaly transition to ENABLE OPERATION
            if (ecat_sensojoint.get_CoE_state() == operation_enabled) {
            if (loop_cnt%1000 == 0){PLOGI << "Enabling joints"<<endl;}
                  if(prev_mode==0.0) {
                ecat_sensojoint.set_control_mode(standby);
                prev_mode = controller_config_interim.control_mode;
            }
                  else {
            //      ecat_sensojoint.set_control_mode(transparent_control);
                    // Change control mode
                    if(controller_config_interim.control_mode != static_cast<robot_control_mode>(prev_mode) ) {
                    //PLOGW << "New modality selected: " << controller_config_interim.control_mode << endl;
//                    PLOGW<<control_input_command.K_gain_input<<endl;
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

        //update variable from sensojoint_control.cpp
        torque_control_interim = ecat_sensojoint.get_torque_control_terms();
        impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();
        adaptive_control_fwd_terms = adaptive_rt_control.get_adaptive_control_terms();




        //********************************//
        //*****Control mode Selection*****//
        //********************************//

        switch(static_cast<int>(controller_config_interim.control_mode)) {

        case robot_control_mode::quit:
            PLOGW << "SHUT DOWN - BRAKE ENGAGED " << endl;
            CSVfile.close();
//            CSVAdaptivetraj.close();
            sensojoint_manager::quit();
            break;

        case robot_control_mode::stop:
            PLOGW << "The Motor will arrest its motion: "<< endl;   //INSERIRE SMOOTH FUNCTION
            ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
            ecat_sensojoint.set_target_output_velocity(0);
            break;

        case robot_control_mode::standby:

            if (controller_config_interim.joint_enabled == true  && ecat_sensojoint.get_CoE_state() == switched_on){
                ecat_sensojoint.set_controlword(15);

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

                torque_output = (-1)*torque_control_interim.compensation_factor*torque_control_interim.gravity_robot_compensation_torque + torque_control_interim.inertia_torque;

            }
            if (controller_config_interim.control_mode == robot_control_mode::transparent_control) {

                torque_output = torque_control_interim.inertia_torque;
            }

           torque_control_interim.control_output=torque_output;
          // if (loop_cnt%1000==0){PLOGI<<"pos"<<adaptive_param_terms.traj_time_scaling<<endl;}
           ecat_sensojoint.torque_control_with_soft_stop(torque_output,elapsed_time_ms - elapsed_time_ms_offset);
           if (loop_cnt%500 == 0){
          // PLOGW<<"P1_ad"<<adaptive_param_terms.traj_amplitude_scaling<<endl;
          //  PLOGW<<"P4_ad"<<adaptive_param_terms.traj_time_scaling<<endl;
            PLOGW<<"P3_ad"<<adaptive_param_terms.simm_scaling<<endl;
           }



       break;

        case robot_control_mode::impedance_control_beta:
            if(repetition_counter==0 || repetition_counter>10 ) {
                count=0;
                repetition_counter=1;

            }
            // reach trajectory starting point
            if (start_trajectory == false) {
                ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_VELOCITY_OUTPUT);
                controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_velocity_output;
                velocity_control = Ks_vel * (P0 - ref_position)*60*M_PI*10 + Kd_vel*motor_terms.velocity_actual_value;
                if (velocity_control > 4000) {
                   velocity_control= 4000;  }//mrpm
                if (loop_cnt%100 == 0) {/*: public esmacat_application*/
                   PLOGI<<"Velocity control target: " << velocity_control<<endl;
//                 PLOGI<<"stifness contribution target: " << Ks_vel * (go_to_traj_starting_pos-ref_position)<<endl;
//                 PLOGI<<"Damping contribution target: " <<Kd_vel*ecat_sensojoint.get_velocity_actual_value_rad_s()<<endl;
                }

                ecat_sensojoint.set_target_output_velocity(static_cast<int32_t>(velocity_control));
                if (abs(P0 - ref_position)< M_PI/128) {
                    ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
                    controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_torque_output;
                    count=0; //Reset of counter of the trajectory in case switched to another control mode and then switchd back to impedance_control_beta
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    start_trajectory = true;
                    if (adaptive_param_terms.traj_adaptation==true) {
                      corr_term=adaptive_param_terms.traj_amplitude_scaling/(pow(2.0,10)*pow(adaptive_param_terms.simm_scaling, (10*adaptive_param_terms.simm_scaling) )*pow((1.0-adaptive_param_terms.simm_scaling),10*(1.0-adaptive_param_terms.simm_scaling)));
                      corr_term=round(10000.0*corr_term)/10000.0;
                      P4=round(EXERCISE_DURATION*adaptive_param_terms.traj_time_scaling);
                      P1=EXERCISE_AMPLITUDE*corr_term/pow(EXERCISE_DURATION*adaptive_param_terms.traj_time_scaling/2,(P3+P5));
                      P3=10.0*adaptive_param_terms.simm_scaling;
                      P5=10.0-P3;
                    }
                    else {
                       P1=EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));
                       P3=5.0;
                       P5=5.0;
                       P4=P2+EXERCISE_DURATION;
                    }
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
               // if (loop_cnt%500== 0 ) {PLOGW<<"Repetition Number:"<<nominal_velocity<<endl;}
                // Beta-Function trajectory generation

                interim_setpoint_exercise = P0 + P1*pow((count-P2),P3) * pow((P4-count),P5);
             /*   if (loop_cnt%500==0){
                PLOGW<<"P1_ad"<<P1<<endl;
                 PLOGW<<"P4_ad"<<P4*adaptive_param_terms.traj_time_scaling<<endl;
                 PLOGW<<"P5_ad"<<(P3+P5)*adaptive_param_terms.simm_scaling<<endl;
                }
                */
                //Beta-function velocity value(derivative of beta trajectory function)
                nominal_velocity=P1*pow(1000.0,P3+P5)*( P3*pow(( (P4-count)/1000.0 ),P5)*pow(count/1000.0,P3-1)-P5*pow(((P4-count)/1000.0),P5-1)*pow(count/1000.0,P3) );
                if (loop_cnt%500== 0 ) {PLOGW<<"Repetition counter:"<<repetition_counter<<endl;}
                // Restart Beta-Function
                if(interim_time_exercise > (P4) ) {
                    PLOGW<<"Total trajectory time"<<interim_time_exercise<<endl;
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    start_trajectory = false;
                    //        support_repetitions++;
                    //        PLOGW << "Repetition #" << support_repetitions;
                }

                // Compute trajectory and run outer impedance control
                 target_k=20*Nm_to_mNm;
                 target_d=2.0*Nm_to_mNm;
                 target_traj  = interim_setpoint_exercise; // WARNING: Trajectory generated in the real-time code to avoid quantization errors.
                 error_traj = interim_setpoint_exercise - (ecat_sensojoint.get_position_actual_value_rad() - POS_OFFSET);
                 //target_torq  = target_k * error_traj - target_d * motor_terms.velocity_actual_value;
                 inverse_dynamics_torque = ecat_sensojoint.compute_inverse_dynamics(ref_position);
                 weight_compensation_torque=(-1)*torque_control_interim.compensation_factor*torque_control_interim.gravity_robot_compensation_torque;//[Nmm]
                 //target_torq = target_torq + weight_compensation_torque;
                 //torque_control_interim.control_output=target_torq;

                 error_traj_velocity= nominal_velocity - ecat_sensojoint.get_velocity_actual_value_rad_s() ;
                 ecat_sensojoint.impedance_control(target_traj,nominal_velocity,inverse_dynamics_torque,interim_time_exercise);
                //ecat_sensojoint.impedance_control(target_traj,target_d*nominal_velocity+inverse_dynamics_torque,interim_time_exercise);
                //Update the terms in the torque_interim and impedance_interim structures
                 torque_control_interim = ecat_sensojoint.get_torque_control_terms();
                 impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();
                //Calculate values for useful indexes
                // it can be changed to a function of sensojoint control and then an update of a structure in struct that cointains all the indexes
                   Error_squared_actual=Error_squared_actual+(pow(error_traj,2)/(total_rep_counter*EXERCISE_DURATION));
//                 Pm2_actual=Pm2_actual+(error_traj^(2))/(total_rep_counter*(EXERCISE_DURATION-time_scaling_f*Pm2_average)));
                  if (loop_cnt%1000 == 0) {
                       PLOGI<<"simmetry_factor: "<<P3/10.0<<endl;
                       PLOGI<<"Total time:"<<P4<<endl;
                       PLOGI<<"Amplitude: " <<adaptive_param_terms.traj_amplitude_scaling/(pow(2.0,10)*pow(adaptive_param_terms.simm_scaling, (10*adaptive_param_terms.simm_scaling) )*pow((1.0-adaptive_param_terms.simm_scaling),10*(1.0-adaptive_param_terms.simm_scaling))) <<endl;
                   }

                        // Update Status
                 count++;
                 if (count == static_cast<int>(P4) ){
                     count = 0;
                     repetition_counter=repetition_counter+1;
                     if (repetition_counter==6) {
                         impedance_terms_interim.K_gain=20*Nm_to_mNm;
                         impedance_terms_interim.D_gain=2*Nm_to_mNm;
                          control_input_command.K_gain_input=  impedance_terms_interim.K_gain ;
                          control_input_command.D_gain_input=impedance_terms_interim.D_gain;

                     }
                 }
             }
         break;
        case robot_control_mode::impedance_control_sigma:
            // reach trajectory starting point
            if (start_trajectory == false){
                ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_VELOCITY_OUTPUT);
                velocity_control = Ks_vel * (P0 - ref_position)*60*M_PI*10 + Kd_vel*motor_terms.velocity_actual_value;    //mrpm
                if (loop_cnt%100 == 0){/*: public esmacat_application*/
                    PLOGI<<"Velocity control target: " << velocity_control<<endl;
//                    PLOGI<<"stifness contribution target: " << Ks_vel * (go_to_traj_starting_pos-ref_position)<<endl;
//                    PLOGI<<"Damping contribution target: " <<Kd_vel*ecat_sensojoint.get_velocity_actual_value_rad_s()<<endl;
                }
                ecat_sensojoint.set_target_output_velocity(static_cast<int32_t>(velocity_control));
                if (abs(P0 - ref_position)< M_PI/128){
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
                nominal_velocity=P1*pow(1000.0,P3+P5)*( P3*pow(( (P4-count)/1000.0 ),P5)*pow(count/1000.0,P3-1)-P5*pow(((P4-count)/1000.0),P5-1)*pow(count/1000.0,P3) );
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

                  ecat_sensojoint.impedance_control(target_traj,nominal_velocity,weight_compensation_torque,interim_time_exercise);
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

        case robot_control_mode::adaptive_control_fwd:
            if(repetition_counter==0) {
                count=0;
                ind_good=0;
//                adaptive_rt_control.set_adaptive_parameter(0.025,0.15,15,10,50);
//                adaptive_rt_control.initialize_adaptive();
            }
            // reach trajectory starting point
            if (start_trajectory == false) {
                ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_VELOCITY_OUTPUT);
                controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_velocity_output;
                velocity_control = Ks_vel * (P0 - ref_position)*60*M_PI*10 + Kd_vel*motor_terms.velocity_actual_value;
                if (velocity_control > 4000) {
                   velocity_control= 4000;  }//mrpm
                if (loop_cnt%100 == 0) {/*: public esmacat_application*/
                   PLOGI<<"Velocity control target: " << velocity_control<<endl;
//                 PLOGI<<"stifness contribution target: " << Ks_vel * (go_to_traj_starting_pos-ref_position)<<endl;
//                 PLOGI<<"Damping contribution target: " <<Kd_vel*ecat_sensojoint.get_velocity_actual_value_rad_s()<<endl;
                }

                ecat_sensojoint.set_target_output_velocity(static_cast<int32_t>(velocity_control));
                if (abs(P0 - ref_position)< M_PI/256) {
                    ecat_sensojoint.set_modes_of_operation(CYCLIC_SYNC_TORQUE_OUTPUT);
                    controller_config_interim.motor_mode_of_operation=CoE_mode_t::cyclic_synchronous_torque_output;
                    count=0; //Reset of counter of the trajectory in case switched to another control mode and then switchd back to impedance_control_beta
                    elapsed_time_ms_offset_exercise = elapsed_time_ms;
                    start_trajectory = true;
                    P1=EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));
                    P3=5.0;
                    P5=5.0;
                    P4=P2+EXERCISE_DURATION;
                    }
                    if  (repetition_counter == 0) {
                        repetition_counter=1;
                     }

                }

            // Compute time variable
            interim_time_exercise = (elapsed_time_ms-elapsed_time_ms_offset_exercise);
            //Calculate values for useful indexes
// it can be changed to a function of sensojoint control and then an update of a structure in struct that cointains all the indexes


            if (start_trajectory ==true) {
               // if (loop_cnt%500== 0 ) {PLOGW<<"Repetition Number:"<<nominal_velocity<<endl;}
                // Beta-Function trajectory generation

                interim_setpoint_exercise = P0 + P1*pow((count-P2),P3) * pow((P4-count),P5);
             /*   if (loop_cnt%500==0){
                PLOGW<<"P1_ad"<<P1<<endl;
                 PLOGW<<"P4_ad"<<P4*adaptive_param_terms.traj_time_scaling<<endl;
                 PLOGW<<"P5_ad"<<(P3+P5)*adaptive_param_terms.simm_scaling<<endl;
                }
                */
                //Beta-function velocity value(derivative of beta trajectory function)
                nominal_velocity=P1*pow(1000.0,P3+P5)*( P3*pow(( (P4-count)/1000.0 ),P5)*pow(count/1000.0,P3-1)-P5*pow(((P4-count)/1000.0),P5-1)*pow(count/1000.0,P3) );
                if (loop_cnt%500== 0 ) {PLOGW<<"Repetition Number:"<<repetition_counter<<endl;}
                // Restart Beta-Function
                if(interim_time_exercise > (P4) ) {
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
                error_traj = interim_setpoint_exercise - (ecat_sensojoint.get_position_actual_value_rad() - POS_OFFSET);
                //target_torq  = target_k * error_traj - target_d * motor_terms.velocity_actual_value;

                inverse_dynamics_torque = ecat_sensojoint.compute_inverse_dynamics(ref_position);
                weight_compensation_torque=(-1)*torque_control_interim.compensation_factor*torque_control_interim.gravity_robot_compensation_torque;//[Nmm]
                //target_torq = target_torq + weight_compensation_torque;
                //torque_control_interim.control_output=target_torq;
                target_k=20*Nm_to_mNm;
                target_d=2.0*Nm_to_mNm;
                error_traj_velocity= nominal_velocity - ecat_sensojoint.get_velocity_actual_value_rad_s() ;
                //if(loop_cnt%100==0){PLOGW<<nominal_velocity<<endl;}
                if (adapt_fwd_torque > 24.0) {
                    adapt_fwd_torque = 24.0;
                }
                if (adapt_fwd_torque< -24.0) {
                    adapt_fwd_torque =-24.0;
                }
                if (isnan(adapt_fwd_torque)) {
                    adapt_fwd_torque=0;
                }
                // ecat_sensojoint.impedance_control(target_traj,nominal_velocity,adapt_fwd_torque*Nm_to_mNm+inverse_dynamics_torque,interim_time_exercise);
                ecat_sensojoint.impedance_control(target_traj,nominal_velocity,adapt_fwd_torque*Nm_to_mNm+target_d*nominal_velocity+inverse_dynamics_torque,interim_time_exercise);
                adaptive_rt_control.gaussian_rbf(-5.0*deg_to_rad,115*deg_to_rad,ref_position);

                adaptive_rt_control.adaptive_virtual_variable(ref_position, error_traj, error_traj_velocity,P0+EXERCISE_AMPLITUDE,interim_time_exercise);
                if (loop_cnt%1== 0 ) {
                 adapt_fwd_torque=adaptive_rt_control.adaptive_torque_fwd();
                }

                //Update the terms in the torque_interim and impedance_interim structures
                torque_control_interim = ecat_sensojoint.get_torque_control_terms();
                impedance_terms_interim = ecat_sensojoint.get_impedance_control_terms();
                adaptive_control_fwd_terms = adaptive_rt_control.get_adaptive_control_terms();
//                trial=adaptive_control_fwd_terms.fwd_virt_variable.size();//adaptive_control_fwd_terms=adaptive_rt_control.get_adaptive_control_terms();
                //Calculate values for useful indexes
// it can be changed to a function of sensojoint control and then an update of a structure in struct that cointains all the indexes

                     Error_squared_actual=Error_squared_actual+(pow(error_traj,2)/(total_rep_counter*EXERCISE_DURATION));
//                   Pm2_actual=Pm2_actual+(error_traj^(2))/(total_rep_counter*(EXERCISE_DURATION-time_scaling_f*Pm2_average)));


                        // Update Status
                count++;
                if (count == static_cast<int>(P4) ){
                    count = 0;
                    repetition_counter=repetition_counter+1;
                }
            }



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

    if (loop_cnt%5 == 0) {
        sensojoint_manager::write2file();
    }
    if(repetition_counter > total_rep_counter ){
        ecat_sensojoint.set_control_mode(robot_control_mode::quit);
    }

    if(loop_cnt > APPLICATION_TIMEOUT_MS ){

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
    control_input_command.gravity_scailing_factor_input = torque_control_interim.weight_assistance ;       // robot weight correction
    control_input_command.weight_scailing_factor_input = torque_control_interim.compensation_factor;        // human user weight correction
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
    adaptive_param_terms=sensojoint_shared_memory.data->adaptive_traj_ref_sm;
    adaptive_control_fwd_terms=sensojoint_shared_memory.data->adaptive_control_fwd_sm;
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
    sensojoint_shared_memory.data ->adaptive_traj_ref_sm             = adaptive_param_terms;
    sensojoint_shared_memory.data ->adaptive_control_fwd_sm          =adaptive_control_fwd_terms;
//    cout << "test"<<ecat_sensojoint.get_position_actual_value()<<endl;

//    cout << "test"<<sensojoint_shared_memory.data->controller_config.output_position<<endl;
}

void sensojoint_manager::quit(){
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
                << "Position   [deg]   :" << fixed << setprecision(3)  << ref_position*rad_to_deg                               << endl
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

    strftime (buffer,80,"SENSOjoint-Test-Edoardo_%Y-%m-%d-%H-%M-%S.csv",now);
    CSVfile.open (buffer);
//    CSVfile.open("../actualTestPredictions.csv");
    if(CSVfile.is_open())
    {
        PLOGI << "Log File Created Successfully";

        CSVfile << endl
                << "J_state" << ","

                << "J_mode_operation" << ","

                << "J_control_mode" << ","

                << "J_elapsed_time_ms" << ","

                << "J_elapsed_time_exercise" << ","

                << "J_position_rad" <<","

                << "J_velocity_rad_s" << ","

                << "Position Error" << ","

                << "J_acceleration" << ","

                << "Velocity_Error"  << ","

                << "T_torque_loadcell_Nm_filt" << ","

                << "T_torque_loadcell_Nm" << ","

                << "T_torque_des" << ","

                << "T_torque_virtual_damper" << ","

                << "T_adapt_ffwd_torque"<< ","

                << "T_adapt_ffwd_torque_err_der"<< ","

                << "inverse dynamics torque" << ","

                << "K Gain" << ","

                << "D Gain" ;

    }
    else {
        PLOGE << "Log File Error";
    }
}

///
////// \brief agree_manager::write2file
/// This functions appends sensors signals, control variables and setpoints to the .csv log file at each timestamp (i.e. 1kHz)
///
void sensojoint_manager::write2file() {

    CSVfile << endl

            << ecat_sensojoint.get_CoE_state() << ","

            << ecat_sensojoint.get_modes_of_operation_display_CoE() << ","

            <<ecat_sensojoint.get_current_control_mode() << ","

            << elapsed_time_ms << ","

            <<count<<","

            << ecat_sensojoint.get_position_actual_value_rad() - (POS_OFFSET) << ","

            << ecat_sensojoint.get_velocity_actual_value_rad_s() << ","

            << error_traj  << ","

            << ecat_sensojoint.motor_accel_actual_value_filt << ","

            << error_traj_velocity  << ","


            << ecat_sensojoint.get_output_torque_actual_value_mNm_filt()*mNm_to_Nm << ","

            << ecat_sensojoint.get_output_torque_actual_value_mNm()*mNm_to_Nm << ","


            << ecat_sensojoint.get_control_output() << ","

            << impedance_terms_interim.impedance_control_torque_mNm  << ","


            <<adapt_fwd_torque << ","

            << adaptive_control_fwd_terms.adapt_torque_err_part+adaptive_control_fwd_terms.adapt_torque_decay_term<< ","

            << inverse_dynamics_torque << ","

            <<impedance_terms_interim.K_gain << ","

             << impedance_terms_interim.D_gain << ","

             << adaptive_param_terms.traj_time_scaling << ","

             << adaptive_param_terms.simm_scaling<< ","

              << adaptive_param_terms.traj_amplitude_scaling;


        //FIX salva i dati solo nella modality traiettoria

}


void sensojoint_manager::closefile() {
    CSVfile.close();
    PLOGI<<"LOG File close correctly"<<endl;
}


