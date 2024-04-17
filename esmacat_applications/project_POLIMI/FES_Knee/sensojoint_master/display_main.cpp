/** @file
 *  @brief This file contains the main program for the EtherCAT Arduino Shield by Esmacat slave
 * example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>

#include "sensojoint_shared_memory.h"
#include "application.h"
#include "sensojoint_structs.h"

using namespace std;
namespace Color {
    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_YELLOW   = 33,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };
}



int main()
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries
    plog::init(plog::info, &consoleAppender); // Initialize the logger

    sensojoint_shared_memory_comm sm;
    controller_configuration display_controller;
    impedance_control_terms display_impedance_terms;
    torque_control_terms display_torque_terms;

    sm.init();
    sm.data->stop=false;
    sm.data->state =1;


    //Modality selection:
    while (sm.data->controller_config.control_mode !=robot_control_mode::quit){
        // Read from shared memory
        double elapsed_time_ms  = sm.data->elapsed_time_ms;
        uint64_t loop_count     = sm.data->loop_cnt;
        controller_configuration    display_controller      = sm.data->controller_config;
        impedance_control_terms     display_impedance_terms = sm.data->impedance_terms;
        torque_control_terms        display_torque_terms    = sm.data->torque_terms;
        motor_values                 display_motor_terms     = sm.data->motor_terms_sm;



        if (display_controller.joint_enabled == true){
            std::cout <<"\033[1;32m SENSOjoint is active - Operation are enabled\033[0m\n"<<fixed<<endl
                      <<"Elapsed Time:" << elapsed_time_ms <<endl
                      <<"Elapsed control LOOP:" << loop_count <<endl
                      <<endl;
            }
        std::cout <<"\033[1;32m MOTOR DATA :\033[0m\n"  <<  endl
                  << "Motor operation modality  :"<< dec << CoE_mode_labels[display_controller.motor_mode_of_operation]             << endl
                  << "Position [deg ]           :" << fixed << setprecision(3)  << display_motor_terms.position_actual_value << endl
                  << "Velocity [rad/s]          :" << fixed << setprecision(3)  << display_motor_terms.velocity_actual_value << endl
                  << "Torque [mNm]              :" << fixed << setprecision(3)  << display_motor_terms.torque_actual_value   << endl
                  <<"\033[1;32m CONTROL DATA\033[0m\n"<< endl
                  << "Control Mode          :" << fixed << setprecision(3)  << display_controller.control_mode               << endl
                  << "TOTAL Torque          :" << fixed << setprecision(3)  << display_impedance_terms.torque_feedback       << endl
                  << "Gravity Torque [mNm]  :" << fixed << setprecision(3)  << display_torque_terms.weight_compensation_torque   << endl
                  << "Inertia Torque [mNm]  :" << fixed << setprecision(3)  << display_torque_terms.inertia_torque              <<endl
                  << "Impedance Torque [mNm]:" << fixed << setprecision(3)  << display_impedance_terms.impedance_control_torque_mNm    << endl
                  << "Controlled Torque     :" << fixed << setprecision(3)  << display_impedance_terms.torque_feedback    << endl

                  << "weight_assistance %   :" << fixed << setprecision(3)  << display_torque_terms.weight_assistance    << endl



                  << endl;
        if ( display_impedance_terms.out_of_boundaries ==1){
            std::cout<<"\033[1;33m Boundary exceeded - SOFT STOP ACTIVE\033[0m\n"<<fixed<<endl
                      <<"Soft Stops Torque :"<<fixed<<display_impedance_terms.soft_stop_torque<<endl ;}
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    sm.detach_shared_memory();
    return 0;
}

















