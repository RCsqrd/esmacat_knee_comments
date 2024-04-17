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

#include "sensojoint_interface.h"
#include "application.h"
#include "sensojoint_structs.h"


using namespace std;

bool kbhit();


int main()
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries
    plog::init(plog::info, &consoleAppender); // Initialize the logger

    sensojoint_interface interface;

    interface.shared_memory.init();

//    interface.readsharedmemory();

    interface.shared_memory.data->stop=false;
    interface.shared_memory.data->state =1;
    interface.readsharedmemory();

    interface.control_mode= interface.interface_controller.control_mode;

    //Modality selection:

    std:cout    << "command: "<< dec << robot_control_labels[interface.control_mode]   << endl
            <<"*************************************************"   << fixed <<endl
            <<"Select modalities typing the assign number (0-6):"   << fixed <<endl
            <<"- QUIT                                         0 "   << fixed <<endl
            <<"- STOP                                         1 "   << fixed <<endl
            <<"- STANDBY                                      2 "   << fixed <<endl
            <<"- ZEROTORQUE                                   z "   << fixed <<endl
            <<"- FREEZE                                       f "   << fixed <<endl
            <<"- ANTI-G                                       a "   << fixed <<endl
            <<"- TRANSPARENT CONTROL                          t "   << fixed <<endl
            <<"- Impedance Standard Trajectory Passive User 6 "   << fixed <<endl
            <<"- Impedance Standard Trajectory Active User 7 " << fixed <<endl
            <<"- Impedance Personalized Trajectory Passive User 8 " << fixed <<endl
            <<"- Impedance Personalized Trajectory Active User 9 " << fixed <<endl
            <<endl;
   /*         <<"- Calibration FES                              3 "   << fixed <<endl
            <<"- Calibration User Weight                      4 "   << fixed <<endl
            <<"- Calibration ANTI_GRAVITY                     5 "   << fixed <<endl
            <<"- Impedance Trajectory Iterative               7 "   << fixed <<endl

            <<"- ZEROTORQUE                                   z "   << fixed <<endl
            <<"- FREEZE                                       f "   << fixed <<endl
            <<"- ANTI-G                                       a "   << fixed <<endl
            <<"- TRANSPARENT CONTROL                          t "   << fixed <<endl
 //           <<endl;

    char input_char = '0';
    input_char = getchar();
    cout << stoi(&input_char)<<endl;
*/
    // inizializzare oggetto interface_impedance_terms.cose

    while (interface.control_mode !=robot_control_mode::quit)
    {
        interface.readsharedmemory();
        if (kbhit())
        {    char input_char = '0';
            input_char = getchar();
            //cout << stoi(&input_char)<<endl;
            switch(input_char){

            case '0':
                interface.control_mode = robot_control_mode::quit;
                interface.interface_controller.control_mode = robot_control_mode::quit;
                break;

            case '1':
                interface.control_mode = robot_control_mode::stop;
                interface.interface_controller.control_mode = robot_control_mode::stop;

                break;
            case '2':
                interface.control_mode = robot_control_mode::standby;
                interface.interface_controller.control_mode = robot_control_mode::standby;

                break;
            case '3':
                interface.control_mode = robot_control_mode::zerotorque_control;
                interface.interface_controller.control_mode = robot_control_mode::zerotorque_control;
                break;
            case '4':

                break;
            case '5':
                    //interface.control_mode = robot_control_mode::antig_calibration;
                break;

            case 'z':
                interface.control_mode = robot_control_mode::zerotorque_control;
                interface.interface_controller.control_mode = robot_control_mode::zerotorque_control;
                interface.enable_joint = true;

                break;
            case 'f':
                interface.control_mode = robot_control_mode::freeze_control;
                interface.interface_controller.control_mode = robot_control_mode::freeze_control;
                interface.enable_joint = true;

                break;
            case 'a':
                interface.control_mode = robot_control_mode::antig_control;
                interface.interface_controller.control_mode = robot_control_mode::antig_control;
                interface.enable_joint = true;
                break;
            case 't':
                cout<<"Traj Amp"<<interface.adaptive_traj_ref.traj_amplitude_scaling<<endl;
                interface.control_mode = robot_control_mode::transparent_control;
                interface.interface_controller.control_mode = robot_control_mode::transparent_control;
                interface.enable_joint = true;
                break;
            case '6':
                interface.adaptive_traj_ref.traj_adaptation=false;
                interface.adaptive_traj_ref.simm_scaling=1;
                interface.adaptive_traj_ref.traj_time_scaling=1;
                interface.adaptive_traj_ref.traj_amplitude_scaling=1;
                interface.control_input_command_interface.K_gain_input =80000;
                interface.control_input_command_interface.D_gain_input = 3500;
                interface.control_mode = robot_control_mode::impedance_control_beta;
                interface.interface_controller.control_mode = robot_control_mode::impedance_control_beta;
                interface.enable_joint = true;
                break;
            case '7':
                interface.adaptive_traj_ref.traj_adaptation=false;
                interface.control_input_command_interface.K_gain_input = 20000;
                interface.control_input_command_interface.D_gain_input = 2000;
                interface.control_mode = robot_control_mode::impedance_control_beta;
                interface.interface_controller.control_mode = robot_control_mode::impedance_control_beta;
                interface.enable_joint = true;
                break;
            case '8':
                interface.adaptive_traj_ref.traj_adaptation=true;
               /* interface.adaptive_traj_ref.traj_time_scaling=0.8;
                interface.adaptive_traj_ref.traj_amplitude_scaling=0.75;
                interface.adaptive_traj_ref.simm_scaling=0.5;
                */
                interface.control_input_command_interface.K_gain_input = 80000;
                interface.control_input_command_interface.D_gain_input = 3500;
                interface.control_mode = robot_control_mode::impedance_control_beta;
                interface.interface_controller.control_mode = robot_control_mode::impedance_control_beta;
                interface.enable_joint = true;
                 break;
            case '9':
              /* interface.adaptive_traj_ref.traj_time_scaling=0.8;
                 interface.adaptive_traj_ref.traj_amplitude_scaling=0.75;
                 interface.adaptive_traj_ref.simm_scaling=0.5;
              */
                interface.adaptive_traj_ref.traj_adaptation=true;
                interface.control_input_command_interface.K_gain_input = 20000;
                interface.control_input_command_interface.D_gain_input = 2000;
                interface.control_mode = robot_control_mode::impedance_control_beta;
                interface.interface_controller.control_mode = robot_control_mode::impedance_control_beta;
                interface.enable_joint = true;
                 break;
            case 'r':
                interface.adaptive_traj_ref.traj_adaptation=false;
                interface.control_input_command_interface.K_gain_input = 20000;
                interface.control_input_command_interface.D_gain_input = 2000;
                interface.control_mode = robot_control_mode::adaptive_control_fwd;
                interface.interface_controller.control_mode = robot_control_mode::adaptive_control_fwd;
                interface.enable_joint = true;
                break;
            case 'K': case 'k':
                while(kbhit()==false){
                    input_char = getchar();
                    if (input_char =='+'){
                        interface.control_input_command_interface.K_gain_input += 0.1;
                    }
                    else if (input_char =='-'){
                        interface.control_input_command_interface.K_gain_input -= 0.1;
                    }
                    else{
                        cout << "Unkown Command"<<endl;

                    }
                }
                break;
            case 'D': case 'd':
                while(interface.kbhit()==false){
                    input_char = getchar();
                    if (input_char =='+'){
                        interface.control_input_command_interface.D_gain_input += 0.1;
                    }
                    else if (input_char =='-'){
                        interface.control_input_command_interface.D_gain_input -= 0.1;
                    }
                    else{
                        cout << "Unkown Command"<<endl;
                    }
                }
                break;
            case 'w': case 'W':
                while(interface.kbhit()==false){
                    input_char = getchar();
                    if (input_char =='+'){
                        interface.control_input_command_interface.weight_scailing_factor_input += 0.1;
                    }
                    else if (input_char =='-'){
                        interface.control_input_command_interface.weight_scailing_factor_input -= 0.1;
                    }
                    else{
                        cout << "Unkown Command"<<endl;
                    }
                }
                break;
            case 'g': case 'G':
                while(interface.kbhit()==false){
                    input_char = getchar();
                    if (input_char =='+'){
                        interface.control_input_command_interface.gravity_scailing_factor_input += 0.1;
                    }
                    else if (input_char =='-'){
                        interface.control_input_command_interface.gravity_scailing_factor_input -= 0.1;
                    }
                    else{
                        cout << "Unkown Command"<<endl;
                    }
                }
                break;
                //case'I': case'i': // scaling factor inertia
        }
        cout <<"You selected the modality :"<< dec <</* robot_control_labels[interface.control_mode]*/ interface.control_mode << endl;
        cout <<"You selected the modality :"<< interface.control_input_command_interface.K_gain_input    << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        interface.writesharedmemory();
    };
    //if (interface.loop_count%5==0) {cout<<interface.control_input_command_interface.K_gain_input<<endl;}
    };

    cout << interface.interface_torque_terms.compensation_factor<<endl;

    //sm.detach_shared_memory();
    return 0;
}


bool kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}





















