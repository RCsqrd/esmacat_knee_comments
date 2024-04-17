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
    sensojoint_shared_memory_comm sm;

    sm.init();

//    interface.readsharedmemory();

    sm.data->stop=false;
    sm.data->state =1;

    interface.readsharedmemory();
    interface.control_mode= robot_control_mode::standby;
    //Modality selection:

    std:cout    << "command: "<< dec << robot_control_labels[interface.control_mode]   << endl
            <<"*************************************************"   << fixed <<endl
            <<"Select modalities typing the assign number (0-6):"   << fixed <<endl
            <<"- QUIT                                         0 "   << fixed <<endl
            <<"- STOP                                         1 "   << fixed <<endl
            <<"- STANDBY                                      2 "   << fixed <<endl            
            <<"- Calibration FES                              3 "   << fixed <<endl
            <<"- Calibration User Weight                      4 "   << fixed <<endl
            <<"- Calibration ANTI_GRAVITY                     5 "   << fixed <<endl
            <<"- Impedance Trajectory                         6 "   << fixed <<endl
            <<"- Impedance Trajectory Iterative               7 "   << fixed <<endl

            <<"- ZEROTORQUE                                   z "   << fixed <<endl
            <<"- FREEZE                                       f "   << fixed <<endl
            <<"- ANTI-G                                       a "   << fixed <<endl
            <<"- TRANSPARENT                                  t "   << fixed <<endl
            <<endl;

    char input_char = '0';
    input_char = getchar();
    cout << stoi(&input_char)<<endl;

    // inizializzare oggetto interface_impedance_terms.cose


    while (interface.control_mode !=robot_control_mode::quit)
    {
        if (kbhit())
        {
            input_char = getchar();

            switch(input_char){

            case '0':
                interface.control_mode = robot_control_mode::quit;
                break;

            case '1':
                interface.control_mode = robot_control_mode::stop;

                break;
            case '2':
                interface.control_mode = robot_control_mode::standby;

                break;
            case '3':

                break;
            case '4':

                break;
            case '5':
                    //interface.control_mode = robot_control_mode::antig_calibration;
                break;

            case 'z':
                interface.control_mode = robot_control_mode::zerotorque_control;
                interface.enable_joint = true;

                break;
            case 'f':
                interface.control_mode = robot_control_mode::freeze_control;
                interface.enable_joint = true;

                break;
            case 'a':
                interface.control_mode = robot_control_mode::antig_control;
                interface.enable_joint = true;
                break;
            case 't':
                interface.control_mode = robot_control_mode::transparent_control;
                interface.enable_joint = true;
                break;
            case '6': case '7':
                interface.control_mode = robot_control_mode::impedance_control_beta;
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
        cout <<"You selected the modality :"<< dec << robot_control_labels[interface.control_mode]   << endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        interface.writesharedmemory();
    };
    };

    cout << interface.interface_torque_terms.compensation_factor<<endl;

//    sm.detach_shared_memory();
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





















