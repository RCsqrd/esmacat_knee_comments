/** @file
 * @brief Contains definitions of functions used for the Series-Elastic Actuator (SEA) Driver
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>

#include "sensojoint_interface.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
sensojoint_interface::sensojoint_interface(){
}

void sensojoint_interface::print_debug_command(){
    std:cout    <<"*************************************************"   << fixed <<endl
                <<"Select modalities typing the assign number (0-6):"   << fixed <<endl
                <<"- QUIT             0 "                                   << fixed <<endl
                <<"- STOP             1 "                                   << fixed <<endl
                <<"- STANDBY          2 "                                   << fixed <<endl
                <<"- ZEROTORQUE       3 "                                   << fixed <<endl
                <<"- FREEZE           4 "                                   << fixed <<endl
                <<"- ANTI_G           5 "                                   << fixed <<endl
                <<"- TRANSPARENT      t "                                   << fixed <<endl
                <<"- IMPEDANCE-P      6 "                                   << fixed <<endl
                <<"- IMPEDANCE-ASS    7 "                                   << fixed <<endl
                <<"- IMPEDANCE-AD-P   8 "                                   << fixed <<endl
                <<"- IMPEDANCE-AD-ASS 9 "                                   << fixed <<endl
                <<"- ADAPTIVE_RT      r "                                   << fixed <<endl


                <<"TO ENABLE JOINT TYPE 10 "                            << fixed <<endl
                <<"Set weigth compensation contribution"                << fixed <<endl
                <<"Set impedance stiffness"                             << fixed <<endl
                <<"Set impedance damping"                               << fixed <<endl
               <<endl;

}

void sensojoint_interface::writesharedmemory(){
    shared_memory.data->controller_config   = interface_controller;
    shared_memory.data->torque_terms        = interface_torque_terms;
    shared_memory.data->impedance_terms     = interface_impedance_terms;
    shared_memory.data->control_input_command_sm = control_input_command_interface;
    shared_memory.data->adaptive_traj_ref_sm  = adaptive_traj_ref;

}


void sensojoint_interface::readsharedmemory(){
    elapsed_time_ms             = shared_memory.data->elapsed_time_ms;
    loop_count                  = shared_memory.data->loop_cnt;
    interface_controller        = shared_memory.data->controller_config;
    interface_impedance_terms   = shared_memory.data->impedance_terms;
    interface_torque_terms      = shared_memory.data->torque_terms;
    control_input_command_interface = shared_memory.data->control_input_command_sm;
    adaptive_traj_ref = shared_memory.data->adaptive_traj_ref_sm;
}

bool sensojoint_interface::kbhit()
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
