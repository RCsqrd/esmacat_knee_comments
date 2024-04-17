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

#include "sensojoint_shared_memory.h"
#include "application.h"
#include "sensojoint_structs.h"


#include <thread>
#include <chrono>
#include <string>
#include <termios.h>
#include <sys/ioctl.h>
//Include the header file for the Esmacat slave you plan to use for e.g. Analog Input slave
#include <unistd.h>

using namespace std;
/*****************************************************************************************
 * CLASSES
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
class sensojoint_interface
{
private:

public:
    /** A constructor- sets initial values for class members */
    sensojoint_interface();

    sensojoint_shared_memory_comm shared_memory;


    controller_configuration interface_controller;
    impedance_control_terms interface_impedance_terms;
    torque_control_terms    interface_torque_terms;
    leg_weight_compensation_config_t leg_weight_comp;
    control_input_command_t control_input_command_interface;

    bool enable_joint = true;
    robot_control_mode control_mode;

    double elapsed_time_ms;
    uint64_t loop_count;


    float weight_assistance;
    float compensation_factor;
    float inertia_factor;



//    void print2screen();
//    void initialize_write2file();
//    void write2file();
//    void closefile();
    void writesharedmemory();
    void readsharedmemory();
    void print_debug_command();
    bool kbhit();
};

#endif // sensojoint_manager_H
