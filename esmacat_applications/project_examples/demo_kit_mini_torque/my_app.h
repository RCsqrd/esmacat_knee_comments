/** @file
 * @brief This file contains the declaration of the class associated with the user-defined
 * application for the EtherCAT Arduino Shield by Esmacat slave example project */

#ifndef MY_APP_H
#define MY_APP_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include "application.h"
using std::cout;
//Include the header file for the Esmacat slave you plan to use for e.g. Analog Input slave
#include "actuator_controller_interface.h"
#include "loadcell_interf.h"


/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/**
 * @brief Description of your custom application class
 *
 * Your custom application class my_app inherits the class 'esmacat_application'
 * Write functions to override the parent functions of 'esmacat_application'
 * Declare an object of your slave class (e.g. ecat_ai)
 * Declare any other variables you might want to add
 * Define the constructor for initialization
 */
class my_app : public esmacat_application
{
private:
    void assign_slave_sequence(); /** identify sequence of slaves and their types */
    void configure_slaves(); /** configure all slaves in communication chain */
    void init(); /** code to be executed in the first iteration of the loop */
    void loop(); /** control loop */
    int32_t encoder_val;
    uint32_t curr_counter;
    string mode;
    double target_for_current_control;
    double target_for_position_control;
    double pot_voltage;
    double pot_command_normalized;
    int    control_mode;
    bool   green_button;
    bool   red_button;

    actuator_controller_interface aci;
    esmacat_loadcell_interface ecat_li;

public:
    /** A constructor- sets initial values for class members */
    my_app()
    {
        curr_counter = 0;
    }
};

#endif // MY_APP_H
