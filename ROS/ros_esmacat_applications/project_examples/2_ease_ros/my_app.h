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
//Include the header file for the Esmacat slave you plan to use for e.g. EASE
#include "ethercat_arduino_shield_by_esmacat.h"
// Include the ROS header files for the Esmacat slave you plan to use e.g. EASE 
#include "ros_ethercat_arduino_shield_by_esmacat.h"

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/**
 * @brief Description of your custom application class
 *
 * Your custom application class my_app inherits the class 'esmacat_application' & 'ros_ethercat_arduino_shield_by_esmacat'
 * Write functions to override the parent functions of 'esmacat_application' & 'ros_ethercat_arduino_shield_by_esmacat'
 * Declare an object of your slave class (e.g. ecat_as)
 * Declare an object of your slave class to interact with ROS Nodes (e.g. ros_ease)
 * Declare any other variables you might want to add
 * Define the constructor for initialization
 */
class my_app : public esmacat_application
{
public:
    /** A constructor- sets initial values for class members */
    my_app(): ros_ease_1("ease1"),ros_ease_2("ease2"),ros_message_1(),ros_message_2() {}
    // ease 1 blinks led when register 5 values become 1
    // ease 2 blinks led when register 5 values become 2
private:
    void assign_slave_sequence(); /** identify sequence of slaves and their types */
    void configure_slaves(); /** configure all slaves in communication chain */
    void init(); /** code to be executed in the first iteration of the loop */
    void loop(); /** control loop */

    esmacat_ethercat_arduino_shield_by_esmacat ease_1; /**< create your Esmacat slave object */
    esmacat_ethercat_arduino_shield_by_esmacat ease_2; /**< create your Esmacat slave object */

    ros_ethercat_arduino_shield_by_esmacat ros_ease_1;
    ros_ethercat_arduino_shield_by_esmacat :: write ros_message_1;
    
    ros_ethercat_arduino_shield_by_esmacat ros_ease_2;
    ros_ethercat_arduino_shield_by_esmacat :: write ros_message_2;
};

#endif // MY_APP_H
