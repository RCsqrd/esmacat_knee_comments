/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the EtherCAT Arduino Shield by Esmacat slave example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "my_app.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 */
void my_app::assign_slave_sequence(){
    // tell the master what type of slave is at which point in the chain
    assign_esmacat_slave_index(&ecat_as,0);

    // Log the position of the slave onto the terminal
    std :: cout << "EASE Slave is at index 0" << std :: endl;
}

/**
 * @brief Configure your Esmacat slave.
 * Link Esmacat slave object with the actual Esmacat slave in the EtherCAT communication chain.
 * Functions beginning with 'configure slave' must only be executed in this function
 */
void my_app::configure_slaves(){
    // add initialization code here
    // Functions starting with "configure_slave" work only in configure_slave() function
}

/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void my_app::init()
{

}

/**
 * @brief Executes functions at the defined loop rate
 */
void my_app::loop(){

    // The ROS object created is used to read the current state of registers from the Esmacat slave object
    //      and store it in a shared memory location
    ros_ease.set_read_registers(&ecat_as);


    // The newly modified qrite registers from the ROS Communication is returned from the shared memory 
    ros_message = ros_ease.get_write_registers();

    // The Esmacat slave object is used to update the corresponding Esmacat slave registers with the 
    //    updated values received from ROS Nodes.
    ecat_as.set_output_variable_5_OUT_GEN_INT5(ros_message.INT5);

    // The updated value is logged onto the terminal
    cout <<" Register 5 = " << ros_message.INT5 << std :: endl;
}
