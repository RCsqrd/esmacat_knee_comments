/** @file
 * @brief This file contains the definition of the ROS related functions associated with the
 * application for the LED Blink on EtherCAT Arduino Shield by Esmacat slave example project */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
# include "ros/ros.h"   // ROS header file to use ROS functionalities
# include "esmacat_high_performance/ease_registers.h"   // The EASE registers message type generated as a header file
# include <boost/thread.hpp>   // To use boost threads
# include <iostream>

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/**
 * @brief Function definition for the Esmacat Write thread which writes data to the Esmacat Node
 *         to modify the EASE registers
 */
void ROS_write_esmacat_thread()
{
    // Create an object similar to the message type used for the ROS Communication to write data
    esmacat_high_performance::ease_registers write_data;

    // Create a handle for the ROS Node that writes data
    ros::NodeHandle data_publisher_node;

    // Specify the frequency that you would like to ROS publishing loop to run at
    ros::Rate loop_rate(100);
    
    // instantiate a publisher using the node defined and passing the <message_type>("Topic name",Buffer Size) as Input
    ros::Publisher data_publisher = data_publisher_node.advertise<esmacat_high_performance::ease_registers>("Esmacat_write_ease",1000);

    while(ros::ok())
    {
        // Set the Register 5 to HIGH
        write_data.INT5 = 1;

        // Publish the modified data
        data_publisher.publish(write_data);
        
        // Turn off ROS communication as per the frequency specified
        loop_rate.sleep();
    }

    //Display data from hard real-time loop to the the terminal.
    ROS_INFO("Register 5 is %i", write_data.INT5);
}

/**
 * @brief Initializes the execution of the ROS communication in a separate thread and
 *          writes data to the Esmacat Node to modify the corresponding slave registers.
 *        Joins the threads after the ROS node or Esmacat Node is shut down.
 * @return
 */
int main(int argc, char **argv)
{
    // Initialize a ROS Node for communication
    ros::init(argc,argv,"Esmacat_ROS_interface");

    // Log info to terminal
    std :: cout << "Creating a ROS Node to communicate with Esmacat" << std :: endl;
    
    // Create a spearate boost thread for writing data 
    boost::thread ROS_write_thread(ROS_write_esmacat_thread);

    //Join threads once execution
    ROS_write_thread.join();
    std :: cout << "Joinning the Created ROS Helper Node Threads" << std :: endl;

    return 0;
}
