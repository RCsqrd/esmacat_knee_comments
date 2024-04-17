/** @file
 *  @brief This file contains the main program for the EtherCAT Arduino Shield by Esmacat slave
 * example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
# include <iostream>
# include "my_app.h"
# include "ros/ros.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/**
 * @brief Initializes the execution of the Ethercat communication and
 *        primary real-time loop for your application for the desired
 *        slave
 * @return
 */

int main(int argc, char **argv)
{
    //this is defined in my_app.cpp and my_app.h
    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender("esmacat_log.csv", 80000, 10); // Create the 1st appender.
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.

    plog::init(plog::warning, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.

    // Initialize the ROS Esmacat Node
    ros::init(argc, argv, "Esmacat_node");

    my_app app;

    // set the cycle time in ns
    app.set_one_cycle_time_ns(2000000L);

    // start the esmacat application customized for your slave
    app.start();

    //the application runs as long as the esmacat master and slave are in communication
    while (app.is_esmacat_app_closed() == false)
    {
        if(!ros::ok())
        {
            app.stop();
        }
    }

    ros::shutdown();   //Shutdown ROS after the EsmaCAT application is complete.    

    return 0;
}
