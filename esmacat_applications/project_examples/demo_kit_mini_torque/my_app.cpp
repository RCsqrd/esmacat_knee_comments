/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the EtherCAT Arduino Shield by Esmacat slave example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "my_app.h"
using namespace std;

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 */
void my_app::assign_slave_sequence(){
    assign_esmacat_slave_index(&ecat_li,0);
    assign_esmacat_slave_index(&aci,1);
}

/**
 * @brief Configure your Esmacat slave.
 * Link Esmacat slave object with the actual Esmacat slave in the EtherCAT communication chain.
 * Functions beginning with 'configure slave' must only be executed in this function
 */
void my_app::configure_slaves(){
    aci.configure_control_type(actuator_controller_interface::control_mode_t::direct_escon_control);
    aci.configure_escon_setpoint_sign(1);
    aci.configure_slave_encoder_clear();

    esmacat_loadcell_interface_channel_config_T loadcell_config;
    loadcell_config.single_ended_diff_ch_0_1 = DIFFERENTIAL_INPUT;
    loadcell_config.single_ended_diff_ch_8_9 = SINGLE_ENDED_INPUT;
    loadcell_config.PGA_ch_0_7 = PGA64;
    loadcell_config.buff_en_ADC_ch_0_7 = 0;
    loadcell_config.buff_en_ADC_ch_8_15 = 0;

    ecat_li.configure_slave_loadcell_interface_adc(loadcell_config);
    IO_Direction dig_io[7] = { IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_INPUT,IO_INPUT,IO_OUTPUT};
    ecat_li.configure_slave_dio_direction(dig_io);
    aci.set_position_control_pid_gain(0.3*1, 0.01*1, 0.013);
    aci.set_one_cycle_time_s(get_one_cycle_time_ns()/1000000000.0);

    //0 = position control with potentiometer, 1 = velocity control with load cell
    control_mode = 0;
    green_button = 1;
    red_button = 1;
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

    //Change LED based on button selected control_mode variable
    if(control_mode == 0 ){
        ecat_li.set_digital_output(0,1);//Red light
        ecat_li.set_digital_output(3,0);//Green light
        ecat_li.set_digital_output(2,0);//Blue light
    }

    else if (control_mode == 1){
        ecat_li.set_digital_output(0,0);//Red light
        ecat_li.set_digital_output(3,0);//Green light
        ecat_li.set_digital_output(2,1);//Blue light
    }

    //Obtain state of the buttons
    red_button = ecat_li.get_digital_input(4);
    green_button = ecat_li.get_digital_input(5);


    // Check if red button was pressed to change to control_mode = 0
    if(red_button == 0){
        control_mode = 0;
        aci.configure_slave_encoder_clear();
    }

    // Check if green buttton was pressed to change to control_mode = 1
    else if(green_button == 0){
        control_mode = 1;
    }

    aci.set_escon_enable(1);
    aci.set_escon_derating_factor(1.0);

    //Read potentiometer voltage
    pot_voltage = ecat_li.get_analog_input_mV(8);

    if (control_mode == 0)
    {
        //Run position control from potentiometer input (normalized)
        pot_command_normalized = (pot_voltage-5000.0)/(5000.0/2048.0);
        target_for_position_control = pot_command_normalized;
        aci.set_desired_position(target_for_position_control);
        mode = "Position Control";

    } else if (control_mode == 1){
        //Run velocity control from loadcell input
        target_for_current_control = ecat_li.get_analog_input_mV(1);
        aci.set_control_setpoint((target_for_current_control-15.00)/300);   //15 is the loadcell offset
        encoder_val = (aci.get_raw_incremental_encoder_reading_cpt()%2048); //2048 is the encoder resolution * 4
        if (encoder_val < 0) encoder_val = encoder_val * -1;
        mode = "Velocity Control";
    }
    if ( elapsed_time_ms > 10000)
    {
        aci.set_escon_enable(0);
        ecat_li.set_digital_output(0,0);//Red light
        ecat_li.set_digital_output(3,0);//Green light
        ecat_li.set_digital_output(2,0);//Blue light
        stop();
    }

    std::cout << elapsed_time_ms << "\t"
              << "Control Mode: " << mode << "\t"
              << "DIO4 Red: " << red_button << "\t"
              << "DIO5 Green: " << green_button << "\t"
              << "Pot Voltage: " << (pot_voltage/1000.0) << "\t"
              << "Load Cell: " << ecat_li.get_analog_input_mV(1) << "\t"
              << std::endl;

}
