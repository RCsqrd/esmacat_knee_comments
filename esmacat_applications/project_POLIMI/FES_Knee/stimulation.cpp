#include "stimulation.h"
#include "headers.h"
#include "sensojoint_fes_app.h"

stimulation::stimulation(){

}

//le smpt sono definite in delle librerie specifiche per il dispositivo di stimolazione

// ml= mid level
void stimulation::initialize_stimulation(){

    smpt_open_serial_port(&device, port_name);
    smpt_clear_ml_init(&ml_init); // clear the ml_init structure to hold init parameters for the device
    ml_init.packet_number = smpt_packet_number_generator_next(&device);
    smpt_send_ml_init(&device, &ml_init);// send the ml_init structure to the device
    smpt_clear_ml_update(&ml_update); // clear the ml_update structure to hold update parameters for the device

    ml_update.enable_channel[Smpt_Channel_Red] = true; //enable the red channel
    ml_update.packet_number = smpt_packet_number_generator_next(&device); //the paket number vary from 0 to 63 (define in smpt_packet_number_generator.h)

    cout << "Stimulator initialization done" << endl;
}

// ll= low level
void stimulation::initialize_ll_stimulation(){

    smpt_open_serial_port(&device, port_name);
    smpt_clear_ll_init(&ll_init); // clear the ll_init structure to hold init parameters for the device
    ll_init.packet_number = packet_number;
    smpt_send_ll_init(&device, &ll_init); // send the ll_init structure to the device

    packet_number ++ ;

    cout << "Stimulator initialization done" << endl;
}

void stimulation::stimulate(){ //mid level stimulation

    ml_update.channel_config[Smpt_Channel_Red].number_of_points = Number_of_points; // [1 - 16] Number of points
    //The ramp is excecuted if the channel is enabled.
    ml_update.channel_config[Smpt_Channel_Red].ramp = Ramp; //[0-15] pulses. Number of linear increasing lower current pulse pattern until the full current is reached
    ml_update.channel_config[Smpt_Channel_Red].period = stimulation_period; // [0,5–16383]ms -> ([<0.1-2000] Hz); mettere periodo 40 se voglio 25Hz

    ml_update.channel_config[Smpt_Channel_Red].points[0].current = stimulation_current;
    ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].current = -stimulation_current;

    ml_update.channel_config[Smpt_Channel_Red].points[0].time = stimulation_pulsewidth;
    ml_update.channel_config[Smpt_Channel_Red].points[1].time = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].time =stimulation_pulsewidth;

    check_data = smpt_send_ml_update(&device, &ml_update);//start of the stimulation (Ml_update)

    // Send packet
    Smpt_ml_get_current_data ml_get_current_data = {0}; // clear the ml_get_current_data structure to hold data for the device
    ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device); 
    ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;

    check_sent=smpt_send_ml_get_current_data(&device, &ml_get_current_data); //This command is used as a keep-alive signal. After Ml_update 
    cout << "check data" << check_data << endl;

}

void stimulation::lowlevel_stimulate(){
    // generate the single pulse
   // cout << "pulse created" << endl;
    ll_channel_config.enable_stimulation = true;
    ll_channel_config.channel = Smpt_Channel_Red;
    ll_channel_config.number_of_points = 3; // number of point in the pulse
    ll_channel_config.packet_number = packet_number;

    ll_channel_config.points[0].current = stimulation_current;
    ll_channel_config.points[0].time = stimulation_pulsewidth;

    ll_channel_config.points[1].time = 0;

    ll_channel_config.points[2].current = -stimulation_current;
    ll_channel_config.points[2].time = stimulation_pulsewidth;

    smpt_send_ll_channel_config(&device,&ll_channel_config);
    packet_number ++;

}

void stimulation::lowlevel_stimulate_calib(int current, int durata){
    // generate the single pulse

    ll_channel_config.enable_stimulation = true;
    ll_channel_config.channel = Smpt_Channel_Red;
    ll_channel_config.number_of_points = 3;
    ll_channel_config.packet_number = packet_number;

    ll_channel_config.points[0].current = current;
    ll_channel_config.points[0].time = durata;

    ll_channel_config.points[1].time = 0;

    ll_channel_config.points[2].current = -current;
    ll_channel_config.points[2].time = durata;

    smpt_send_ll_channel_config(&device,&ll_channel_config);
    packet_number ++;

    cout << "current" << current << endl;
}

void stimulation::lowlevel_stimulation_close(){

    smpt_send_ll_stop(&device, packet_number);
    smpt_close_serial_port(&device);
}

