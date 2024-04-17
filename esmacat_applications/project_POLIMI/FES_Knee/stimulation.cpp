#include "stimulation.h"
#include "headers.h"
#include "sensojoint_fes_app.h"


stimulation::stimulation()
{

}


void stimulation::initialize_stimulation(){

    smpt_open_serial_port(&device, port_name);
    smpt_clear_ml_init(&ml_init);
    ml_init.packet_number = smpt_packet_number_generator_next(&device);
    smpt_send_ml_init(&device, &ml_init);
    smpt_clear_ml_update(&ml_update);

    ml_update.enable_channel[Smpt_Channel_Red] = true;
    ml_update.packet_number = smpt_packet_number_generator_next(&device);

    cout << "Stimulator initialization done" << endl;
}


void stimulation::initialize_ll_stimulation(){

    smpt_open_serial_port(&device, port_name);
    smpt_clear_ll_init(&ll_init);
    ll_init.packet_number = packet_number;
    smpt_send_ll_init(&device, &ll_init);

    packet_number ++ ;

    cout << "Stimulator initialization done" << endl;
}

void stimulation::stimulate(){


    ml_update.channel_config[Smpt_Channel_Red].number_of_points = Number_of_points;
    ml_update.channel_config[Smpt_Channel_Red].ramp = Ramp;
    ml_update.channel_config[Smpt_Channel_Red].period = stimulation_period; //mettere periodo 40 se voglio 25Hz

    ml_update.channel_config[Smpt_Channel_Red].points[0].current = stimulation_current;
    ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].current = -stimulation_current;

    ml_update.channel_config[Smpt_Channel_Red].points[0].time = stimulation_pulsewidth;
    ml_update.channel_config[Smpt_Channel_Red].points[1].time = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].time =stimulation_pulsewidth;

    check_data = smpt_send_ml_update(&device, &ml_update);

    // Send packet
    Smpt_ml_get_current_data ml_get_current_data = {0};
    ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
    ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;
    check_sent=smpt_send_ml_get_current_data(&device, &ml_get_current_data);

    cout << "check sent" << check_sent << endl;
    cout << "check data" << check_data << endl;


}

void stimulation::lowlevel_stimulate(){
    // generate the single pulse
   // cout << "pulse created" << endl;
    ll_channel_config.enable_stimulation = true;
    ll_channel_config.channel = Smpt_Channel_Red;
    ll_channel_config.number_of_points = 3;
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

