#include "stimulation.h"
#include "headers.h"


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



void stimulation::stimulate(){

    ml_update.channel_config[Smpt_Channel_Red].number_of_points = Number_of_points;
    ml_update.channel_config[Smpt_Channel_Red].ramp = Ramp;
    ml_update.channel_config[Smpt_Channel_Red].period = stimulation_period; //mettere periodo 40 se voglio 25Hz

    ml_update.channel_config[Smpt_Channel_Red].points[0].current = stimulation_amplitude;
    ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].current = -stimulation_amplitude;

    ml_update.channel_config[Smpt_Channel_Red].points[0].time = stimulation_pulsewidth;
    ml_update.channel_config[Smpt_Channel_Red].points[1].time = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].time =stimulation_pulsewidth;

    smpt_send_ml_update(&device, &ml_update);

    // Send packet
    Smpt_ml_get_current_data ml_get_current_data = {0};
    ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
    ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;
    smpt_send_ml_get_current_data(&device, &ml_get_current_data);

}

