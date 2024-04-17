#ifndef STIMULATION_H
#define STIMULATION_H
#include "headers_fes.h"

// Stimulation headers
#include "smpt_ll_client.h"
#include "smpt_client.h"
#include "smpt_ml_client.h"
#include "smpt_messages.h"
#include "smpt_packet_number_generator.h"
#include "smpt_ll_packet_validity.h"

#define Number_of_points 3
#define Ramp 3



class stimulation
{
public:

    stimulation();
    void initialize_stimulation();
    void initialize_ll_stimulation();
    void lowlevel_stimulation_close();
    void stimulate();
    void lowlevel_stimulate();
    void lowlevel_stimulate_calib(int current, int durata);

    /* Stimulator variables */
    const char *port_name= "/dev/ttyUSB0";
    Smpt_device device= {0};
    Smpt_ml_init ml_init = {0};
    Smpt_ml_update ml_update = {0};
    Smpt_ml_get_current_data ml_get_current_data = {0};

    uint8_t packet_number = 0;
    Smpt_ll_init ll_init = {0};
    Smpt_ll_channel_config ll_channel_config = {0};


    int stimulation_period = 25; // Defined in milliseconds (0.025 s)
    double stimulation_frequency = 1/stimulation_period; // Defined in Hz (40hz)
    double stimulation_current = 0;
    double stimulation_pulsewidth = 0;

    bool check_sent = 0;
    bool check_data = 0;



};

#endif // STIMULATION_H
