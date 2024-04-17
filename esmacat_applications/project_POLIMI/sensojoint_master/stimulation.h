#ifndef STIMULATION_H
#define STIMULATION_H
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
    void stimulate();

    /* Stimulator variables */
    const char *port_name= "/dev/ttyUSB0";
    Smpt_device device= {0};
    Smpt_ml_init ml_init = {0};
    Smpt_ml_update ml_update = {0};
    Smpt_ml_get_current_data ml_get_current_data = {0};

    int stimulation_period = 25; // Defined in milliseconds (0.025 s)
    double stimulation_frequency = 1/stimulation_period; // Defined in Hz (40hz)
    int stimulation_amplitude = 0;
    int stimulation_pulsewidth = 0;

};

#endif // STIMULATION_H
