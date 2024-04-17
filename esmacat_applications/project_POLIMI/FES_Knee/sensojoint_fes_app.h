#ifndef SENSOJOINT_FES_APP_H
#define SENSOJOINT_FES_APP_H
#include "headers_fes.h"
#include "sensojoint_master/sensojoint_shared_memory.h"
#include "stimulation.h"


#define EXERCISE_DURATION 5000.0
#define EXERCISE_START (M_PI/180.0)*150 //(M_PI/180.0)*150  M_PI/36
#define EXERCISE_AMPLITUDE -(M_PI/180.0)*90 //-(M_PI/180.0)*90 13*M_PI/36
#define EXERCISE_AMPLITUDE_BETA 70*(M_PI/180) // 13*M_PI/36=65 deg  70*(M_PI/180)
#define EXERCISE_START_BETA 0      // M_PI/36 = 5deg M_PI/18 = 10deg
#define POS_OFFSET_BETA M_PI*345/180


class sensojoint_fes_app
{
public:
    sensojoint_fes_app();
    sensojoint_shared_memory_comm c;
    stimulation stim;
    void FES_loop();
    double charge_waveform_generation(double iteration, int counter_waveform);
    double charge_waveform_generation_hyb(double iteration, int counter_waveform);
    //void stimulation_t(double stim_param[4]); stimulation without calibration
    void stimulation_t(vector <double> &calib); //stimulation with calibration
    void stimulation_calibration();
    void open_calibration_file();
    void write_calibration_file();
    void close_calibration_file();
    unsigned int counter;
    unsigned int t_overflow;
    uint16_t fes_mode = 0;
    //uint16_t last_fes_mode = 99;
    bool change_mode = false;
    unsigned int prova;

    //CSV file
    void write_sensojoint_file();
    void open_sensojoint_file();
    void close_sensojoint_file();
    void open_ROM_file();
    void close_ROM_file();
    void write_ROM_file();

    void generate_betafunction();
    double desired_betaf[5000] = {0};
    double point_betaf = 0.0;
    int time_bf = 0;
    double P0_f= 0.0;
    double P1_f= 0.0;
    double P2_f= 0.0;
    double P3_f= 0.0;
    double P4_f= 0.0;
    double P5_f = 0.0;
    double pos_act_trig = 0.0;
    int index_act_pos   = 0;
    double beta_position = 0.0;
    uint64_t ms = 0;

    //int campioni;
    int num_iter = 0;
    double q_adjusted = 0.0;

    double stiffness_nm_rad = 0.0;
    double damping_nms_rad = 0.0;

    double delta = 0.0;
    double mean = 0.0;
    double M2 = 0.0;
    double q_2;

    double charge_old = 0.0;
    double charge_dot = 0.0;
    double charge_dot_old = 0.0;
    double T_limit = 0.0;
    int t_old = 0;

    int counter_waveform = 0;
    int counter_iteration = 0;

    double q = 0;
//    double IMIN = 3;
//    double IMAX = 46;
//    double PWMIN = 300;
//    double PWMAX = 500;
    double IMIN = 0;
    double IMAX = 0;
    double PWMIN = 0;
    double PWMAX = 0;
    double stim_param[4] = {IMIN, IMAX, PWMIN, PWMAX};

    double coeff_allocation = 1;

    double pos_err = 0;
    double des_pos;
    double act_pos;
//    double act_torque;
//    double des_torque;
//    double act_velocity;
    double sum_pos_err = 0;
    double mean_position_error = 0;
    double std_position_error = 0;
    double sum_std_mean_position_error = 0.0;
    double mean_std_mean_position_error = 0.0;
    double sum_mean_position_error = 0.0;
    double mean_mean_position_error = 0.0;

    /* Stimulation Calibration variables */
    bool initialize=0;
    int stop_calibration=0;
    double corrente=2;
    double durata=200;
    double torque;
    double prev_torque;
    bool flag_min=0;
    double Imin=0;
    double PWmin=0;
    double Imax=0;
    double PWmax=0;
    double q_calib= 0.0;
    int campioni;
    #define DELTA_TORQUE 100
    vector <double> calib ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    Smpt_ll_channel_config ll_channel_config = {0};
    uint8_t packet_number = 0;
    Smpt_device device= {0};

//    double IMIN = 0;
//    double IMAX = 0;
//    double PWMIN = 0;
//    double PWMAX = 0;

    //EMG
    bool emg_samples = false;
    bool emg_samples_old = false;
    bool check_uvc = true;
    bool stimulated = false;
    bool exercise_start=false;
    unsigned int emg_th = 20;
    bool emg_calib;
    double timer = 0.0;
    unsigned int emg_th_final = 0;


    double P0_beta;
    bool flag_trigger_found = 0;
    vector <double> ROM;
    //double p = 0.0;
    //double ROM[10]={};
    //double velocity = 0.0;
    //double velocity_old = 0.0;
    double position_transp = 0.0;
    double position_transp_old = 0.0;
    double max_pos = 0.0;
    bool flag_transp = 0;
    double end_transp_traj = 0.0;
    double end_transp_traj_1 = 0.0;
    uint ROM_size = 0;
    vector <double> range ={0.0,0.0,0.0};

};

#endif // SENSOJOINT_FES_APP_H
