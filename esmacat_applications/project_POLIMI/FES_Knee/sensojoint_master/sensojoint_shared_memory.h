/** @file
 * @brief Contains declarations used for AGREE exoskeleton's shared memory
 *
*/

#ifndef AGREE_ESMACAT_SHARED_MEMORY_COMM_H
#define AGREE_ESMACAT_SHARED_MEMORY_COMM_H
#define DEFAULT_ROS_KEY_ID  2121

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <iostream>
#include "sensojoint_structs.h"
using namespace std;
#include <vector>
#include "ros/datatypes.h" //added

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class sensojoint_shared_memory_comm
{
    struct shared_memory_packet {

        double      elapsed_time_ms = 0;
        int         err_msg_count = 0;
        uint64_t    state =1;
        double      elapsed_time;
         float      trial_value;
        bool        stop = false;
//        bool        enable_joint;

        int         sensojoint_mode = -1;

        robot_control_mode      control_mode_command;
//        robot_control_mode      control_mode_status = control_mode_command;
        controller_configuration controller_config;
        impedance_control_terms impedance_terms;
        torque_control_terms    torque_terms;
        motor_values             motor_terms_sm;

        control_input_command_t  control_input_command_sm;
        //        robot_control_mode control_mode_command = robot_control_mode::standby;
        //        robot_control_mode control_mode_status = robot_control_mode::standby;
        float weight_assistance;
        float compensation_factor;
        float inertia_correction;


        //Performance index
         float Error_actual;
         float average_velocity;


        uint64_t    loop_cnt = 0;              // count of loop
        int stimulation_phase; //valentina
        double current;        //valentina
        double meanTorque;     //valentina
        double position_value; //valentina
        int maxTorque; //valentina
        int threshold; //valentina


        //EMG
        bool emg;
        bool stim;
        uint16_t modality;
        bool start_traj;
        unsigned int prov;
        unsigned int EMG_th;
        double P0_betaf = M_PI/9;
        bool ex_start = false;
        double inv_dyn_torque;

        double betafunction_sm[5000] = {0};
        int ind_act_pos = 0;
        double rom = 0.0;

    };

private:
    key_t key;
    bool is_the_shared_memory_detached= 0;
    int shmid = 0;

public:
    //    static int number_of_process_attached_in_shared_memory;
    shared_memory_packet* data;
    sensojoint_shared_memory_comm();
    ~sensojoint_shared_memory_comm();


    bool init();
    void change_shared_memory_key(key_t k){key= k;} // only use this function before init
    key_t get_shared_memory_key(){return key;}
    void detach_shared_memory();

//    void set_sensojoint_command(robot_control_mode command){data->control_mode_command = command;}
//    void set_sensojoint_status(robot_control_mode status){data->control_mode_status = status;}
};

#endif // AGREE_ESMACAT_SHARED_MEMORY_COMM_H
