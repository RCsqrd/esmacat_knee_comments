#ifndef trajectory_generation_H
#define trajectory_generation_H




/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "error_list.h"
// #include "sensojoint_master/sensojoint_structs.h"
#include "sensojoint_master/sensojoint_shared_memory.h"
#include <unistd.h>
#include "file_handling/include/json_handling.h"
#include <fstream>
#include <vector>
#define EXERCISE_DURATION 5000.0
#define EXERCISE_AMPLITUDE  17*M_PI/36    // 70 deg
#define EXERCISE_START 0      // 10 deg
#define POS_OFFSET M_PI*345/180
/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class trajectory_gen  {
private:
//int n_points; // n of points of the vectors//
// Data_structure
int n_tot_points;
int n_points_per_rep;
int n_data_to_read;
std::vector<std::vector<float>> data_reps_array;
std::vector<float>pos_err_time_vector_average;
std::vector<float>vel_err_time_vector_average;
//Nominal trajectory variables

float ex_init_pos;
float ex_amplitude;
float ex_t_tot;
float ex_P3_coeff;
//Useful Kinematic variables//
float traj_ref_pos;
float traj_ref_vel;
float actual_pos;
float actual_vel;
float err_pos;
float err_vel;
float err_pos_squared;
float err_vel_squared;
//Trajctory time and count variables
int counter=0;
float elapsed_time_ms=0;
int time_exercise=0;
int ind_good=0;
bool start_trajectory=false;
float elapsed_time_ms_offset_exercise=0;
//Output Trajectory variables

float t_average_peak;
float t_average_tot;
float max_position_average;
float abs_max_position=0;
float max_position_treshold=1.2*(ex_init_pos+ex_amplitude);
float min_position_treshold=0.7*(ex_init_pos+ex_amplitude);
float final_vel_treshold=0.3;
adaptive_traj_offline_t adaptive_traj_off_param;
public:
int repetition_counter=0;
int rep_number=8;
std::vector<int> good_rep_list;
 sensojoint_shared_memory_comm           trajectory_gen_shared_memory;
        trajectory_gen();
    void traj_ref_computation(float elapsed_time_ms);
    void read_ad_file();
    void compute_average_quantities();
    void discard_rep();
    void set_nominal_beta_function_param(float in_pos,float amplitude,float duration,float exp_p3);
    void set_pos_treshold(float min_treshold_k,float max_treshold_k);
//    void trajectory_generation_alg();
    void write_ad_beta_param();
    void write_traj_data();
    void inizialize_write_traj_data();
    adaptive_traj_offline_t get_adapt_offline_param();
 ofstream CSVAdaptivetraj;
};


 /* void main {


if {
   rep_counte

    traj
C
} */
//

#endif // TRAJECTORY_GEN_H



    //

