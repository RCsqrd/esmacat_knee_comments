#ifndef SENSOJOINT_ADAPTIVE_H
#define SENSOJOINT_ADAPTIVE_H
#include "sensojoint_structs.h"
#include "file_handling/include/json_handling.h"

class sensojoint_adaptive
{
private:
float n_Gauss_fun=0;
array<float,500>centroids;
array<float,500>gauss_vector;
array<float,500>fwd_virt_variable;
array<float,500>prev_fwd_virt_variable;
array<float,500>err_based_term;
array<float,500>decay_term;
float L2_norm_gauss_vector=0;
float fwd_variable_norm=0;
bool ret=false;
float corr_torque=0;
public:
    float sigma_param;
    float gamma_param;
    float delta_param;
    float forgot_fct;
    adaptive_traj_online_forward adaptive_control_online_terms;
    adaptive_traj_online_forward get_adaptive_control_terms();
    sensojoint_adaptive();
   // void set_vector_size();
    void set_adaptive_parameter(float sigma ,float gamma ,float  delta,float f_factor,float n_fun);
    void initialize_adaptive();
    void gaussian_rbf(float theta_in,float theta_amp,float actual_pos);
    void adaptive_virtual_variable(float actual_pos, float err_traj,float err_vel_traj,float theta_max,float time_exercise);
    float adaptive_torque_fwd();
};

#endif // SENSOJOINT_ADAPTIVE_H
