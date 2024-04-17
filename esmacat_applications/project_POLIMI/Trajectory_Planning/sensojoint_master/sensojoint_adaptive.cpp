
#include "sensojoint_adaptive.h"
#include "sensojoint.h"
#include <iomanip>
#include <string>
#include "application.h"
/*float sensojoint_adaptive::sensojoint_adaptive_t(float n_fun,float actual_pos,float err_traj, float err_vel_traj) {
     sensojoint_adaptive::gaussian_rbf(n_fun,actual_pos);
     sensojoint_adaptive::adaptive_virtual_variable(n_fun, actual_pos, err_traj,err_vel_traj);
     sensojoint_adaptive::adaptive_torque_fwd(n_fun);
}
*/
sensojoint_adaptive::sensojoint_adaptive(){

}

void sensojoint_adaptive::set_adaptive_parameter(float sigma ,float gamma ,float  delta, float f_factor, float n_fun){
    sigma_param= sigma;
    gamma_param= gamma;
    delta_param=delta;
    forgot_fct=f_factor;
    n_Gauss_fun=n_fun;
    //Saving term in the structure
      adaptive_control_online_terms.sigma_param =sigma;
      adaptive_control_online_terms.gamma_param =gamma;
      adaptive_control_online_terms.delta_param=delta;
      adaptive_control_online_terms.forgot_fct=f_factor;

}
adaptive_traj_online_forward sensojoint_adaptive::get_adaptive_control_terms(){
        return adaptive_control_online_terms;
}
/*void sensojoint_adaptive::set_vector_size(){
    centroids.resize(n_Gauss_fun);
    gauss_vector.resize(n_Gauss_fun);
    fwd_virt_variable.resize(n_Gauss_fun);
    prev_fwd_virt_variable.resize(n_Gauss_fun);
    err_based_term.resize(n_Gauss_fun);
    decay_term.resize(n_Gauss_fun);

}
*/
void  sensojoint_adaptive::initialize_adaptive() {
     err_based_term={0};
     decay_term={0};
     centroids={0};
     gauss_vector={0};
     fwd_virt_variable={0};
     prev_fwd_virt_variable={0};
}
 void sensojoint_adaptive::gaussian_rbf(float theta_in,float theta_amp,float actual_pos) {

    float L2_norm_gauss_vector=0;
    for (int i=0; i<n_Gauss_fun; i++) {
        centroids[i]=theta_in+theta_amp*static_cast<float>(i)/(n_Gauss_fun-1);
        gauss_vector[i]=exp(-(actual_pos-centroids[i])*(actual_pos-centroids[i])/(2*sigma_param*sigma_param));
        gauss_vector[n_Gauss_fun+i]=exp(-(actual_pos-centroids[i])*(actual_pos-centroids[i])/(2*sigma_param*sigma_param));
        if (abs(actual_pos-( theta_in+theta_amp ))< theta_amp/(n_Gauss_fun-1) ) {
        L2_norm_gauss_vector=adaptive_control_online_terms.norm_gauss_vector;
        }
        else {
          L2_norm_gauss_vector=L2_norm_gauss_vector+gauss_vector[i]*gauss_vector[i];
        }
       }
    L2_norm_gauss_vector=std::round(1000.0*L2_norm_gauss_vector)/1000.0;
    //Saving term in the structure
    adaptive_control_online_terms.norm_gauss_vector=L2_norm_gauss_vector;
    adaptive_control_online_terms.centroids=centroids;
    adaptive_control_online_terms.gauss_vector=gauss_vector;

}

void sensojoint_adaptive::adaptive_virtual_variable( float actual_pos ,float err_traj,float err_vel_traj,float theta_max,float time_exercise) {
  float adapt_torque_err_part=0;
  float adapt_torque_decay_term=0;
  float adapt_torque=0;
  bool inversion=false;
  fwd_variable_norm=0;
  prev_fwd_virt_variable=adaptive_control_online_terms.fwd_virt_variable;
  //cout<< prev_fwd_virt_variable[43]<<endl;
  L2_norm_gauss_vector=adaptive_control_online_terms.norm_gauss_vector;
  adapt_torque=adaptive_control_online_terms.adapt_torque;
  if( abs(actual_pos+err_traj - theta_max)< 0.0001 ) {
  ret=true;
  inversion=true;
  }
  if(time_exercise<5) {
  ret=false;
  }
  if (ret==false) {
   for (int i=0;i<n_Gauss_fun;i++) {
   err_based_term[i]=+1/gamma_param*gauss_vector[i]*(err_vel_traj+delta_param*err_traj);
   decay_term[i]=-1.0/forgot_fct*gauss_vector[i]*adapt_torque/L2_norm_gauss_vector;
   fwd_virt_variable[i]=(err_based_term[i]+decay_term[i])*0.001+prev_fwd_virt_variable[i];

   //

  adapt_torque_err_part=adapt_torque_err_part+err_based_term[i]*0.001*gauss_vector[i];
  adapt_torque_decay_term=adapt_torque_decay_term+decay_term[i]*0.001*gauss_vector[i];
  //Updating previous term of the structure
  prev_fwd_virt_variable[i]=fwd_virt_variable[i];
  fwd_variable_norm=fwd_variable_norm+fwd_virt_variable[i]*fwd_virt_variable[i];
  }
  }
  if (ret==true) {
      for (int i=0;i<n_Gauss_fun;i++) {
      err_based_term[i]=+1/gamma_param*gauss_vector[n_Gauss_fun+i]*(err_vel_traj+delta_param*err_traj);
      decay_term[i]=-1.0/forgot_fct*gauss_vector[n_Gauss_fun+i]*adapt_torque/L2_norm_gauss_vector;
      fwd_virt_variable[n_Gauss_fun+i]=(err_based_term[i]+decay_term[i])*0.001+prev_fwd_virt_variable[n_Gauss_fun+i];

      //

     adapt_torque_err_part=adapt_torque_err_part+err_based_term[i]*0.001*gauss_vector[n_Gauss_fun+i];
     adapt_torque_decay_term=adapt_torque_decay_term+decay_term[i]*0.001*gauss_vector[n_Gauss_fun+i];
     //Updating previous term of the structure
     prev_fwd_virt_variable[n_Gauss_fun+i]=fwd_virt_variable[n_Gauss_fun+i];
     fwd_variable_norm=fwd_variable_norm+fwd_virt_variable[n_Gauss_fun+i]*fwd_virt_variable[n_Gauss_fun+i];
     }
     if (inversion==true) {
       for (int j=0;j<n_Gauss_fun;j++) {
           corr_torque=corr_torque+fwd_virt_variable[n_Gauss_fun+j]*gauss_vector[n_Gauss_fun+j];
       }
       corr_torque=corr_torque-adapt_torque;
       for (int i=0;i<n_Gauss_fun;i++) {
        fwd_virt_variable[n_Gauss_fun+i]=fwd_virt_variable[n_Gauss_fun+i]+corr_torque*gauss_vector[n_Gauss_fun+i]/L2_norm_gauss_vector;
       }
     }
   }
  // Saving terms in the structure
  adaptive_control_online_terms.upg_term=-1.0/forgot_fct*adaptive_control_online_terms.adapt_torque/L2_norm_gauss_vector+1/gamma_param*(err_vel_traj+delta_param*err_traj);
  adaptive_control_online_terms.adapt_torque_decay_term=adapt_torque_decay_term;
  adaptive_control_online_terms.adapt_torque_err_part=adapt_torque_err_part;
  adaptive_control_online_terms.fwd_virt_variable_norm=fwd_variable_norm;
  adaptive_control_online_terms.fwd_virt_variable=prev_fwd_virt_variable;
}

float sensojoint_adaptive::adaptive_torque_fwd() {
  float adapt_torque=0;
  float previous_adapt_torque=0;
  previous_adapt_torque=adaptive_control_online_terms.adapt_torque;
  gauss_vector= adaptive_control_online_terms.gauss_vector;
  fwd_virt_variable=adaptive_control_online_terms.fwd_virt_variable;
  if (ret==0) {
   for (int i=0;i<n_Gauss_fun;i++) {
        adapt_torque=adapt_torque+fwd_virt_variable[i]*gauss_vector[i];
      }
  }
  if (ret==1) {
   for (int i=0;i<n_Gauss_fun;i++) {
        adapt_torque=adapt_torque+fwd_virt_variable[n_Gauss_fun+i]*gauss_vector[n_Gauss_fun+i];
      }
  }
 adaptive_control_online_terms.adapt_torque=adapt_torque;
 return adapt_torque;
}


