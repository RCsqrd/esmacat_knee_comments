      #include <thread>
#include <chrono>
#include "application.h"
#include "trajectory_generation.h"
#include "sensojoint_master/sensojoint_structs.h"
#include "sensojoint_master/sensojoint_shared_memory.h"
#include <iomanip>
#include <string>
int main() {
    int ac_pos;

    sensojoint_shared_memory_comm Traj_sm;
    trajectory_gen Traj;
    Traj_sm.init();
    if (  Traj_sm.init())
    {
        PLOGI << "User Interface shared memory initialized with key " << hex << static_cast<int>(Traj_sm.get_shared_memory_key()) << dec ;    // start the shared memory communication
    }
   else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
       Traj_sm.detach_shared_memory();

   }
   Traj_sm.data->stop=false;
   Traj_sm.data->state =1;
   Traj.inizialize_write_traj_data();
    Traj.set_nominal_beta_function_param(EXERCISE_START,EXERCISE_AMPLITUDE,EXERCISE_DURATION,5);
    Traj.set_pos_treshold(0.7,1.2);
    while (Traj_sm.data->controller_config.control_mode ==0 || Traj_sm.data->controller_config.control_mode == robot_control_mode::standby ){
       sleep(1);

    }
   while ( Traj_sm.data->controller_config.control_mode == robot_control_mode::impedance_control_beta)   {

    ac_pos= (Traj_sm.data->motor_terms_sm.position_actual_value);
    //cout<< Traj_sm.data->motor_terms_sm.position_actual_value<<endl;
   }


   while(  Traj_sm.data->controller_config.control_mode == robot_control_mode::transparent_control  ) {
  //      auto t0 = std::chrono::high_resolution_clock::now();
 //       cout<< Traj_sm.data->motor_terms_sm.position_actual_value;
        if( (Traj_sm.data->adaptive_traj_ref_sm.traj_is_adapted == false ) && (Traj.repetition_counter<=Traj.rep_number) )  {
         Traj.set_nominal_beta_function_param(EXERCISE_START,EXERCISE_AMPLITUDE,EXERCISE_DURATION,5);
         Traj.traj_ref_computation(Traj_sm.data->elapsed_time_ms);
        }
    else if( (Traj_sm.data->adaptive_traj_ref_sm.traj_is_adapted == false) && (Traj.repetition_counter>Traj.rep_number) ) {
        Traj.read_ad_file();
        Traj.discard_rep();
        if (Traj.good_rep_list.size() ==0) {
            return 0;
        }
        Traj.compute_average_quantities();
        Traj.write_ad_beta_param();
        Traj_sm.data->adaptive_traj_ref_sm=Traj.get_adapt_offline_param();
        Traj_sm.data->adaptive_traj_ref_sm.traj_is_adapted =true;
       // cout<<Traj_sm.data->adaptive_traj_ref_sm.traj_amplitude_scaling<<endl;
      }

std::this_thread::sleep_for(std::chrono::milliseconds(1));
/* auto t1 = std::chrono::high_resolution_clock::now();
 std::cout << "took "
              << std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count()
               << " milliseconds\n";
*/
 }

  // if( Traj_sm.data->controller_config.control_mode == robot_control_mode::impedance_control_beta)
  // {

  while( (Traj_sm.data->controller_config.control_mode != robot_control_mode::quit) && (Traj_sm.data->controller_config.control_mode != robot_control_mode::transparent_control) && (Traj_sm.data->controller_config.control_mode != robot_control_mode::impedance_control_beta) ) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      //cout <<"wait"<<endl;
  }

   //Traj_sm.detach_shared_memory();
   return 0;
}
