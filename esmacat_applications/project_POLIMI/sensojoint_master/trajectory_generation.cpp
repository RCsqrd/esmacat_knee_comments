
#include "error_list.h"
#include "sensojoint_master/sensojoint_structs.h"
#include "trajectory_generation.h"
#include <unistd.h>
#include "file_handling/include/json_handling.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
//#include <thread>
#include <chrono>
using namespace std;
using std::chrono::duration_cast;
using std::chrono::milliseconds;


 trajectory_gen::trajectory_gen() {
}
/*
The function trajectory computation extracts from each transparent execution
and saves the following variables:
the repetition number;
ind_good( a boolean variable that expresses if the repetition is valid or not=;
actual position;
the time instant from the beginning of the repetition;
actual velocity;
position error(with respect to the reference of the standard trajectory that is passed through memory from sensojoint_manager;

*/

void trajectory_gen::traj_ref_computation(float elapsed_time) {


     //Get velocity and position data from shared memory
     trajectory_gen_shared_memory.init();

     actual_pos= trajectory_gen_shared_memory.data->motor_terms_sm.position_actual_value;
     actual_vel= trajectory_gen_shared_memory.data->motor_terms_sm.velocity_actual_value;
     long int ti;
     elapsed_time_ms=elapsed_time;
/*     if (static_cast<int>(elapsed_time_ms)%100 == 0) {
     cout<<"time"<<elapsed_time<<endl;
     }
*/     if ( (actual_vel>0.075) && (actual_pos>EXERCISE_START) && (start_trajectory==false) ) {
         start_trajectory=true;
         elapsed_time_ms_offset_exercise = elapsed_time_ms;
         abs_max_position=0;
         if (repetition_counter==0) {
             repetition_counter=1;
         ti = static_cast<long int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
         }
         cout<<"repetition_started"<<endl;
     }
      if (start_trajectory==true) {
     time_exercise=elapsed_time_ms - elapsed_time_ms_offset_exercise;
      if (time_exercise <= ex_t_tot) {
       counter=time_exercise;
       }
      else  {
        counter = static_cast<int>(ex_t_tot);
      }
     traj_ref_pos=ex_init_pos+(ex_amplitude)/pow(ex_t_tot/2,(10))*pow((counter),ex_P3_coeff)*pow((ex_t_tot-counter),(10-static_cast<int>(ex_P3_coeff) ));;
     traj_ref_vel=(ex_amplitude/pow((ex_t_tot)/2,(10))*ex_P3_coeff*pow((ex_t_tot-counter),ex_P3_coeff-1)*pow((counter),ex_P3_coeff-1)*(2*counter-ex_t_tot));

     err_pos= traj_ref_pos-actual_pos;
     err_vel= traj_ref_vel-actual_vel;

     //Save rep data
//        long int t2 = static_cast<long int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
     if ( time_exercise%50==0 ) {
         trajectory_gen::write_traj_data();
         // The function trajectory_gen::write_traj_data() is used to save the data described before in a data structure//
//         long int ti = static_cast<long int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
     }

     if ( (actual_pos ) > abs_max_position) {
        abs_max_position=actual_pos;
     }

     if (abs_max_position>abs(max_position_treshold) )   {
       ind_good=1;
     }

     if (counter > (static_cast<int>(0.4*ex_t_tot))) {

                     if ( ( ( actual_pos<(EXERCISE_START+0.75*abs(EXERCISE_AMPLITUDE+EXERCISE_START)) )  && ( abs(actual_vel)<final_vel_treshold) )   || (static_cast<int>(time_exercise) == 2*ex_t_tot)  )   {

                     if( (static_cast<int>(time_exercise) == 2*ex_t_tot) || (abs_max_position<abs(min_position_treshold)) ) {
                     ind_good=1;

                     }
                     trajectory_gen::write_traj_data();
                     // The function trajectory_gen::write_traj_data() is used to save the data described before in a data structure//
                     repetition_counter=repetition_counter+1;
                     cout<<"repetition_completed"<<endl;
                     elapsed_time_ms_offset_exercise = elapsed_time_ms;
                     time_exercise=0;
                     counter=0;
                     start_trajectory=false;
                     ind_good=0;
                     }

               }


         }
//     trajectory_gen_shared_memory.data->adaptive_traj_ref_sm.counter=counter;
//     trajectory_gen_shared_memory.data->adaptive_traj_ref_sm.time_exercise=time_exercise;

     }
     // Check if the velocity is correct in its formulation//

/*
The function read_ad_file()
is used to extract the values from the csv file Rep_Data created by the function trajectory_gen::write_traj_data() and to
save the data into a data array
*/

void trajectory_gen::read_ad_file() {
    ifstream inputFile;
    int i;
    int c;
    std::vector<std::vector<float>> data_array;
    int n_data_to_read=7;
   inputFile.open("Rep_data.csv",ios::in);
  if (inputFile.is_open())  {
//   while (inputFile.good()) {
    std::string line;
    std::string value;
     i=0;
     c=0;
    while(std::getline( inputFile,line)) {
     std::stringstream s(line);
     std::vector<float> row;
     i=0;
      while (std::getline(s, value ,',')) {
      row.push_back(std::stof(value));
//      PLOGW<<"  Ok-2  "<< row.back()<<endl;
      if (i==n_data_to_read-1) {
       data_array.push_back(row);
       }
      i++;
      }
      c=row.size();
      if (c!=n_data_to_read) {
       PLOGI <<"Error Missing Data" ;
       break;
       }

     }
    std::cout<< "The number of points is:" << data_array.size();
    std::cout <<"The number of data variables for each point is:\n" <<data_array[0].size() ;
     n_tot_points=data_array.size();
     data_reps_array=data_array;
//    }

  }
  else {
     // show message:
     std::cout << "Error opening file";
    }


}

/*
The function trajectory_gen::discard_rep()
is used to eliminate from the previous generated data array the lines corresponding to values related to repetitions that are invalid
(the value of boolean ind_good=1)
*/


void trajectory_gen::discard_rep() {
    bool good=true;
/*
   1. The array containing data is checked row by row :
   at the end of every repetition the first element of the row(position [0]) changes w.r.t the following row:
   the repetition is added to a list of good ones if and only the index at position [1] is 0 for each row
   corresponding to that repetition
*/
    for (int j=0; j<(static_cast<int>(data_reps_array.size())-1);j++) {
      good = true;
      if ( (static_cast <int>(data_reps_array[j][1]) == 1) && (good==true)) {
         good=false;

           }
      if (good==true &&(static_cast <int>(data_reps_array[j][0]) !=static_cast <int>(data_reps_array[j+1][0])))  {
          good_rep_list.push_back(static_cast <int>(data_reps_array[j][0]));

           }

      if (good==false &&(static_cast <int>(data_reps_array[j][0]) !=static_cast <int>(data_reps_array[j+1][0]))) {
          good=true;

          }

      if (good==true &&(static_cast <int>(data_reps_array[j][1])==0) && ( j==(static_cast <int>(data_reps_array.size())-2) ) ) {
           good_rep_list.push_back(static_cast <int>(data_reps_array[j][0]));
    //std::cout<<"The number of good reps is:"<<good_rep_list.size()<<endl;
          }

       }
    std::cout<<"The number of good reps is:"<<good_rep_list.size()<<endl;
    if (good_rep_list.size()==0) {
        // show message:
       std::cout << "No good rep";
    //    adaptive_traj_off_param.traj_time_scaling=1;
    //    adaptive_traj_off_param.traj_amplitude_scaling=1;
    //    adaptive_traj_off_param.simm_scaling=1;
    return;}
//    PLOGW<<"Ok_3"<<endl;
    //PLOGW<<good_rep_list.size()<<endl;
    //PLOGW<<good_rep_list[2]<<endl;
    //PLOGW<<good_rep_list[3]<<endl;
    //

   /*
    *
      2. Every row corresponding to a repetition in the good list is copied to a new data array
      (to remove the data corresponding to discarded repetitions
   */
    int i=0;
    std::vector<std::vector<float>> data_repetition_g;
    n_points_per_rep=n_tot_points/rep_number;
      for (int j=0; j<(static_cast<int>(data_reps_array.size()));j++)  {
       if(static_cast<int>(data_reps_array[j][0])>good_rep_list[i]) {
        i++;
       }
       if (static_cast<int>(data_reps_array[j][0])==good_rep_list[i]) {
         data_repetition_g.push_back(data_reps_array[j]);
         }
       }
      data_reps_array=data_repetition_g;
}

/*
The function trajectory_gen::compute_average_quantities()
is used to read the data from the data structure and for each time instant compute the average values
of the peak position time t_average_peak, the total time t_average_tot and the the peak value of the position profile  max_position_average
Then from those quantities adaptive_traj_off_param.traj_time_scaling ,adaptive_traj_off_param.traj_amplitude_scaling, adaptive_traj_off_param.simm_scaling are computed

*/


void trajectory_gen::compute_average_quantities() {
    std::vector<float> t_peak_v(good_rep_list.size(),0);
    std::vector<float> t_tot_v(good_rep_list.size(),0);
    std::vector<float> max_position_v(good_rep_list.size(),0);
    int pos_err_time_average_t=0;
    int vel_err_time_average_t=0;
    float correction_amplitude=0;
    t_average_peak=0;
    t_average_tot=0;
    max_position_average=0;
    const double epsilon=final_vel_treshold;
    cout<<":  "<<n_points_per_rep<<endl;
    int j=0;
    int jp=0;
    //std::cout<<"The number of good reps is:"<<n_points_per_rep<<endl;
    for (int k=0; k<good_rep_list.size(); k++)  {
    jp=jp+j;
    j=1;

    abs_max_position=0;
    if (jp+j < (data_reps_array.size()-1)) {
     while(static_cast<int>(data_reps_array[jp+j][0])==static_cast<int>(data_reps_array[jp+j+1][0]))  {
     pos_err_time_average_t=data_reps_array[jp+j+1][2];      vel_err_time_average_t=data_reps_array[jp+j+1][4];
          if  (k==0)  {
          pos_err_time_vector_average.push_back(pos_err_time_average_t/(good_rep_list.size()));
          vel_err_time_vector_average.push_back(vel_err_time_average_t/(good_rep_list.size()));
          }
          else if((k>0) && ( j>(pos_err_time_vector_average.size()+1) ) ) {
          pos_err_time_vector_average.push_back(pos_err_time_average_t);
          vel_err_time_vector_average.push_back(vel_err_time_average_t);
          }
          else {
          pos_err_time_vector_average[j]=(pos_err_time_vector_average[j])+pos_err_time_average_t/(good_rep_list.size());
          vel_err_time_vector_average[j]=(vel_err_time_vector_average[j])+vel_err_time_average_t/(good_rep_list.size());
          }
          if (abs(data_reps_array[(jp+j)][2])>abs_max_position) {
             abs_max_position=abs(data_reps_array[(jp+j)][2]);
          }
        if ( (data_reps_array[(jp+j)][4]>=0) &&(data_reps_array[jp+j+1][4]<0)  && (abs(data_reps_array[jp+j][2])>abs(0.8*(EXERCISE_START+EXERCISE_AMPLITUDE)))  && static_cast<int>(t_peak_v[k])==0  )  {
          t_peak_v[k]=(data_reps_array[jp+j][3]);
          max_position_average=max_position_average+(data_reps_array[jp+j][2])/good_rep_list.size();
         }
        else if ( (abs(data_reps_array[jp+j+1][4])<= epsilon) && t_peak_v[k] > t_tot_v[k]) {
            if (jp+j < (static_cast<int>(data_reps_array.size())-2)) {
            if ((static_cast<int>(data_reps_array[jp+j+2][3])==0) ) {
               t_tot_v[k]=(data_reps_array[jp+j+1][3]);
               cout<<"T_peak_v_k"<<(j+jp+1)<<endl;
               cout<<"\n"<<j<<endl;
             }
            }
           }

      j++;
      if (jp+j == (data_reps_array.size()-1)) {
       break; }
      }

      }

    }
    pos_err_time_vector_average[0]=pos_err_time_vector_average[1];
    vel_err_time_vector_average[0]=0;
    pos_err_time_vector_average.insert(pos_err_time_vector_average.begin(),pos_err_time_vector_average[0]);
   vel_err_time_vector_average.insert(vel_err_time_vector_average.begin(),vel_err_time_vector_average[0]);
   for (int k=0; k<good_rep_list.size(); k++)  {
      if (static_cast<int>(t_tot_v[k])==0){
    t_tot_v[k]=2*t_peak_v[k];
    }
      t_average_peak=t_average_peak+(t_peak_v[k])/good_rep_list.size();
      t_average_tot= t_average_tot+(t_tot_v[k])/good_rep_list.size();
          cout<<"T_average_tot "<<t_average_tot<<endl;
         cout<<"T_average_peak "<<t_average_peak<<endl;
      int r=0;
   }
   // Computation and saving of the trajectory parameters in the data structure  adaptive_traj_off_param //
   adaptive_traj_off_param.traj_time_scaling=1+((t_average_tot)-ex_t_tot)/(ex_t_tot);
   adaptive_traj_off_param.simm_scaling=t_average_peak/t_average_tot;
   adaptive_traj_off_param.traj_amplitude_scaling=max_position_average/((ex_init_pos+ex_amplitude));
   adaptive_traj_off_param.traj_amplitude_scaling=roundf(adaptive_traj_off_param.traj_amplitude_scaling * 1000) / 1000;
   adaptive_traj_off_param.simm_scaling=roundf(adaptive_traj_off_param.simm_scaling * 1000) / 1000;
    cout<<adaptive_traj_off_param.traj_time_scaling<<endl;
    cout<<adaptive_traj_off_param.traj_amplitude_scaling<<endl;
    cout<<adaptive_traj_off_param.simm_scaling<<endl;

    //Write to shared-memory
     trajectory_gen_shared_memory.data->adaptive_traj_ref_sm              = adaptive_traj_off_param;


 }
adaptive_traj_offline_t trajectory_gen::get_adapt_offline_param(){
        return adaptive_traj_off_param;
}


void trajectory_gen::set_nominal_beta_function_param(float in_pos,float amplitude,float duration,float exp_p3){
    ex_init_pos=in_pos;
    ex_t_tot=duration;
    ex_amplitude=amplitude;
    ex_P3_coeff=exp_p3;
 }


void trajectory_gen::set_pos_treshold(float min_treshold_k, float max_treshold_k) {
     min_position_treshold=min_treshold_k*(ex_init_pos+ex_amplitude);
     max_position_treshold=max_treshold_k*(ex_init_pos+ex_amplitude);
}

/*
trajectory_gen::write_ad_beta_param saves the personalized trajectory parameters into a csv file
*/

void trajectory_gen::write_ad_beta_param() {
    ofstream outFile("Beta_param.csv");
   if(outFile.is_open())  {
                  outFile << endl
            << adaptive_traj_off_param.traj_time_scaling << ","
            << adaptive_traj_off_param.traj_amplitude_scaling << ","
            << adaptive_traj_off_param.simm_scaling ;
   //     }
  }
}


void trajectory_gen::inizialize_write_traj_data() {
    ifstream inputFile;
    inputFile.open("Rep_data.csv",ios::trunc);
    inputFile.close();
    //Buffer creation for new data
    time_t t_t = time(nullptr);   // get time now
    struct tm * now = localtime( & t_t );
    char buffer_ad [80];
    strftime (buffer_ad,80,"Rep_data.csv",now);
     CSVAdaptivetraj.open(buffer_ad);
}
void  trajectory_gen::write_traj_data() {
     CSVAdaptivetraj
           << repetition_counter << ","
           << ind_good<< ","
           << actual_pos<< ","
           << time_exercise << ","
           << actual_vel<<","
           << err_pos << ","
            << err_vel;
    CSVAdaptivetraj <<endl;
    CSVAdaptivetraj.flush();
}




