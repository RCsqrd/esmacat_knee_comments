#include "sensojoint_fes_app.h"
#include "stimulation.h"
#include <chrono>

ofstream CSV_sensojoint_file;
ifstream CSVnewfile_read;
ofstream CSV_calibration_file;
ifstream CSV_calibration_file_read;
ofstream CSV_ROM;
ifstream CSV_ROM_read;

using namespace std::chrono;

// USER calibration parameters
float A=5.0;
float B=200.0;
float C=0.0;
float D=0.0;
double ex_amplitude = 0.0;

// USER ID
char userID[10];

// Wait for keyboard interrupt
bool kbhit();

sensojoint_fes_app::sensojoint_fes_app() 
{

}

void read_calibration_file(vector <double> &calib);
void read_ROM_file(vector <double> &range);

void sensojoint_fes_app::FES_loop(){

    write_sensojoint_file();

    des_pos = c.data->impedance_terms.impedance_control_setpoint;
    act_pos = c.data->motor_terms_sm.position_actual_value;
    //act_torque = c.data->
    //des_torque = c.data->
    //act_velocity = c.data->
    emg_samples = c.data->emg;
    c.data->stim = stimulated;
    c.data->prov = counter;
    c.data->EMG_th = emg_th;

  //Main switch loop

  if(counter%1==0) {
    ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    switch (fes_mode) {
      case 0:
        if(change_mode){
          c.data->controller_config.control_mode = robot_control_mode::standby;
          cout<<"STANDBY mode"<<endl;
          change_mode = false;
        }
      break;

      case 1:
        if(change_mode){
          cout<<"FES + MOTOR"<<endl;
          //              cout << "Sync FES" << endl;
          stim.initialize_ll_stimulation();
          //              stim.stimulation_current= 10;
          //              stim.stimulation_pulsewidth=300;
          //              stim.lowlevel_stimulate();

          read_calibration_file(calib);
          read_ROM_file(range);
          c.data->rom = ex_amplitude;

          campioni=0;
          num_iter=0;
          q_adjusted=0.2;

          //       coeff_allocation = 0.25;

          //        stiffness_nm_rad = 10.0; //10.0
          //        damping_nms_rad = 1.0;

          delta = 0.0;
          mean = 0.0;
          M2 = 0.0;

          charge_old= 0.0;
          charge_dot = 0.0;
          charge_dot_old= 0.0;
          T_limit = 0.65;

          //Initialize waveform
          counter_waveform = 0;
          counter_iteration = 0;
          change_mode = false;
        }

        //c.data->controller_config.control_mode  = robot_control_mode::go_to_position;
        //c.data->controller_config.control_mode  = robot_control_mode::impedance_control_beta;

        q = charge_waveform_generation(counter_iteration,counter_waveform)*q_adjusted;
      
    //print info every 100 iterations
        if(counter%100==0){
          cout <<"Charge "<< q << endl;
          cout << "counter_iteration " << counter_iteration << endl;
          cout << "counter_waveform " << counter_waveform << endl;
          cout << "Actual position " << act_pos << endl;
          cout << "START TRAJ" << c.data->start_traj << endl;
        }

        //Write shared memory
        c.data->controller_config.control_mode  = robot_control_mode::impedance_control_beta;
        //c.data->control_input_command_sm.K_gain_input = stiffness_nm_rad;
        //c.data->control_input_command_sm.D_gain_input = damping_nms_rad;

  //        if(counter_waveform>0 && counter_waveform<=EXERCISE_DURATION/2){
  //            if(counter%50==0){
  //                stimulation_t(stim_param);
  //            }
  //        }

  //          if(counter_waveform>0 && counter_waveform<=EXERCISE_DURATION/2){
  //              if(counter%25==0){
  //                  stimulation_t(calib);
  //              }
  //          }

        // Compute trajectory error with/without sign
        //loop that last half of the exercise
        if(counter_waveform>0 && counter_waveform<=EXERCISE_DURATION/2){  //counter_waveform>0 && counter_waveform<=EXERCISE_DURATION/2  counter_waveform>1500 && counter_waveform<=3333
          stimulated=true;
          if(counter%25==0){
            stimulation_t(calib);
          }

          // Compute position error
          pos_err=des_pos-act_pos;
          // cumulate position error
          sum_pos_err+=pos_err;

          // Compute Standard Deviation with streaming data
          delta = abs(pos_err) - mean;
          mean = mean + delta/static_cast<double>(counter_waveform+1);
          M2 = M2+delta*(abs(pos_err)-mean);
        }
        else {
          stimulated=false;
        }

        // After knee extension compute mean position error and mean std
        if(counter_waveform==EXERCISE_DURATION/2){  //counter_waveform==EXERCISE_DURATION/2  counter_waveform==3333
          num_iter++;
          mean_position_error=sum_pos_err/counter_waveform;

          std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

          sum_std_mean_position_error+=std_position_error;
          mean_std_mean_position_error = sum_std_mean_position_error/num_iter;

          sum_mean_position_error+=mean_position_error;
          mean_mean_position_error=sum_mean_position_error/counter_waveform;

          charge_old= 0.0;
          charge_dot = 0.0;
          charge_dot_old= 0.0;
        }

        if(num_iter > 1){
            // Iterative learning

          if (counter_waveform>=3500 && counter_waveform<3501){

            double threshold_pos_error = 0.0;
            double threshold_std_pos_error = 0.1;
            double K_adjusted = 0.1;
            cout << "**************************" << endl;
            cout << "Iterative Learning Control" << endl;
            cout << "Mean error: " << mean_position_error*rad_to_deg << endl;
            cout << "Std  error: " << std_position_error*rad_to_deg << endl;
            cout << "Thr error: " << threshold_pos_error << endl;
            cout << "Thr std: " << threshold_std_pos_error << endl;

          // Adjust charge
          if(mean_position_error*rad_to_deg <= (threshold_pos_error-threshold_std_pos_error)){
            cout << "Fast movement: decrease charge" << endl;
            q_adjusted += K_adjusted*((mean_position_error*rad_to_deg+(threshold_pos_error+threshold_std_pos_error)));
          }
          else if(mean_position_error*rad_to_deg >= (threshold_pos_error+threshold_std_pos_error)){
            cout << "Slow movement: increase charge" << endl;
            q_adjusted += K_adjusted*((mean_position_error*rad_to_deg-(threshold_pos_error+threshold_std_pos_error)));
          }
          else {
            cout << "Ok movement: Ok charge" << endl;
          }
          // Copyright SDG please.
          q_adjusted = std::max(0.0,std::min(1.0,q_adjusted));
          cout << "Charge adjusted: " << q_adjusted << endl;
        }
      }

        if (c.data->start_traj==true){
          if(counter_waveform < EXERCISE_DURATION){
            counter_waveform++;
            campioni++;
          }

          // After knee flexion/extension reset counters
          else if (counter_waveform==EXERCISE_DURATION) {
            sum_pos_err=0;
            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;
            counter_waveform=0;
            campioni=0;
            T_limit=1.0;
            counter_iteration++;
          }
        }
      break;

      case 100:
        if(change_mode){
  //     cout << "Sync FES" << endl;
  //     stim.initialize_ll_stimulation();

  //     stim.stimulation_current= 10;
  //     stim.stimulation_pulsewidth=300;
  //     stim.lowlevel_stimulate();
        cout << "Only motor" << endl;
        read_ROM_file(range);
        c.data->rom = ex_amplitude;
        cout<<"ROM: "<<c.data->rom<<endl;

        //Initialize waveform
        counter_waveform = 0;
        counter_iteration = 0;
        change_mode = false;
        }

        c.data->controller_config.control_mode  = robot_control_mode::impedance_control_beta;

        if(counter%100==0){
          cout << "counter_iteration " << counter_iteration << endl;
          cout << "counter_waveform " << counter_waveform << endl;
          cout << "Actual position " << act_pos << endl;
          cout << "START TRAJ" << c.data->start_traj << endl;
        }

        if (c.data->start_traj==true){
          if(counter_waveform < EXERCISE_DURATION){
            counter_waveform++;
            campioni++;
          }

        // After knee flexion/extension reset counters
          else if (counter_waveform==EXERCISE_DURATION) {
            counter_waveform=0;
            counter_iteration++;
            campioni=0;
          }
        }
    break;

    case 102:
        if (change_mode){
          cout<<"Weight compensation coeff calibration"<<endl;
          read_ROM_file(range);
          c.data->rom = ex_amplitude;

          //Initialize waveform
          counter_waveform = 0;
          counter_iteration = 0;
          change_mode=false;
        }

        c.data->controller_config.control_mode  = robot_control_mode::impedance_control_beta;

        if(counter%100==0){
          cout << "counter_iteration " << counter_iteration << endl;
          cout << "counter_waveform " << counter_waveform << endl;
          cout << "Actual position " << act_pos << endl;
          cout << "Weight coeff " << c.data->torque_terms.weight_assistance << endl;
          cout << "START TRAJ" << c.data->start_traj << endl;
        }

        // Compute trajectory error with/without sign
        if(counter_waveform>2300 && counter_waveform<=2700){ //counter_waveform>2000 && counter_waveform<=4500 counter_waveform>2000 && counter_waveform<=3000
          // Compute position error
          pos_err=des_pos-act_pos;
          // cumulate position error
          sum_pos_err+=pos_err;

  //      // Compute Standard Deviation with streaming data
  //         delta = abs(pos_err) - mean;
  //         mean = mean + delta/static_cast<double>(counter_waveform+1);
  //         M2 = M2+delta*(abs(pos_err)-mean);
        }

          // After knee extension compute mean position error and mean std
            if(counter_waveform==3333){
              num_iter++;
              mean_position_error=sum_pos_err/counter_waveform;

  //          std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

  //          sum_std_mean_position_error+=std_position_error;
  //          mean_std_mean_position_error = sum_std_mean_position_error/num_iter;

  //          sum_mean_position_error+=mean_position_error;
  //          mean_mean_position_error=sum_mean_position_error/counter_waveform;
            }

          if(num_iter > 1){
            // Iterative learning

              if (counter_waveform>=3500 && counter_waveform<3501){
                double threshold_pos_error = 0.0;
                double threshold_std_pos_error = 0.1;
                cout << "**************************" << endl;
                cout << "Iterative Learning Control" << endl;
                cout << "Mean error: " << mean_position_error*rad_to_deg << endl;
                //cout << "Std  error: " << std_position_error*rad_to_deg << endl;
                cout << "Thr error: " << threshold_pos_error << endl;
                cout << "Thr std: " << threshold_std_pos_error << endl;

                if(mean_position_error*rad_to_deg <= (threshold_pos_error-threshold_std_pos_error)){
                  cout << "Over compensated movement: decrease weight coefficient" << endl;
                  c.data->torque_terms.weight_assistance = c.data->torque_terms.weight_assistance - 0.05;
                }
                else if(mean_position_error*rad_to_deg >= (threshold_pos_error+threshold_std_pos_error)){
                  cout << "Under compensated movement: increase weight coefficient" << endl;
                  c.data->torque_terms.weight_assistance = c.data->torque_terms.weight_assistance + 0.05;
                }
                else {
                  cout << "Ok movement"<< endl;
                  cout << "Final weight coefficient: "<< c.data->torque_terms.weight_assistance << endl;
                }
              }
            }

            if (c.data->start_traj==true){
              if(counter_waveform < EXERCISE_DURATION){
                counter_waveform++;
                campioni++;
              }

              // After knee flexion/extension reset counters
              else if (counter_waveform==EXERCISE_DURATION) {
                counter_waveform=0;
                counter_iteration++;
                sum_pos_err=0;
                campioni=0;
              }
            }
      break;

      case 2:
        if(change_mode){
  //              cout << "Sync FES" << endl;
  //              stim.initialize_ll_stimulation();

  //              stim.stimulation_current= 10;
  //              stim.stimulation_pulsewidth=300;
  //              stim.lowlevel_stimulate();

          cout<<"TRANSPARENT mode"<<endl;
          change_mode = false;
        }
        c.data->controller_config.control_mode  = robot_control_mode::transparent_control;
  //          velocity = c.data->motor_terms_sm.velocity_actual_value;
        position_transp = c.data->motor_terms_sm.position_actual_value;
  //          cout<<"pos "<<position_transp<<endl;
  //          cout<<"pos_old "<<position_transp_old<<endl;

        if (position_transp>7*deg_to_rad){

          if(position_transp>position_transp_old){
              max_pos=position_transp;
            }

          else if(position_transp<position_transp_old && max_pos>=(60*deg_to_rad)){ //&& position_transp<(50*deg_to_rad)
              flag_transp=1;
            }
        }
        else if(position_transp<(7*deg_to_rad) && flag_transp==1){
          ROM.push_back(max_pos);
          ROM_size = ROM.size();
          flag_transp = 0;
          max_pos = 0;
  //              for(uint i=0; i <ROM.size(); i++) {

  //                  cout <<ROM.at(i) << endl; }
        }
        position_transp_old = position_transp;

  //if ROM size is 10, calculate the average of the ROM and change mode to write ROM file
        if(ROM_size==10){
          for(uint i=0; i <ROM.size(); i++) {
              end_transp_traj_1 += ROM[i];
            }
          end_transp_traj = end_transp_traj_1/10;
          cout<<end_transp_traj<<endl;
          change_mode=true;
          fes_mode = 201;
          ROM_size = 11;
        }

  //          if((velocity<0.0 && velocity_old>0.0)){
  //              p = c.data->motor_terms_sm.position_actual_value;
  //              ROM.push_back(p);
  ////              for(uint i=0; i <ROM.size(); i++) {

  ////                     cout <<ROM.at(i) << endl; }
  //          }

  //          velocity_old = velocity;
      break;

      case 201:
        if(change_mode ){
          cout<<"ROM generated"<<endl;
          open_ROM_file();
          write_ROM_file();
          close_ROM_file();
          change_mode=false;
        }
      break;

      case 3:
        if(change_mode){
          close_sensojoint_file();
          close_calibration_file();
          stim.lowlevel_stimulation_close();
          change_mode = false;
        }

        c.data->controller_config.control_mode  = robot_control_mode::quit;
        cout<<"QUIT mode"<<endl;
        exit(0);

      case 4:
        if(change_mode){
          c.data->controller_config.control_mode  = robot_control_mode::freeze_control;
          cout<<"FREEZE control mode"<<endl;
          change_mode = false;
        }
      break;

      case 5:
        if(change_mode){
          c.data->controller_config.control_mode  = robot_control_mode::target_position_control;
          cout<<"Target position control "<<endl;
          change_mode = false;
        }
      break;

      case 6:
        if(change_mode){
          c.data->controller_config.control_mode  = robot_control_mode::antig_control;
          cout<<"Anti-g control "<<endl;
          change_mode = false;
        }
      break;

      case 7:
        if(change_mode){
          c.data->controller_config.control_mode  = robot_control_mode::go_to_position;
          cout<<"Go to Position "<<endl;
          change_mode = false;
        }
      break;

      case 15:
        if(change_mode){
          cout << "Generation of the desired offline trajectory";
          read_ROM_file(range);
          c.data->rom = ex_amplitude;
          counter = 0;
          P0_f = EXERCISE_START_BETA;
          P2_f = 0.0;
          P3_f = 5.0; //5ht Order
          P4_f = P2_f+EXERCISE_DURATION;
          P5_f = 5.0; //5th Order
          P1_f = (c.data->rom)/pow(EXERCISE_DURATION/2,(P3_f+P5_f));

          change_mode=false;
        }

        if(counter<EXERCISE_DURATION){
          c.data->betafunction_sm[counter]= P0_f + P1_f*pow((counter-P2_f),P3_f) * pow((P4_f-counter),P5_f);
        }

        if(counter%100==0){
          cout<< c.data->betafunction_sm[counter]<<endl;
        }

  //    if exercise is finished, change to case 0
        if(counter>EXERCISE_DURATION){
          //            c.data->controller_config.control_mode!=robot_control_mode::standby;
          fes_mode = 0;
          change_mode=true;
        }
      break;

      case 8:
        if(change_mode){
          //cout << "Sync FES" << endl;
          stim.initialize_ll_stimulation();

          //               stim.stimulation_current= 10;
          //               stim.stimulation_pulsewidth=300;
          //               stim.lowlevel_stimulate();

          cout<<"FES + MOTOR + EMG"<<endl;
          read_calibration_file(calib);
          read_ROM_file(range);
          c.data->rom = ex_amplitude;
          campioni=0;
          num_iter=0;

          //       coeff_allocation = 0.25;
          //        stiffness_nm_rad = 10.0; //10.0
          //        damping_nms_rad = 1.0;

          delta = 0.0;
          mean = 0.0;
          M2 = 0.0;
          q_adjusted=0.2;

          charge_old= 0.0;
          charge_dot = 0.0;
          charge_dot_old= 0.0;
          T_limit = 0.65;

          //Initialize waveform
          counter_waveform = 0;
          counter_iteration = 0;

          //               c.data->controller_config.control_mode  = robot_control_mode::freeze_control;

          change_mode = false;
        }

        fes_mode = 801;
        change_mode = true; //così entrerebbe in case 801,altimenti si esce dall'if(change_mode) iniziale

      break;

      case 801:
        // TRANSPARENT + WAIT FOR EMG TRIGGER
        if(change_mode){
          timer=counter;
          change_mode=false;
        }
        c.data->controller_config.control_mode=robot_control_mode::transparent_control;
        if ((emg_samples == true && emg_samples_old == false) || counter==timer+5000){
  //        P0_beta = act_pos;
          pos_act_trig = act_pos;
          change_mode = true;
          fes_mode = 802;
        }
        emg_samples_old = emg_samples;

      break;

      case 802:
        if(change_mode){
          // 4950 because 5000 (dimension of betafunction_sm array) and 50 the increment of i
          for (int i=0; i<4950; i++){
            //cout << "sono nel for" << endl;
            //                  if((round(pos_act_trig*100)/100)==(round(rad_to_deg*(c.data->betafunction_sm[i])*100))/100){
            if(c.data->betafunction_sm[i]<pos_act_trig<c.data->betafunction_sm[i+50]){
              index_act_pos = i+50;
              cout << "found trigger at deg: " << c.data->betafunction_sm[i+50] << " at index: " << index_act_pos << endl;
              c.data->ind_act_pos = index_act_pos;
              counter_waveform = index_act_pos;
              t_old = counter_waveform;
              i=4950;
              //q=0;
              change_mode=false;
            }
          }
        }
        c.data->controller_config.control_mode=robot_control_mode::impedance_control_beta;

        beta_position = c.data->betafunction_sm[counter_waveform];

  //counter_waveform is a time index
        q = charge_waveform_generation_hyb(counter_iteration,counter_waveform)*q_adjusted;
        //q_2 = charge_waveform_generation_hyb(counter_iteration,counter_waveform);

        if(counter%100==0){
          cout <<"Charge "<< q << endl;
          // cout <<"Charge 2 "<< q_2 << endl;
          cout << "counter_iteration " << counter_iteration << endl;
          cout << "num_iter " << num_iter << endl;
          cout << "counter_waveform " << counter_waveform << endl;
          cout << "Actual position " << act_pos << endl;
          cout << "beta_position " << c.data->betafunction_sm[counter_waveform] << endl;
          //             cout << "Exercise " << c.data->start_traj << endl;
          //             cout << "P0_beta" << P0_beta;
          //             cout << "P0_beta_shm" <<(c.data->P0_betaf);
        }

        //               if (emg_samples == true && emg_samples_old == false){
        //                   exercise_start = true;

        //               }
        //               emg_samples_old = emg_samples;

        //               if(exercise_start ==true){
        //                       c.data->controller_config.control_mode  = robot_control_mode::impedance_control_beta;
        //               }

        //Write shared memory
        //c.data->controller_config.control_mode  = robot_control_mode::impedance_control_beta;
        //c.data->control_input_command_sm.K_gain_input = stiffness_nm_rad;
        //c.data->control_input_command_sm.D_gain_input = damping_nms_rad;

        //           if (counter%25==0){
        //               stimulation_t(stim_param);
        //           }
        if(counter_waveform>0 && counter_waveform<=EXERCISE_DURATION/2){ //counter_waveform>0 && counter_waveform<=EXERCISE_DURATION/2
          stimulated=true;
          if(counter%25==0){
            //stimulation_t(stim_param);
            stimulation_t(calib);
            //stimulated = true;
          }
          //                   else {
          //                       stimulated = false;
          //                   }
        }
        else {
          stimulated = false;
        }

        // Compute trajectory error with/without sign
        if(counter_waveform<=EXERCISE_DURATION/2){ //counter_waveform<=EXERCISE_DURATION/2  counter_waveform>1500 && counter_waveform<=3333

          // Compute position error
          pos_err=des_pos-act_pos;
          // cumulate position error
          sum_pos_err+=pos_err;

          // Compute Standard Deviation with streaming data
          delta = abs(pos_err) - mean;
          mean = mean + delta/static_cast<double>(counter_waveform+1);
          M2 = M2+delta*(abs(pos_err)-mean);
        }

        // After knee extension compute mean position error and mean std
        if(counter_waveform==EXERCISE_DURATION/2){ //EXERCISE_DURATION/2  3333

          num_iter++;
          mean_position_error=sum_pos_err/counter_waveform;

          std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

          sum_std_mean_position_error+=std_position_error;
          mean_std_mean_position_error = sum_std_mean_position_error/num_iter;

          sum_mean_position_error+=mean_position_error;
          mean_mean_position_error=sum_mean_position_error/counter_waveform;

          charge_old= 0.0;
          charge_dot = 0.0;
          charge_dot_old= 0.0;
        }

        if(num_iter > 1){ //half movement was performed
          // Iterative learning

          if (counter_waveform>=3500 && counter_waveform<3501){

            double threshold_pos_error = 0.0;
            double threshold_std_pos_error = 0.1;
            double K_adjusted = 0.1;
            cout << "**************************" << endl;
            cout << "Iterative Learning Control" << endl;
            cout << "Mean error: " << mean_position_error*rad_to_deg << endl;
            cout << "Std  error: " << std_position_error*rad_to_deg << endl;
            cout << "Thr error: " << threshold_pos_error << endl;
            cout << "Thr std: " << threshold_std_pos_error << endl;

            if(mean_position_error*rad_to_deg <= (threshold_pos_error-threshold_std_pos_error)){
              cout << "Fast movement: decrease charge" << endl;
              q_adjusted += K_adjusted*((mean_position_error*rad_to_deg+(threshold_pos_error+threshold_std_pos_error)));
            }
            else if(mean_position_error*rad_to_deg >= (threshold_pos_error+threshold_std_pos_error)){
              cout << "Slow movement: increase charge" << endl;
              q_adjusted += K_adjusted*((mean_position_error*rad_to_deg-(threshold_pos_error+threshold_std_pos_error)));
            }

            else {
              cout << "Ok movement: Ok charge" << endl;
            }
            // Copyright SDG please.
            q_adjusted = std::max(0.0,std::min(1.0,q_adjusted));
            cout << "Charge adjusted: " << q_adjusted << endl;
          }
        }

        if (c.data->start_traj==true){
          if(counter_waveform < EXERCISE_DURATION){
            counter_waveform++;
            campioni++;
          }

          // After knee flexion/extension reset counters
          else if (counter_waveform==EXERCISE_DURATION) {
            sum_pos_err=0;
            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;
            counter_waveform=0;
            campioni=0;
            T_limit=1.0;
            counter_iteration++;
            //fes_mode = 803;
            change_mode=true;
            fes_mode=801;
            q=0;
          }
        }
        //          change_mode=true;
        //          fes_mode=801;

      break;

  //      case 803:
  //          c.data->controller_config.control_mode=robot_control_mode::go_to_position;
  //          if((act_pos-M_PI/36)<M_PI/180){
  //              fes_mode = 801;
  //              change_mode=true;
  //          }
  //          break;
      
      case 101:
        if(change_mode){
          change_mode=false;
          cout << " minimum" << endl;
        }
        //corrente=2 and durata=400
        A=round(corrente);
        B=round(durata);
        cout << "MINIMO CALIBRAZIONE IDENTIFICATO" << endl;

        fes_mode=9;
        change_mode = true;
      break;

      case 9:
      /* STIMULATION CALIBRATION
      current and pulsewidth are increased by a fixed value each iteration. The current is increased by 0.0035 and the pulsewidth by 0.03.
      if campioni is greater than 1000, and multiple of 25, the stimulation is activated with the current and pulsewidth values.
      when campioni= 1000 current= 8.5; after campioni>1000 there is a stimulation every 25 campioni
      if the current is greater than 40 or the pulsewidth is greater than 500, gets the current and the pulsewidth
      */
        if(change_mode){
          change_mode=false;
          cout << " STIM CALIBRATION mode" << endl;
          stim.initialize_ll_stimulation();
          corrente=5;
          durata=200;
          q_calib=0.0;
          campioni=0;
        }

        // STANDARD CALIBRATION IN FREEZE
        c.data->controller_config.control_mode  = robot_control_mode::transparent_control;
        // Giunto molto rigido 60.0/40.0 - 6.0/4.0

        if(campioni>1000){
          if(campioni%25==0){
            stim.lowlevel_stimulate_calib(corrente,durata);
            cout << "current " << corrente << endl;
            cout << "pw " << durata << endl;
          }
        }

        if(corrente>40 || durata>500 ){
          stim.ll_channel_config.enable_stimulation = false;
          stim.ll_channel_config.channel = Smpt_Channel_Red;
          fes_mode = 10; 
          change_mode = true; // then it enters in the case 10
          C=round(corrente);
          D=round(durata);
          cout << "MASSIMO CALIBRAZIONE IDENTIFICATO" << endl;
        }

        corrente+=0.0035; //0.0035 0.0045
        durata+=0.03;
        prev_torque=torque;
        campioni++;
      break;

      case 10:
        if(change_mode){
          change_mode=false;
          cout << " STOP CALIBRATION mode" << endl;
          }
        C=round(corrente);
        D=round(durata);
        cout << "MASSIMO CALIBRAZIONE IDENTIFICATO" << endl;

        fes_mode=11;
        change_mode = true; // then it enters in the case 11
        stim.ll_channel_config.enable_stimulation = false;
        stim.ll_channel_config.channel = Smpt_Channel_Red;
      break;

      case 11:
        if(change_mode){
          change_mode=false;
          cout << " END Calibration mode" << endl;
          stim.ll_channel_config.enable_stimulation = false;
          stim.ll_channel_config.channel = Smpt_Channel_Red;
        }

        Imin= A;
        PWmin=B;
        Imax=C;
        PWmax=D;

        open_calibration_file();
        write_calibration_file();
        close_calibration_file();

        cout << " MIN CURRENT" << Imin << endl;
        cout << " MIN PULSEWIDTH" <<PWmin << endl;
        cout << " MAX CURRENT" << Imax << endl;
        cout << " MAX PULSEWIDTH" << PWmax << endl;

        fes_mode = 12;
        change_mode = true;
      break;

      case 12:
        if( change_mode){
          change_mode=false;
          cout << " STOP mode" << endl;
          read_calibration_file(calib);
        }
  //            stiffness_nm_rad = 5.0;
  //            damping_nms_rad = 1.0;
  //            coeff_allocation = 0.0;
        c.data->controller_config.control_mode = robot_control_mode::standby;
        stim.ll_channel_config.enable_stimulation = false;
        stim.ll_channel_config.channel = Smpt_Channel_Red;
      break;

      case 13:
        if (change_mode){
          cout << "EMG threshold calibration mode" << endl;
          emg_th = 20;
          emg_calib = true;
          cout << "Press + to increase threshold "
                  "Press - to decrease threshold" << endl;
          change_mode = false;
        }

        if (emg_calib == true) {
          c.data->controller_config.control_mode=robot_control_mode::transparent_control;

          if(kbhit()){
            char input_char = '0';
            input_char = getchar();
            switch (input_char) {
              case '+':
                emg_th = emg_th + 2;
                break;
              case '-':
                emg_th = emg_th - 2;
                break;
              case 's':
                cout << "Stop emg calibration" << endl;
                c.data->controller_config.control_mode=robot_control_mode::freeze_control;
                emg_th_final = emg_th;
                emg_calib = false;

              break;

  //          default:
  //             cout << "INVALID INPUT - stopping calibration" << endl;
  //          break;
            }

            if(input_char!='+' && input_char!='-' && input_char!='s'){
              cout << "Invalid input \n"
                      "Press + to increase threshold \n"
                      "Press - to decrease threshold" << endl;
            }
            else if(input_char=='+'){
              cout << "Threshold increased to: " << emg_th << endl;
            }
            else if(input_char=='-'){
              cout << "Threshold decreased to: " << emg_th << endl;
            }
            else if(input_char=='s'){
              cout << "Final threshold is: " << emg_th << endl;
            }
          }
        }
      break;

//      case 14:

//              if(change_mode){
//                  cout << "Sync FES" << endl;
//                  stim.initialize_ll_stimulation();
//                  stim.stimulation_current= 10;
//                  stim.stimulation_pulsewidth=300;
//                  stim.lowlevel_stimulate();
//                  stim.lowlevel_stimulation_close();
//              }

//          break;

    }
      //Save FES modality in shared memory
      c.data->modality=fes_mode;
  }

}

double sensojoint_fes_app::charge_waveform_generation(double iteration, int t) {
  // Declare beta function parameters
    double P0,P1,P2,P3,P4,P5;
    double T = EXERCISE_DURATION/2.0; // ms
    double T_d=200.0;                 // ms
    //double K= 0.5;
    double Q_slope = 0.002; // slope increases if q_slope decreases 

  // support variables
    //double beta_function;
    double charge = 0;

  // Rising edge
//            if(fes_mode==802){
//                P0 = c.data->P0_betaf;
//            }
//            else {
//                P0 = EXERCISE_START;
//            }
    P0 = EXERCISE_START;
    P2 = 0.0;
    P3 = 5.0; //5ht Order

    //Falling edge
    P4 = P2+EXERCISE_DURATION;
    P5 = 5.0; //5th Order

    P1 = EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));

  // Compute beta-function
    if(c.data->start_traj == true){
      // 1. First beta-function
      //if(t >= 0 && t < (T_limit*T - T_d)  ){
      if(t<EXERCISE_DURATION/2){
        // Compute charge during first phase
        //charge = sin(P0+P1*pow((t+T_d-P2),P3)*pow((P4-(t+T_d)),P5)); previous waveform
        charge = sin(c.data->betafunction_sm[t]);

        // Compute differential charge
        charge_dot = charge - charge_old;

        // Compute max or min (inversion of sign)
        if((charge_dot<0 && charge_dot_old>=0 && t>100)||(charge_dot>0 && charge_dot_old<=0 && t>100)){
          T_limit = t/T;
          cout << "Max found at "<< T_limit << endl;
        }

        // Store values
        charge_old = charge;
        charge_dot_old = charge_dot;
      }

//                    // 2. Plateau
//                    else if(t < (T_limit*T +T_d)){
//                        charge = 1;
//                    }

//                    // 3. Second beta-function
//                    else if(t <= T){
//                        charge = sin(P0+P1*pow((t-P2),P3)*pow((P4-t),P5));
//                    }

//                    // 4. Descending slope
//                    else if(t >= T && t < 1.5*T){
//                        charge = sin(EXERCISE_AMPLITUDE+EXERCISE_START) - Q_slope*(t-T);
//                        if(charge < 0 ) charge = 0;
//                    }

//                    // 5. Ascending slope
//                    else if(t >= 1.5*T && t <= 2*T){
//                        if(t > 2*T - sin(EXERCISE_START)/Q_slope){
//                            charge = (t-2*T+sin(EXERCISE_START)/Q_slope)*Q_slope;
//                        }
//                        else{
//                            charge = 0;
//                        }
//                    }

        // X. else
        else{
          charge = 0;
        }
      }

//    else {

//        // 0. First ascending slope
//        if(t >= 1.5*T && t <= 2*T){
//            if(t > - sin(EXERCISE_START/Q_slope)){
//                charge = (t+EXERCISE_START/Q_slope)*Q_slope;
//            }
//            else{
//                charge = 0;
//            }
//        }
//        // X. else
//        else{
//            charge = 0;
//        }

//    }
  return charge;
}

double sensojoint_fes_app::charge_waveform_generation_hyb(double iteration, int t) {

  // Declare beta function parameters
    double P0,P1,P2,P3,P4,P5;
    double T = (EXERCISE_DURATION/2.0); // ms
    double T_d=200.0;                 // ms
    //double K= 0.5;
    double Q_slope = 0.002; // slope increases if q_slope decreases

  // support variables
    //double beta_function;
    double charge = 0;

        // Rising edge
//          if(fes_mode==802){
//            P0 = c.data->P0_betaf;
//          }
//          else {
//            P0 = EXERCISE_START;
//          }

    P0 = EXERCISE_START;
    P2 = 0.0;
    P3 = 5.0; //5ht Order

    //Falling edge
    P4 = P2+EXERCISE_DURATION;
    P5 = 5.0; //5th Order

    P1 = (EXERCISE_AMPLITUDE)/pow((EXERCISE_DURATION/2),(P3+P5));

  // Compute beta-function
    if(c.data->start_traj == true){
      // 1. First beta-function
//      if(t >= 0 && t < (T_limit*T - T_d)  ){
        if(t<EXERCISE_DURATION/2.0 && t!=t_old){
          // Compute charge during first phase
          charge = sin(c.data->betafunction_sm[t]);
//        charge = sin(P0+P1*pow((t+T_d-P2),P3)*pow((P4-(t+T_d)),P5));

          // Compute differential charge
          charge_dot = charge - charge_old;

          // Compute max or min (inversion of sign)
          if((charge_dot<0 && charge_dot_old>=0 && t>100)||(charge_dot>0 && charge_dot_old<=0 && t>100)){
            T_limit = t/T;
//          cout << "Max found at "<< T_limit << endl;
          }

          // Store values
          charge_old = charge;
          charge_dot_old = charge_dot;
        }
              
//     // 2. Plateau
//        else if(t>=(T_limit*T - T_d) && t< (T_limit*T +T_d)){
//              charge = 1;
//             }

//     // 3. Second beta-function
//        else if(t>=(T_limit*T +T_d) &&  t < T){
//               charge=sin(c.data->betafunction_sm[t]);
////             charge = sin(P0+P1*pow((t-P2),P3)*pow((P4-t),P5));
//              }

//     // 4. Descending slope
//         else if(t >= T && t < 1.5*T){
//                charge = sin(EXERCISE_AMPLITUDE+EXERCISE_START) - Q_slope*(t-T);
//                if(charge < 0 ) charge = 0;
//              }

//     // 5. Ascending slope
//          else if(t >= 1.5*T && t <= 2*T){
//                if(t > 2*T - sin(EXERCISE_START)/Q_slope){
//                  charge = (t-2*T+sin(EXERCISE_START)/Q_slope)*Q_slope;
//                 }
//               else{
//                charge = 0;
//               }
//          }
    // X. else
          else{
            charge = 0;
          }
    }

    t_old = t;

      //    else {

      //        // 0. First ascending slope
      //        if(t >= 1.5*T && t <= 2*T){
      //            if(t > - sin(EXERCISE_START/Q_slope)){
      //                charge = (t+EXERCISE_START/Q_slope)*Q_slope;
      //            }
      //            else{
      //                charge = 0;
      //            }
      //        }
      //        // X. else
      //        else{
      //            charge = 0;
      //        }

      //    }

    return charge;
}

void sensojoint_fes_app::stimulation_t(vector <double> &calib){

  IMIN= calib[0];
  PWMIN= calib[1];
  IMAX= calib[2];
  PWMAX= calib[3];

  if (campioni>0 && campioni<EXERCISE_DURATION){
    stim.stimulation_current=IMIN+sqrt(q)*(IMAX-IMIN);
    stim.stimulation_pulsewidth=PWMIN+sqrt(q)*(PWMAX-PWMIN);
  }
  else {
    stim.stimulation_current=0;
    stim.stimulation_pulsewidth=0;
  }
  stim.lowlevel_stimulate();
}

//void sensojoint_fes_app::stimulation_t(double stim_p[4]){

//    if (counter_waveform>0 && counter_waveform<EXERCISE_DURATION){ //campioni

//        stim.stimulation_current=IMIN+sqrt(q)*(IMAX-IMIN);
//        stim.stimulation_pulsewidth=PWMIN+sqrt(q)*(PWMAX-PWMIN);

//    }
//    stim.lowlevel_stimulate();

//}

void sensojoint_fes_app::stimulation_calibration(){
  //torque=c.data->motor_terms_sm.torque_actual_value;
  //cout << "pulse created" << endl;
  ll_channel_config.enable_stimulation = true;
  ll_channel_config.channel = Smpt_Channel_Red;
  ll_channel_config.number_of_points = 3;
  ll_channel_config.packet_number = packet_number;

  ll_channel_config.points[0].current = corrente;
  ll_channel_config.points[0].time = durata;

  ll_channel_config.points[1].time = 0;

  ll_channel_config.points[2].current = -corrente;
  ll_channel_config.points[2].time = durata;

  smpt_send_ll_channel_config(&device,&ll_channel_config);

  packet_number ++;
//    if(torque>prev_torque+DELTA_TORQUE && flag_min==0){
//            // IDENTIFICAZIONE MINIMO
//            A=round(corrente);
//            B=round(durata);
//            cout << "MINIMO CALIBRAZIONE IDENTIFICATO" << endl;

//            flag_min=1;
//        }

  if(corrente>30 || durata>500 ){
    stim.ll_channel_config.enable_stimulation = false;
    stim.ll_channel_config.channel = Smpt_Channel_Red;
    fes_mode = 10;
    change_mode = true;
    C=round(corrente);
    D=round(durata);
    cout << "MASSIMO CALIBRAZIONE IDENTIFICATO" << endl;
  }

  corrente+=0.12;
  durata+=1.5;
//        prev_torque=torque;
}

//CALIBRATION FILES
void sensojoint_fes_app:: open_calibration_file(){

  // Get system  time
  time_t t = time(nullptr);
  struct tm * now = localtime( & t );
  char buffer [80];

  // Open user-specific calibration file
  snprintf (buffer,80,"/home/esmacat/esmacat_rt/build-release/esmacat_applications/calib_FES_%s.csv", userID);
  CSV_calibration_file.open(buffer);

  if(CSV_calibration_file.is_open()){
    cout << "USER Calibration file opened." << endl;
  }
}

void sensojoint_fes_app::close_calibration_file(){

  if(CSV_calibration_file.is_open()){
    CSV_calibration_file.close();
    cout << "USER Calibration file closed correctly." << endl;
  }

}

void sensojoint_fes_app::write_calibration_file(){

  CSV_calibration_file

    << 1 << ","
    << A << ","
    << B << ","
    << C << ","
    << D << ","
//           << H << ","
//            << W << ","
//            << E << "," //coeff_weight_assistance << ","
//            << F << "," //<< avg_mean_position_error<< ","
//            << G << "," //<< avg_std_position_error << ","
    << userID ;
}

void read_calibration_file( vector <double> &calib){

  // Open user-specific calibration file
  char buffer [80];
  snprintf (buffer,80,"/home/esmacat/esmacat_rt/build-release/esmacat_applications/calib_FES_%s.csv", userID);
  CSV_calibration_file_read.open(buffer, ios::in);

  if(CSV_calibration_file_read.is_open()){
    cout << "USER Calibration file opened." << endl;
  }

  // Get the roll number
  // of which the data is required
  int rollnum=1, roll2, count = 0;

  // Read the Data from the file
  // as String Vector
  vector<string> row;
  string line, word, temp;

  row.clear();

  // read an entire row and
  // store it in a string variable 'line'
  getline(CSV_calibration_file_read, line);

  // used for breaking words
  stringstream s(line);

  // read every column data of a row and
  // store it in a string variable, 'word'
  while (getline(s, word, ',')) {
    // add all the column data
    // of a row to a vector
    row.push_back(word);
  }

  // convert string to integer for comparision
  roll2 = stoi(row[0]);

  // Compare the roll number
  if (roll2 == rollnum) {
    // Print the found data
    count = 1;
//        cout << "Imin   " << row[1] << "\n";
//        cout << "PWmin  " << row[2] << "\n";
//        cout << "Imax   " << row[3] << "\n";
//        cout << "PWmax  " << row[4] << "\n";
//        cout << "Weight Ass" << row[7] << "\n";
//        cout << "Avg Pos Err" << row[8] << "\n";
//        cout << "Avg std Pos Err" << row[9]<< "\n";
//        cout << "UserID " << row[10] << endl;

    calib[0]=stod(row[1]);
    A = calib[0];
    calib[1]=stod(row[2]);
    B = calib[1];
    calib[2]=stod(row[3]);
    C = calib[2];
    calib[3]=stod(row[4]);
    D = calib[3];
//        calib[4]=stod(row[5]);
//        H = calib[4];
//        calib[5]=stod(row[6]);
//        W = calib[5];
//        calib[6]=stod(row[7]);
//        E = calib[6];
//        calib[7]=stod(row[8]);
//        F = calib[7];
//        calib[8]=stod(row[9]);
//        G = calib[8];
//        calib[9]=stod(row[10]);
  }

  if (count == 0)
  {
    cout << "Record not found\n";// File pointer
  }

  if(CSV_calibration_file_read.is_open())
  {
    CSV_calibration_file_read.close();
    cout << "USER Calibration file closed correctly" << endl;
  }
}


//Functions for CSV file

void sensojoint_fes_app::open_sensojoint_file(){

    // Get system  time
    time_t t = time(nullptr);
    struct tm * now = localtime( & t );
    char buffer [80];

    // Log directory
    strftime (buffer,80,"FES_Knee-%Y-%m-%d-%H-%M-%S.csv",now);
    CSV_sensojoint_file.open (buffer);
    if(CSV_sensojoint_file.is_open())
    {
        PLOGI << "SENSOJOINT Log File Created";
        CSV_sensojoint_file << endl

                          << "mode" << ","
                          << "time" << ","
                          << "quadriceps_current" << ","
                          << "quadriceps_pulsewidth" << ","
                          << "quadriceps_charge" << ","
                          << "quadriceps_charge_correction" << ","
                          << "actual_position" << ","
                          << "desired_position" << ","
                          << "actual_torque" << ","
                          << "desired_torque" << ","
                          << "actual_velocity" << ","
                          << "stiffness" << ","
                          << "damping" << ","
                          << "allocation_factor" << ","
                          << "coeff_weight_assistance" << ","
//                          << "total_dynamic_torque" << ","
                          << "robot_dynamic_torque" << ","
                          << "leg_dynamic_torque" << ","
                          << "robot_allocated_dynamic_torque" << ","
                          << "fes_allocated_dynamic_torque" << ","
                          << "counter_waveform" << ","
                          << "start_traj" << "," //added
                          << "stimulated" << "," //added
                          << "mean_position_error" << ","
                          << "std_position_error" << ","
                          << "emg_trigger" << ","
//                          << "stimulation_phase" << ","
//                          << "current" << ","
                          << "final_threshold" << ","
                          << "Inverse dynamics torque" << ","
                          << "impedance_control_torque_mNm" << ","
                          << "torque_feedback" << ","
                          << "tot_gravity_torque_mNm" << ","
                          << "torque_feedfoward" << ","
                          << "beta_position" << ","
                          << "ms" << ","
                          << "ROM" << ",";
//                          << "avg_mean_position_error" <<","
//                          << "avg_std_position_error" << ",";
    }

    else{
        PLOGE << "SENSOJOINT Log File Error";
    }
}

void sensojoint_fes_app::write_sensojoint_file(){

    CSV_sensojoint_file << endl

               << fes_mode << ","
               << c.data->elapsed_time_ms << ","
               << stim.stimulation_current << ","
               << stim.stimulation_pulsewidth << ","
               << q << ","
               << q_adjusted <<","
               << act_pos << ","
               << des_pos << ","
               << c.data->motor_terms_sm.output_torque_actual_value << ","
               << c.data->impedance_terms.torque_tot << ","
               << c.data->motor_terms_sm.velocity_actual_value << ","
               << c.data->impedance_terms.K_gain << ","
               << c.data->impedance_terms.D_gain << "," //<< damping_nms_rad << ","
               << coeff_allocation << "," //<< coeff_allocation << ","
               << c.data->torque_terms.weight_assistance << ","
     //          << c.data->impedance_terms. << ","
               << c.data->torque_terms.gravity_robot_compensation_torque << ","
               << c.data->torque_terms.weight_compensation_torque << ","
               << c.data->torque_terms.gravity_robot_compensation_torque+c.data->torque_terms.weight_compensation_torque*coeff_allocation << "," //it should be only c.data->torque_terms.gravity_robot_compensation_torque*coeff_allocation
               << c.data->torque_terms.weight_compensation_torque*(1.0 - coeff_allocation) << ","
               << counter_waveform << ","
               << c.data->start_traj << "," //added
               << stimulated << "," //added
               << mean_position_error << ","
               << std_position_error << ","
               << c.data->emg << ","
//               << c.data->motor_terms_sm.position_actual_value*rad_to_deg << "," //valentina
//               << c.data->threshold << "," //valentina
//               << c.data->stimulation_phase << "," //valentina
//               << c.data->current << "," //valentina
//               << c.data->meanTorque << "," //valentina
//               << c.data->maxTorque << "," //valentina
               << emg_th_final << ","
               << c.data->inv_dyn_torque << ","
               << c.data->impedance_terms.impedance_control_torque_mNm << ","
               << c.data->impedance_terms.torque_feedback << ","
               << c.data->impedance_terms.tot_gravity_torque_mNm<< ","
               << c.data->impedance_terms.torque_feedfoward<< ","
               << beta_position<< ","
               << ms<< ","
               << ROM.size()<< ",";
//               << threshold_pos_error << ","
//               << threshold_std_pos_error << ",";
}

void sensojoint_fes_app::close_sensojoint_file(){

     CSV_sensojoint_file.close();

}


void sensojoint_fes_app:: open_ROM_file(){
   
    // Get system  time
    time_t t = time(nullptr);
    struct tm * now = localtime( & t );
    char buffer [80];

    // Open user-specific calibration file
    snprintf (buffer,80,"/home/esmacat/esmacat_rt/build-release/esmacat_applications/ROM_%s.csv", userID);
    CSV_ROM.open(buffer);

    if(CSV_ROM.is_open()){
        cout << "USER ROM file opened." << endl;
    }
}

void sensojoint_fes_app::close_ROM_file(){

    if(CSV_ROM.is_open()){
        CSV_ROM.close();
        cout << "USER ROM file closed correctly." << endl;
    }

}

void sensojoint_fes_app::write_ROM_file(){

    CSV_ROM

            << 1 << ","
            << end_transp_traj <<",";

            //<< userID ;

}

void read_ROM_file( vector <double> &range){

    // Open user-specific calibration file
    char buffer [80];
    snprintf (buffer,80,"/home/esmacat/esmacat_rt/build-release/esmacat_applications/ROM_%s.csv", userID);
    CSV_ROM_read.open(buffer, ios::in);

    if(CSV_ROM_read.is_open()){
        cout << "USER ROM file opened." << endl;
    }

    // Get the roll number
    // of which the data is required
    int rollnum=1, roll2, count = 0;


    // Read the Data from the file
    // as String Vector
    vector<string> row;
    string line, word, temp;

    row.clear();

    // read an entire row and
    // store it in a string variable 'line'
    getline(CSV_ROM_read, line);


    // used for breaking words
    stringstream s(line);

    // read every column data of a row and
    // store it in a string variable, 'word'
    while (getline(s, word, ',')) {

        // add all the column data
        // of a row to a vector
        row.push_back(word);
    }

    // convert string to integer for comparision
    roll2 = stoi(row[0]);

    // Compare the roll number
    if (roll2 == rollnum) {

        // Print the found data
        count = 1;

        range[0]=stod(row[1]);
        ex_amplitude = range[0];


    }

    if (count == 0)

        cout << "Record not found\n";// File pointer


    if(CSV_ROM_read.is_open())
    {
        CSV_ROM_read.close();
        cout << "USER ROM file closed correctly" << endl;
    }

}



void sensojoint_fes_app::generate_betafunction(){

}
