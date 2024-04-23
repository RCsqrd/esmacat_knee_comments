#include "sensojoint_fes_manager.h"
bool kbhit();

//// Text Color Identifiers
// string boldred_key = "\033[1;31m";
// string red_key = "\033[31m";
// string boldpurple_key = "\033[1;35m";
// string yellow_key = "\033[33m";
// string blue_key = "\033[36m";
// string green_key = "\033[32m";
// string color_key = "\033[0m";

sensojoint_FES_manager::sensojoint_FES_manager(){
    // Initializing the shared memory
    if (app.c.init())
    {
        PLOGI << "User Interface shared memory initialized with key: " << hex << app.c.get_shared_memory_key();// start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        app.c.detach_shared_memory();
    }
}

void sensojoint_FES_manager::getinfo(){
    struct sched_param param;
    int policy;

    sched_getparam(0, &param);
    printf("Priority of this process: %d\n\r", param.sched_priority);

    pthread_getschedparam(pthread_self(), &policy, &param);

    printf("Priority of the thread: %d, current policy is: %d\n\r",
              param.sched_priority, policy);
}

// Thread functions

// Non REAL-TIME thread - used for interface (i.e. no need for deterministic behaviour)

void sensojoint_FES_manager::nrt_thread(){
    /* NRT code */
   
    loop_time_stats non_realtime_loop_time_stats("non_realtime_loop_time_stats.txt",loop_time_stats::output_mode::screenout_only);

    struct timespec t_start, t_now, t_next, t_period, t_prev,t_result;

    PLOGW << "NRT Thread";

    getinfo();

    clock_gettime(CLOCK_REALTIME, &t_start);
    t_next = t_start;
    unsigned long int loop_count = 0;
    unsigned long int t_overflow = 0;

    /* Calculate the time for the execution of this task*/
    t_period.tv_sec = 0;
    t_period.tv_nsec = DEFAULT_LOOP_TIME_NS;

    print_command_keys();

    while(app.c.data->stop==false){
        t_prev = t_next;
        non_realtime_loop_time_stats.loop_starting_point(); // record the starting time of each iteration

        // Do here whatever you want to do within the thread
        interface();

        if(loop_count%1000==0){
            timespec_sub(&t_result,&t_now,&t_start); // calculate the time passed since the start of the thread
            //PLOGI << yellow_key << "NRT Clock: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;
        }
        loop_count++;

        // Calculate the next iteration time
        timespec_add_nsec(&t_next, &t_prev, DEFAULT_LOOP_TIME_NS*10);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t_next, NULL);
        clock_gettime ( CLOCK_REALTIME, &t_now);

        t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
        if(t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
        {
//        cout << red_key << "NRT Overflow: " << t_overflow << color_key  << endl;
        }
    }

    stop_nrt_thread();
}

//REAL-TIME thread

//void sensojoint_FES_manager::rt_thread(){

//    loop_time_stats realtime_loop_time_stats("realtime_loop_time_stats.txt",loop_time_stats::output_mode::screenout_only);

//    struct timespec t_start, t_now, t_next, t_period, t_prev,t_result;

//    PLOGW << "RT Thread";

//    getinfo();

//    clock_gettime(CLOCK_MONOTONIC, &t_start);
//    clock_gettime(CLOCK_MONOTONIC, &t_now);
//    t_next = t_now;
////    unsigned long int loop_count = 0;
//    unsigned long loop_count=0;
//    unsigned long int t_overflow = 0;

//    /* Calculate the time for the execution of this task*/
//    t_period.tv_sec = 0;
//    t_period.tv_nsec = DEFAULT_LOOP_TIME_NS;

//    while(app.c.data->stop==false)
//    {
//        t_prev = t_next;
//        realtime_loop_time_stats.loop_starting_point();

//        // Do here whatever you want to do within the thread

//        //cout << "Im in the loop" << endl;
////        app.FES_loop();
//        if(app.counter%1000000==0){
//        cout << app.counter << endl;}

//        if(app.counter%1000==0){
//            timespec_sub(&t_result,&t_now,&t_start);
////            PLOGI << yellow_key << "RT Clock: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;
//        }

//        app.counter++;
//        timespec_add_nsec(&t_next, &t_prev, DEFAULT_LOOP_TIME_NS*10);
//        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t_next, NULL);
//        clock_gettime ( CLOCK_REALTIME, &t_now);

//        t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
//                if(t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
//                {
//        //            cout << red_key << "RT Overflow: " << t_overflow << color_key  << endl;
//                }
//    }

//    stop_rt_thread();

//}

//REAL-TIME thread
void sensojoint_FES_manager::rt_thread(){

    loop_time_stats realtime_loop_time_stats("realtime_loop_time_stats.txt",loop_time_stats::output_mode::screenout_only);

    struct timespec t_start, t_now, t_next, t_period, t_prev,t_result;

    PLOGW << "RT Thread";

    getinfo();

    clock_gettime(CLOCK_MONOTONIC, &t_start);
    clock_gettime(CLOCK_MONOTONIC, &t_now);
    t_next = t_now;

    /* Calculate the time for the execution of this task*/
    t_period.tv_sec = 0;
    t_period.tv_nsec = DEFAULT_LOOP_TIME_NS;

    while(app.c.data->stop==false){
        realtime_loop_time_stats.loop_starting_point();
        t_period.tv_sec = 0;
        t_period.tv_nsec =DEFAULT_LOOP_TIME_NS;
        TIMESPEC_INCREMENT(t_next,t_period);

        // Do here whatever you want to do within the thread

        //calculate the elapsed time since the start of the thread
        if(app.counter%1==0){
            timespec_sub(&t_result,&t_now,&t_start);
            app.FES_loop();
        }

//        if(app.counter%1000==0){
//            timespec_sub(&t_result,&t_now,&t_start);
////            PLOGI << yellow_key << "RT Clock: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;
//        }

        app.counter++;
//        timespec_add_nsec(&t_next, &t_prev, DEFAULT_LOOP_TIME_NS*10);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, NULL);
        clock_gettime ( CLOCK_MONOTONIC, &t_now);

        app.t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
                if(app.t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
                {
        //            cout << red_key << "RT Overflow: " << t_overflow << color_key  << endl;
                }
    }

    realtime_loop_time_stats.store_loop_time_stats();
    stop_rt_thread();

}


void sensojoint_FES_manager::join_nrt_thread()
{
    (void) pthread_join(pthread_nrt_input, NULL);
}

void sensojoint_FES_manager::join_rt_thread()
{
    (void) pthread_join(pthread_rt_input, NULL);
}

void sensojoint_FES_manager::stop_nrt_thread(){

}

void sensojoint_FES_manager::stop_rt_thread(){

}

bool sensojoint_FES_manager::start_nrt_thread(){

    // Scheduler variables
    int policy; // policy of the thread
    struct sched_param prio; 
    pthread_attr_t attr; // thread attributes

    pthread_attr_init(&attr); // initialize the thread attributes with default values
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); // set the thread attributes to be explicitly scheduled

    policy = SCHED_OTHER; 

    pthread_attr_setschedpolicy(&attr, policy);
    prio.sched_priority = 1; // priority range should be btw -20 to +19
    pthread_attr_setschedparam(&attr,&prio);

    if ( pthread_create(&pthread_nrt_input, &attr, internal_nrt, this) ){
        PLOGE << "Error: nrt thread" ; // print error message if return value is not 0
        return 1;
    }
    return 0;
}

bool sensojoint_FES_manager::start_rt_thread(){

    // Scheduler variables
    int policy;
    struct sched_param prio;
    pthread_attr_t attr;

    pthread_attr_init( &attr);
    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);

    policy = SCHED_RR;

    pthread_attr_setschedpolicy( &attr, policy);
    prio.sched_priority = 1; // priority range should be btw -20 to +19
    pthread_attr_setschedparam(&attr,&prio);

    if ( pthread_create(&pthread_rt_input, &attr, internal_rt, this) ){
        PLOGE << "Error: rt thread" ;
        return 1;
    }
    return 0;
}

/*use to manage the interface of the application, the selected 'mode' is then pass to 'fes_mode'variable
also use in the 'seneojoint_FES_app' where each mode is explained in details
*/
void sensojoint_FES_manager::interface(){

    // Non-blocking function, if keyboard hit is detected, do something.
    if(kbhit()) {
        char input_char = '0';
        uint16_t mode = 1;

        // Get character
//        cin >> input_char;
        input_char = getchar();

        switch (input_char){
        case ' ':
            print_command_keys();
            app.change_mode = false; 
            break;
        case '0':
            cout << "MOD changed to STANDBY" << endl;
            mode=0;
            break;
        case '1':
            cout << "MOD changed to FES + MOTOR" << endl;
            mode=1;
            break;
        case 'm':
            cout << "MOD changed to ONLY MOTOR" << endl;
            mode=100;
            break;
        case 'c':
            cout << "MOD changed to weight coeff CALIBRATION" << endl;
            mode=102;
            break;
        case '2':
            cout << "MOD changed to TRANSPARENT" << endl;
            mode=2;
            break;
        case '3':
            cout << "MOD changed to QUIT" << endl;
            mode=3;
            break;
        case '4':
            cout << "MOD changed to FREEZE" << endl;
            mode=4;
            break;
        case '5':
            cout << "MOD changed to TARGET POSITION CONTROL" << endl;
            mode=5;
            break;
        case '6':
            cout << "MOD changed to ANTI-G CONTROL" << endl;
            mode=6;
            break;
        case '7':
            cout << "MOD changed to GO TO POSITION CONTROL" << endl;
            mode=7;
            break;
        case '8':
            cout << "MOD changed to FES + MOTOR + EMG" << endl;
            mode=8;
            break;
        case '9':
            cout << "MOD changed to FES CALIBRATION" << endl;
            mode=9;
            break;
        case 'z':
            // se schiaccio z passo nella fase di stop della calibrazione
            mode=10;
            break;
        case 'a':
            mode=101;
            break;
        case 'e':
            mode=13;
            break;
//        case 's':
//            mode=14;
//            break;
        case 'b':
            mode=15;
            break;

//        default:
//            cout << red_key << "INVALID INPUT - stopping test"<< color_key << endl;
//            break;
        }

        if(input_char!=' ' && input_char!='0' && input_char!='1'  && input_char!='2' && input_char!='3'  && input_char!='4'  && input_char!='5'  && input_char!='6' && input_char!='7' && input_char!='8' && input_char!='9' && input_char!='z' && input_char!='e' && input_char!='a' && input_char!='m' && input_char!='b' && input_char!='c'){
         app.change_mode = false;
         print_command_keys();
         }
        else if(input_char=='0' || input_char=='1'  || input_char=='2' || input_char=='3'|| input_char=='4' || input_char=='5' || input_char=='6' || input_char=='7' || input_char=='8' || input_char=='9' || input_char=='z' || input_char=='e' || input_char=='a' || input_char=='m' || input_char=='b' || input_char=='c'){
           //update fes_mode variable and change_mode flag --> will be used in 'sensojoint_fes_app.cpp'
            app.fes_mode = mode;
            app.change_mode = true;
 //         print_command_keys();
        }
    }
}

// Wait for keyboard interrupt
bool kbhit()
{
    termios term; // terminal structure
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON; // disable canonical mode --> input available immediately
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting); // check if there are bytes waiting to be read

    tcsetattr(0, TCSANOW, &term); // restore the terminal settings

    return byteswaiting > 0;
}

// Print commands on terminal
void sensojoint_FES_manager::print_command_keys()
{
  std::cout   << ": Enter command for test"  << endl;
  std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
  std::cout << blue_key << "\'0\'" << color_key << ": standby mode" << "\n";
  std::cout << blue_key << "\'m\'" << color_key << ": MOTOR mode"<< "\n";
  std::cout << blue_key << "\'c\'" << color_key << ": WEIGHT coeff CALIBRATION mode"<< "\n";
  std::cout << blue_key << "\'1\'" << color_key << ": FES+MOTOR mode"<< "\n";
  std::cout << blue_key << "\'2\'" << color_key << ": TRANSPARENT mode"<< "\n";
  std::cout << blue_key << "\'3\'" << color_key << ": QUIT mode"<< "\n";
  std::cout << blue_key << "\'4\'" << color_key << ": FREEZE CONTROL mode"<< "\n";
  std::cout << blue_key << "\'5\'" << color_key << ": TARGET POSITION CONTROL mode"<< "\n";
  std::cout << blue_key << "\'6\'" << color_key << ": ANTIG CONTROL mode"<< "\n";
  std::cout << blue_key << "\'7\'" << color_key << ": GO TO POSITION mode"<< "\n";
  std::cout << blue_key << "\'8\'" << color_key << ": FES+MOTOR+EMG mode"<< "\n";
  std::cout << blue_key << "\'9\'" << color_key << ": FES CALIBRATION mode"<< "\n";
  std::cout << blue_key << "\'a\'" << color_key << ": MIN CURRENT DETECTION mode"<< "\n";
  std::cout << blue_key << "\'z\'" << color_key << ": MAX CURRENT DETECTION mode"<< "\n";
  std::cout << blue_key << "\'e\'" << color_key << ": EMG CALIBRATION mode"<< "\n";
  std::cout << blue_key << "\'b\'" << color_key << ": Beta fucntion generation"<< "\n";
  cout << endl;
}
