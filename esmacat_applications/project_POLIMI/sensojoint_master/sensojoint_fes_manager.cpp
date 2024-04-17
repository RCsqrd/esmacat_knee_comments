#include "sensojoint_fes_manager.h"
bool kbhit();



// Text Color Identifiers
 string boldred_key = "\033[1;31m";
 string red_key = "\033[31m";
 string boldpurple_key = "\033[1;35m";
 string yellow_key = "\033[33m";
 string blue_key = "\033[36m";
 string green_key = "\033[32m";
 string color_key = "\033[0m";

sensojoint_FES_manager::sensojoint_FES_manager()
{
    // Initializing the shared memory
    if (app.c.init())
    {
        PLOGI << "User Interface shared memory initialized with key: " << hex << app.c.get_shared_memory_key();    // start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        app.c.detach_shared_memory();
    }
}

void sensojoint_FES_manager::getinfo()
{
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

    while(app.c.data->stop==false)
    {
        t_prev = t_next;
        non_realtime_loop_time_stats.loop_starting_point();

        // Do here whatever you want to do within the thread
        interface();
//    cout << "Im in the loop" << endl;

        if(loop_count%1000==0){
            timespec_sub(&t_result,&t_now,&t_start);
            //PLOGI << yellow_key << "NRT Clock: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;
        }
        loop_count++;
        timespec_add_nsec(&t_next, &t_prev, DEFAULT_LOOP_TIME_NS*10);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t_next, NULL);
        clock_gettime ( CLOCK_REALTIME, &t_now);

        t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
                if(t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
                {
        //            cout << red_key << "NRT Overflow: " << t_overflow << color_key  << endl;
                }
    }

    stop_nrt_thread();

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

bool sensojoint_FES_manager::start_nrt_thread(){


    // Scheduler variables
    int policy;
    struct sched_param prio;
    pthread_attr_t attr;

    pthread_attr_init( &attr);
    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);

    policy = SCHED_OTHER;

    pthread_attr_setschedpolicy( &attr, policy);
    prio.sched_priority = 1; // priority range should be btw -20 to +19
    pthread_attr_setschedparam(&attr,&prio);

    if ( pthread_create(&pthread_nrt_input, &attr, internal_nrt, this) ){
        PLOGE << "Error: nrt thread" ;
        return 1;
    }
    return 0;
}


void sensojoint_FES_manager::interface()
{

    // Non-blocking function, if keyboard hit is detected, do something.
    if(kbhit())

    {

        char input_char = '0';
        uint16_t mode = 1;

        // Get character
//        cin >> input_char;
        input_char = getchar();

        switch (input_char){

        case ' ':
            print_command_keys();
            break;
        case '0':
            cout << "MOD changed to 0" << endl;
            mode=0;
            break;
        case '1':
            cout << "MOD changed to 1" << endl;
            mode=1;
            break;
        case '2':
            cout << "MOD changed to 2" << endl;
            mode=2;
            break;
        case '3':
            cout << "MOD changed to 3" << endl;
            mode=3;
            break;
        default:
            cout << red_key << "INVALID INPUT - stopping test"<< color_key << endl;
            break;
        }

        if(input_char!=' ' && input_char!='0' && input_char!='1'  && input_char!='2' && input_char!='3'){
//         app.rehamove_mode = mode;
//         app.change_mode = true;
         print_command_keys();
         }

    }



}

// Wait for keyboard interrupt
bool kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

// Print commands on terminal
void sensojoint_FES_manager::print_command_keys()
{
  std::cout   << ": Enter command for test"  << endl;
  std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
  std::cout << blue_key << "\'0\'" << color_key << ": 0 mode" << "\n";
  std::cout << blue_key << "\'1\'" << color_key << ": 1 mode"<< "\n";
  std::cout << blue_key << "\'2\'" << color_key << ": 2 mode"<< "\n";
  std::cout << blue_key << "\'3\'" << color_key << ": 3 mode"<< "\n";
  cout << endl;

}
