#ifndef SENSOJOINT_FES_MANAGER_H
#define SENSOJOINT_FES_MANAGER_H
#include "sensojoint_fes_app.h"
#include "headers_fes.h"


#define DEFAULT_LOOP_TIME_NS 1000000L
#define DEFAULT_APP_DURATION_COUNTS 10000
#define ALLOWED_LOOPTIME_OVERFLOW_NS 200000L

#ifdef _POSIX_PRIORITY_SCHEDULING
#define POSIX "POSIX 1003.1b\n";
#endif
#ifdef _POSIX_THREADS
#ifdef _POSIX_THREAD_PRIORITY_SCHEDULING
// #define POSIX "POSIX 1003.1c\n";
#endif
#endif

// Text Color Identifiers
const string boldred_key = "\033[1;31m";
const string red_key = "\033[31m";
const string boldpurple_key = "\033[1;35m";
const string yellow_key = "\033[33m";
const string blue_key = "\033[36m";
const string green_key = "\033[32m";
const string color_key = "\033[0m";


class sensojoint_FES_manager
{
    private:
    static void *internal_nrt(void * This) {((sensojoint_FES_manager *)This)->nrt_thread(); return nullptr;}
    static void *internal_rt(void * This) {((sensojoint_FES_manager *)This)->rt_thread(); return nullptr;}
    static void getinfo();

    // Pthreads variables
    pthread_t pthread_nrt_input;
    pthread_t pthread_rt_input;
public:

    sensojoint_fes_app app;
    sensojoint_FES_manager();
    bool start_nrt_thread();
    bool start_rt_thread();
    void nrt_thread();
    void rt_thread();
    void stop_nrt_thread();
    void stop_rt_thread();
    void interface();
    void print_command_keys();
    void join_nrt_thread();
    void join_rt_thread();
};

#endif // SENSOJOINT_FES_MANAGER_H
