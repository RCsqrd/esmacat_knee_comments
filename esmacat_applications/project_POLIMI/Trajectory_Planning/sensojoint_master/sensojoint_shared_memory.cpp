/** @file
 * @brief Contains definitions of functions used for the primary executable of Harmony SHR
 *
*/
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "sensojoint_shared_memory.h"


/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
sensojoint_shared_memory_comm::sensojoint_shared_memory_comm()
{
    key = ftok("/bin",DEFAULT_ROS_KEY_ID);
}
sensojoint_shared_memory_comm::~sensojoint_shared_memory_comm(){
    detach_shared_memory();
}

bool sensojoint_shared_memory_comm::init(){
    // shmget returns an identifier in shmid
    shared_memory_packet temp;
    shmid = shmget(key, sizeof(temp),0666|IPC_CREAT);
    // shmat to attach to shared memory
    data = (shared_memory_packet*) shmat(shmid,(void*)0,0);
    //data->mode = control_mode_t::standby;
    data->stop = false;
    if (shmid > 0 && key > 0) return true; // no error
    else return false; // error
}

void sensojoint_shared_memory_comm::detach_shared_memory(){
    shmdt(data);
    shmctl(shmid,IPC_RMID,NULL);
}





