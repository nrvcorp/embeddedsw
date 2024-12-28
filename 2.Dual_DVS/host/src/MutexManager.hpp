#ifndef MUTEXMANAGER_HPP
#define MUTEXMANAGER_HPP

#include <pthread.h>
#include <thread>
#include <atomic>

class MutexManager
{
public:
    MutexManager();
   /**
    * returns member variable 'mutex' 
    */
    pthread_mutex_t &getMutex();
   /**
    * returns member variable 'cond_ready' 
    */
    int getReady();
   /**
    * sets member variable 'cond_ready'
    * @param ready value to set cond_ready to
    */
    void setReady(int ready);
   /**
    * locks the mutex, so that only one thread can acquire this mutex at a time
    * useful for multiple identical threads with pipelinind structure
    * prerequisites:
    *   > cond_ready is set to 1
    */
    void lock_pipeline();
   /**
    * unlocks the mutex, waking another random thread sharing this mutex. 
    * useful for multiple identical threads with pipelinind structure
    */
    void unlock_pipeline();
   /**
    * locks the mutex, without waiting for cond signaling.
    * useful for single-writer multiple-reader situations
    */
    void lock_single_writer();
    /**
    * unlocks the mutex, signaling a random reader to wake it up.
    * useful for single-writer multiple-reader situations
    * @param is_update if 0, does not wake up any readers. 
    */
    void unlock_single_writer(int is_update= 1);
    /**
    * locks the mutex, waiting for cond signaling.
    * useful for single-writer multiple-reader situations
    */
    void lock_multiple_reader();
    /**
    * unlocks the mutex without cond signaling.
    * useful for single-writer multiple-reader situations
    */
    void unlock_multiple_reader();
    ~MutexManager();
    /**
    * simply locks the mutex
    */
    void lock_display();
    /**
    * simply unlocks the mutex
    */
    void unlock_display();
    /**
    function to wake waiting threads in order to terminate them properly. 
    */
    void terminate();
    
private:
    //pthread mutex
    pthread_mutex_t mutex;
    //pthread cond 
    pthread_cond_t cond;
    //ready flag for cond
    int cond_ready;
    
};

#endif // MUTEXMANAGER_HPP
