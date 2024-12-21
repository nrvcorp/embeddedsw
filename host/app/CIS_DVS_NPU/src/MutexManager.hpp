#ifndef MUTEXMANAGER_HPP
#define MUTEXMANAGER_HPP

#include <pthread.h>
#include <thread>
#include <atomic>

class MutexManager
{
public:
    MutexManager();
    pthread_mutex_t &getMutex();
    int getReady();
    void setReady(int ready);
    void lock_pipeline();
    void unlock_pipeline();
    void lock_single_writer();
    void unlock_single_writer(int is_update);
    void lock_multiple_reader();
    void unlock_multiple_reader();
    ~MutexManager();
    void lock_display();
    void unlock_display();
    void terminate();
    
private:
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int cond_ready;
    
};

#endif // MUTEXMANAGER_HPP
