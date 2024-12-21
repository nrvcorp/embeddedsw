#include "MutexManager.hpp"
MutexManager::MutexManager()
    {
        pthread_mutex_init(&mutex, NULL);
        pthread_cond_init(&cond, NULL);
        cond_ready = 0;
    }
    pthread_mutex_t &MutexManager::getMutex()
    {
        return mutex;
    }
    int MutexManager::getReady()
    {
        return cond_ready;
    }
    void MutexManager::setReady(int ready)
    {
        cond_ready = ready;
    }
    void MutexManager::lock_pipeline()
    {
        pthread_mutex_lock(&mutex);
        while (!cond_ready)
        {
            pthread_cond_wait(&cond, &mutex);
        }
        cond_ready = 0;
    }
    void MutexManager::unlock_pipeline()
    {
        cond_ready = 1;
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&mutex);
    }
    void MutexManager::lock_single_writer()
    {
        pthread_mutex_lock(&mutex);
    }
    void MutexManager::unlock_single_writer(int is_update = 1)
    {
        if (is_update)
        {
            cond_ready = 1;
            pthread_cond_signal(&cond);
        }
        pthread_mutex_unlock(&mutex);
    }
    void MutexManager::lock_multiple_reader()
    {
        pthread_mutex_lock(&mutex);
        while ( !cond_ready)
        {
            pthread_cond_wait(&cond, &mutex);
        }
        cond_ready = 0;
    }
    void MutexManager::unlock_multiple_reader()
    {
        pthread_mutex_unlock(&mutex);
    }
    MutexManager::~MutexManager()
    {
        pthread_mutex_destroy(&mutex);
        pthread_cond_destroy(&cond);
    }
    void MutexManager::lock_display()
    {
        pthread_mutex_lock(&mutex);
    }
    void MutexManager::unlock_display()
    {
        pthread_mutex_unlock(&mutex);
    }
    void MutexManager::terminate(){
        cond_ready = 1;
        pthread_cond_broadcast(&cond);
    }