#ifndef MUTEXMANAGER_HPP
#define MUTEXMANAGER_HPP

#include <pthread.h>

class MutexManager
{
public:
    MutexManager()
    {
        pthread_mutex_init(&display_mutex, nullptr);
    }

    pthread_mutex_t &getMutex()
    {
        return display_mutex;
    }

    void lock_display()
    {
        pthread_mutex_lock(&display_mutex);
    }

    void unlock_display()
    {
        pthread_mutex_unlock(&display_mutex);
    }
    ~MutexManager()
    {
        pthread_mutex_destroy(&display_mutex);
    }

private:
    pthread_mutex_t display_mutex;
};

class MutexCond
{
public:
    MutexCond()
    {
        pthread_mutex_init(&mutex, NULL);
        pthread_cond_init(&cond, NULL);
        cond_ready = 0;
    }
    pthread_mutex_t &getMutex()
    {
        return mutex;
    }
    int getReady()
    {
        return cond_ready;
    }
    void lock_pipeline()
    {
        pthread_mutex_lock(&mutex);
        while (!cond_ready)
        {
            pthread_cond_wait(&cond, &mutex);
        }
        cond_ready = 0;
    }
    void unlock_pipeline()
    {
        cond_ready = 1;
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&mutex);
    }
    void lock_single_writer()
    {
        pthread_mutex_lock(&mutex);
    }
    void unlock_single_writer(int is_update = 1)
    {
        if (is_update)
        {
            cond_ready = 1;
            pthread_cond_signal(&cond);
        }
        pthread_mutex_unlock(&mutex);
    }
    void lock_multiple_reader()
    {
        pthread_mutex_lock(&mutex);
        while (!cond_ready)
        {
            pthread_cond_wait(&cond, &mutex);
        }
        cond_ready = 0;
    }
    void unlock_multiple_reader()
    {
        pthread_mutex_unlock(&mutex);
    }
    ~MutexCond()
    {
        pthread_mutex_destroy(&mutex);
        pthread_cond_destroy(&cond);
    }

private:
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int cond_ready;
};

#endif // MUTEXMANAGER_HPP
