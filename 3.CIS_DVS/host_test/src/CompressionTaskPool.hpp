#pragma once

#include "config.hpp" // CV_WORKER_CORES_LIST, CV_WORKER_CORES_NUM
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <fcntl.h>
#include <mutex>
#include <queue>
#include <string>
#include <sys/uio.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <zlib.h>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

//------------------------------------------------------------------------------
// CompressionTaskPool: 고성능 병렬 압축·저장 풀
//  - std::function/iostream 제거, lock-minimized 경량 큐
//  - thread-local 재사용 버퍼 + writev 사용
//  - SCHED_FIFO 우선순위, CPU 코어 고정
//------------------------------------------------------------------------------
class CompressionTaskPool
{
  public:
    struct Task
    {
        const uint8_t *data;
        size_t size;
        std::string filename;
    };

    explicit CompressionTaskPool(const std::vector<int> &cores)
        : stop_flag(false)
    {
        for (int core : cores)
        {
            workers_.emplace_back([this, core]()
                                  {
                set_affinity(core);
                elevate_to_max_priority();
                workerLoop(); });
        }
    }

    ~CompressionTaskPool()
    {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            stop_flag = true;
        }
        cv_.notify_all();
        for (auto &th : workers_)
        {
            if (th.joinable())
                th.join();
        }
    }

    // 논블로킹 enqueue
    void enqueue(const uint8_t *data, size_t size, std::string filename)
    {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            tasks_.push(Task{data, size, std::move(filename)});
        }
        cv_.notify_one();
    }

  private:
    std::vector<std::thread> workers_;
    std::queue<Task> tasks_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_flag;

    static void set_affinity(int core_id)
    {
#ifdef __linux__
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_id, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
#endif
    }

    static void elevate_to_max_priority()
    {
#ifdef __linux__
        struct sched_param sch{};
        sch.sched_priority = sched_get_priority_max(SCHED_FIFO);
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
#endif
    }

    void workerLoop()
    {
        // thread-local compression buffer
        thread_local std::vector<uint8_t> compBuf;
        thread_local uLongf lastBound = 0;

        while (true)
        {
            Task task;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait(lk, [this]
                         { return stop_flag || !tasks_.empty(); });
                if (stop_flag && tasks_.empty())
                    return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            // 압축
            uLongf bound = compressBound(task.size);
            if (bound > lastBound)
            {
                compBuf.resize(bound);
                lastBound = bound;
            }
            uLongf compSize = bound;
            compress2(compBuf.data(), &compSize, task.data, task.size, Z_BEST_SPEED); // Z_BEST_SPEED Z_DEFAULT_COMPRESSION Z_BEST_COMPRESSION

            // 파일 쓰기 (writev)
            int fd = ::open(task.filename.c_str(), O_CREAT | O_WRONLY | O_TRUNC | O_CLOEXEC, 0644);
            if (fd < 0)
            {
                _exit(1);
            }
            uint64_t orig = task.size, comp = compSize;
            struct iovec iov[3];
            iov[0].iov_base = &orig;
            iov[0].iov_len = sizeof(orig);
            iov[1].iov_base = &comp;
            iov[1].iov_len = sizeof(comp);
            iov[2].iov_base = compBuf.data();
            iov[2].iov_len = compSize;
            size_t toWrite = sizeof(orig) + sizeof(comp) + compSize;
            size_t written = 0;
            while (written < toWrite)
            {
                ssize_t w = ::writev(fd, iov, 3);
                if (w < 0)
                {
                    _exit(1);
                }
                written += w;
            }
            ::close(fd);
        }
    }
};
