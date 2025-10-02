#pragma once

#include "OpenCVWorker.hpp" // OpenCVWorker, OpenCVTask, OpenCVTaskType
#include <atomic>
#include <memory>
#include <vector>

//------------------------------------------------------------------------------
// OpenCVTaskManager
//   - 여러 OpenCVWorker 인스턴스를 멀티코어에 걸쳐 관리
//   - 지정 모드: core ID 리스트에 명시적으로 고정
//   - 자동 모드: 시스템 코어 수 기반 풀 생성
//------------------------------------------------------------------------------
class OpenCVTaskManager
{
  public:
    // 지정 모드 생성자: core_ids에서 명시한 CPU 코어에 각각 고정된 워커 생성
    static std::unique_ptr<OpenCVTaskManager> createFixed(const std::vector<int> &core_ids)
    {
        std::vector<std::unique_ptr<OpenCVWorker>> workers;
        workers.reserve(core_ids.size());
        for (int core : core_ids)
        {
            workers.emplace_back(std::make_unique<OpenCVWorker>(core));
        }
        return std::unique_ptr<OpenCVTaskManager>(new OpenCVTaskManager(std::move(workers)));
    }

    // 자동 모드 생성자: 지정된 워커 개수만큼 코어 고정 없이 생성
    static std::unique_ptr<OpenCVTaskManager> createAuto(int num_workers)
    {
        std::vector<std::unique_ptr<OpenCVWorker>> workers;
        workers.reserve(num_workers);
        for (int i = 0; i < num_workers; ++i)
        {
            workers.emplace_back(std::make_unique<OpenCVWorker>(-1)); // -1: affinity 설정 없음
        }
        return std::unique_ptr<OpenCVTaskManager>(new OpenCVTaskManager(std::move(workers)));
    }

    // 단일 task를 라운드 로빈으로 분산 enqueue
    void enqueueTask(const OpenCVTask &task)
    {
        size_t idx = rrCounter_++ % workers_.size();
        workers_[idx]->enqueue(task);
    }

    // 모든 워커에게 동일 task를 브로드캐스트 enqueue
    void broadcastTask(const OpenCVTask &task)
    {
        for (auto &w : workers_)
        {
            w->enqueue(task);
        }
    }

  private:
    explicit OpenCVTaskManager(std::vector<std::unique_ptr<OpenCVWorker>> &&workers)
        : workers_(std::move(workers)), rrCounter_(0) {}

    std::vector<std::unique_ptr<OpenCVWorker>> workers_; // 풀 내부 워커들
    std::atomic<size_t> rrCounter_;                      // 라운드 로빈 인덱스
};
