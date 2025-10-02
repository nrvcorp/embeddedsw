#include <condition_variable>
#include <mutex>

#ifndef COUNTING_SEMAPHORE_HPP
#define COUNTING_SEMAPHORE_HPP

/* 사용방법 예시
비어있는 버퍼 수를 세는 객체와 차있는 버퍼 수를 세는 객체 생성
생성자에는 카운터 초기값 전달 (각각 버퍼 크기, 0 전달)

acquire(n) => n개의 버퍼가 준비되지 않았으면 sleep
release(n) => n개의 버퍼를 준비하고 sleep 상태의 쓰레드를 깨움

공급 쓰레드 {
        비어있는버퍼카운터.acquire(n)
        //n개 공급
        차있는버퍼카운터.release(n)
}
소비 쓰레드 {
        차있는버퍼카운터.acquire(m)
        //m개 소비
        비어있는버퍼카운터.release(m)
}*/

class CountingSemaphore
{
  public:
    explicit CountingSemaphore(unsigned init = 0)
        : count_(init) {}

    void acquire(unsigned n = 1)
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]
                 { return count_ >= n; });
        count_ -= n;
    }
    void release(unsigned n = 1)
    {
        {
            std::lock_guard<std::mutex> lk(m_);
            count_ += n;
        }
        cv_.notify_all();
    }
    void release_all()
    {
        {
            std::lock_guard<std::mutex> lk(m_);
            count_ = UINT_MAX;
        }
        cv_.notify_all();
    }
    unsigned int get_count()
    {
        {
            std::lock_guard<std::mutex> lk(m_);
            return count_;
        }
    }

  private:
    std::mutex m_;
    std::condition_variable cv_;
    unsigned int count_;
};

#endif
