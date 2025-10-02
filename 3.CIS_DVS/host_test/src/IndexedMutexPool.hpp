#pragma once
#include <vector>
#include <mutex>
#include <cassert>

/*
 * IndexedMutexPool
 * ----------------
 *  N개의 std::mutex를 인덱스로 관리하는 경량 래퍼.
 *  C++17 이상에서 사용 가능.
 *
 *  사용 예:
 *      IndexedMutexPool mp(8);
 *      {
 *          auto g = mp.acquire(i);  // lock
 *          ...                      // 보호 구간
 *      }                            // 자동 unlock
 */
class IndexedMutexPool {
public:
    explicit IndexedMutexPool(std::size_t n) : mtx_(n) {}

    /* RAII Guard: 스코프 종료 시 자동 unlock */
    class Guard {
    public:
        explicit Guard(std::mutex& m) : lock_(m) {}
    private:
        std::unique_lock<std::mutex> lock_;
    };

    /* 인덱스 i 잠금 획득 */
    [[nodiscard]]
    Guard acquire(std::size_t i) {
        assert(i < mtx_.size());
        return Guard{ mtx_[i] };   // NRVO: 실질적 복사 비용 0
    }

    /* 수동 lock / unlock 이 필요할 때 */
    void lock(std::size_t i)   { mtx_[i].lock();   }
    void unlock(std::size_t i) { mtx_[i].unlock(); }

    /* 원 mutex 참조 반환 */
    std::mutex& raw(std::size_t i) { return mtx_[i]; }

private:
    std::vector<std::mutex> mtx_;
};

