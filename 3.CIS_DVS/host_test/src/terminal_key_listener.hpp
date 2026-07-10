/* terminal_key_listener.hpp */
#pragma once
#include "shutdown_flag.hpp"
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <thread>
#include <functional>
#include <chrono>

class TerminalKeyListener {
public:
    /**  quit      : 모든 스레드가 공유하는 ShutdownFlag
     *   on_A_key  : 'a' 또는 'A' 눌렀을 때 호출할 함수(없으면 기본 = do nothing)
     *   rt_prio   : SCHED_FIFO 우선순위 (1-99), 0 이면 기본 스케줄러 사용    */
    TerminalKeyListener(ShutdownFlag quit,
                        std::function<void()> on_A_key = {},
                        int rt_prio = 80)
        : quit_(std::move(quit)), onA_(std::move(on_A_key))
    {
        th_ = std::thread(&TerminalKeyListener::loop, this, rt_prio);
    }
    ~TerminalKeyListener()           /* RAII 정리 */
    {
        quit_->store(true);
        if (th_.joinable()) th_.join();
    }

private:
    /* ────────── 터미널을 RAW 모드로 (RAII) ────────── */
    struct RawTerm {
        termios old{};
        RawTerm()  {
            tcgetattr(STDIN_FILENO, &old);
            termios raw = old;
            raw.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
        ~RawTerm() { tcsetattr(STDIN_FILENO, TCSANOW, &old); }
    };

    /* ────────── 실시간 우선순위 설정 ────────── */
    static void setRT(int prio)
    {
        if (prio <= 0) return;           // 기본 스케줄러 사용
        sched_param p{ .sched_priority = prio };
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &p) != 0)
            perror("pthread_setschedparam");
    }

    /* ────────── 입력 감시 루프 ────────── */
    void loop(int prio)
    {
        setRT(prio);
        RawTerm raw;

        pollfd pfd{ STDIN_FILENO, POLLIN, 0 };

        while (!quit_->load(std::memory_order_relaxed)) {
            int r = ::poll(&pfd, 1, 100);                 // 100 ms timeout
            if (r > 0 && (pfd.revents & POLLIN)) {
                char ch;
                if (::read(STDIN_FILENO, &ch, 1) == 1) {
                    if (ch == 27) {                       // ESC
                        quit_->store(true, std::memory_order_release);
                    }
                    else if (ch == 'a' || ch == 'A') {    // ‘a’
                        if (onA_) onA_();
                    }
                }
            }
        }
    }

    ShutdownFlag            quit_;
    std::function<void()>   onA_;
    std::thread             th_;
};

