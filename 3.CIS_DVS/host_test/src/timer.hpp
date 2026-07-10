// timer.hpp — lightweight timing helpers (rev‑2)
#pragma once

#include "DebugConfig.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <vector>
// ────────────────────────────────────────────────────────────────
//  Compile‑time switches (override with -D…)
//   LOG_ALL_INTERVALS   1 → 모든 샘플을 출력 (기본 0)
//   TIMING_REPORT_INTERVAL_SEC    N → N 초마다 통계 출력(기본 1)
// ────────────────────────────────────────────────────────────────
#ifndef LOG_ALL_INTERVALS
#define LOG_ALL_INTERVALS 0
#endif
#ifndef TIMING_REPORT_INTERVAL_SEC
#define TIMING_REPORT_INTERVAL_SEC 1
#endif

namespace timing
{
using clock = std::chrono::steady_clock;
using ns = std::chrono::nanoseconds;
using d_ms = std::chrono::duration<double, std::milli>;

//-----------------------------------------------------------------
// StatCounter — min / max / avg aggregator
//-----------------------------------------------------------------
struct StatCounter
{
    long long min = std::numeric_limits<long long>::max();
    long long max = 0;
    long long sum = 0;
    long long cnt = 0;
#if LOG_ALL_INTERVALS
    std::vector<double> samples;
#endif

    void add(long long v_ns)
    {
        if (v_ns < min)
            min = v_ns;
        if (v_ns > max)
            max = v_ns;
        sum += v_ns;
        ++cnt;
#if LOG_ALL_INTERVALS
        samples.push_back(v_ns / 1e6);
#endif
    }
    void reset()
    {
        min = std::numeric_limits<long long>::max();
        max = sum = cnt = 0;
#if LOG_ALL_INTERVALS
        samples.clear();
#endif
    }
};

//-----------------------------------------------------------------
// Metrics — per‑tag global aggregator & reporter
//-----------------------------------------------------------------
class Metrics
{
  public:
    static Metrics &instance()
    {
        static Metrics s;
        return s;
    }

    void addGap(const char *tag, long long ns)
    {
        get(tag).gap.add(ns);
    }
    void addBody(const char *tag, long long ns)
    {
        get(tag).body.add(ns);
    }

    void maybeReport(const char *tag)
    {
        auto &data = get(tag);
        auto now = clock::now();
        if (now - data.lastReport >= std::chrono::seconds(TIMING_REPORT_INTERVAL_SEC) && data.body.cnt)
        {
            data.lastReport = now;
            print(tag, data);
            data.gap.reset();
            data.body.reset();
        }
    }

  private:
    struct Pair
    {
        StatCounter gap;
        StatCounter body;
        clock::time_point lastReport{clock::now()};
    };
    std::unordered_map<std::string, Pair> map_;

    Pair &get(const char *tag)
    {
        return map_[tag];
    }

    static void print(const char *tag, const Pair &p)
    {
        auto toMs = [](long long ns)
        { return ns / 1e6; };
        std::cout << tag << " (" << p.body.cnt << " samples)\n";
        std::cout << "  GAP   min " << toMs(p.gap.min) << " ms  max " << toMs(p.gap.max)
                  << " ms  avg " << (p.gap.cnt ? toMs(p.gap.sum) / p.gap.cnt : 0) << " ms\n";
        std::cout << "  BODY  min " << toMs(p.body.min) << " ms  max " << toMs(p.body.max)
                  << " ms  avg " << (p.body.cnt ? toMs(p.body.sum) / p.body.cnt : 0) << " ms\n";
#if LOG_ALL_INTERVALS
        std::cout << "  Samples: ";
        for (double v : p.body.samples)
            std::cout << v << ' ';
        std::cout << "\n";
#endif
        std::cout << "------------------------------\n";
    }
};

//-----------------------------------------------------------------
// CallGap — measures wall‑time gap between consecutive calls
//-----------------------------------------------------------------
/*class CallGap
{
  public:
    explicit CallGap(const char *tag) noexcept : tag_(tag)
    {
        auto now = clock::now();
        static thread_local clock::time_point prev = now;
        static thread_local bool first = true;
        if (!first)
        {
            auto diff = std::chrono::duration_cast<ns>(now - prev).count();
            Metrics::instance().addGap(tag_, diff);
        }
        first = false;
        prev = now;
    }

  private:
    const char *tag_;
};*/
class CallGap
{
  public:
    explicit CallGap(const char *tag) noexcept
        : tag_str_(tag), tag_(tag_str_.c_str())
    {
        record();
    }

    CallGap(const char *tag, int index) noexcept
        : tag_str_(tag + std::string("_") + std::to_string(index)), tag_(tag_str_.c_str())
    {
        record();
    }

  private:
    std::string tag_str_;
    const char *tag_;

    void record() noexcept
    {
        auto now = clock::now();
        static thread_local clock::time_point prev = now;
        static thread_local bool first = true;
        if (!first)
        {
            auto diff = std::chrono::duration_cast<ns>(now - prev).count();
            Metrics::instance().addGap(tag_, diff);
        }
        first = false;
        prev = now;
    }
};
//-----------------------------------------------------------------
// ScopeTimer — RAII block execution timer
//-----------------------------------------------------------------
/*class ScopeTimer
{
  public:
    explicit ScopeTimer(const char *tag) noexcept : tag_(tag), start_(clock::now()) {}
    ~ScopeTimer() noexcept
    {
        auto diff = std::chrono::duration_cast<ns>(clock::now() - start_).count();
        auto &m = Metrics::instance();
        m.addBody(tag_, diff);
        m.maybeReport(tag_);
    }

  private:
    const char *tag_;
    clock::time_point start_;
};*/
class ScopeTimer
{
  public:
    explicit ScopeTimer(const char *tag) noexcept
        : tag_str_(tag), tag_(tag_str_.c_str()), start_(clock::now()) {}

    ScopeTimer(const char *tag, int index) noexcept
        : tag_str_(tag + std::string("_") + std::to_string(index)),
          tag_(tag_str_.c_str()),
          start_(clock::now()) {}

    ~ScopeTimer() noexcept
    {
        auto diff = std::chrono::duration_cast<ns>(clock::now() - start_).count();
        auto &m = Metrics::instance();
        m.addBody(tag_, diff);
        m.maybeReport(tag_);
    }

  private:
    std::string tag_str_;
    const char *tag_;
    clock::time_point start_;
};
} // namespace timing
