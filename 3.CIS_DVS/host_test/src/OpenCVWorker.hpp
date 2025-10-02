#pragma once

#include "config.hpp"
#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
#include <thread>

//------------------------------------------------------------------------------
// OpenCVTaskType: 작업 종류
//------------------------------------------------------------------------------
enum class OpenCVTaskType
{
    InitToGrayFrame,   // 프레임을 중간 밝기의 회색으로 초기화
    DvsToGrayscaleMap, // DVS 포맷 → 8비트 LUT 변환
    SaveAsPNG,         // PNG 저장 (분할 저장 포함)
    MapAndSavePNG      // Part별 DVS→Gray 변환 후 바로 PNG 저장
};

//------------------------------------------------------------------------------
// OpenCVTask: 워커에게 전달할 작업 정보
//------------------------------------------------------------------------------
struct OpenCVTask
{
    OpenCVTaskType type;
    std::shared_ptr<cv::Mat> input; // packed 또는 gray 이미지 데이터
    cv::Mat *output = nullptr;      // 출력 Mat (InitToGrayFrame, DvsToGrayscaleMap)
    std::string filename;           // PNG 저장 시 사용
    int part_id = 0;                // 분할 처리 시 파트 번호
    int part_count = 1;             // 분할 처리 시 전체 파트 수
};

//------------------------------------------------------------------------------
// OpenCVWorker: 단일 코어에 고정되어 작업 처리
//------------------------------------------------------------------------------
class OpenCVWorker
{
  public:
    // DVS 2bit → 8bit 그레이스케일 LUT
    static constexpr std::array<uint8_t, 4> DVS_TO_GRAY_LUT = {128, 255, 0, 128};

    explicit OpenCVWorker(int core_id = -1)
        : stop_flag(false)
    {
        worker_thread = std::thread([this, core_id]()
                                    {
            if (core_id >= 0)
                set_affinity(core_id);
            thread_loop(); });
    }

    ~OpenCVWorker()
    {
        {
            std::lock_guard<std::mutex> lk(mtx);
            stop_flag = true;
        }
        cv_cond.notify_all();
        if (worker_thread.joinable())
            worker_thread.join();
    }

    // 작업 큐에 추가
    void enqueue(const OpenCVTask &task)
    {
        {
            std::lock_guard<std::mutex> lk(mtx);
            tasks.push(task);
        }
        cv_cond.notify_one();
    }

  private:
    std::thread worker_thread;
    std::mutex mtx;
    std::condition_variable cv_cond;
    std::queue<OpenCVTask> tasks;
    std::atomic<bool> stop_flag;

    void thread_loop()
    {
        cv::setNumThreads(std::thread::hardware_concurrency());
        while (!stop_flag)
        {
            OpenCVTask task;
            {
                std::unique_lock<std::mutex> lk(mtx);
                cv_cond.wait(lk, [this]
                             { return stop_flag || !tasks.empty(); });
                if (stop_flag && tasks.empty())
                    break;
                task = tasks.front();
                tasks.pop();
            }
            run(task);
        }
    }

    void run(const OpenCVTask &task)
    {
        switch (task.type)
        {
        case OpenCVTaskType::InitToGrayFrame:
            if (task.output)
            {
                task.output->create(DVS_FRAME_H, DVS_FRAME_W, CV_8UC1);
                task.output->setTo(cv::Scalar(127));
            }
            break;

        case OpenCVTaskType::DvsToGrayscaleMap:
            if (task.input && task.output && task.part_count > 0)
            {
                const cv::Mat &in = *task.input;
                cv::Mat &out = *task.output;
                out.create(DVS_FRAME_H, DVS_FRAME_W, CV_8UC1);

                const uint8_t *data = in.data;
                constexpr size_t bits_per_row = size_t(DVS_FRAME_W) * 2;

                int total_rows = DVS_FRAME_H;
                int step = total_rows / task.part_count;
                int y0 = task.part_id * step;
                int y1 = (task.part_id == task.part_count - 1)
                             ? total_rows
                             : y0 + step;
                if (y0 >= total_rows)
                    break;

                for (int y = y0; y < y1; ++y)
                {
                    uint8_t *out_row = out.ptr<uint8_t>(y);
                    size_t row_off = size_t(y) * bits_per_row;
                    for (int x = 0; x < DVS_FRAME_W; ++x)
                    {
                        size_t bit_idx = row_off + size_t(x) * 2;
                        size_t byte_idx = bit_idx >> 3;
                        int shift = 6 - int(bit_idx & 7);
                        uint8_t two_bits = (data[byte_idx] >> shift) & 0x03;
                        out_row[x] = DVS_TO_GRAY_LUT[two_bits];
                    }
                }
            }
            break;

        case OpenCVTaskType::MapAndSavePNG:
            if (task.input && !task.filename.empty() && task.part_count > 0)
            {
                const uint8_t *data = task.input->data;
                constexpr size_t bits_per_row = size_t(DVS_FRAME_W) * 2;

                int total_rows = DVS_FRAME_H;
                int step = total_rows / task.part_count;
                int y0 = task.part_id * step;
                int y1 = (task.part_id == task.part_count - 1)
                             ? total_rows
                             : y0 + step;
                if (y0 >= total_rows)
                    break;

                cv::Mat slice(y1 - y0, DVS_FRAME_W, CV_8UC1);
                for (int y = y0; y < y1; ++y)
                {
                    uint8_t *out_row = slice.ptr<uint8_t>(y - y0);
                    size_t row_off = size_t(y) * bits_per_row;
                    for (int x = 0; x < DVS_FRAME_W; ++x)
                    {
                        size_t bit_idx = row_off + size_t(x) * 2;
                        size_t byte_idx = bit_idx >> 3;
                        int shift = 6 - int(bit_idx & 7);
                        uint8_t two_bits = (data[byte_idx] >> shift) & 0x03;
                        out_row[x] = DVS_TO_GRAY_LUT[two_bits];
                    }
                }

                std::string base = task.filename;
                std::string ext = ".png";
                if (auto dot = base.rfind('.'); dot != std::string::npos)
                {
                    ext = base.substr(dot);
                    base = base.substr(0, dot);
                }
                std::string out_name = base + "_part" + std::to_string(task.part_id) + ext;
                cv::imwrite(out_name, slice, {cv::IMWRITE_PNG_COMPRESSION, CV_IMWRITE_PNG_COMPRESSION_LEVEL});
            }
            break;

        default:
            break;
        }
    }

    void set_affinity(int core_id)
    {
#ifdef __linux__
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_id, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
#endif
    }
};
