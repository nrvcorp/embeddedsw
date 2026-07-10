#pragma once

#include "config.hpp" // DVS_FRAME_W, DVS_FRAME_H, CV_WORKER_CORES_LIST, FRAME_HEADER_BYTES
#include <array>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include <zlib.h>

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error "No filesystem support"
#endif

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

/// @class DatToPngConverter
/// @brief 멀티코어로 .dat 파일 압축 해제 → DVS LUT 매핑 → PNG 저장
class DatToPngConverter
{
  public:
    /// @brief 기본 생성자: config.hpp의 CV_WORKER_CORES_LIST 로 스레드풀 코어 설정
    DatToPngConverter()
        : cores_{CV_WORKER_CORES_LIST} {}

    /**
     * @brief 지정 폴더 내 .dat 파일을 병렬로 변환
     * @param input_folder       .dat 파일들이 들어있는 디렉터리 경로
     * @param output_folder_arg  PNG 저장 디렉터리 경로 (빈 문자열이면 자동 생성)
     */
    void convertFolder(const std::string &input_folder,
                       const std::string &output_folder_arg) const
    {
        fs::path in_dir(input_folder);
        if (!fs::is_directory(in_dir))
        {
            std::cerr << "[Error] 입력 폴더가 유효하지 않습니다: " << input_folder << "\n";
            return;
        }

        // output_folder_arg 가 비어 있으면 "png"+입력폴더명 으로 자동 설정
        fs::path out_dir = output_folder_arg.empty()
                               ? in_dir.parent_path() / ("png" + in_dir.filename().string())
                               : fs::path(output_folder_arg);

        try
        {
            fs::create_directories(out_dir);
        }
        catch (const fs::filesystem_error &e)
        {
            std::cerr << "[Error] 출력 디렉터리 생성 실패: " << out_dir
                      << " (" << e.what() << ")\n";
            return;
        }

        // .dat 파일 목록 수집
        std::vector<fs::path> files;
        for (auto &e : fs::directory_iterator(in_dir))
        {
            if (e.is_regular_file() && e.path().extension() == ".dat")
                files.push_back(e.path());
        }

        size_t N = files.size();
        size_t T = cores_.size();
        if (N == 0 || T == 0)
        {
            std::cout << "[Info] 변환할 .dat 파일이 없거나 코어 설정이 잘못되었습니다.\n";
            return;
        }

        // DVS 2-bit → 8-bit LUT
        static constexpr std::array<uint8_t, 4> LUT = {128, 255, 0, 128};
        const int W = DVS_FRAME_W, H = DVS_FRAME_H;
        const size_t bits_per_row = size_t(W) * 2;

        // 스레드 워커 람다
        auto worker = [&](size_t tid)
        {
#ifdef __linux__
            // CPU affinity & FIFO 우선순위
            cpu_set_t cpus;
            CPU_ZERO(&cpus);
            CPU_SET(cores_[tid], &cpus);
            pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);
            sched_param sch{};
            sch.sched_priority = sched_get_priority_max(SCHED_FIFO);
            pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
#endif
            for (size_t i = tid; i < N; i += T)
            {
                const auto &dat = files[i];
                std::ifstream ifs(dat, std::ios::binary);
                if (!ifs)
                    continue;

                uint64_t orig_sz = 0, comp_sz = 0;
                ifs.read(reinterpret_cast<char *>(&orig_sz), sizeof(orig_sz));
                ifs.read(reinterpret_cast<char *>(&comp_sz), sizeof(comp_sz));
                std::vector<uint8_t> cbuf(comp_sz);
                ifs.read(reinterpret_cast<char *>(cbuf.data()), comp_sz);
                ifs.close();

                std::vector<uint8_t> raw(orig_sz);
                uLongf dst = orig_sz;
                if (uncompress(raw.data(), &dst, cbuf.data(), comp_sz) != Z_OK || dst != orig_sz)
                    continue;

                // --- [헤더에서 timestamp와 frame_num 추출] ---
#if (FRAME_HEADER_BYTES == 8)
                uint32_t timestamp = (raw[0]) | (raw[1] << 8) | (raw[2] << 16) | (raw[3] << 24);
                uint32_t frame_num = (raw[4]) | (raw[5] << 8) | (raw[6] << 16) | (raw[7] << 24);
#elif (FRAME_HEADER_BYTES == 16)
                uint64_t ext_header = // 암시적 캐스팅은 32비트 타입이라 그 이상 크기의 시프트는 명시적 캐스팅 해야됨
                    (static_cast<std::uint64_t>(raw[0])) |
                    (static_cast<std::uint64_t>(raw[1]) << 8) |
                    (static_cast<std::uint64_t>(raw[2]) << 16) |
                    (static_cast<std::uint64_t>(raw[3]) << 24) |
                    (static_cast<std::uint64_t>(raw[4]) << 32) |
                    (static_cast<std::uint64_t>(raw[5]) << 40) |
                    (static_cast<std::uint64_t>(raw[6]) << 48) |
                    (static_cast<std::uint64_t>(raw[7]) << 56);
                uint64_t sensor_cfg_index = ext_header & ((1ULL << 63) - 1);
                uint32_t timestamp = (raw[8]) | (raw[9] << 8) | (raw[10] << 16) | (raw[11] << 24);
                uint32_t frame_num = (raw[12]) | (raw[13] << 8) | (raw[14] << 16) | (raw[15] << 24);
#endif
                // --- [헤더 제거 후 실제 이미지 데이터만 추출] ---
                const uint8_t *img_data = raw.data() + FRAME_HEADER_BYTES;

                cv::Mat gray(H, W, CV_8UC1);
                for (int y = 0; y < H; ++y)
                {
                    uint8_t *out = gray.ptr<uint8_t>(y);
                    size_t off = size_t(y) * bits_per_row;
                    for (int x = 0; x < W; ++x)
                    {
                        size_t bidx = (off + x * 2) >> 3;
                        int shift = (off + x * 2) & 7;
                        out[x] = LUT[(img_data[bidx] >> shift) & 0x03];
                    }
                }

                std::ostringstream fname;
                fname << dat.stem().string()
                      << "_f" << frame_num
                      << "_t" << timestamp
                      << ".png";
#if (FRAME_HEADER_BYTES == 16)
                // sensor_cfg_index 별 하위 폴더: out_dir / ("c" + index)
                fs::path cfg_dir = out_dir / ("c" + std::to_string(sensor_cfg_index));
                try
                {
                    fs::create_directories(cfg_dir);
                }
                catch (const fs::filesystem_error &e)
                {
                    std::cerr << "[Error] 하위 디렉터리 생성 실패: " << cfg_dir
                              << " (" << e.what() << ")\n";
                    continue;
                }
                fs::path png = cfg_dir / fname.str();
#else
                // 8바이트 헤더일 때는 sensor_cfg_index가 없으므로 그냥 out_dir에 저장
                fs::path png = out_dir / fname.str();
#endif
                // 출력 파일명 생성: "원본이름_c센서설정인덱스_f프레임번호_t타임스탬프.png"
                /*std::ostringstream fname;
                fname << dat.stem().string() << "_c" << sensor_cfg_index << "_f" << frame_num << "_t" << timestamp << ".png";
                fs::path png = out_dir / fname.str();*/

                if (cv::imwrite(png.string(), gray))
                {
                    std::cout << "Created: " << png.filename().string() << "\n";
                }

                /*std::vector<uint8_t> raw(orig_sz);
                uLongf dst = orig_sz;
                if (uncompress(raw.data(), &dst, cbuf.data(), comp_sz) != Z_OK || dst != orig_sz)
                    continue;

                cv::Mat gray(H, W, CV_8UC1);
                for (int y = 0; y < H; ++y)
                {
                    uint8_t *out = gray.ptr<uint8_t>(y);
                    size_t off = size_t(y) * bits_per_row;
                    for (int x = 0; x < W; ++x)
                    {
                        size_t bidx = (off + x * 2) >> 3;
                        int shift = (off + x*2) & 7;
                        out[x] = LUT[(raw[bidx] >> shift) & 0x03];
                    }
                }

                fs::path png = out_dir / (dat.stem().string() + ".png");
                if (cv::imwrite(png.string(), gray))
                {
                    std::cout << "Created: " << png.filename().string() << "\n";
                }*/
            }
        };

        // 스레드 실행
        std::vector<std::thread> threads;
        threads.reserve(T);
        for (size_t t = 0; t < T; ++t)
            threads.emplace_back(worker, t);
        for (auto &th : threads)
            th.join();

#ifdef __linux__
        std::cout << "[Info] 권한 변경중(0777)...: " << "\n";
        // 모든 디렉토리+파일의 권한을 0777로 설정
        try
        {
            // out_dir
            ::chmod(out_dir.string().c_str(), 0777);

            // 하위 모든 디렉토리+파일에 대해 0777
            for (const auto &entry : fs::recursive_directory_iterator(out_dir))
            {
                ::chmod(entry.path().string().c_str(), 0777);
            }
        }
        catch (const fs::filesystem_error &e)
        {
            std::cerr << "[Warning] 권한 변경 중 오류 발생: " << e.what() << "\n";
        }
#endif

        std::cout << "[Info] 모든 파일 변환 완료: " << out_dir << "\n";
    }

  private:
    std::vector<int> cores_;
};
