#include <memory>
#include <pthread.h>
#include <sched.h>

#include <X11/Xlib.h> //  XInitThreads(); imshow()를 여러 쓰레드에서 사용
#undef Status

#include <cassert>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <getopt.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <regex>
#include <sstream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/time.h>
#include <sys/types.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <linux/capability.h>
#include <linux/sched.h> // for SCHED_DEADLINE
#include <linux/types.h>
#include <sys/syscall.h> // for SYS_sched_setattr

#include <chrono>
#include <csignal>

#include "CIS.hpp" // Include CIS class
#include "DVS.hpp" // Include DVS class
#include "DebugConfig.hpp"
#include "config.hpp"

#include "shutdown_flag.hpp"

using namespace cv;
using namespace std;

using namespace std::chrono;

steady_clock::time_point prog_start;
steady_clock::time_point last_heavy_start;

void set_key_input_priority(int prio = 90)
{
    struct sched_param param;
    param.sched_priority = prio;

    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0)
    {
        perror("pthread_setschedparam failed");
    }
}

// affinity 설정
void setThreadAffinity(std::thread &t, int cpu_id)
{
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(cpu_id, &cpus);
    auto handle = t.native_handle(); // pthread_t
    if (pthread_setaffinity_np(handle, sizeof(cpus), &cpus) != 0)
    {
        perror("pthread_setaffinity_np");
    }
}

void print_summary_and_exit(int signum)
{
    auto now = steady_clock::now();
    auto total_us = duration_cast<microseconds>(now - prog_start).count();

    std::cout << "\n[INFO] Program ran for " << total_us / 1000000.0 << " seconds.\n";
    std::cout << "[INFO] Gracefully exiting.\n";
    exit(0);
}

static std::shared_ptr<std::atomic<bool>> *g_quit_ptr = nullptr;
extern "C" void sigint_handler(int) noexcept
{
    if (g_quit_ptr && *g_quit_ptr)
    {
        (*g_quit_ptr)->store(true, std::memory_order_release);
    }
}

void lockMemory()
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
        std::cerr << "mlockall failed: " << std::strerror(errno) << "\n";
        // 필요하다면 exit(1)…
    }
}

enum Mode
{
    CIS_DISPLAY = 1,
    DVS_DISPLAY,
    DVS_CHECK_FRAME_DROP,
    CIS_DVS_DISPLAY,
    DVS_STORE,
    FIL_STORE,
    DVS_FIL_STORE,
    DVS_ROI,
    CIS_DVS_BBOX,
    CIS_DVS_OVERLAY,
    DVS_FPS_CHECK,
    CIS_DVS_DISPLAY_FPS,
    DVS_FIL_DISPLAY_FPS,
    DVS_DISPLAY_FPS,
    FIL_DISPLAY_FPS,
    DVS_DISPLAY_AND_STORE,
    CIS_ONLY_ROI,
    DVS_BIN_TO_VID,
    DVS_BIN_TO_PNG,
    DVS_DAT_TO_PNG,
    CIS_DVS_STORE_PNG
};

// Function declarations
void printBanner();
void handleMode(Mode mode);
Mode parseArguments(int argc, char *argv[]);

void configureXdmaIrqAffinity(const std::string &dev_path, int cpu);

int main(int argc, char *argv[])
{
    // std::signal(SIGINT, on_sigint);

    lockMemory();

    XInitThreads();

    // Print application banner
    printBanner();

    // Parse command-line arguments to determine the mode
    Mode mode = parseArguments(argc, argv);

    // Handle the selected mode
    handleMode(mode);

    return 0;
}

void printBanner()
{
    printf(ANSI_COLOR_BLUE);
    printf("************************************************\n");
    printf("*******                              ***********\n");
    printf("*******  CIS, DVS Video Application  ***********\n");
    printf("*******                              ***********\n");
    printf("************************************************\n");
    printf(ANSI_COLOR_RESET);
}

void handleMode(Mode mode)
{

    MutexManager mutexManager; // Initialize mutex manager

    auto quit = std::make_shared<std::atomic<bool>>(false);
    g_quit_ptr = &quit;
    std::signal(SIGINT, sigint_handler);
    // std::signal(SIGTERM, sigint_handler);
    // std::signal(SIGHUP, sigint_handler);
    // std::signal(SIGQUIT, sigint_handler);

    // Create pointers for CIS and DVS objects
    CIS *cis = nullptr;
    DVS *dvs = nullptr;
    DVS *fil = nullptr;

    Bbox bbox;
    bool terminate;
    MutexManager bbox_mutex;
    // Declare the vector outside the switch block
    std::vector<std::thread>
        threads;
    cv::Mat frame_shared;
    cv::Rect dvs_rect, cis_rect;
    int dvs_width, dvs_height;
    char bin_path[200];
    char folder_name[100];
    char bin_file_name[100];
    char vid_file_name[200];
    char *dot = nullptr;

    auto start = std::chrono::steady_clock::now();
    switch (mode)
    {
    case CIS_DISPLAY:
        printf("CIS only display mode\n");
        cis = new CIS(
            CIS_FRAME_H, CIS_FRAME_W,
            CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR,
            CIS_BUFFER_NUM,
            C2H_DEVICE_CIS, H2C_DEVICE_CIS,
            mutexManager);
        cis->display_stream(); // Call CIS display stream
        delete cis;            // Cleanup
        cis = NULL;
        break;

    case DVS_DISPLAY:
        printf("DVS only display mode\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / DISPLAY_FPS), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);

        dvs->display_stream(true); // Call DVS display stream
        delete dvs;                // Cleanup
        dvs = NULL;
        break;

    case DVS_CHECK_FRAME_DROP:
        printf("DVS only check mode\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);
        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->check_frame_drop(); });
            setThreadPriorityMax(threads.back());
        }
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        delete dvs; // Cleanup
        dvs = NULL;
        break;

    case CIS_DVS_DISPLAY:
        printf("CIS and DVS display mode\n");
        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / (DISPLAY_FPS)), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);

        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis]()
                                 { cis->display_stream(); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->display_stream(true); });
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete cis; // Cleanup
        delete dvs; // Cleanup
        cis = NULL;
        dvs = NULL;
        break;

    case DVS_STORE:
    {
        printf("DVS compression and store mode\n");

        dvs = new DVS(ID_DVS, DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_COMMIT_INDEX_ADDR,
                      DVS_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, HOST_FRAME_BATCH_BUFFER_NUM, 1, quit);
        configureXdmaIrqAffinity(C2H_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        configureXdmaIrqAffinity(H2C_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        if (dvs)
        {
            threads.emplace_back([dvs]() { /*set_realtime_priority(99); */

                                dvs->multiple_buf_pcie_reader(); });
        }
        setThreadPriorityMax(threads.back()); // Set priority after thread creation
        setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);

        if (dvs)
        {
            threads.emplace_back([dvs]() { /*set_realtime_priority(99); */
                                 dvs->multiple_buf_dat_writer(); });
        }
        setThreadPriorityHigh(threads.back()); // Set priority after thread creation
        setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete dvs;
        dvs = NULL;
        break;
    }

    case FIL_STORE:
    {
        printf("Filtered DVS compression and store mode\n");

        fil = new DVS(ID_FIL, DVS_FRAME_H, DVS_FRAME_W, true, 1, FIL_FRAME_RDY_BASEADDR, FIL_FRAME_BASEADDR, FIL_COMMIT_INDEX_ADDR,
                      FIL_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, HOST_FRAME_BATCH_BUFFER_NUM, 1, quit);

        configureXdmaIrqAffinity(C2H_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        configureXdmaIrqAffinity(H2C_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        if (fil)
        {
            threads.emplace_back([fil]() { /*set_realtime_priority(99); */

                                fil->multiple_buf_pcie_reader(); });
        }
        setThreadPriorityMax(threads.back()); // Set priority after thread creation
        setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);
        if (fil)
        {
            threads.emplace_back([fil]() { /*set_realtime_priority(99); */
                                 fil->multiple_buf_dat_writer(); });
        }
        setThreadPriorityHigh(threads.back()); // Set priority after thread creation
        setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete fil;
        fil = NULL;
        break;
    }

    case DVS_FIL_STORE:
    {
        printf("DVS & Filtered DVS compression and store mode\n");

        dvs = new DVS(ID_DVS, DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_COMMIT_INDEX_ADDR,
                      DVS_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, HOST_FRAME_BATCH_BUFFER_NUM, 1, quit);

        fil = new DVS(ID_FIL, DVS_FRAME_H, DVS_FRAME_W, true, 1, FIL_FRAME_RDY_BASEADDR, FIL_FRAME_BASEADDR, FIL_COMMIT_INDEX_ADDR,
                      FIL_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, HOST_FRAME_BATCH_BUFFER_NUM, 1, quit);

        configureXdmaIrqAffinity(C2H_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        configureXdmaIrqAffinity(H2C_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);

        if (dvs)
        {
            threads.emplace_back([dvs]() { /*set_realtime_priority(99); */

                                dvs->multiple_buf_pcie_reader(); });
        }
        setThreadPriorityMax(threads.back()); // Set priority after thread creation
        setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);

        if (dvs)
        {
            threads.emplace_back([dvs]() { /*set_realtime_priority(99); */
                                 dvs->multiple_buf_dat_writer(); });
        }
        setThreadPriorityHigh(threads.back()); // Set priority after thread creation
        setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);

        if (fil)
        {
            threads.emplace_back([fil]() { /*set_realtime_priority(99); */

                                fil->multiple_buf_pcie_reader(); });
        }
        setThreadPriorityMax(threads.back()); // Set priority after thread creation
        setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);
        if (fil)
        {
            threads.emplace_back([fil]() { /*set_realtime_priority(99); */
                                 fil->multiple_buf_dat_writer(); });
        }
        setThreadPriorityHigh(threads.back()); // Set priority after thread creation
        setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER_2);

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete dvs;
        dvs = NULL;
        delete fil;
        fil = NULL;
        break;
    }

    case DVS_ROI:
        printf("DVS ROI mode\n ");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / DISPLAY_FPS), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, &bbox, &bbox_mutex, &terminate);
        dvs->set_DVS_ROI(ROI_EVENT_SCORE, ROW_SCORE_THRESHOLD, ROI_HEIGHT_MIN_THRESHOLD, DVS_ROI_MIN_SIZE, 1.0);
        // run old algorithm
        // dvs->dvs_roi_average_based(1, 1, true, true);
        // run new algorithm
        dvs->dvs_roi_proposed(1, 1, true, true);
        dvs = NULL;
        delete dvs;
        break;

    case CIS_DVS_BBOX:
        printf("CIS DVS BBOX mode\n ");

        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager, &bbox_mutex, &bbox, &terminate);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / DISPLAY_FPS), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, &bbox, &bbox_mutex, &terminate);

        // set relative parameters between the two sensors
        cis->set_DVS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y);
        dvs->set_CIS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y, CIS_FRAME_W, CIS_FRAME_H, ROI_EVENT_SCORE, ROW_SCORE_THRESHOLD, ROI_HEIGHT_MIN_THRESHOLD, CIS_ROI_MIN_SIZE, ROI_INFLATION);
        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis]()
                                 { cis->crop_dvs_roi(); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->dvs_roi_proposed(1, 1, true); });
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        delete cis;
        delete dvs;
        dvs = NULL;
        cis = NULL;
        break;
    case CIS_DVS_OVERLAY:
        printf("CIS DVS overlay mode for tuning\n");
        printf("when red grid shows up, use it to estimate scale between DVS and CIS images\n");
        printf("open up config.hpp\n");
        printf("if DVS image is larger sideways, try to shrink CIS_DVS_SCALE_X\n");
        printf("if DVS image is shorter vertically, increase CIS_DVS_SCALE_Y\n");
        printf("if grid has 10 regions horizontally, width of each region = 0.1\n");
        printf("if DVS image is shunted left from CIS image by 3 regions, increase CIS_DVS_OFFSET_X by 0.3\n");
        printf("if DVS image is shunted down from CIS image by 2 regions, decrease CIS_DVS_OFFSET_Y by 0.2\n");
        frame_shared = cv::Mat::zeros(DVS_FRAME_H, DVS_FRAME_W, CV_8UC1);
        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager, &bbox_mutex, &bbox, &terminate);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / DISPLAY_FPS), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, &bbox, &bbox_mutex, &terminate);

        // set relative parameters between the two sensors
        cis->set_DVS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y);
        dvs->set_CIS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y, CIS_FRAME_W, CIS_FRAME_H, ROI_EVENT_SCORE, ROW_SCORE_THRESHOLD, ROI_HEIGHT_MIN_THRESHOLD, CIS_ROI_MIN_SIZE, ROI_INFLATION);
        // Start threads for CIS and DVS

        cis->set_roi(dvs_rect, cis_rect, dvs_width, dvs_height);
        while (true)
        {
            dvs->send_frame((cv::Mat *)(&frame_shared), true);
            bool retval = cis->overlay_dvs((cv::Mat *)(&frame_shared), dvs_rect, cis_rect, dvs_width, dvs_height, DVS_WEIGHT_ALPHA, CIS_DVS_CALIBRATION_GRID_NUM);
            if (retval)
                break;
        }

        delete cis;
        delete dvs;
        dvs = NULL;
        cis = NULL;
        break;

    case DVS_FPS_CHECK:
        printf("DVS FPS check mode\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);
        dvs->fps_count();
        delete dvs; // Cleanup
        dvs = NULL;
        break;

    case CIS_DVS_DISPLAY_FPS:
    {

        printf("CIS DVS display with fps check mode\n");

        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager);

        dvs = new DVS(ID_DVS, DVS_FRAME_H, DVS_FRAME_W, true, FRAME_ACCUM_COUNT, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_COMMIT_INDEX_ADDR,
                      DVS_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager,
                      HOST_FRAME_BATCH_BUFFER_NUM, (DISPLAY_DOWNSAMPLE_NUM / NUM_FRAMES_PER_TRANSFER), quit);
        configureXdmaIrqAffinity(C2H_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        configureXdmaIrqAffinity(H2C_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis]()
                                 { cis->display_stream(); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->multiple_buf_display_fps_pcie_reader(); });
            setThreadPriorityMax(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);
            threads.emplace_back([dvs]()
                                 { dvs->multiple_buf_display_fps_render(true); });
            setThreadPriorityHigh(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete cis; // Cleanup
        delete dvs; // Cleanup
        cis = NULL;
        dvs = NULL;
        break;
    }

    case DVS_FIL_DISPLAY_FPS:
    {
        printf("DVS & Filtered DVS display with fps check mode\n");

        dvs = new DVS(ID_DVS, DVS_FRAME_H, DVS_FRAME_W, true, FRAME_ACCUM_COUNT, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_COMMIT_INDEX_ADDR,
                      DVS_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager,
                      HOST_FRAME_BATCH_BUFFER_NUM, (DISPLAY_DOWNSAMPLE_NUM / NUM_FRAMES_PER_TRANSFER), quit);
        fil = new DVS(ID_FIL, DVS_FRAME_H, DVS_FRAME_W, true, FRAME_ACCUM_COUNT, FIL_FRAME_RDY_BASEADDR, FIL_FRAME_BASEADDR, FIL_COMMIT_INDEX_ADDR,
                      FIL_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager,
                      HOST_FRAME_BATCH_BUFFER_NUM, (DISPLAY_DOWNSAMPLE_NUM / NUM_FRAMES_PER_TRANSFER), quit);

        configureXdmaIrqAffinity(C2H_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        configureXdmaIrqAffinity(H2C_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->multiple_buf_display_fps_pcie_reader(); });
            setThreadPriorityMax(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);
            threads.emplace_back([dvs]()
                                 { dvs->multiple_buf_display_fps_render(true); });
            setThreadPriorityHigh(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER_2);
        }
        if (fil)
        {
            threads.emplace_back([fil]()
                                 { fil->multiple_buf_display_fps_pcie_reader(); });
            setThreadPriorityMax(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);
            threads.emplace_back([fil]()
                                 { fil->multiple_buf_display_fps_render(true); });
            setThreadPriorityHigh(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);
        }
#if (COMPARE_FIL_DVS)
        // compare fil & dvs
        {
            threads.emplace_back([&]()
                                 {
                static uint32_t last_fil_frame = 0;

            
                while (!quit->load(std::memory_order_acquire)){
                        uint32_t raw_frame_num = dvs->latest_header.frame_num.load(std::memory_order_relaxed);
                        uint32_t raw_timestamp = dvs->latest_header.timestamp.load(std::memory_order_relaxed);

                        uint32_t fil_frame_num = fil->latest_header.frame_num.load(std::memory_order_relaxed);
                        uint32_t fil_timestamp = fil->latest_header.timestamp.load(std::memory_order_relaxed);

                        if (fil_frame_num == last_fil_frame)
                        {
                            std::this_thread::sleep_for(std::chrono::microseconds(100));
                            continue;
                        }

                        last_fil_frame = fil_frame_num;

                        int frame_gap = static_cast<int>(fil_frame_num - raw_frame_num);
                        int time_gap = static_cast<int>(fil_timestamp - raw_timestamp);

                        printf("[FIL frame %u] framenum_diff=%d, timestamp_diff=%d \n",
                               fil_frame_num, frame_gap, time_gap);

                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                } });
            setThreadPriorityHigh(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_COMPARATOR);
        }
#endif
        /*if (dvs)
        {
            threads.emplace_back([dvs]()
                                 {
                                 set_key_input_priority();
                                 dvs->key_input(); });
        }*/

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete fil; // Cleanup
        delete dvs; // Cleanup
        fil = NULL;
        dvs = NULL;
        break;
    }
    case DVS_DISPLAY_FPS:
    {
        printf("DVS display with fps check mode\n");

        dvs = new DVS(ID_DVS, DVS_FRAME_H, DVS_FRAME_W, true, FRAME_ACCUM_COUNT, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_COMMIT_INDEX_ADDR,
                      DVS_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager,
                      HOST_FRAME_BATCH_BUFFER_NUM, (DISPLAY_DOWNSAMPLE_NUM / NUM_FRAMES_PER_TRANSFER), quit);
        configureXdmaIrqAffinity(C2H_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        configureXdmaIrqAffinity(H2C_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->multiple_buf_display_fps_pcie_reader(); });
            setThreadPriorityMax(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);
            threads.emplace_back([dvs]()
                                 { dvs->multiple_buf_display_fps_render(true); });
            setThreadPriorityHigh(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete dvs; // Cleanup
        dvs = NULL;
        break;
    }

    case FIL_DISPLAY_FPS:
    {
        printf("Filtered DVS display with fps check mode\n");

        fil = new DVS(ID_FIL, DVS_FRAME_H, DVS_FRAME_W, true, FRAME_ACCUM_COUNT, FIL_FRAME_RDY_BASEADDR, FIL_FRAME_BASEADDR, FIL_COMMIT_INDEX_ADDR,
                      FIL_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager,
                      HOST_FRAME_BATCH_BUFFER_NUM, (DISPLAY_DOWNSAMPLE_NUM / NUM_FRAMES_PER_TRANSFER), quit);
        configureXdmaIrqAffinity(C2H_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        configureXdmaIrqAffinity(H2C_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        if (fil)
        {
            threads.emplace_back([fil]()
                                 { fil->multiple_buf_display_fps_pcie_reader(); });
            setThreadPriorityMax(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);
            threads.emplace_back([fil]()
                                 { fil->multiple_buf_display_fps_render(true); });
            setThreadPriorityHigh(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete fil; // Cleanup
        fil = NULL;
        break;
    }
    case DVS_DISPLAY_AND_STORE:
        printf("DVS display and store mode\n");

        dvs = new DVS(ID_DVS, DVS_FRAME_H, DVS_FRAME_W, true, FRAME_ACCUM_COUNT, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_COMMIT_INDEX_ADDR,
                      DVS_BUFFER_NUM_BURST, NUM_FRAMES_PER_TRANSFER, DVS_FPS, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager,
                      HOST_FRAME_BATCH_BUFFER_NUM, (DISPLAY_DOWNSAMPLE_NUM / NUM_FRAMES_PER_TRANSFER), quit);
        configureXdmaIrqAffinity(C2H_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        configureXdmaIrqAffinity(H2C_DEVICE_DVS, CPU_CORE_BUF_PRODUCER);
        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->mbuf_DnW_pcie_reader(); });
            setThreadPriorityMax(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_PRODUCER);
            threads.emplace_back([dvs]()
                                 { dvs->mbuf_DnW_display_render(true); });
            setThreadPriorityHigh(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);
            threads.emplace_back([dvs]()
                                 { dvs->mbuf_DnW_dat_writer(); });
            setThreadPriorityHigh(threads.back());
            setThreadAffinity(threads.back(), CPU_CORE_BUF_CONSUMER);
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete dvs; // Cleanup
        dvs = NULL;
        break;
    case CIS_ONLY_ROI:
        printf("CIS only ROI through background subtraction and contour mode\n");
        cis = new CIS(
            CIS_FRAME_H, CIS_FRAME_W,
            CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR,
            CIS_BUFFER_NUM,
            C2H_DEVICE_CIS, H2C_DEVICE_CIS,
            mutexManager);
        cis->background_subtraction(); // Call CIS display stream
        delete cis;                    // Cleanup
        cis = NULL;
        break;
    case DVS_BIN_TO_VID:
        printf("convert the bin file from DVS_STORE mode to a grayscale video\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, true);
        cout << "Path to bin file:\n";
        cin.getline(bin_file_name, 100);
        cout << "Path to video file:\n";
        cin.getline(vid_file_name, 100);
        dvs->bin_to_vid((char *)bin_file_name, (char *)vid_file_name);
        delete dvs;
        dvs = NULL;
        break;

    case DVS_BIN_TO_PNG:
        printf("convert the bin file from DVS_STORE mode to a collection of PNGs\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1 /*(2000 / 60)*/, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, true);

        cout << "bin file:\n";
        cin.getline(bin_file_name, 100);
        snprintf(bin_path, sizeof(bin_path), "./bin_files/%s", bin_file_name);
        strncpy(folder_name, bin_file_name, sizeof(folder_name));
        folder_name[sizeof(folder_name) - 1] = '\0';

        dot = strrchr(folder_name, '.');
        if (dot != nullptr)
            *dot = '\0';
        snprintf(vid_file_name, sizeof(vid_file_name), "./bin_files/%s", folder_name);
        if (access(vid_file_name, F_OK) == -1)
        {
            mkdir(vid_file_name, 0777);
        }
        // cout << "Path to folder that will contain PNG images:\n";
        // cin.getline(vid_file_name, 100);
        printf("%s, %s\n", bin_path, vid_file_name);
        dvs->bin_to_png((char *)bin_path, (char *)vid_file_name);
        delete dvs;
        break;

    case DVS_DAT_TO_PNG:
        printf("convert the dat to png\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1 /*(2000 / 60)*/, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, true);

        char png_dir[256];

        std::cout << "dat folder name (inside ./bin_files):\n> ";
        std::cin.getline(bin_file_name, sizeof(bin_file_name));

        // 입력 폴더: "./bin_files/<bin_file_name>"
        std::snprintf(bin_path, sizeof(bin_path),
                      "./bin_files/%s", bin_file_name);

        // 출력 폴더: 뒤에 "_png" 추가  "./bin_files/<bin_file_name>_png"
        std::snprintf(png_dir, sizeof(png_dir),
                      "./bin_files/%s_png", bin_file_name);

        std::cout << "Input  folder: " << bin_path << "\n";
        std::cout << "Output folder: " << png_dir << "\n";

        dvs->dat_to_png((char *)bin_path, (char *)png_dir);
        delete dvs;
        break;

    case CIS_DVS_STORE_PNG:
        printf("Save CIS and DVS images in PNG format, synchronized at 60FPS\n");
        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager, &bbox_mutex, &bbox, &terminate);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1 /*(DVS_FPS / DISPLAY_FPS)*/, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, &bbox, &bbox_mutex, &terminate);
        // set relative parameters between the two sensors
        cis->set_DVS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y);
        dvs->set_CIS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y, CIS_FRAME_W, CIS_FRAME_H, ROI_EVENT_SCORE, ROW_SCORE_THRESHOLD, ROI_HEIGHT_MIN_THRESHOLD, CIS_ROI_MIN_SIZE, ROI_INFLATION);
        cout << "Path to folder that will contain CIS images:\n";
        cin.getline(bin_file_name, 100);
        cout << "Path to folder that will contain DVS images:\n";
        cin.getline(vid_file_name, 100);
        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis, bin_file_name]()
                                 { cis->save_png_stream((char *)bin_file_name); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs, vid_file_name]()
                                 { dvs->save_png_stream((char *)vid_file_name, true); });
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        delete cis;
        delete dvs;
        dvs = NULL;
        cis = NULL;
        break;
    default:
        fprintf(stderr, "Error: Unknown mode\n");
        exit(EXIT_FAILURE);
    }
}

void configureXdmaIrqAffinity(const std::string &dev_path, int cpu)
{
    namespace fs = std::filesystem;

    // 1) /dev entry → major:minor
    struct stat st;
    if (stat(dev_path.c_str(), &st) < 0)
    {
        std::cerr << "WARN: cannot stat " << dev_path << "\n";
        return;
    }
    if (!S_ISCHR(st.st_mode))
    {
        std::cerr << "WARN: not a char device: " << dev_path << "\n";
        return;
    }
    int maj = major(st.st_rdev);
    int min = minor(st.st_rdev);

    // 2) major:minor → sysfs device symlink → PCI 주소
    fs::path dev_sys = "/sys/dev/char/" +
                       std::to_string(maj) + ":" + std::to_string(min) + "/device";
    if (!fs::is_symlink(dev_sys))
    {
        std::cerr << "WARN: no sysfs device link for " << dev_path << "\n";
        return;
    }
    std::string pci_addr = fs::read_symlink(dev_sys).filename().string();
    std::cout << "INFO: " << dev_path << " → PCI device " << pci_addr << "\n";

    // 3) PCI → msi_irqs 디렉토리
    fs::path msi_dir = "/sys/bus/pci/devices/" + pci_addr + "/msi_irqs";
    if (!fs::exists(msi_dir) || !fs::is_directory(msi_dir))
    {
        std::cerr << "ERROR: msi_irqs not found for PCI " << pci_addr << "\n";
        return;
    }

    // 4) affinity mask 계산 (cpu==2 → mask=0x4)
    uint64_t mask = (1ULL << cpu);
    std::ostringstream hex;
    hex << std::hex << mask;
    std::string mask_str = hex.str();

    // 5) 각 벡터(0,1,2…)의 irq 파일 읽고 /proc/irq/<n>/smp_affinity 에 쓰기
    for (auto &ent : fs::directory_iterator(msi_dir))
    {
        fs::path irq_path = ent.path() / "irq";
        if (!fs::exists(irq_path))
            continue;

        int irq = 0;
        std::ifstream(irq_path) >> irq;
        std::string affinity_file =
            "/proc/irq/" + std::to_string(irq) + "/smp_affinity";

        std::ofstream out(affinity_file);
        if (!out)
        {
            std::cerr << "FAIL: cannot open " << affinity_file << "\n";
            continue;
        }
        out << mask_str;
        std::cout << " → IRQ " << irq
                  << " affinity set to CPU " << cpu
                  << " (mask 0x" << mask_str << ")\n";
    }
}

Mode parseArguments(int argc, char *argv[])
{
    Mode mode = DVS_DISPLAY_FPS; // Default mode

    // Define command-line options
    struct option long_options[] = {
        {"cis", no_argument, nullptr, 'c'},
        {"dvs", no_argument, nullptr, 'd'},
        {"check", no_argument, nullptr, 'x'},
        {"cis-dvs", no_argument, nullptr, 's'},
        {"write-dvs", no_argument, nullptr, 'w'},
        {"write-fil", no_argument, nullptr, 'W'},
        {"write-dvs-fil", no_argument, nullptr, 'z'},
        {"roi", no_argument, nullptr, 'r'},
        {"bbox", no_argument, nullptr, 'b'},
        {"overlay", no_argument, nullptr, 'o'},
        {"dvs-fps", no_argument, nullptr, 'f'},
        {"cis-dvs-fps", no_argument, nullptr, 'p'},
        {"dvs-fil-fps", no_argument, nullptr, 'B'},
        {"dvs-fps-d", no_argument, nullptr, 'D'},
        {"fil-fps", no_argument, nullptr, 'F'},
        {"dvs-display-and-write", no_argument, nullptr, 'a'},
        {"cis-roi", no_argument, nullptr, 'i'},
        {"dvs-bin-to-vid", no_argument, nullptr, 'v'},
        {"dvs-bin-to-png", no_argument, nullptr, 'g'},
        {"dvs-dat-to-png", no_argument, nullptr, 'G'},
        {"cis-dvs-store-png", no_argument, nullptr, 't'},
        {nullptr, 0, nullptr, 0}};

    // Parse command-line arguments
    int opt;
    while ((opt = getopt_long(argc, argv, "cdxswWzrbofpBDFaivgGt", long_options, nullptr)) != -1)
    {
        switch (opt)
        {
        case 'c':
            // cis only display
            mode = CIS_DISPLAY;
            // break;
            [[fallthrough]];
        case 'd':
            // dvs only display
            mode = DVS_DISPLAY;
            [[fallthrough]];
        case 'x':
            // prints error whenever there is a frame drop
            // error can happen in other execution modes due to heavy host code
            mode = DVS_CHECK_FRAME_DROP;
            // break;
            [[fallthrough]];
        case 's':
            // shows both CIS and DVS display
            // to get DVS fps correctly, run ./main -p
            mode = CIS_DVS_DISPLAY;
            // break;
            [[fallthrough]];
        case 'w':
            // subdirectory bin_files must reside under CIS_DVS
            // store compressed files containing DVS frames using multiple buffering
            // also runs error checks like in ./main -x
            mode = DVS_STORE;
            break;
        case 'W':
            // subdirectory bin_files must reside under CIS_DVS
            // store compressed files containing Filtered DVS frames using multiple buffering
            // also runs error checks like in ./main -x
            mode = FIL_STORE;
            break;
        case 'z':
            mode = DVS_FIL_STORE;
            break;
        case 'r':
            // runs DVS ROI detection
            // draws square bounding box around ROI
            mode = DVS_ROI;
            // break;
            [[fallthrough]];
        case 'b':
            // you should run ./main -o to calibrate between CIS and DVS first
            // shows ROI onto DVS and CIS perspective simultaneously
            mode = CIS_DVS_BBOX;
            // break;
            [[fallthrough]];
        case 'o':
            // shows red grid in order to calibrate between CIS and DVS viewpoint
            // modify config.hpp according to console messages
            mode = CIS_DVS_OVERLAY;
            // break;
            [[fallthrough]];
        case 'f':
            // prints DVS fps to the console
            // no error checking functionality
            mode = DVS_FPS_CHECK;
            // break;
            [[fallthrough]];
        case 'p':
            // displays CIS, DVS video streams
            // displays their FPS
            // runs error checks
            // since DVS is downsampled, the screen might be laggy or shaded less
            mode = CIS_DVS_DISPLAY_FPS;
            break;
        case 'B':
            // displays DVS, Filtered DVS video streams
            // displays their FPS
            // runs error checks
            // since DVS is downsampled, the screen might be laggy or shaded less
            mode = DVS_FIL_DISPLAY_FPS;
            break;
        case 'D':
            // displays DVS video streams
            // displays FPS
            // runs error checks
            // since DVS is downsampled, the screen might be laggy or shaded less
            mode = DVS_DISPLAY_FPS;
            break;
        case 'F':
            // Filtered DVS video streams
            // displays FPS
            // runs error checks
            // since DVS is downsampled, the screen might be laggy or shaded less
            mode = FIL_DISPLAY_FPS;
            break;
        case 'a':
            // DVS video streams
            // displays FPS
            // runs error checks
            // store compressed files containing DVS frames
            mode = DVS_DISPLAY_AND_STORE;
            break;
        case 'i':
            // runs background subtraction on CIS images
            mode = CIS_ONLY_ROI;
            // break;
            [[fallthrough]];
        case 'v':
            // converts bin file to .avi video file
            // the path to bin file, and the path to video is required
            mode = DVS_BIN_TO_VID;
            [[fallthrough]];
        case 'g':
            // converts bin file into a collection of .png files
            //  the path to bin file, and the path to png file folder is requred
            mode = DVS_BIN_TO_PNG;
            break;
        case 'G':
            // converts .dat files to .png files
            //  the path to bin file, and the path to png file folder is requred
            mode = DVS_DAT_TO_PNG;
            break;
        case 't':
            // captures CIS and DVS images in png format, synchronized at 60fps
            // the path to CIS, and the path to DVS image folders are required.
            mode = CIS_DVS_STORE_PNG;
            [[fallthrough]];
        default:
            mode = DVS_DISPLAY_FPS;
            break;
            /*fprintf(stderr, "Usage: %s [--cis | --dvs | --check | --cis-dvs | --write-dvs | --write-fil | --write-dvs-fil | --roi | --bbox | --overlay | --dvs-fps | --cis-dvs-fps | --dvs-fil-fps | --dvs-fps-d | --fil-fps | --dvs-display-and-write | --cis-roi | --dvs-bin-to-vid | --dvs-bin-to-png | --dvs-dat-to-png | --cis-dvs-store-png ]\n", argv[0]);
            exit(EXIT_FAILURE);*/
        }
    }

    return mode;
}
