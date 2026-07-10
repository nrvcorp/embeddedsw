#include "DVS.hpp"

#include <pthread.h>
#include <sched.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

#include "CompressionTaskPool.hpp"
#include "CountingSemaphore.hpp"
#include "DatToPngConverter.hpp"
#include "DebugConfig.hpp"
#include "IndexedMutexPool.hpp"
#include "OpenCVTaskManager.hpp"

#include "MutexManager.hpp"
#include "PCIe.hpp"
#include "bbox.hpp"
#include "config.hpp"
#include "shutdown_flag.hpp"

#include "timer.hpp"
using namespace timing;
using namespace std::chrono;

#define BILLION 1000000000L

void setThreadPriorityMax(std::thread &t)
{
    pthread_t handle = t.native_handle();

    // Set scheduling parameters
    sched_param sch;
    sch.sched_priority = sched_get_priority_max(SCHED_FIFO); // Highest priority

    if (pthread_setschedparam(handle, SCHED_FIFO, &sch) != 0)
    {
        std::cerr << "Failed to set thread priority.\n";
    }
}
void setThreadPriorityHigh(std::thread &t)
{
    pthread_t handle = t.native_handle();

    // Set scheduling parameters
    sched_param sch;
    sch.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1; // Highest priority

    if (pthread_setschedparam(handle, SCHED_FIFO, &sch) != 0)
    {
        std::cerr << "Failed to set thread priority.\n";
    }
}

DVS::DVS( // normal constructor
    int frame_h, int frame_w, bool is_header, int accum_num,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr, int buffer_num,
    const char *c2h_dev, const char *h2c_dev, MutexManager &display_mutex,
    bool double_buffering, int display_downsample_num)
    : frame_h(frame_h),
      frame_w(frame_w),
      accum_num(accum_num),
      is_header(is_header),
      header_bytes(FRAME_HEADER_BYTES),
      rdy_baseaddr(rdy_baseaddr),
      frame_baseaddr(frame_baseaddr),
      buffer_num(buffer_num),
      fpt(1),
      pcie(c2h_dev, h2c_dev),
      rd_ptr(0),
      display_mutex(display_mutex),
      terminate(nullptr),
      display_downsample_num(display_downsample_num)
{
    fpt = 1;
    pcie_read_min_interval_us = 200;
    last_pcie_read = steady_clock::now();
    std::cout << "TEST1" << std::endl;
    // set total frame bytes
    frame_bytes =
        (is_header) ? (frame_h * frame_w) / 4 + header_bytes : (frame_h * frame_w) / 4;

    // set total pixel num
    pixel_num = frame_h * frame_w;

    // pre-calculate buffer address for ZCU106 PCIE XDMA access
    buffer_rdy_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    buffer_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    for (int i = 0; i < buffer_num; i++)
    {
        buffer_rdy_addr[i] = rdy_baseaddr + i;
        buffer_addr[i] = frame_baseaddr + ((frame_bytes)*i);
    }
    buffer_rdy = (char *)malloc(1 * sizeof(char));
    is_buffer_rdy = (char *)malloc(1 * sizeof(char));
    buffer_done = (char *)malloc(1 * sizeof(char));
    is_buffer_rdy[0] = 0x01;
    buffer_done[0] = 0x00;

    // allocate buffers
    buffer = (char *)malloc(frame_bytes * sizeof(char));

    // set data pointer behind header
    frame_start = (is_header) ? buffer + header_bytes : buffer;

    // allocate mutex and buffer for double buffering
    if (!double_buffering)
    {
        dbuf_mutex[0] = NULL;
        dbuf_mutex[1] = NULL;
        double_buffer = NULL;
    }
    else
    {
        double_buffer = (char *)malloc(frame_bytes * sizeof(char));
        dbuf_mutex[0] = new MutexManager();
        dbuf_mutex[1] = new MutexManager();
        terminate = new bool;
    }

    // don't init CIS related params right now
    convert_cis = false;
}

DVS::DVS( // pcie burst
    int id, int frame_h, int frame_w, bool is_header, int accum_num,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr, uintptr_t buffer_commit_index_addr,
    int buffer_num,
    int num_frames_per_transter, int dvs_fps, const char *c2h_dev,
    const char *h2c_dev, MutexManager &display_mutex, int host_buffer_num,
    int display_downsample_num, ShutdownFlag sdflag)
    : id(id),
      frame_h(frame_h),
      frame_w(frame_w),
      accum_num(accum_num),
      is_header(is_header),
      rdy_baseaddr(rdy_baseaddr),
      frame_baseaddr(frame_baseaddr),
      buffer_commit_index_addr(buffer_commit_index_addr),
      buffer_num(buffer_num),
      fpt(num_frames_per_transter),
      pcie(c2h_dev, h2c_dev),
      rd_ptr(0),
      display_mutex(display_mutex),
      host_buffer_num(host_buffer_num),
      locks(std::make_unique<IndexedMutexPool>(host_buffer_num)),
      filledSlots(0),
      displayedSlots(0),
      freeSlots(host_buffer_num),
      terminate(nullptr),
      display_downsample_num(display_downsample_num),
      init_mode(MODE_MULT),
      quit(std::move(sdflag)),

      test_header(0)
{
    std::cout << "TEST MULT" << std::endl;
    std::cout << "frame_baseaddr = 0x" << std::hex << frame_baseaddr << std::dec
              << std::endl;

    last_pcie_read = steady_clock::now();
    tps = dvs_fps / fpt;
    pcie_read_min_interval_us = (1e6 / tps) * 0.7;

    std::cout << "num_frames_per_transter = " << fpt
              << std::endl;
    std::cout << "pcie_read_min_interval_us = " << pcie_read_min_interval_us
              << std::endl;
    std::cout << "(display) FPS = " << dvs_fps / (accum_num * display_downsample_num)
              << std::endl;
    std::cout << "(display) frame_accum_num = " << accum_num
              << std::endl;
    std::cout << "(display) downsample_num = " << display_downsample_num
              << std::endl;

    // set total frame bytes
    header_bytes = (is_header) ? FRAME_HEADER_BYTES : 0;
    frame_bytes = (frame_h * frame_w) / 4 + header_bytes;
    frame_bytes *= fpt;
    // set total pixel num
    pixel_num = frame_h * frame_w;

    int B = 127 / accum_num;
    DVS_TO_GRAY_LUT_SMOOTH = {
        /*code 0*/ 0,
        /*code 1*/ static_cast<int8_t>(+B),
        /*code 2*/ static_cast<int8_t>(-B),
        /*code 3*/ 0};

    // pre-calculate buffer address for ZCU106 PCIE XDMA access
    buffer_rdy_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    buffer_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    for (int i = 0; i < buffer_num; i++)
    {
#if (CHECK_ALL_READY_FLAGS)
        buffer_rdy_addr[i] = rdy_baseaddr + (i * fpt);
#else
        buffer_rdy_addr[i] =
            rdy_baseaddr + (i * fpt) +
            (fpt - 1); // 한 번에 읽어오는 프레임의 중 마지막 프레임
#endif
        buffer_addr[i] = frame_baseaddr + ((frame_bytes)*i);
    }

#if (CHECK_ALL_READY_FLAGS)
    buffer_rdy = (char *)malloc(fpt * sizeof(char));
    is_buffer_rdy = (char *)malloc(fpt * sizeof(char));
    buffer_done = (char *)malloc(fpt * sizeof(char));
    memset(is_buffer_rdy, 0x01, fpt * sizeof(char));
    memset(buffer_done, 0x00, fpt * sizeof(char));
#else
    buffer_rdy = (char *)malloc(1 * sizeof(char));
    is_buffer_rdy = (char *)malloc(1 * sizeof(char));
    buffer_done = (char *)malloc(1 * sizeof(char));
    is_buffer_rdy[0] = 0x01;
    buffer_done[0] = 0x00;
#endif

    buffer_flush = (char *)malloc(buffer_num * fpt * sizeof(char));
    memset(buffer_flush, 0x00, (buffer_num * fpt) * sizeof(char));
    buffer_commit_index = (char *)malloc(4 * sizeof(char));

    // allocate buffers
    host_buffer.resize(host_buffer_num);
    // mbuf_mutex.resize(host_buffer_num);
    host_buffer_ready.resize(host_buffer_num);

    for (int i = 0; i < host_buffer_num; i++)
    {
        host_buffer[i] = (char *)malloc(frame_bytes * sizeof(char));
        // mbuf_mutex[i] = new MutexManager();
        host_buffer_ready[i] = 0;
        // std::cout << "host buffer address: " <<
        // static_cast<void*>(host_buffer[i]) << std::endl;
    }

    terminate = new bool;
    // set data pointer behind header
    frame_start = buffer + header_bytes;

    // don't init CIS related params right now
    convert_cis = false;

    // disable double buffering
    double_buffer = NULL;
    dbuf_mutex[0] = NULL;
    dbuf_mutex[1] = NULL;
}

DVS::DVS( // crop_roi constructor
    int frame_h, int frame_w, bool is_header, int accum_num,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr, int buffer_num,
    const char *c2h_dev, const char *h2c_dev, MutexManager &display_mutex,
    Bbox *bbox, MutexManager *thread_mutex, bool *terminate,
    int display_downsample_num)
    : frame_h(frame_h),
      frame_w(frame_w),
      accum_num(accum_num),
      is_header(is_header),
      header_bytes(FRAME_HEADER_BYTES),
      rdy_baseaddr(rdy_baseaddr),
      frame_baseaddr(frame_baseaddr),
      buffer_num(buffer_num),
      pcie(c2h_dev, h2c_dev),
      rd_ptr(0),
      display_mutex(display_mutex),
      bbox(bbox),
      thread_mutex(thread_mutex),
      terminate(terminate),
      display_downsample_num(display_downsample_num)
{
    fpt = 1;
    pcie_read_min_interval_us = 200;
    last_pcie_read = steady_clock::now();
    std::cout
        << "TEST2" << std::endl;
    // set total frame bytes
    frame_bytes =
        (is_header) ? (frame_h * frame_w) / 4 + header_bytes : (frame_h * frame_w) / 4;

    // set total pixel num
    pixel_num = frame_h * frame_w;

    // pre-calculate buffer address for ZCU106 PCIE XDMA access
    buffer_rdy_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    buffer_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    for (int i = 0; i < buffer_num; i++)
    {
        buffer_rdy_addr[i] = rdy_baseaddr + i;
        buffer_addr[i] = frame_baseaddr + ((frame_bytes)*i);
    }
    buffer_rdy = (char *)malloc(1 * sizeof(char));
    buffer_done = (char *)malloc(1 * sizeof(char));
    buffer_done[0] = 0x00;

    // allocate buffer
    buffer = (char *)malloc(frame_bytes * sizeof(char));

    // set data pointer behind header
    frame_start = (is_header) ? buffer + header_bytes : buffer;

    // disable double buffering
    double_buffer = NULL;
    dbuf_mutex[0] = NULL;
    dbuf_mutex[1] = NULL;

    // don't init CIS related params right now
    convert_cis = false;
}

void DVS::convert2BitToBR()
{
    // for BGR
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        int pixel = (frame_start[byteIndex] >> bitOffset) & 0x03;
        if (pixel == 1)
        {
            // for off events, bias current pixel closer to red
            frame.data[i * 3] = 255;
            frame.data[i * 3 + 1] = 0;
            frame.data[i * 3 + 2] = 0;
        }
        else if (pixel == 2)
        {
            // for off events, bias current pixel closer to blue
            frame.data[i * 3] = 0;
            frame.data[i * 3 + 1] = 0;
            frame.data[i * 3 + 2] = 255;
        }
        else
        {
            frame.data[i * 3] = 255;
            frame.data[i * 3 + 1] = 255;
            frame.data[i * 3 + 2] = 255;
        }
    }
}

void DVS::convert2BitToBR_accum()
{
    // for BGR
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        int pixel = (frame_start[byteIndex] >> bitOffset) & 0x03;
        if (pixel == 1)
        {
            // for off events, bias current pixel closer to red
            frame.data[i * 3] = 255;
            frame.data[i * 3 + 1] = 0;
            frame.data[i * 3 + 2] = 0;
        }
        else if (pixel == 2)
        {
            // for off events, bias current pixel closer to blue
            frame.data[i * 3] = 0;
            frame.data[i * 3 + 1] = 0;
            frame.data[i * 3 + 2] = 255;
        }
    }
}

void DVS::convert2BitTo8Bit()
{
    // for each pixel
    /*for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        uint8_t pixel =
            (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
        // dst[i] = pixel * 85;
        // Map 2-bit value to 8-bit value (0, 85, 170, 255)
        frame.data[i] = (pixel == 0) ? 128 : // no event
                            (pixel == 1) ? 255
                                         : // on event
                            (pixel == 2) ? 0
                                         : // off event
                            0;
    }*/

    for (int i = 0; i < pixel_num; i += 4)
    {
        int byteIndex = i >> 2;
        int bitOffset = (i & 3) << 1;
        uint8_t pixel =
            (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits

        frame.data[i] = DVS_TO_GRAY_LUT[pixel];
    }
}

void DVS::convert2BitTo8Bit_accum()
{
    // for each pixel
    /*for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        uint8_t pixel =
            (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits

        // simply stack events on top of previous frames
        // can be replaced with an alternate image processing method
        if (pixel == 1)
        {
            // for on events, set grayscale pixel to 255
            frame.data[i] = 255;
        }
        else if (pixel == 2)
        {
            // for off events, set grayscale pixel to 0
            frame.data[i] = 0;
        }
    }*/

    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i >> 2;
        int bitOffset = (i & 3) << 1;
        uint8_t pixel =
            (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
        if (pixel)
        {
            frame.data[i] = DVS_TO_GRAY_LUT[pixel];
        }
    }
}

void DVS::convert2BitTo8Bit_accumSmooth()
{
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i >> 2;
        int bitOffset = (i & 3) << 1;
        uint8_t pixel =
            (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits

        frame.data[i] += DVS_TO_GRAY_LUT_SMOOTH[pixel];
    }
}

void DVS::convert2BitToBGR_accum()
{
    // for each pixel
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        int pixel = (frame_start[byteIndex] >> bitOffset) & 0x03;
        if (pixel == 1)
        {
            // for on events, bias current pixel closer to red
            frame.data[i * 3] =
                (frame.data[i * 3] < 40) ? 0 : (frame.data[i * 3] - 40);
            frame.data[i * 3 + 2] = (frame.data[i * 3 + 2] + 40 > 255)
                                        ? 255
                                        : (frame.data[i * 3 + 2] + 40);
        }
        else if (pixel == 2)
        {
            // for off events, bias current pixel closer to blue
            frame.data[i * 3] =
                (frame.data[i * 3] + 40 > 255) ? 255 : (frame.data[i * 3] + 40);
            frame.data[i * 3 + 2] =
                (frame.data[i * 3 + 2] < 40) ? 0 : (frame.data[i * 3 + 2] - 40);
        }
    }
}

void DVS::decode_header(const char *buffer, uint64_t &sensor_cfg_index,
                        int &frame_num,
                        uint32_t &timestamp)
{

    static const int header_ext_size = 8;
    static uint64_t prev_id = 0;
    // 암시적 캐스팅은 32비트 타입이라 그 이상 크기의 시프트는 명시적 캐스팅 해야됨
    test_header = ((static_cast<uint64_t>(buffer[7] & 0xffu) << 56) |
                   (static_cast<uint64_t>(buffer[6] & 0xffu) << 48) |
                   (static_cast<uint64_t>(buffer[5] & 0xffu) << 40) |
                   (static_cast<uint64_t>(buffer[4] & 0xffu) << 32) |
                   (static_cast<uint64_t>(buffer[3] & 0xffu) << 24) |
                   (static_cast<uint64_t>(buffer[2] & 0xffu) << 16) |
                   (static_cast<uint64_t>(buffer[1] & 0xffu) << 8) |
                   static_cast<uint64_t>(buffer[0] & 0xffu));

    // Extract frame number from buffer
    frame_num = ((static_cast<int>(static_cast<unsigned char>(buffer[7 + header_ext_size]) << 24)) |
                 (static_cast<int>(static_cast<unsigned char>(buffer[6 + header_ext_size]) << 16)) |
                 (static_cast<int>(static_cast<unsigned char>(buffer[5 + header_ext_size]) << 8)) |
                 (static_cast<int>(static_cast<unsigned char>(buffer[4 + header_ext_size]))));

    // Extract timestamp from buffer
    timestamp = ((static_cast<uint32_t>(static_cast<unsigned char>(buffer[3 + header_ext_size]) << 24)) |
                 (static_cast<uint32_t>(static_cast<unsigned char>(buffer[2 + header_ext_size]) << 16)) |
                 (static_cast<uint32_t>(static_cast<unsigned char>(buffer[1 + header_ext_size]) << 8)) |
                 (static_cast<uint32_t>(static_cast<unsigned char>(buffer[0 + header_ext_size]))));

    // 하위 63비트
    uint64_t cfg_id = test_header & ((1ULL << 63) - 1);
    if (prev_id != cfg_id)
    {
        prev_id = cfg_id;
        std::cout << "cfg_id: " << std::dec << cfg_id
                  << std::endl;
    }
}

void DVS::decode_header(const char *buffer, int &frame_num,
                        uint32_t &timestamp)
{

    // Extract frame number from buffer
    frame_num = ((static_cast<int>(static_cast<unsigned char>(buffer[7]) << 24)) |
                 (static_cast<int>(static_cast<unsigned char>(buffer[6]) << 16)) |
                 (static_cast<int>(static_cast<unsigned char>(buffer[5]) << 8)) |
                 (static_cast<int>(static_cast<unsigned char>(buffer[4]))));

    // Extract timestamp from buffer
    timestamp = ((static_cast<uint32_t>(static_cast<unsigned char>(buffer[3]) << 24)) |
                 (static_cast<uint32_t>(static_cast<unsigned char>(buffer[2]) << 16)) |
                 (static_cast<uint32_t>(static_cast<unsigned char>(buffer[1]) << 8)) |
                 (static_cast<uint32_t>(static_cast<unsigned char>(buffer[0]))));
}

void DVS::read_frame(char *dvs_buffer)
{

#if (CHECK_READ_FRAME)
    CallGap _gap("READ_FRAME", id);
    ScopeTimer _tim("READ_FRAME", id);
#endif

    // wait for ready flag
    // by polling through PCIE connection
    while (true)
    {
#if (CHECK_READ_FLAG)
        CallGap _gap("READ_FLAG");
        ScopeTimer _tim("READ_FLAG");
#endif
        auto n = steady_clock::now();

        // std::cout << "ddddd " << (duration_cast<microseconds>(n - lr).count()) << "\n";
        if (duration_cast<microseconds>(n - last_pcie_read).count() >=
            pcie_read_min_interval_us)
        {
            last_pcie_read = n;
#if (CHECK_ALL_READY_FLAGS)
            pcie.c2h(buffer_rdy, fpt, buffer_rdy_addr[rd_ptr]);
            if (memcmp(buffer_rdy, is_buffer_rdy, fpt) == 0)
                break;
#else
            pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);
            if (buffer_rdy[0] == 0x01)
                break;
#endif
        }
    }

    pcie.c2h(dvs_buffer, frame_bytes, buffer_addr[rd_ptr]);

    // set flag to DONE through PCIE
#if (CHECK_ALL_READY_FLAGS)
    pcie.h2c(buffer_done, fpt, buffer_rdy_addr[rd_ptr]);
#else
    pcie.h2c(buffer_done, fpt, buffer_rdy_addr[rd_ptr] - (fpt - 1));
#endif

    //  change the address for ready flag and DVS frame
    rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
}

#if (READ_HEADER_ONLY_IF_NOT_NEEDED)
void DVS::read_header(char *dvs_buffer)
{

#if (CHECK_READ_HEADER)
    CallGap _gap("READ_HEADER", id);
    ScopeTimer _tim("READ_HEADER", id);
#endif

    // wait for ready flag
    // by polling through PCIE connection
    while (true)
    {
#if (CHECK_READ_FLAG)
        CallGap _gap("READ_FLAG");
        ScopeTimer _tim("READ_FLAG");
#endif
        auto n = steady_clock::now();
        if (duration_cast<microseconds>(n - last_pcie_read).count() >=
            pcie_read_min_interval_us)
        {
            last_pcie_read = n;
#if (CHECK_ALL_READY_FLAGS)
            pcie.c2h(buffer_rdy, fpt, buffer_rdy_addr[rd_ptr]);
            if (memcmp(buffer_rdy, is_buffer_rdy, fpt) == 0)
                break;
#else
            pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);
            if (buffer_rdy[0] == 0x01)
                break;
#endif
        }
    }

    // read DVS first frame header through PCIE
    pcie.c2h(dvs_buffer, header_bytes, buffer_addr[rd_ptr]);

    // set flag to DONE through PCIE
#if (CHECK_ALL_READY_FLAGS)
    pcie.h2c(buffer_done, fpt, buffer_rdy_addr[rd_ptr]);
#else
    pcie.h2c(buffer_done, fpt, buffer_rdy_addr[rd_ptr] - (fpt - 1));
#endif

    //  change the address for ready flag and DVS frame
    rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
}
#endif

//
void DVS::flush_buffer_rdy(void)
{
#if (CHECK_ALL_READY_FLAGS)
    pcie.h2c(buffer_flush, (buffer_num * fpt), buffer_rdy_addr[0]);
#else
    pcie.h2c(buffer_flush, (buffer_num * fpt), buffer_rdy_addr[0] - (fpt - 1));
#endif
}

void DVS::init_rd_ptr(void)
{
    flush_buffer_rdy();
    pcie.c2h(buffer_commit_index, 4, buffer_commit_index_addr); // u32

    int ptr_temp = ((static_cast<unsigned char>(buffer_commit_index[3]) << 24) |
                    (static_cast<unsigned char>(buffer_commit_index[2]) << 16) |
                    (static_cast<unsigned char>(buffer_commit_index[1]) << 8) |
                    static_cast<unsigned char>(buffer_commit_index[0]) << 0);

    rd_ptr = ((ptr_temp / fpt) + 1) % buffer_num;
    printf("rd ptr = %d\n", rd_ptr);
}

void DVS::calc_fps(double &fps, int &display_fps, int &frameCount, double &startTime,
                   cv::Mat &frame)
{
    static int display_fps_cap = 0;
    frameCount += accum_num * display_downsample_num * fpt;
    display_fps++;
    double end = cv::getTickCount();
    double elapsedTime = (end - startTime) / cv::getTickFrequency();
    // auto end = (std::chrono::high_resolution_clock::now());
    // auto elapsedTime =
    // (std::chrono::duration_cast<std::chrono::microseconds>(end - startTime)) ;
    if (elapsedTime >= 1.0)
    {
        startTime = end;
        fps = frameCount / elapsedTime;
        frameCount = 0;
        display_fps_cap = display_fps;
        display_fps = 0;
    }

    std::ostringstream oss, oss2;
    oss << "processed FPS: " << static_cast<int>(fps)
        << " downsample num: " << static_cast<int>(display_downsample_num)
        << " accum num: " << static_cast<int>(accum_num);
    cv::putText(frame, oss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);

    oss2 << "rendered FPS: " << static_cast<int>(display_fps_cap);
    cv::putText(frame, oss2.str(), cv::Point(10, 65), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
}

void DVS::display_stream(bool is_flip)
{
    // double fps = 0.0,
    // int display_fps = 0;
    // int frameCount = 0;
    // double startTime = cv::getTickCount();
    while (true)
    {
        // initialize cv::Mat frame
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);

        // stack frames using member functions
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            read_frame(buffer);
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }
        }

        // show image
        if (is_flip)
        {
            cv::flip(frame, frame, 0);
        }
        // calc_fps(fps, display_fps, frameCount, startTime, frame);

        // display using mutex locking
        display_mutex.lock_display();

        cv::imshow("DVS camera", frame);

        // press ESC to quit
        if (cv::waitKey(1) == 27)
        {
            display_mutex.unlock_display();

            // free frame
            frame.release();
            break;
        }
        display_mutex.unlock_display();
    }
}

void DVS::save_png_stream(char *output_folder_name, bool is_flip)
{
    // double fps = 0.0;
    // int frameCount = 0;
    // double startTime = cv::getTickCount();
    int frame_count = 0;

    while (true)
    {
        // initialize cv::Mat frame
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);

        // stack frames using member functions
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            read_frame(buffer);
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }
        }
        // show image
        if (is_flip)
        {
            cv::flip(frame, frame, 0);
        }
        if (thread_mutex->try_lock_reader() == 1)
        {
            // Save the frame as a PNG image
            std::ostringstream filename;
            filename << output_folder_name << "/frame_" << std::setw(5)
                     << std::setfill('0') << frame_count << ".png";
            cv::imwrite(filename.str(), frame);
            thread_mutex->unlock_multiple_reader();
            frame_count++;
            // Generate filename for the PNG image
        }
        // press ESC to quit
        if (cv::waitKey(1) == 27)
        {
            // free frame
            frame.release();
            break;
        }
    }
}

void DVS::multiple_buf_display_fps_pcie_reader()
// note : thread_mutex needs to be an array like thread_mutex[2]
{
    std::cout << "TEST MULT3" << std::endl;

    int prev_frame_num;
    uint32_t prev_timestamp;
    bool check_init = 0;
    int error_num = 0;
    int multiple_buffer_idx = 0;
    int frame_num;
    uint32_t timestamp;
    long long timestamp_itv = 0;
    char *dvs_buffer;
#if (FRAME_HEADER_BYTES == 16)
    uint64_t sensor_cfg_index;
#endif

    // while reading raw sensor data, check frame num consistency too

    printf("\nstarting display...\n");
    init_rd_ptr();
    while (!quit->load(std::memory_order_acquire))
    {

        freeSlots.acquire();
        /*if (host_buffer_ready[multiple_buffer_idx]){
                std::this_thread::yield();
                continue;
        }*/
#if (CHECK_multiple_buf_display_fps_pcie_reader)
        CallGap _gap("multiple_buf_display_fps_pcie_reader");
        ScopeTimer _tim("multiple_buf_display_fps_pcie_reader");
#endif
        /*if (check_init < (tps / display_downsample_num) * 5)
        {
            check_init++;
        }*/

        // mbuf_mutex[multiple_buffer_idx]->lock_single_writer();

        {
            auto g = locks->acquire(multiple_buffer_idx);

            dvs_buffer = host_buffer[multiple_buffer_idx];

            for (int i = 0; i < display_downsample_num; i++)
            {
                // read raw data

                if (i == display_downsample_num - 1)
                    read_frame(dvs_buffer);
                else
#if (READ_HEADER_ONLY_IF_NOT_NEEDED)
                    read_header(dvs_buffer);
#else
                    read_frame(dvs_buffer);
#endif

                // check frame num consistency
#if (FRAME_HEADER_BYTES == 16)
                decode_header(dvs_buffer, sensor_cfg_index, frame_num, timestamp);
#else
                decode_header(dvs_buffer, frame_num, timestamp);
#endif
                int frame_num_gap = (frame_num - prev_frame_num) > 0 ? (frame_num - prev_frame_num) : (frame_num - prev_frame_num) + 256;
                if (frame_num_gap != fpt)
                {
                    if (check_init)
                    {
                        error_num++;
                        std::cout << "====================================================="
                                     "================================================"
                                  << std::endl;
                        std::cout << "(p) ERROR NUM: " << std::dec << error_num
                                  << std::endl;
                        std::cout << "prev_timestamp: " << std::dec << prev_timestamp
                                  << ", prev_frame_num: " << prev_frame_num << std::endl;
                        std::cout << "timestamp: " << std::dec << timestamp
                                  << ", frame_num: " << frame_num << std::endl;
                        std::cout << "====================================================="
                                     "================================================"
                                  << std::endl;
                    }
                }

                prev_frame_num = frame_num;
                // timestamp_itv += (timestamp - prev_timestamp);
                prev_timestamp = timestamp;
#if (COMPARE_FIL_DVS)
                latest_header.frame_num.store(frame_num, std::memory_order_relaxed);
                latest_header.timestamp.store(timestamp, std::memory_order_relaxed);
#endif
            }

            // mbuf_mutex[multiple_buffer_idx]->unlock_single_writer(1);
            // host_buffer_ready[multiple_buffer_idx] = 1;

            multiple_buffer_idx = (multiple_buffer_idx + 1) % host_buffer_num;

            filledSlots.release();
            check_init = 1;
        }

        if (*terminate)
            break;
    }
    filledSlots.release_all();
}

void DVS::multiple_buf_display_fps_render(bool is_flip)
{
    std::cout << "TEST MULT4" << std::endl;

    double fps = 0.0;
    int display_fps = 0;
    int frameCount = 0;
    double startTime = cv::getTickCount();
    double end = cv::getTickCount();
    double elapsedTime = (end - startTime) / cv::getTickFrequency();
    int multiple_buffer_idx = 0;

    // initialize cv::Mat frame
    const cv::Size sz(frame_w, frame_h);
    cv::Mat base(sz, CV_8UC1, cv::Scalar(128));

    std::string winname = "DVS camera " + std::to_string(id);

    while (!quit->load(std::memory_order_acquire))
    {
        filledSlots.acquire(accum_num);
#if (CHECK_multiple_buf_display_fps_render)
        CallGap _gap("multiple_buf_display_fps_render");
        ScopeTimer _tim("multiple_buf_display_fps_render");
#endif

        // reset frame
        base.copyTo(frame);
        // stack frames using member functions
        int frame_grp_num = 0;
        for (int frame_grp_num = 0; frame_grp_num < (accum_num); frame_grp_num++)
        {

            auto g = locks->acquire(multiple_buffer_idx);
            frame_start = host_buffer[multiple_buffer_idx] + header_bytes;
            // convert2BitTo8Bit_accumSmooth();
            convert2BitTo8Bit_accum();
            multiple_buffer_idx = (multiple_buffer_idx + 1) % host_buffer_num;
        }
        freeSlots.release(accum_num);

        // show image
        if (is_flip)
        {
            cv::flip(frame, frame, 0);
        }
        calc_fps(fps, display_fps, frameCount, startTime, frame);

        // display using mutex locking
        // display_mutex.lock_display();

        cv::imshow(winname, frame);

        if (*terminate || cv::waitKey(1) == 27)
        {
            *terminate = true;
            // cleanup
            frame.release();
            // display_mutex.unlock_display();
            break;
        }
        // display_mutex.unlock_display();
    }
    freeSlots.release_all();
}
void DVS::check_frame_drop()
{
    int prev_frame_num;
    uint32_t prev_timestamp;
    int check_init = 0;
    int error_num = 0;
    int frame_num;
    uint32_t timestamp;
#if (FRAME_HEADER_BYTES == 16)
    uint64_t sensor_cfg_index;
#endif

    auto start = std::chrono::high_resolution_clock::now();
    while (1)
    {
        read_frame(buffer);
        if (check_init < (fpt * tps * 2))
        {
            check_init++;
        }
        else
        {
            break;
        }
    }
    while (1)
    {
        // wait some time after the sensor starts up

        // get frame num and headers
        read_frame(buffer);
#if (FRAME_HEADER_BYTES == 16)
        decode_header(buffer, sensor_cfg_index, frame_num, timestamp);
#else
        decode_header(buffer, frame_num, timestamp);
#endif
        // std::cout << "frame_pointer value: " << std::dec << rd_ptr  << " ";
        // std::cout << "frame_num value (decimal): " << std::dec << frame_num <<
        // "(hex): 0x" << std::hex << frame_num << "  "; std::cout << "timestamp
        // value (decimal): " << std::dec << timestamp << "(hex): 0x" << std::hex <<
        // timestamp << std::endl;

        // check if the frame num increments by 1 consistently
        if ((prev_frame_num + 1 != frame_num) &&
            (prev_frame_num != (frame_num + 255)))
        {
            // if frame num is inconsistent, print error message
            error_num++;
            std::cout << "ERROR NUM: " << std::dec << error_num << std::endl;
        }

        prev_frame_num = frame_num;
        prev_timestamp = timestamp;
        // if(error_num > 1000) {
        //     break;
        // }
    }
    // std::cout << "ERROR NUM: " << std::dec << error_num << std::endl;
}
void DVS::fps_count()
{
    int frame_count = 0;
    auto start = std::chrono::high_resolution_clock::now();

    while (1)
    {
        frame_count++;
        // read DVS frame
        read_frame(buffer);
        // whenever 4096 frames are read
        if (frame_count >= 4096)
        {
            // Record the end time
            auto end = std::chrono::high_resolution_clock::now();

            // Calculate the elapsed time in microseconds
            auto elapsed =
                std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            // calculate true fps
            std::cout << "FPS : " << ((4096 * 1000000.0) / (int)(elapsed.count()))
                      << std::endl;
            start = end;
            frame_count = 0;
        }
    }
}
void *DVS::multiple_buf_pcie_reader()
// note : thread_mutex needs to be an array like thread_mutex[2]
{
    int prev_frame_num;
    uint32_t prev_timestamp;
    bool check_init = 0;
    int error_num = 0;
    int multiple_buffer_idx = 0;
    // int prev_idx = 0;
    int frame_num;
    uint32_t timestamp;
    char *dvs_buffer;
#if (FRAME_HEADER_BYTES == 16)
    uint64_t sensor_cfg_index;
#endif

    if (posix_memalign((void **)&dvs_buffer, 4096, frame_bytes) != 0)
    {
        perror("posix_memalign failed");
    }

    printf("\nstarting save...\n");
    init_rd_ptr();
    // while reading raw sensor data, check frame num consistency too
    while (!quit->load(std::memory_order_acquire))
    {
        freeSlots.acquire();
#if (CHECK_multiple_buf_pcie_reader)
        CallGap _gap("multiple_buf_pcie_reader");
        ScopeTimer _tim("multiple_buf_pcie_reader");
#endif

        // maintain (host_buffer_num) buffers and (host_buffer_num) mutexes for
        // multiple buffering

        // mbuf_mutex[multiple_buffer_idx]->lock_single_writer();
        {
            auto g = locks->acquire(multiple_buffer_idx);
            dvs_buffer = host_buffer[multiple_buffer_idx];

            // read raw data
            read_frame(dvs_buffer);

            // check frame num consistency
#if (FRAME_HEADER_BYTES == 16)
            decode_header(dvs_buffer, sensor_cfg_index, frame_num, timestamp);
#else
            decode_header(dvs_buffer, frame_num, timestamp);
#endif
            int frame_num_gap = (frame_num - prev_frame_num) > 0 ? (frame_num - prev_frame_num) : (frame_num - prev_frame_num) + 256;
            if (frame_num_gap != fpt)
            {
                if (check_init)
                {

                    error_num++;
                    std::cout << "==========================================================="
                                 "=========================================="
                              << std::endl;
                    std::cout << "(w) ERROR NUM: " << std::dec << error_num << std::endl;
                    std::cout << "prev_timestamp: " << std::dec << prev_timestamp
                              << ", prev_frame_num: " << prev_frame_num << std::endl;
                    std::cout << "timestamp: " << std::dec << timestamp
                              << ", frame_num: " << frame_num << std::endl;
                    std::cout << "==========================================================="
                                 "=========================================="
                              << std::endl;
                }
            }

            prev_frame_num = frame_num;
            prev_timestamp = timestamp;
#if (COMPARE_FIL_DVS)
            latest_header.frame_num.store(frame_num, std::memory_order_relaxed);
            latest_header.timestamp.store(timestamp, std::memory_order_relaxed);
#endif
            // host_buffer_ready[multiple_buffer_idx] = 1;

            // mbuf_mutex[multiple_buffer_idx]->unlock_single_writer(1);

            multiple_buffer_idx = (multiple_buffer_idx + 1) % host_buffer_num;
            filledSlots.release();
            check_init = 1;
        }
        /*if (multiple_buffer_idx != (prev_idx + 1) % host_buffer_num)
            d_c++;
        prev_idx = multiple_buffer_idx;*/
    }
    filledSlots.release_all();
    return nullptr;
}

void *DVS::multiple_buf_bin_writer()
{
    char *bin_name = NULL;
    int multiple_buffer_idx = 0;
    // int prev_idx = 0;

    time_t current_time;
    struct tm *local_time;
    std::cout << "File Write////////////////////////////////////////////"
              << std::endl;
    // based on current time, determine the name of output file
    time(&current_time);
    local_time = localtime(&current_time);
    if (asprintf(&bin_name,
                 "./bin_files/data_%04d-%02d-%02d_%02d-%02d-%02d_ID-%d.bin",
                 local_time->tm_year + 1900, local_time->tm_mon + 1,
                 local_time->tm_mday, local_time->tm_hour, local_time->tm_min,
                 local_time->tm_sec, id) == -1)
    {
        perror("Error creating bin file name");
        return nullptr;
    }

    // open file descriptor to write bin file to
    std::ofstream file(bin_name, std::ios::binary | std::ios::app);
    if (!file.is_open())
    {
        perror("Failed to open file for writing.");
        return nullptr;
    }

    while (!quit->load(std::memory_order_acquire))
    {

        filledSlots.acquire();
#if (CHECK_MULTIPLE_BUF_BIN_WRITER)
        CallGap _gap("MULTIPLE_BUF_BIN_WRITER");
        ScopeTimer _tim("MULTIPLE_BUF_BIN_WRITER");
#endif
        auto g = locks->acquire(multiple_buffer_idx);
        file.write(host_buffer[multiple_buffer_idx], frame_bytes);

        multiple_buffer_idx = (multiple_buffer_idx + 1) % host_buffer_num;
        freeSlots.release();
        /*if (multiple_buffer_idx != (prev_idx + 1) % host_buffer_num)
            d_c++;
        prev_idx = multiple_buffer_idx;*/
    }
    freeSlots.release_all();
    file.close();
    free(bin_name);
    return nullptr;
}

void *DVS::multiple_buf_png_writer()
{
    // 1) 세션 디렉토리 준비
    const std::string out_base = "./bin_files";
    std::filesystem::create_directories(out_base);
    auto now0 = std::chrono::system_clock::now();
    auto tt0 = std::chrono::system_clock::to_time_t(now0);
    std::tm tm0{};
    localtime_r(&tt0, &tm0);

    std::ostringstream diross;
    diross << out_base << '/'
           << "data_" << std::put_time(&tm0, "%Y-%m-%d_%H-%M-%S")
           << "_ID-" << id;
    const std::string session_dir = diross.str();
    std::filesystem::create_directories(session_dir);

    // 2) TaskManager 생성 (코어 리스트는 config.hpp의 매크로 사용)
    std::vector<int> cores = {CV_WORKER_CORES_LIST};
    auto manager = OpenCVTaskManager::createFixed(cores);
    const int numWorkers = CV_WORKER_CORES_NUM;

    int multiple_buffer_idx = 0;
    int frame_count = 0;

    while (!quit->load(std::memory_order_acquire))
    {
        filledSlots.acquire();
        auto guard = locks->acquire(multiple_buffer_idx);

        // 1) PCIE 버퍼에서 raw frame pointer 획득
        char *frame_start = host_buffer[multiple_buffer_idx] + header_bytes;
        size_t byte_count = DVS_FRAME_H * DVS_FRAME_W * 2 / 8;

        // 2) packed data를 Mat으로 래핑
        cv::Mat packed(1, static_cast<int>(byte_count), CV_8UC1, frame_start);
        // → shared_ptr로 감싸야 안전합니다
        auto packed_sp = std::make_shared<cv::Mat>(packed);

        // 3) 출력할 gray Mat은 워커 내부에서 slice로 생성하므로 여기서는 생략

        // 4) 파일명 준비
        std::string path = session_dir + '/' + std::to_string(frame_count) + ".png";

        // 5) Part별 MapAndSavePNG 작업 enqueue
        for (int part = 0; part < numWorkers; ++part)
        {
            manager->enqueueTask(OpenCVTask{
                OpenCVTaskType::MapAndSavePNG,
                packed_sp, // shared_ptr로 넘기기
                nullptr,   // output Mat 은 내부 처리
                path,      // base filename
                part,      // this worker’s part_id
                numWorkers // total parts
            });
        }

        ++frame_count;
        multiple_buffer_idx = (multiple_buffer_idx + 1) % host_buffer_num;
        freeSlots.release();
    }
    freeSlots.release_all();
    return nullptr;
}

void *DVS::multiple_buf_dat_writer()
{
    // 1) 세션 디렉터리 준비
    const std::string out_base = "./bin_files";
    std::filesystem::create_directories(out_base);
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream ss;
    ss << out_base << '/' << "data_"
       << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
       << "_ID-" << id;
    const std::string session_dir = ss.str();
    std::filesystem::create_directories(session_dir);

    // 2) 멀타코어 고정 풀 생성 (config.hpp 에 CV_WORKER_CORES_LIST, CV_WORKER_CORES_NUM)
    std::vector<int> cores = {CV_WORKER_CORES_LIST};
    CompressionTaskPool pool(cores);

    int multiple_buffer_idx = 0;
    int frame_block_idx = 0;
    const size_t byte_count = DVS_FRAME_H * DVS_FRAME_W * 2 / 8 + header_bytes;

    while (!quit->load(std::memory_order_acquire))
    {
        filledSlots.acquire(CV_WORKER_CORES_NUM);
        {
            std::vector<IndexedMutexPool::Guard> guards;
            guards.reserve(CV_WORKER_CORES_NUM);
            for (int i = 0; i < CV_WORKER_CORES_NUM; ++i)
            {
                int idx = (multiple_buffer_idx + i) % host_buffer_num;
                guards.emplace_back(locks->acquire(idx));
            }

            // 3) 코어 개수만큼 시작 포인터 배열 준비
            std::array<const uint8_t *, CV_WORKER_CORES_NUM> ptrs;
            for (int i = 0; i < CV_WORKER_CORES_NUM; ++i)
            {
                int idx = (multiple_buffer_idx + i) % host_buffer_num;
                ptrs[i] = reinterpret_cast<const uint8_t *>(
                    host_buffer[idx]);
            }

            // 4) 각 프레임을 개별 파일로 논블로킹 enqueue
            for (int i = 0; i < CV_WORKER_CORES_NUM; ++i)
            {

#if (!STORE_EVERY_FRAME)
                // 헤더의 최상위 비트 체크
                const uint8_t *p = ptrs[i];

                bool should_save = (static_cast<unsigned char>(p[7]) & 0x80u) != 0;
                if (!should_save)
                    continue; // 이 프레임은 바로 드랍
#endif

                int global_index = frame_block_idx * CV_WORKER_CORES_NUM + i;
                std::ostringstream fn;
                fn << session_dir << '/'
                   << global_index << ".dat";

                pool.enqueue(
                    ptrs[i],    // data pointer
                    byte_count, // length
                    fn.str()    // filename
                );
            }
        }

        ++frame_block_idx;
        multiple_buffer_idx = (multiple_buffer_idx + CV_WORKER_CORES_NUM) % host_buffer_num;
        freeSlots.release(CV_WORKER_CORES_NUM);
    }
    // 함수 종료 시 pool 소멸자가 자동으로 모든 작업 완료 대기
    freeSlots.release_all();
    return nullptr;
}

void DVS::mbuf_DnW_pcie_reader()
{
    int prev_frame_num;
    uint32_t prev_timestamp;
    bool check_init = 0;
    int error_num = 0;
    int multiple_buffer_idx = 0;
    int frame_num;
    uint32_t timestamp;
    char *dvs_buffer;
#if (FRAME_HEADER_BYTES == 16)
    uint64_t sensor_cfg_index;
#endif

    if (posix_memalign((void **)&dvs_buffer, 4096, frame_bytes) != 0)
    {
        perror("posix_memalign failed");
    }

    printf("\nstarting save...\n");
    init_rd_ptr();
    // while reading raw sensor data, check frame num consistency too
    while (!quit->load(std::memory_order_acquire))
    {
        freeSlots.acquire();
#if (CHECK_mbuf_DnW_pcie_reader)
        CallGap _gap("mbuf_DnW_pcie_reader");
        ScopeTimer _tim("mbuf_DnW_pcie_reader");
#endif

        {
            auto g = locks->acquire(multiple_buffer_idx);
            dvs_buffer = host_buffer[multiple_buffer_idx];

            // read raw data
            read_frame(dvs_buffer);

            // check frame num consistency
#if (FRAME_HEADER_BYTES == 16)
            decode_header(dvs_buffer, sensor_cfg_index, frame_num, timestamp);
#else
            decode_header(dvs_buffer, frame_num, timestamp);
#endif
            int frame_num_gap = (frame_num - prev_frame_num) > 0 ? (frame_num - prev_frame_num) : (frame_num - prev_frame_num) + 256;
            if (frame_num_gap != fpt)
            {
                if (check_init)
                {

                    error_num++;
                    std::cout << "==========================================================="
                                 "=========================================="
                              << std::endl;
                    std::cout << "(w) ERROR NUM: " << std::dec << error_num << std::endl;
                    std::cout << "prev_timestamp: " << std::dec << prev_timestamp
                              << ", prev_frame_num: " << prev_frame_num << std::endl;
                    std::cout << "timestamp: " << std::dec << timestamp
                              << ", frame_num: " << frame_num << std::endl;
                    std::cout << "==========================================================="
                                 "=========================================="
                              << std::endl;
                }
            }

            prev_frame_num = frame_num;
            prev_timestamp = timestamp;

            multiple_buffer_idx = (multiple_buffer_idx + 1) % host_buffer_num;
            filledSlots.release();
            check_init = 1;
        }
        if (*terminate)
            break;
    }
    filledSlots.release_all();
}

void DVS::mbuf_DnW_display_render(bool is_flip)
{
    std::cout << "TEST MULT4" << std::endl;

    double fps = 0.0;
    int display_fps = 0;
    int frameCount = 0;
    double startTime = cv::getTickCount();
    double end = cv::getTickCount();
    double elapsedTime = (end - startTime) / cv::getTickFrequency();
    int multiple_buffer_idx = 0;

    // initialize cv::Mat frame
    const cv::Size sz(frame_w, frame_h);
    cv::Mat base(sz, CV_8UC1, cv::Scalar(128));
    // reset frame
    base.copyTo(frame);

    std::string winname = "DVS camera " + std::to_string(id);

    // no accum
    // display_downsample_num = display_downsample_num * accum_num;
    // accum_num = 1;

    unsigned int display_downsample_count = 0;
    unsigned int accum_count = 0;
    while (!quit->load(std::memory_order_acquire))
    {
        filledSlots.acquire();
#if (CHECK_mbuf_DnW_display_render)
        CallGap _gap("mbuf_DnW_display_render");
        ScopeTimer _tim("mbuf_DnW_display_render");
#endif

        if (display_downsample_count--) // display_downsample_count > 0, display_downsample_count - 1
        {
            multiple_buffer_idx = (multiple_buffer_idx + 1) % host_buffer_num;
            displayedSlots.release();
        }
        else
        {
            display_downsample_count = (display_downsample_num - 1);
            {
                auto g = locks->acquire(multiple_buffer_idx);
                frame_start = host_buffer[multiple_buffer_idx] + header_bytes;
                convert2BitTo8Bit_accum();
                multiple_buffer_idx = (multiple_buffer_idx + 1) % host_buffer_num;

                displayedSlots.release();
            }
            if (accum_count-- == 0)
            {
                accum_count = (accum_num - 1);
                // show image
                if (is_flip)
                {
                    cv::flip(frame, frame, 0);
                }
                calc_fps(fps, display_fps, frameCount, startTime, frame);

                cv::imshow(winname, frame);

                if (*terminate || cv::waitKey(1) == 27)
                {
                    *terminate = true;
                    // cleanup
                    frame.release();
                    break;
                }

                // reset frame
                base.copyTo(frame);
            }
        }
    }
    displayedSlots.release_all();
}

void *DVS::mbuf_DnW_dat_writer()
{ // 1) 세션 디렉터리 준비
    const std::string out_base = "./bin_files";
    std::filesystem::create_directories(out_base);
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream ss;
    ss << out_base << '/' << "data_"
       << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
       << "_ID-" << id;
    const std::string session_dir = ss.str();
    std::filesystem::create_directories(session_dir);

    // 2) 멀타코어 고정 풀 생성 (config.hpp 에 CV_WORKER_CORES_LIST, CV_WORKER_CORES_NUM)
    std::vector<int> cores = {CV_WORKER_CORES_LIST};
    CompressionTaskPool pool(cores);

    int multiple_buffer_idx = 0;
    int frame_block_idx = 0;
    const size_t byte_count = DVS_FRAME_H * DVS_FRAME_W * 2 / 8 + header_bytes;

    while (!quit->load(std::memory_order_acquire))
    {
        displayedSlots.acquire(CV_WORKER_CORES_NUM);
        {

            std::vector<IndexedMutexPool::Guard> guards;
            guards.reserve(CV_WORKER_CORES_NUM);
            for (int i = 0; i < CV_WORKER_CORES_NUM; ++i)
            {
                int idx = (multiple_buffer_idx + i) % host_buffer_num;
                guards.emplace_back(locks->acquire(idx));
            }

            // 3) 코어 개수만큼 시작 포인터 배열 준비
            std::array<const uint8_t *, CV_WORKER_CORES_NUM> ptrs;
            for (int i = 0; i < CV_WORKER_CORES_NUM; ++i)
            {
                int idx = (multiple_buffer_idx + i) % host_buffer_num;
                ptrs[i] = reinterpret_cast<const uint8_t *>(
                    host_buffer[idx]);
            }

            // 4) 각 프레임을 개별 파일로 논블로킹 enqueue
            for (int i = 0; i < CV_WORKER_CORES_NUM; ++i)
            {

#if (!STORE_EVERY_FRAME)
                // 헤더의 최상위 비트 체크
                const uint8_t *p = ptrs[i];

                bool should_save = (static_cast<unsigned char>(p[7]) & 0x80u) != 0;
                if (!should_save)
                    continue; // 이 프레임은 바로 드랍
#endif

                int global_index = frame_block_idx * CV_WORKER_CORES_NUM + i;
                std::ostringstream fn;
                fn << session_dir << '/'
                   << global_index << ".dat";

                pool.enqueue(
                    ptrs[i],    // data pointer
                    byte_count, // length
                    fn.str()    // filename
                );
            }
        }

        ++frame_block_idx;
        multiple_buffer_idx = (multiple_buffer_idx + CV_WORKER_CORES_NUM) % host_buffer_num;
        freeSlots.release(CV_WORKER_CORES_NUM);
        if (*terminate)
            break;
    }
    // 함수 종료 시 pool 소멸자가 자동으로 모든 작업 완료 대기
    freeSlots.release_all();
    return nullptr;
}

void DVS::bin_to_vid(char *path_to_bin, char *output_vid_name)
{
    std::ifstream bin_file(path_to_bin, std::ios::binary);

    if (!bin_file)
    {
        std::cerr << "Error opening binary file!" << std::endl;
        return;
    }
    // std::ofstream vid_file(output_vid_name, std::ios_binary | std::ios_app);
    //  if (!vid_file.is_open())
    // {
    //     perror("Failed to open file for writing.");
    //     return nullptr;
    // }
    cv::VideoWriter videoWriter(output_vid_name,
                                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
                                cv::Size(frame_w, frame_h), false);
    while (bin_file.read(buffer, frame_bytes))
    {
        std::streamsize bytesRead = bin_file.gcount(); // Get actual bytes read

        if (bytesRead < frame_bytes)
        {
            if (bin_file.eof())
            {
                std::cout << "Reached end of file. Read only " << bytesRead
                          << " bytes.\n";
            }
            else if (bin_file.fail() || bin_file.bad())
            {
                std::cerr << "File read error occurred!\n";
            }
        }
        else
        {
            // std::cout << "Read " << bytesRead << " bytes successfully.\n";
            frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
            convert2BitTo8Bit();

            // If the format is BGR but needs conversion
            // cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

            videoWriter.write(frame);
        }
    }
    bin_file.close();
    videoWriter.release();
}

void DVS::bin_to_png(char *path_to_bin, char *output_folder_name)
{
    std::ifstream bin_file(path_to_bin, std::ios::binary);

    if (!bin_file)
    {
        std::cerr << "Error opening binary file!" << std::endl;
        return;
    }

    // int frame_count = 0; // Counter for frame naming

    // while (bin_file.read(buffer, frame_bytes))
    // {
    //     std::streamsize bytesRead = bin_file.gcount(); // Get actual bytes read
    //     if (bytesRead < frame_bytes)
    //     {
    //         if (bin_file.eof())
    //         {
    //             std::cout << "Reached end of file. Read only " << bytesRead <<
    //             " bytes.\n";
    //         }
    //         else if (bin_file.fail() || bin_file.bad())
    //         {
    //             std::cerr << "File read error occurred!\n";
    //         }
    //     }
    //     else
    //     {
    //         // Initialize frame and convert data
    //         frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
    //         convert2BitTo8Bit();

    //         // Generate filename for the PNG image
    //         std::ostringstream filename;
    //         filename << output_folder_name << "/frame_" << std::setw(5) <<
    //         std::setfill('0') << frame_count << ".png";

    //         // Save the frame as a PNG image
    //         cv::imwrite(filename.str(), frame);

    //         frame_count++; // Increment frame count
    //     }
    // }
    // bin_file.close();

    int frame_count = 0; // Counter for frame naming
    while (!bin_file.eof())
    {
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            bin_file.read(buffer, frame_bytes);
            std::streamsize bytesRead = bin_file.gcount(); // get actual bytes read

            // frame read error
            if (bytesRead < frame_bytes)
            {
                if (bin_file.eof())
                {
                    std::cout << "Reached end of file. Read only " << bytesRead
                              << " bytes.\n";
                }
                else if (bin_file.fail() || bin_file.bad())
                {
                    std::cerr << "File read error occurred!\n";
                }
            }

            // accumulate
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }
        }

        cv::flip(frame, frame, 0);

        // Generate filename for the PNG image
        std::ostringstream filename;
        filename << output_folder_name << "/frame_" << std::setw(5)
                 << std::setfill('0') << frame_count << ".png";

        // Save the frame as a PNG image
        cv::imwrite(filename.str(), frame);
        frame_count++; // Increment frame count
    }
}

void DVS::dat_to_png(char *input_folder_name, char *output_folder_name)
{
    DatToPngConverter converter;
    converter.convertFolder(
        input_folder_name, // .dat 파일 폴더
        output_folder_name // PNG 출력 폴더
    );
}

void DVS::dvs_roi_average_based(int img_show, int is_update, bool is_flip,
                                bool print_latency)
{
    float algorithm_avg = 0.0, frame_read_avg = 0.0;
    int frame_cnt = 0;
    std::chrono::high_resolution_clock::time_point algorithm_start, algorithm_end,
        frame_read_start, frame_read_end;
    std::chrono::duration<double, std::milli> algorithm_elapsed,
        frame_read_elapsed;
    while (true)
    {
        if (print_latency)
        {
            frame_read_start = std::chrono::high_resolution_clock::now();
        }
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
        // buffers to count the number of events per column and row
        int *x_count = (int *)calloc(frame_w, sizeof(int));
        int *y_count = (int *)calloc(frame_h, sizeof(int));

        // total number of events accumulated throughout several frames
        int sum = 0;

        // stack several frames and count their number of events
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            read_frame(buffer);
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }
            if (print_latency)
            {
                algorithm_start = std::chrono::high_resolution_clock::now();
            }
            sum += roi_count_average(x_count, y_count, is_flip);
            if (print_latency)
            {
                algorithm_end = std::chrono::high_resolution_clock::now();
                algorithm_elapsed = algorithm_end - algorithm_start;
                algorithm_avg += algorithm_elapsed.count();
            }
        }
        Bbox b_box_dvs, b_box_cis;
        if (print_latency)
        {
            algorithm_start = std::chrono::high_resolution_clock::now();
        }
        // calculate event ROI in the form of a bounding box
        int is_roi =
            roi_alg_average_based(x_count, y_count, sum, &b_box_dvs, &b_box_cis);
        if (print_latency)
        {
            algorithm_end = std::chrono::high_resolution_clock::now();
            algorithm_elapsed = algorithm_end - algorithm_start;
            algorithm_avg += algorithm_elapsed.count();
        }
        // cleanup
        free(x_count);
        free(y_count);
        x_count = NULL;
        y_count = NULL;

        // acquire mutex
        thread_mutex->lock_single_writer();

        // if there are enough events to draw a ROI
        // write bounding box information in a shared struct variable
        if (is_roi)
        {
            bbox->lx = b_box_cis.lx;
            bbox->ly = b_box_cis.ly;
            bbox->hx = b_box_cis.hx;
            bbox->hy = b_box_cis.hy;
        }
        else if (!is_update)
        {
            bbox->lx = -1;
            bbox->ly = -1;
            bbox->hx = -1;
            bbox->hy = -1;
        }
        thread_mutex->unlock_single_writer(1); // is_roi || is_update);

        // display DVS video and ROI if img_show == 1
        display_mutex.lock_display();
        if (img_show)
        {
            if (is_flip)
            {
                cv::flip(frame, frame, 0);
            }
            cv::Point p1(b_box_dvs.lx, b_box_dvs.ly);
            cv::Point p2(b_box_dvs.hx, b_box_dvs.hy);
            cv::rectangle(frame, p1, p2, cv::Scalar(255), 2, cv::LINE_8);
            cv::imshow("DVS camera", frame);
        }

        // if ESC pressed, exit...
        // or some other thread detects ESC press, exit.
        if (*terminate || cv::waitKey(1) == 27)
        {
            *terminate = true;
            // wake up any waiting threads
            thread_mutex->terminate();
            // cleanup
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        frame.release();
        display_mutex.unlock_display();
        if (print_latency)
        {
            frame_read_end = std::chrono::high_resolution_clock::now();
            frame_read_elapsed = frame_read_end - frame_read_start;
            frame_read_avg += frame_read_elapsed.count();
            if (frame_cnt == 99)
            {
                frame_cnt = 0;
                std::cout << "Frame read latency : " << frame_read_avg / 100.0
                          << ", Algorithm latency : " << algorithm_avg / 100.0 << "ms"
                          << std::endl;
                algorithm_avg = 0.0;
                frame_read_avg = 0.0;
            }
            else
            {
                frame_cnt++;
            }
        }
    }
}

void DVS::send_frame(cv::Mat *dest_frame, bool is_flip)
{
    frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
    // stack several frames and count their number of events
    for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
    {
        read_frame(buffer);
        if (frame_grp_num == 0)
        {
            convert2BitToBR();
        }
        else
        {
            convert2BitToBR_accum();
        }
    }
    if (is_flip)
    {
        cv::flip(frame, frame, 0);
    }
    thread_mutex->lock_single_writer();
    // clone image to send to CIs
    *dest_frame = frame.clone();
    thread_mutex->unlock_single_writer(1);
}

int DVS::roi_count_average(int *x_count, int *y_count, bool is_flip)
{
    int sum = 0;
    for (int h = 0; h < frame_h; h++)
    {
        for (int byteIndex = 0; byteIndex < (frame_w >> 2); ++byteIndex)
        {
            uint8_t prev_pixel =
                (h == 0) ? 0 : (frame_start[(h - 1) * (frame_w >> 2) + byteIndex]);
            uint8_t cur_pixel = frame_start[h * (frame_w >> 2) + byteIndex];
            uint8_t pixel_count_val =
                pixel_count(prev_pixel) + pixel_count(cur_pixel);
            if (pixel_count_val >= 2)
            {
                for (int bitOffset = 0; bitOffset < 8; bitOffset += 2)
                {
                    uint8_t pixel = (cur_pixel >> bitOffset) & 0x03; // Extract 2 bits
                    if (pixel != 0)
                    {
                        // count events, events per column, and events per row
                        sum++;
                        x_count[(byteIndex << 2) | (bitOffset >> 1)]++;
                        if (is_flip)
                        {
                            y_count[frame_h - 1 - h]++;
                        }
                        else
                        {
                            y_count[h]++;
                        }
                    }
                }
            }
        }
    }
    return sum;
}

int DVS::roi_alg_average_based(int *x_count, int *y_count, int sum,
                               Bbox *b_box_dvs, Bbox *b_box_cis)
{
    // check columns with number of events above average
    int x_avg = sum / frame_w;
    // to reduce noise, set minimum of events to 5
    if (x_avg < 5)
        x_avg = 5;
    int x_thresh_count = 0;
    int x_min = 0, x_max = 0;
    for (int w = 0; w < frame_w; w++)
    {
        if (x_count[w] > x_avg)
        {
            x_thresh_count++;
            // check consecutive <roi_height_min_threshold> num of columns with number
            // of events above average
            if (x_thresh_count >= roi_height_min_threshold)
            {
                if (x_min == 0)
                {
                    // set as ROI left boundary
                    x_min = w - roi_height_min_threshold + 1;
                    x_max = w;
                }
                else if (w > x_max)
                {
                    // set as ROI right boundary
                    x_max = w;
                }
            }
        }
        else
        {
            // if no consecutive <roi_height_min_threshold> columns exist, reset count
            x_thresh_count = 0;
        }
    }
    // check rows with number of events above average
    int y_avg = sum / frame_h;
    // to reduce noise, set minimum of events to 5
    if (y_avg < 5)
        y_avg = 5;
    int y_thresh_count = 0;
    int y_min = 0, y_max = 0;
    for (int h = 0; h < frame_h; h++)
    {
        if (y_count[h] > y_avg)
        {
            y_thresh_count++;
            // check consecutive <roi_height_min_threshold> num of rows with number of
            // events above average
            if (y_thresh_count >= roi_height_min_threshold)
            {
                if (y_min == 0)
                {
                    // set as ROI top boundary
                    y_min = h - roi_height_min_threshold + 1;
                    y_max = h;
                }
                else if (h > y_max)
                {
                    // set as ROI bottom boundary
                    y_max = h;
                }
            }
        }
        else
        {
            // if no consecutive <roi_height_min_threshold> rows exist, reset count
            y_thresh_count = 0;
        }
    }
    if (x_min != 0 && y_min != 0)
    {
        // if valid ROI is detected
        draw_square_roi(b_box_dvs, x_min, y_min, x_max, y_max, frame_w, frame_h);
        if (convert_cis)
        {
            // convert DVS coordinates to CIS
            x_min = (int)(cis_x_scale * x_min) + cis_x_offset;
            y_min = (int)(cis_y_scale * y_min) + cis_y_offset;
            x_max = (int)(cis_x_scale * x_max) + cis_x_offset;
            y_max = (int)(cis_y_scale * y_max) + cis_y_offset;
            // draw ROI for CIS
            draw_square_roi(b_box_cis, x_min, y_min, x_max, y_max, cis_frame_w,
                            cis_frame_h);
        }
        // return if valid ROI detected
        return 1;
    }
    else
    {
        return 0;
    }
}
void DVS::set_CIS(float x_scale, float y_scale, float x_offset, float y_offset,
                  int frame_w, int frame_h, int roi_event_score_,
                  int row_score_threshold_, int roi_height_min_threshold_,
                  int roi_min_size_, float roi_inflation_ratio_)
{
    // detect ROI for CIS instead of DVS
    convert_cis = true;

    // using the following parameters :
    cis_x_scale = x_scale;
    cis_y_scale = y_scale;
    cis_x_offset = (int)(x_offset * frame_w);
    cis_y_offset = (int)(y_offset * frame_h);
    cis_frame_w = frame_w;
    cis_frame_h = frame_h;
    roi_event_score = roi_event_score_;
    row_score_threshold = row_score_threshold_;
    roi_height_min_threshold = roi_height_min_threshold_;
    roi_min_size = roi_min_size_;
    roi_inflation_ratio = roi_inflation_ratio_;
}
void DVS::set_DVS_ROI(int roi_event_score_, int row_score_threshold_,
                      int roi_height_min_threshold_, int roi_min_size_,
                      float roi_inflation_ratio_)
{
    // detect ROI for DVS
    roi_event_score = roi_event_score_;
    row_score_threshold = row_score_threshold_;
    roi_height_min_threshold = roi_height_min_threshold_;
    roi_min_size = roi_min_size_;
    roi_inflation_ratio = roi_inflation_ratio_;
}
void DVS::draw_square_roi(Bbox *b_box, int x_min, int y_min, int x_max,
                          int y_max, int width, int height)
{
    // draw square ROI around given x and y coordinate ranges
    // while making sure ROI doesn't exceed the entire frame

    // move coordinates to inside the frame
    if (x_min < 0)
        x_min = 0;
    if (y_min < 0)
        y_min = 0;
    if (x_max >= width)
        x_max = width - 1;
    if (y_max >= height)
        y_max = height - 1;

    // determine square ROI size
    int inflated_size = (int)(roi_inflation_ratio * (x_max - x_min));
    if (inflated_size > height)
    {
        inflated_size = height;
    }
    else if (inflated_size < roi_min_size)
    {
        inflated_size = roi_min_size;
    }

    int inflated_size_y = (int)(roi_inflation_ratio * (y_max - y_min));
    if (inflated_size_y > height)
    {
        inflated_size_y = height;
    }
    else if (inflated_size_y < roi_min_size)
    {
        inflated_size_y = roi_min_size;
    }

    // give margin around x_max and x_min to determine ROI position
    int min_offset = (inflated_size - (x_max - x_min)) >> 1;

    // if ROI exceeds frame, move it inside
    if (x_min - min_offset < 0)
    {
        b_box->lx = 0;
        b_box->hx = inflated_size - 1;
    }
    else if (x_min + inflated_size - min_offset >= width)
    {
        b_box->hx = width;
        b_box->lx = width - inflated_size + 1;
    }
    else
    {
        b_box->lx = x_min - min_offset;
        b_box->hx = x_min - min_offset + inflated_size - 1;
    }

    // give margin around y_max and y_min to determine ROI position
    int min_offset_y = (inflated_size_y - (y_max - y_min)) >> 1;

    // if ROI exceeds frame, move it inside
    if (y_min - min_offset_y < 0)
    {
        b_box->ly = 0;
        b_box->hy = inflated_size_y - 1;
    }
    else if (y_min + inflated_size_y - min_offset_y >= height)
    {
        b_box->hy = height;
        b_box->ly = height - inflated_size_y + 1;
    }
    else
    {
        b_box->ly = y_min - min_offset_y;
        b_box->hy = y_min - min_offset_y + inflated_size_y - 1;
    }
}

int DVS::pixel_count(uint8_t x)
{
    // calculate how many 2-bit events happen inside 8-bit word
    int cnt = 0;
    for (int i = 0; i < 4; i++)
    {
        if (x && 0x03)
            cnt++;
        x = x >> 2;
    }
    return cnt;
}
void DVS::convert2BitTo8Bit_count(bool is_flip)
{
    for (int h = 0; h < frame_h; h++)
    {
        for (int byteIndex = 0; byteIndex < (frame_w >> 2); ++byteIndex)
        {
            // apply a simple spatial filter before written to frame.
            // 8-bit word that includes current pixel U 8-bit word 1 row before.
            // if these 8 pixels contain less than 2 pixels, current pixel not written
            // to frame.
            uint8_t prev_pixel =
                (h == 0) ? 0 : (frame_start[(h - 1) * (frame_w >> 2) + byteIndex]);
            uint8_t cur_pixel = frame_start[h * (frame_w >> 2) + byteIndex];
            uint8_t pixel_count_val =
                pixel_count(prev_pixel) + pixel_count(cur_pixel);

            for (int bitOffset = 0; bitOffset < 8; bitOffset += 2)
            {
                uint8_t pixel = (cur_pixel >> bitOffset) & 0x03; // Extract 2 bits
                int frame_idx;
                // if DVS flipped, write to frame upside down
                if (is_flip)
                {
                    frame_idx =
                        (frame_h - h - 1) * frame_w + (byteIndex << 2) + (bitOffset >> 1);
                }
                else
                {
                    frame_idx = h * frame_w + (byteIndex << 2) + (bitOffset >> 1);
                }
                // write gray pixel values
                if (pixel_count_val >= 2 && pixel == 1)
                {
                    frame.data[frame_idx] = 224;
                }
                else if (pixel_count_val >= 2 && pixel == 2)
                {
                    frame.data[frame_idx] = 32;
                }
                else
                {
                    frame.data[frame_idx] = 128;
                }
            }
        }
    }
}

void DVS::convert2BitTo8Bit_count_accum(bool is_flip)
{
    for (int h = 0; h < frame_h; h++)
    {
        for (int byteIndex = 0; byteIndex < (frame_w >> 2); ++byteIndex)
        {
            // apply a simple spatial filter before written to frame.
            // 8-bit word that includes current pixel U 8-bit word 1 row before.
            // if these 8 pixels contain less than 2 pixels, current pixel not written
            // to frame.
            uint8_t prev_pixel =
                (h == 0) ? 0 : (frame_start[(h - 1) * (frame_w >> 2) + byteIndex]);
            uint8_t cur_pixel = frame_start[h * (frame_w >> 2) + byteIndex];
            uint8_t pixel_count_val =
                pixel_count(prev_pixel) + pixel_count(cur_pixel);
            if (pixel_count_val >= 2)
            {
                for (int bitOffset = 0; bitOffset < 8; bitOffset += 2)
                {
                    uint8_t pixel = (cur_pixel >> bitOffset) & 0x03; // Extract 2 bits
                    int frame_idx;
                    // if DVS flipped, write to frame upside down
                    if (is_flip)
                    {
                        frame_idx = (frame_h - h - 1) * frame_w + (byteIndex << 2) +
                                    (bitOffset >> 1);
                    }
                    else
                    {
                        frame_idx = h * frame_w + (byteIndex << 2) + (bitOffset >> 1);
                    }
                    // accumulate to gray pixels (functionality not used for now)
                    if (frame.data[frame_idx] > 128)
                    {
                        if (pixel != 0)
                            frame.data[frame_idx] += 1;
                    }
                    else if (frame.data[frame_idx] < 128)
                    {
                        if (pixel != 0)
                            frame.data[frame_idx] -= 1;
                    }
                    // stack frames, overwriting pixels with no events
                    else if (pixel == 1)
                    {
                        frame.data[frame_idx] = 224;
                    }
                    else if (pixel == 2)
                    {
                        frame.data[frame_idx] = 32;
                    }
                }
            }
        }
    }
}
int DVS::roi_alg_proposed(Bbox *dvs, Bbox *cis)
{
    int row_start_idx = 0;
    // final roi values
    int global_x_min = frame_w, global_x_max = 0;
    int global_y_min = 0, global_y_max = -1;
    // height-wise line width threshold apply
    int roi_row_streak = 0;
    // calculate x-direction min, max for a few rows
    int candidate_x_min, candidate_x_max;
    candidate_x_min = frame_w;
    candidate_x_max = 0;
    // calculate max sum of consecutive partial sequence
    // where some values are -1 and others are roi_event_score(currently 5)
    for (int h = 0; h < frame_h; h++)
    {
        int local_max_score = 0, local_cur_score = 0;
        int local_max_left = frame_w, local_max_right = 0;
        int local_cur_score_left = 0;
        // incremental algorithm
        for (int w = 0; w < frame_w; w++)
        {
            // cur_val : current score of sequence element
            int cur_val;
            int pix_val = frame.data[row_start_idx + w];
            if (pix_val > 128)
            {
                cur_val =
                    roi_event_score; // 2+width_count;//((pix_val-128)>>2)+width_count;
            }
            else if (pix_val < 128)
            {
                cur_val =
                    roi_event_score; // 2+width_count;//((128-pix_val)>>2)+width_count;
            }
            else
            {
                cur_val = -1;
            }
            // local_cur_score : max value of partial sequence whose right end is w
            local_cur_score = local_cur_score + cur_val;
            // if local_cur_score < 0, 0(no elements in partial sequence) is larger,
            // empty sequence corresponding to local_cur_score
            if (local_cur_score < 0)
            {
                local_cur_score = 0;
                // set the left end of partial sequence to current sequence element
                local_cur_score_left = w;
            }
            // record maximum partial consecutive sequence
            // and its left & right ends
            if (local_cur_score > local_max_score)
            {
                local_max_score = local_cur_score;
                local_max_left = local_cur_score_left;
                local_max_right = w;
            }
        }
        // printf("row %d min %d max %d local_max_score = %d\n", h,local_max_left,
        // local_max_right, local_max_score ); if max score exceeds threshold
        if (local_max_score >= row_score_threshold)
        {
            roi_row_streak++;
            // union max & min with few adjacent rows
            if (local_max_left < candidate_x_min)
                candidate_x_min = local_max_left;
            if (local_max_right > candidate_x_max)
                candidate_x_max = local_max_right;
            // if consecutive rows' max score exceed row_score_threshold
            // modify final ROI
            if (roi_row_streak >= roi_height_min_threshold)
            {
                if (global_y_min == 0)
                    global_y_min = h - roi_height_min_threshold + 1;
                global_y_max = h;
                if (candidate_x_min < global_x_min)
                    global_x_min = candidate_x_min;
                if (candidate_x_max > global_x_max)
                    global_x_max = candidate_x_max;
            }
        }
        else
        {
            // if consecutive rows don't exist
            // initialize max, min and row count
            candidate_x_min = frame_w;
            candidate_x_max = 0;
            roi_row_streak = 0;
        }
        row_start_idx += frame_w;
    }
    if (global_y_max != -1)
    {
        // if valid ROI is detected
        //  printf("%d %d %d %d\n", global_x_min, global_y_min, global_x_max,
        //  global_y_max);
        // draw dvs ROI
        draw_square_roi(dvs, global_x_min, global_y_min, global_x_max, global_y_max,
                        frame_w, frame_h);
        if (convert_cis)
        {
            // convert DVS coordinates to CIS
            int x_min = (int)(cis_x_scale * global_x_min) + cis_x_offset;
            int y_min = (int)(cis_y_scale * global_y_min) + cis_y_offset;
            int x_max = (int)(cis_x_scale * global_x_max) + cis_x_offset;
            int y_max = (int)(cis_y_scale * global_y_max) + cis_y_offset;
            // draw ROI for CIS
            draw_square_roi(cis, x_min, y_min, x_max, y_max, cis_frame_w,
                            cis_frame_h);
        }
        // return if valid ROI detected
        return 1;
    }
    else
    {
        return 0;
    }
}
void DVS::dvs_roi_proposed(int img_show, int is_update, bool is_flip,
                           bool print_latency)
{
    float algorithm_avg = 0.0, frame_read_avg = 0.0;
    int frame_cnt = 0;
    std::chrono::high_resolution_clock::time_point algorithm_start, algorithm_end,
        frame_read_start, frame_read_end;
    std::chrono::duration<double, std::milli> algorithm_elapsed,
        frame_read_elapsed;
    while (true)
    {
        if (print_latency)
        {
            frame_read_start = std::chrono::high_resolution_clock::now();
        }
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            read_frame(buffer);
            // generate DVS frame
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit_count(is_flip);
            }
            else
            {
                convert2BitTo8Bit_count_accum(is_flip);
            }
        }
        Bbox b_box_dvs, b_box_cis;

        if (print_latency)
        {
            algorithm_start = std::chrono::high_resolution_clock::now();
        }
        // calculate event ROI in the form of a bounding box
        int is_roi = roi_alg_proposed(&b_box_dvs, &b_box_cis);
        if (print_latency)
        {
            algorithm_end = std::chrono::high_resolution_clock::now();
            algorithm_elapsed = algorithm_end - algorithm_start;
            algorithm_avg += algorithm_elapsed.count();
        }
        // acquire mutex
        thread_mutex->lock_single_writer();

        // if there are enough events to draw a ROI
        // write bounding box information in a shared struct variable
        if (is_roi)
        {
            bbox->lx = b_box_cis.lx;
            bbox->ly = b_box_cis.ly;
            bbox->hx = b_box_cis.hx;
            bbox->hy = b_box_cis.hy;
            // printf("%d %d %d %d\n", bbox->lx, bbox->ly, bbox->hx, bbox->hy);
        }
        else if (!is_update)
        {
            bbox->lx = -1;
            bbox->ly = -1;
            bbox->hx = -1;
            bbox->hy = -1;
        }
        thread_mutex->unlock_single_writer(1); // is_roi || is_update);

        // display DVS video and ROI if img_show == 1
        display_mutex.lock_display();
        if (img_show)
        {
            if (is_roi)
            {
                cv::Point p1(b_box_dvs.lx, b_box_dvs.ly);
                cv::Point p2(b_box_dvs.hx, b_box_dvs.hy);
                cv::rectangle(frame, p1, p2, cv::Scalar(255), 2, cv::LINE_8);
            }
            cv::imshow("DVS camera", frame);
        }

        // if ESC pressed, exit...
        // or some other thread detects ESC press, exit.
        if (*terminate || cv::waitKey(1) == 27)
        {
            *terminate = true;
            // wake up any waiting threads
            thread_mutex->terminate();
            // cleanup
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        frame.release();
        display_mutex.unlock_display();
        if (print_latency)
        {
            frame_read_end = std::chrono::high_resolution_clock::now();
            frame_read_elapsed = frame_read_end - frame_read_start;
            frame_read_avg += frame_read_elapsed.count();
            if (frame_cnt == 99)
            {
                frame_cnt = 0;
                std::cout << "Frame read latency : " << frame_read_avg / 100.0
                          << ", Algorithm latency : " << algorithm_avg / 100.0 << "ms"
                          << std::endl;
                algorithm_avg = 0.0;
                frame_read_avg = 0.0;
            }
            else
            {
                frame_cnt++;
            }
        }
    }
}

void DVS::key_input()
{
    unsigned int free_cnt, filled_cnt;
    while (!quit->load(std::memory_order_acquire))
    {
        /*int key = cv::waitKey(1);
        if (key == 27)
        {
            quit->store(true, std::memory_order_release);
            freeSlots.release_all();
            filledSlots.release_all();
            break;
        }
        else if (key == 'a' || key == 'A')
        {
            printf("a key pressed\n");
            unsigned int free_cnt, filled_cnt;
            free_cnt = freeSlots.get_count();
            filled_cnt = filledSlots.get_count();
            printf("free: %d, filled = %d\n", free_cnt, filled_cnt);
        }*/

        free_cnt = freeSlots.get_count();
        filled_cnt = filledSlots.get_count();
        printf("=========================================================\n");
        printf("free: %d, filled = %d\n", free_cnt, filled_cnt);

        std::this_thread::sleep_for(std::chrono::minutes(1));
    }
}

DVS::~DVS()
{
    if (init_mode == MODE_MULT)
    {
        free(buffer_addr);
        free(buffer_rdy);
        free(is_buffer_rdy);
        free(buffer_rdy_addr);
        free(buffer_done);
        free(buffer_flush);
        free(buffer_commit_index);

        for (int i = 0; i < host_buffer_num; i++)
        {
            free(host_buffer[i]);
            //	delete mbuf_mutex[i];
        }

        delete terminate;
        std::cout << "===terminate(1)===" << std::endl;
    }
    else
    {
        if (double_buffer != NULL)
        {
            free(double_buffer);
            delete dbuf_mutex[0];
            delete dbuf_mutex[1];
            if (terminate != NULL)
            {
                delete terminate;
            }
        }
        free(buffer);
        free(buffer_rdy);
        free(buffer_done);
        std::cout << "===terminate(2)===" << std::endl;
    }
}
