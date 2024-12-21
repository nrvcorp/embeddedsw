#ifndef DVS_HPP
#define DVS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <dirent.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <fstream>

#include <condition_variable>
#include "PCIe.hpp"
#include "MutexManager.hpp"
#include "bbox.hpp"
class DVS
{
private:
    // PCIe connection
    PCIe pcie;

    // Frame cfg
    int frame_h, frame_w;
    int pixel_num;
    int frame_bytes;
    bool is_header;
    int header_bytes;
    int accum_num;

    // Frame data, ctrl
    cv::Mat frame;
    char *buffer;
    char *double_buffer;
    char *frame_start;
    char *buffer_rdy;
    char *buffer_done;

    uintptr_t rdy_baseaddr;
    uintptr_t frame_baseaddr;
    int buffer_num;
    uintptr_t *buffer_rdy_addr;
    uintptr_t *buffer_addr;
    int rd_ptr;
    MutexManager &display_mutex;
    MutexManager *thread_mutex;
    int roi_thresh;
    int roi_min_size;
    float roi_inflation_ratio;
    Bbox *bbox;
    bool *terminate;
    MutexManager* dbuf_mutex[2];

public:
    DVS(
        int frame_h, int frame_w,
        bool is_header, int accum_num,
        uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
        int buffer_num,
        const char *c2h_dev, const char *h2c_dev,
        MutexManager &display_mutex, int double_buffering = 0);
    // DVS(
    //     int frame_h, int frame_w,
    //     bool is_header, int accum_num,
    //     uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    //     int buffer_num,
    //     const char *c2h_dev, const char *h2c_dev,
    //     MutexManager &display_mutex,
    //     MutexManager *thread_mutex);
    DVS(
        int frame_h, int frame_w,
        bool is_header, int accum_num,
        uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
        int buffer_num,
        const char *c2h_dev, const char *h2c_dev,
        MutexManager &display_mutex, int roi_thresh, int roi_min_size,
        float roi_inflation_ratio, Bbox *bbox, MutexManager *thread_mutex = NULL,
        bool *terminate = NULL);

    void convert2BitTo8Bit();
    void convert2BitTo8Bit_accum();
    void convert2BitToBGR_accum();
    void decode_header(
        const char *buffer, int &frame_num, uint32_t &timestamp);

    void display_stream();
    void read_frame();
    void check_frame_drop();
    void *double_buf_reader();
    void *double_buf_bin_writer();
    void crop_coord(int img_show);
    int event_accum(int *x_count, int *y_count, int width, int height);
    int event_roi(int *x_count, int *y_count, int width, int height, int sum, Bbox *b_box);

    ~DVS();
};

#endif // DVS_HPP
