#ifndef CIS_HPP
#define CIS_HPP

#include <opencv2/opencv.hpp>
#include "MutexManager.hpp"
#include "PCIe.hpp"
#include "bbox.hpp"

class CIS
{
private:
    // PCIe connection
    PCIe pcie;

    // Frame cfg
    int frame_h, frame_w;
    int frame_bytes;

    // Frame data, ctrl
    cv::Mat frame;
    uintptr_t rdy_baseaddr;
    uintptr_t frame_baseaddr;
    int buffer_num;
    uintptr_t *buffer_rdy_addr;
    uintptr_t *buffer_addr;
    int rd_ptr;
    char *buffer_rdy;
    char *buffer_done;

    MutexManager &mutexManager;
    MutexCond *mutexCond;
    Bbox *bbox;

public:
    CIS(
        int frame_h, int frame_w,
        uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
        int buffer_num,
        const char *c2h_dev, const char *h2c_dev,
        MutexManager &mutexManager);
    CIS(
        int frame_h, int frame_w,
        uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
        int buffer_num,
        const char *c2h_dev, const char *h2c_dev,
        MutexManager &mutexManager, MutexCond *mutexCond, Bbox *bbox);
    void display_stream();
    void crop_dvs_roi(float x_scale, float y_scale, int x_offset, int y_offset);
    ~CIS();
};

#endif // CIS_HPP