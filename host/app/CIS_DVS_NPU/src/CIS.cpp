#include "CIS.hpp"

// constructor
CIS::CIS(
    int frame_h, int frame_w,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &display_mutex,
    MutexManager *thread_mutex,
    Bbox *bbox, bool* terminate) : frame_h(frame_h), frame_w(frame_w),
                  rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
                  buffer_num(buffer_num),
                  pcie(c2h_dev, h2c_dev),
                  rd_ptr(0),
                  display_mutex(display_mutex),
                  thread_mutex(thread_mutex),
                  bbox(bbox),
                  terminate(terminate)
{
    frame_bytes = frame_h * frame_w * 3;

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
    pcie_mutex = new MutexManager();
    pcie_mutex->setReady(1);
}

CIS::CIS(
    int frame_h, int frame_w,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &display_mutex) : frame_h(frame_h), frame_w(frame_w),
                                  rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
                                  buffer_num(buffer_num),
                                  pcie(c2h_dev, h2c_dev),
                                  rd_ptr(0),
                                  display_mutex(display_mutex),
                                  thread_mutex(NULL),
                                  bbox(NULL),
                                  terminate(NULL)
{

    frame_bytes = frame_h * frame_w * 3;

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
    pcie_mutex = new MutexManager();
    pcie_mutex->setReady(1);
}
int CIS::get_frame_h() { return frame_h; }
int CIS::get_frame_w() { return frame_w; }

void CIS::read_frame(cv::Mat &frame)
{
    pcie_mutex->lock_pipeline();
    while (true)
    {
        pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);
        if ((buffer_rdy[0] & 0x01) == 1)
        {
            break;
        }
    }

    pcie.c2h((char *)frame.data, frame_bytes, buffer_addr[rd_ptr]);
    pcie.h2c(buffer_done, 1, buffer_rdy_addr[rd_ptr]);
    rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
    pcie_mutex->unlock_pipeline();
}
void CIS::scale_bbox(Bbox *b_box, float x_scale, float y_scale, int x_offset, int y_offset)
{
    thread_mutex->lock_multiple_reader();
    if(*terminate) thread_mutex->setReady(1);
    b_box->lx = (int)(x_scale * bbox->lx) + x_offset;
    b_box->ly = (int)(y_scale * bbox->ly) + y_offset;
    b_box->hx = (int)(x_scale * bbox->hx) + x_offset;
    b_box->hy = (int)(y_scale * bbox->hy) + y_offset;
    thread_mutex->unlock_multiple_reader();
}
void CIS::display_stream()
{
    frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
    while (true)
    {
        read_frame(frame);
        display_mutex.lock_display();
        cv::imshow("CIS camera", frame);
        if (cv::waitKey(1) == 27)
        {
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        display_mutex.unlock_display();
    }
}

void CIS::crop_dvs_roi(float x_scale, float y_scale, int x_offset, int y_offset)
{
    frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
    while (true)
    {
        read_frame(frame);
        
        Bbox bbox_cis;
        scale_bbox(&bbox_cis, x_scale, y_scale, x_offset, y_offset);
        cv::Point p1(bbox_cis.lx, bbox_cis.ly);
        cv::Point p2(bbox_cis.hx, bbox_cis.hy);
        cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        cv::Point DVS_UL(x_offset, y_offset);
        cv::Point DVS_LR((int)(x_scale * 960) + x_offset,
                         (int)(y_scale * 720) + y_offset);
        cv::rectangle(frame, DVS_UL, DVS_LR, cv::Scalar(0, 255, 0), 2, cv::LINE_8);

        display_mutex.lock_display();
        cv::imshow("CIS camera", frame);
        if (*terminate || cv::waitKey(1) == 27)
        {
            *terminate = 1;
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        display_mutex.unlock_display();
    }
}

CIS::~CIS()
{

    delete pcie_mutex;
    free(buffer_rdy);
    free(buffer_done);
}