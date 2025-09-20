#include "CIS.hpp"

// constructor
CIS::CIS(
    int frame_h, int frame_w,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &mutexManager,
    MutexCond *mutexCond,
    Bbox *bbox) : frame_h(frame_h), frame_w(frame_w),
                  rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
                  buffer_num(buffer_num),
                  pcie(c2h_dev, h2c_dev),
                  rd_ptr(0),
                  mutexManager(mutexManager),
                  mutexCond(mutexCond),
                  bbox(bbox)
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
}

CIS::CIS(
    int frame_h, int frame_w,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &mutexManager) : frame_h(frame_h), frame_w(frame_w),
                                  rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
                                  buffer_num(buffer_num),
                                  pcie(c2h_dev, h2c_dev),
                                  rd_ptr(0),
                                  mutexManager(mutexManager),
                                  mutexCond(NULL),
                                  bbox(NULL)
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
}

void CIS::display_stream()
{
    while (true)
    {
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
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

        mutexManager.lock_display();
        cv::imshow("CIS camera", frame);
        if (cv::waitKey(1) == 27)
        {
            frame.release();
            mutexManager.unlock_display();
            break;
        }
        mutexManager.unlock_display();

        rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
    }
}

void CIS::crop_dvs_roi(float x_scale, float y_scale, int x_offset, int y_offset)
{
    while (true)
    {
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
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

        mutexCond->lock_multiple_reader();
        Bbox bbox_cis;
        bbox_cis.lx = (int)(x_scale * bbox->lx) + x_offset;
        bbox_cis.ly = (int)(y_scale * bbox->ly) + y_offset;
        bbox_cis.hx = (int)(x_scale * bbox->hx) + x_offset;
        bbox_cis.hy = (int)(y_scale * bbox->hy) + y_offset;
        mutexCond->unlock_multiple_reader();

        cv::Point p1(bbox_cis.lx, bbox_cis.ly);
        cv::Point p2(bbox_cis.hx, bbox_cis.hy);
        cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        cv::Point DVS_UL(x_offset, y_offset);
        cv::Point DVS_LR((int)(x_scale * 960) + x_offset,
                         (int)(y_scale * 720) + y_offset);
        cv::rectangle(frame, DVS_UL, DVS_LR, cv::Scalar(0, 255, 0), 2, cv::LINE_8);

        mutexManager.lock_display();
        cv::imshow("CIS camera", frame);
        if (cv::waitKey(1) == 27)
        {
            frame.release();
            mutexManager.unlock_display();
            break;
        }
        mutexManager.unlock_display();

        rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
    }
}

CIS::~CIS()
{
    free(buffer_rdy);
    free(buffer_done);
}