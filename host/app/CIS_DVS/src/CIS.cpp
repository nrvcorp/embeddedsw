#include "CIS.hpp"

// constructor
CIS::CIS(
    int frame_h, int frame_w,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &display_mutex,
    MutexManager *thread_mutex,
    Bbox *bbox, bool *terminate) : frame_h(frame_h), frame_w(frame_w),
                                   rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
                                   buffer_num(buffer_num),
                                   pcie(c2h_dev, h2c_dev),
                                   rd_ptr(0),
                                   display_mutex(display_mutex),
                                   thread_mutex(thread_mutex),
                                   bbox(bbox),
                                   terminate(terminate)
{
    // set total frame bytes
    frame_bytes = frame_h * frame_w * 3;

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

    // initialize mutex for PCIE access
    pcie_mutex = new MutexManager();

    // set mutex cond ready, so that one thread can initially acquire the mutex
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
    // set total frame bytes
    frame_bytes = frame_h * frame_w * 3;

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

    // initialize mutex for PCIE access
    pcie_mutex = new MutexManager();
    // set mutex cond ready, so that one thread can initially acquire the mutex
    pcie_mutex->setReady(1);
}
int CIS::get_frame_h() { return frame_h; }
int CIS::get_frame_w() { return frame_w; }

void CIS::read_frame(cv::Mat &frame)
{
    // acquire mutex
    pcie_mutex->lock_pipeline();

    // wait for ready flag
    // by polling through PCIE connection
    while (true)
    {
        pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);
        if ((buffer_rdy[0] & 0x01) == 1)
        {
            break;
        }
    }

    // read CIS frame through PCIE
    pcie.c2h((char *)frame.data, frame_bytes, buffer_addr[rd_ptr]);

    // set flag to DONE through PCIE
    pcie.h2c(buffer_done, 1, buffer_rdy_addr[rd_ptr]);

    // change the address for ready flag and DVS frame
    rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;

    // release mutex
    pcie_mutex->unlock_pipeline();
}
void CIS::set_DVS(float x_scale_, float y_scale_, float x_offset_, float y_offset_)
{
    // set DVS parameters relative to CIS
    // to show DVS view range on top of CIS video stream
    x_offset = (int)(x_offset_ * frame_w);
    y_offset = (int)(y_offset_ * frame_h);
    x_scale = x_scale_;
    y_scale = y_scale_;
}

bool CIS::scale_bbox(Bbox *b_box)
{
    // acquire mutex to obtain bounding box info from DVS object
    thread_mutex->lock_multiple_reader();
    bool retval;
    // if ESC is pressed in some thread, keep setting cond_ready to 1 so that all listeners can wake and receive the broadcast from DVS thread
    if (*terminate)
        thread_mutex->setReady(1);

    // receive bbox information
    if (bbox->lx >= 0 && bbox->ly >= 0 && bbox->hx - bbox->lx >= 100 && bbox->hy - bbox->ly >= 100 && bbox->hx - bbox->lx <= frame_w && bbox->hy - bbox->ly <= frame_h)
    {
        b_box->lx = bbox->lx;
        b_box->ly = bbox->ly;
        b_box->hx = bbox->hx;
        b_box->hy = bbox->hy;
        retval = true;
    }
    else
    {
        retval = false;
    }
    thread_mutex->unlock_multiple_reader();
    return retval;
}

void CIS::calc_fps(double &fps, int &frameCount, double &startTime, cv::Mat &frame)
{
    frameCount++;
    double elapsedTime = (cv::getTickCount() - startTime) / cv::getTickFrequency();
    if (elapsedTime >= 1.0)
    {
        fps = frameCount / elapsedTime;
        frameCount = 0;
        startTime = cv::getTickCount();
    }

    std::ostringstream oss;
    oss << "FPS: " << static_cast<int>(fps);
    cv::putText(frame, oss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
}
void CIS::background_subtraction()
{
    cv::Ptr<cv::BackgroundSubtractor> MOG2_subtractor = cv::createBackgroundSubtractorMOG2(true);
    cv::Ptr<cv::BackgroundSubtractor> bg_subtractor = MOG2_subtractor;

    cv::Mat frame, foreground_mask, threshold_img, dilated;
    frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
    while (true)
    {
        read_frame(frame);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

        // Apply the background subtractor to get the foreground mask
        bg_subtractor->apply(frame, foreground_mask);

        // Apply threshold to create a binary image
        cv::threshold(foreground_mask, threshold_img, 120, 255, cv::THRESH_BINARY);

        // Dilate the threshold image to thicken the regions of interest
        cv::dilate(threshold_img, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 2);

        // Find contours in the dilated image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw bounding boxes for contours that exceed a certain area threshold
        for (size_t i = 0; i < contours.size(); i++)
        {
            if (cv::contourArea(contours[i]) > 150)
            {
                cv::Rect bounding_box = cv::boundingRect(contours[i]);
                cv::rectangle(frame, bounding_box, cv::Scalar(255, 255, 0), 2);
            }
        }
        display_mutex.lock_display();

        // Show the different outputs
        cv::imshow("Subtractor", foreground_mask);
        cv::imshow("Threshold", threshold_img);
        cv::imshow("Detection", frame);

        // exit if ESC is pressed
        if (cv::waitKey(1) == 27)
        {
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        display_mutex.unlock_display();
    }
}

void CIS::display_stream()
{
    double fps = 0.0;
    int frameCount = 0;
    double startTime = cv::getTickCount();
    frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
    while (true)
    {
        // read frame through PCIE
        read_frame(frame);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        calc_fps(fps, frameCount, startTime, frame);

        display_mutex.lock_display();

        // display frame
        cv::imshow("CIS camera", frame);

        // exit if ESC is pressed
        if (cv::waitKey(1) == 27)
        {
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        display_mutex.unlock_display();
    }
}

void CIS::crop_dvs_roi()
{
    frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
    while (true)
    {
        // read frame
        read_frame(frame);

        // receive bbox information
        Bbox bbox_cis;
        scale_bbox(&bbox_cis);

        // display bounding box as a rectangle
        cv::Point p1(bbox_cis.lx, bbox_cis.ly);
        cv::Point p2(bbox_cis.hx, bbox_cis.hy);
        cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), 2, cv::LINE_8);

        // display DVS view range
        cv::Point DVS_UL((x_offset < 0 ? 0 : x_offset),
                         (y_offset < 0 ? 0 : y_offset));
        int temp_x = (int)(x_scale * 960) + x_offset;
        int temp_y = (int)(y_scale * 720) + y_offset;
        cv::Point DVS_LR((temp_x >= frame_w ? (frame_w - 1) : temp_x),
                         (temp_y >= frame_h ? (frame_h - 1) : temp_y));
        cv::rectangle(frame, DVS_UL, DVS_LR, cv::Scalar(0, 255, 0), 2, cv::LINE_8);

        // display frame
        display_mutex.lock_display();
        cv::imshow("CIS camera", frame);

        // terminate if ESC pressed or some other thread terminates
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
void CIS::resize_dvs(cv::Mat *dvs_frame, cv::Mat *dvs_resize, int width, int height)
{
    thread_mutex->lock_multiple_reader();
    cv::resize(*dvs_frame, *dvs_resize, cv::Size(width, height));
    thread_mutex->unlock_multiple_reader();
}
void CIS::set_roi(cv::Rect &dvs_rect, cv::Rect &cis_rect, int &dvs_width, int &dvs_height)
{

    // calculate overlayed region in CIS frame
    int xmin = x_offset < 0 ? 0 : x_offset;
    int ymin = y_offset < 0 ? 0 : y_offset;
    int temp_x = (int)(x_scale * 960) + x_offset;
    int temp_y = (int)(y_scale * 720) + y_offset;
    int xmax = temp_x >= frame_w ? (frame_w - 1) : (temp_x);
    int ymax = temp_y >= frame_h ? (frame_h - 1) : (temp_y);
    // output resized target sizes for DVS
    dvs_width = (int)(x_scale * 960);
    dvs_height = (int)(y_scale * 720);
    // calculate the same region from resized DVS frame's perspective
    int dvs_crop_xmin = x_offset < 0 ? (-x_offset) : 0;
    int dvs_crop_ymin = y_offset < 0 ? (-y_offset) : 0;
    int dvs_crop_xmax = temp_x >= frame_w ? (dvs_width + frame_w - 1 - temp_x) : (dvs_width);
    int dvs_crop_ymax = temp_y >= frame_h ? (dvs_height + frame_h - 1 - temp_y) : (dvs_height);
    // store the results in cv::Rect object
    cv::Point dvs_crop_p1(dvs_crop_xmin, dvs_crop_ymin);
    cv::Point dvs_crop_p2(dvs_crop_xmax, dvs_crop_ymax);
    cv::Point cis_crop_p1(xmin, ymin);
    cv::Point cis_crop_p2(xmax, ymax);
    dvs_rect = cv::Rect(dvs_crop_p1, dvs_crop_p2);
    cis_rect = cv::Rect(cis_crop_p1, cis_crop_p2);
}
bool CIS::overlay_dvs(cv::Mat *dvs_frame, cv::Rect &dvs_rect, cv::Rect &cis_rect, int dvs_width, int dvs_height, float alpha, int numRegions)
{

    frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
    // read frame
    read_frame(frame);
    // receive DVS frmae
    cv::Mat dvs_resize;
    resize_dvs(dvs_frame, &dvs_resize, dvs_width, dvs_height);
    // crop DVS frame to fit in CIS frame
    cv::Mat dvs_crop = (dvs_resize)(dvs_rect);
    // transform DVS from grayscale to BGR
    //  cv::Mat dvs_bgr;
    //  cv::cvtColor(dvs_crop, dvs_bgr, cv::COLOR_GRAY2BGR);
    // obtain cropped CIS frame
    cv::Mat cis_crop = (frame)(cis_rect);
    // overlay CIS and DVS
    cv::addWeighted(cis_crop, 1 - alpha, dvs_crop, alpha, 0.0, cis_crop);
    // draw grid for calibration
    draw_grid(frame, numRegions);
    // display frame
    display_mutex.lock_display();
    cv::imshow("CIS camera", frame);

    // terminate if ESC pressed or some other thread terminates
    if (*terminate || cv::waitKey(1) == 27)
    {
        *terminate = 1;
        frame.release();
        display_mutex.unlock_display();
        return true;
    }
    display_mutex.unlock_display();
    return false;
}
void CIS::draw_grid(cv::Mat &image, int numRegions)
{

    // Calculate the step size for grid lines
    int stepX = frame_w / numRegions;
    int stepY = frame_h / numRegions;

    // Draw vertical grid lines
    for (int i = 1; i < numRegions; ++i)
    {
        int x = i * stepX;
        cv::line(image, cv::Point(x, 0), cv::Point(x, frame_h), cv::Scalar(0, 0, 255), 1); // Red lines
    }

    // Draw horizontal grid lines
    for (int i = 1; i < numRegions; ++i)
    {
        int y = i * stepY;
        cv::line(image, cv::Point(0, y), cv::Point(frame_w, y), cv::Scalar(0, 0, 255), 1); // Red lines
    }
}

CIS::~CIS()
{

    delete pcie_mutex;
    free(buffer_rdy);
    free(buffer_done);
}