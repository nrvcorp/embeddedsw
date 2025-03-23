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
#include <chrono>

#include <condition_variable>
#include "PCIe.hpp"
#include "MutexManager.hpp"
#include "bbox.hpp"

/**
 * Function to set thread priority to highest.
 * @param t : thread alias to set priority to
 */
void setThreadPriority(std::thread &t);

// class to manage DVS object
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
    int display_downsample_num;

    // Frame data
    // raw sensor data buffer
    char *buffer;
    // double buffering raw sensor data buffer
    char *double_buffer;
    // frame for opencv display
    cv::Mat frame;
    // pointer to skip header and go to actual sensor data
    char *frame_start;

    // on-ZCU106 buffer address management
    int buffer_num;
    uintptr_t rdy_baseaddr;
    uintptr_t frame_baseaddr;
    char *buffer_rdy;
    char *buffer_done;
    uintptr_t *buffer_rdy_addr;
    uintptr_t *buffer_addr;
    // controls which on-ZCU106 frame buffer to access
    int rd_ptr;

    // mutex for opencv display
    MutexManager &display_mutex;

    // mutex for DVS-CIS multithreading
    MutexManager *thread_mutex;

    // parameters for DVS ROI calculation
    int roi_event_score;
    int roi_min_score;
    int roi_line_width;
    int roi_min_size;
    float roi_inflation_ratio;

    // if true, calculate ROI bbox from CIS viewpoint
    bool convert_cis;

    // bounding box shared memory
    Bbox *bbox;

    // pointer to multithreading termination flag
    bool *terminate;

    // mutex for write-to-bin file double buffering
    MutexManager *dbuf_mutex[2];

    // DVS to CIS relative frame size scale (0~1)
    float cis_x_scale;
    float cis_y_scale;

    // DVS frame offset inside CIS frame (in pixels )
    int cis_x_offset;
    int cis_y_offset;

    // CIS frame size
    int cis_frame_w;
    int cis_frame_h;

public:
    /**
     * Constructor for normal DVS application and write-to-bin double buffering
     * @param frame_h height of DVS frame
     * @param frame_w width of DVS frame
     * @param is_header true if frame num and timestamp are prepended
     * @param accum_num number of stacked images for opencv display
     * @param rdy_baseaddr (ZCU106 MMap IO) ready flag array base address
     * @param frame_baseaddr (ZCU106 MMap IO) base address of multiple frame buffers
     * @param buffer_num (ZCU106 MMap IO) number of multiple frame buffers
     * @param c2h_dev ("/dev/xdma_dvs0_c2h_0") c2h port alias of xdma driver
     * @param h2c_dev ("/dev/xdma_dvs0_h2c_0") h2c port alias of xdma driver
     * @param display_mutex mutex object for opencv display
     * @param double_buffering true if double buffering is enabled for write-to-bin purposes
     */
    DVS(
        int frame_h, int frame_w,
        bool is_header, int accum_num,
        uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
        int buffer_num,
        const char *c2h_dev, const char *h2c_dev,
        MutexManager &display_mutex, bool double_buffering = false, int display_downsample_num = 1);
    /**
     * constructor for DVS-CIS multithreading and ROI calculation
     * @param frame_h height of DVS frame
     * @param frame_w width of DVS frame
     * @param is_header true if frame num and timestamp are prepended
     * @param accum_num number of stacked images for opencv display
     * @param rdy_baseaddr (ZCU106 MMap IO) ready flag array base address
     * @param frame_baseaddr (ZCU106 MMap IO) base address of multiple frame buffers
     * @param buffer_num (ZCU106 MMap IO) number of multiple frame buffers
     * @param c2h_dev ("/dev/xdma_dvs0_c2h_0") c2h port alias of xdma driver
     * @param h2c_dev ("/dev/xdma_dvs0_h2c_0") h2c port alias of xdma driver
     * @param display_mutex mutex object for opencv display
     * @param bbox bounding box shared memory between CIS and DVS
     * @param thread_mutex mutex object for DVS-CIS multithreading
     * @param terminate pointer to shared terminate flag for terminating multithreading properly
     */
    DVS(
        int frame_h, int frame_w,
        bool is_header, int accum_num,
        uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
        int buffer_num,
        const char *c2h_dev, const char *h2c_dev,
        MutexManager &display_mutex, Bbox *bbox, MutexManager *thread_mutex = NULL,
        bool *terminate = NULL,
        int display_downsample_num = 1);

    /**
     * Function to convert 2-bit image data to 8-bit for opencv display
     */
    void convert2BitTo8Bit();
    /**
     * Function to stack 2-bit image data to 8-bit for opencv display
     */
    void convert2BitTo8Bit_accum();
    /**
     * Function to accumulate 2-bit image data to 8-bit for red and blue opencv display
     */
    void convert2BitToBGR_accum();
    /**
     *function to show events as red & blue on white background for overlay
     */
    void convert2BitToBR();
    /**
     *function to show events as red & blue on white background for overlay
     */
    void convert2BitToBR_accum();
    /**
     * Get frame num and timestamp from frame data
     *
     * prerequisites
     *   > is_header == true
     *   > DVS sensor header enabled
     * @param buffer pointer location to header data
     * @param[out] frame_num frame num sent from DVS sensor
     * @param[out] timestamp timestamp set from DVS sensor
     *
     */
    void decode_header(
        const char *buffer, int &frame_num, uint32_t &timestamp);
    /**
     * set ROI parameters for DVS opencv video
     * for creating a square ROI bounding box inside DVS frame
     * which detects frequent events inside the ROI
     *
     * @param roi_event_score_ score for including event pixel in row-wise sliced ROI
     * @param roi_min_score_ min score for considering particular row as part of final ROI
     * @param roi_line_width_ required number of consecutive rows inside DVS frame with high scores
     * @param roi_min_size_ minimum ROI bounding box size (in pixels)
     * @param roi_inflation_ratio_ enlarge bounding box size to center and zoom out ROI for NPU object detection purposes
     */
    void set_DVS_ROI(int roi_event_score_, int roi_min_score_, int roi_line_width_, int roi_min_size_, float roi_inflation_ratio_);

    /**
     * set ROI parameters and DVS position relative to CIS
     * for DVS-CIS multithreading
     * for creating a square ROI bounding box inside CIS frame
     * which detects frequent events inside the ROI
     *
     * define :
     *   > DVS_view_frame : the box region inside CIS frame corresponding to DVS view range
     *
     * @param x_scale DVS_view_frame width / CIS frame width
     * @param y_scale DVS_view_frame height / CIS frame height
     * @param x_offset (distance between left edges of DVS_view_frame and CIS frame / CIS frame width)
     * @param y_offset (distance between top edges of DVS_view_frame and CIS frame / CIS frame height)
     * @param frame_w CIS frame width in pixels
     * @param frame_h CIS frame height in pixels
     * @param roi_event_score_ score for including event pixel in row-wise sliced ROI
     * @parvoid setThreadPriority(std::thread &t)am roi_min_score_ min score for considering particular row as part of final ROI
     * @param roi_line_width_ required number of consecutive rows inside DVS frame with high scores
     * @param roi_min_size_ minimum ROI bounding box size (in pixels)
     * @param roi_inflation_ratio_ enlarge bounding box size to center and zoom out ROI for NPU object detection purposes
     */
    void set_CIS(float x_scale, float y_scale, float x_offset, float y_offset, int frame_w, int frame_h, int roi_event_score_, int roi_min_score_,
                 int roi_line_width_, int roi_min_size_, float roi_inflation_ratio_);

    /**
     * calculate DVS fps and display on opencv frame
     * @param[out] fps val of FPS, needs to be live variable when calc_fps repeatedly called
     * @param[out] frameCount current frame count, needs to be live variable when calc_fps repeatedly called
     * @param[inout] startTime value of last time measurement inside calc_fps
     * @param[out] frame frame to write FPS text to
     */
    void calc_fps(double &fps, int &frameCount, double &startTime, cv::Mat &frame);

    /**
     * displays DVS frame to opencv video, multithreading-safe
     * @param is_flip horizontal flip image.
     */
    void display_stream(bool is_flip = false);
    /**
     * @brief for use with CIS::save_png_stream
     * Saves DVS frames in PNG format in unison with CIS::save_png_stream saving CIS images at 60Hz.
     * @param output_folder_name path to folder to output DVS png images to
     * @param is_flip horizontal flip image
     */
    void save_png_stream(char *output_folder_name, bool is_flip);
    /**
     * writes sensor data to double buffer
     * skips 1 frame for every 2 frames to match convert2bitto8bit latency
     */
    void double_buf_display_fps_writer();

    /**
     * reads data from double buffer and displays DVS screen.
     * @param is_flip horizontal flip image.
     */
    void double_buf_display_fps_reader(bool is_flip = false);
    /**
     * read frame safely from ZCU106 over PCI express using ready and done flags
     *
     * reads from MMAP-io:
     *   > buffer_rdy
     *   > buffer
     *
     * writes to MMAP-io :
     *   > buffer_done
     * @param dvs_buffer buffer to write sensor data to
     */
    void read_frame(char *dvs_buffer);
    /**
     prints error message to console whenever DVS experiences a frame drop.
     */
    void check_frame_drop();
    /**
     * checks current FPS
     */
    void fps_count();
    /**
     * thread to put DVS sensor data in 2 buffers using double buffering
     */
    void *double_buf_reader();
    /*
     * thread to read from double buffer and write to bin file
     * inside directory ./bin_files
     */
    void *double_buf_bin_writer();
    /**
     * Reconstructs a video file from the bin file stored by DVS_STORE mode (double_buf_reader and double_buf_bin_writer)
     * @param path_to_bin path to input bin file
     * @param output_vid_name path to output video name (mp4)
     */
    void bin_to_vid(char *path_to_bin, char *output_vid_name);
    /**
     * Reconstructs a collection of PNG images from the bin file stored by DVS_STORE mode (double_buf_reader and double_buf_bin_writer)
     * @param path_to_bin path to input bin file
     * @param output_folder_name path to where png files will be stored
     */
    void bin_to_png(char *path_to_bin, char *output_folder_name);
    /**
     * draw a square roi bounding box including coordinates (x_min,y_min), (x_max, y_max)
     * inside frame of size (width, height)
     *
     * @param[out] b_box pointer to shared bounding box struct
     * @param x_min minimum value of x coordinate
     * @param y_min minimum value of y coordinate
     * @param x_max maximum value of x coordinate
     * @param y_max maximum value of y coordinate
     * @param width width of target frame
     * @param height height of target frame
     */
    void draw_square_roi(Bbox *b_box, int x_min, int y_min, int x_max, int y_max, int width, int height);
    /**
     * Calculates and displays ROI bounding box along with DVS opencv video.
     *
     * @param img_show displays opencv video, 0 when used only for multithreading purposes
     * @param is_update if 1, wakes up listener threads only if valid ROI bbox appears
     * @param is_flip if the DVS image is flipped using a mirror.
     * @param print_latency prints average algorithm and per-frame latency to terminal
     */
    void crop_coord(int img_show = 1, int is_update = 0, bool is_flip = false, bool print_latency = false);
    /**
     *sends DVS frame data to dest_frame.
     *@param[out] dest_frame pointer to write DVS frame to
     *@param is_flip send horizontally flipped image to CIS
     */
    void send_frame(cv::Mat *dest_frame, bool is_flip = false);
    /**
     * counts the number of events per frame row and column.
     *
     * @param[out] x_count Container for event counts per column
     * @param[out] y_count Container for event counts per row
     * @return total number of events
     */
    int event_accum(int *x_count, int *y_count, bool is_flip = false);
    /**
     * calculates ROI bounding box for events, given stacked DVS frames
     *
     * @param x_count Container for event counts per column
     * @param y_count Container for event counts per row
     * @param sum total count of events for stacked frames
     * @param[out] b_box_dvs pointer to bounding box for dvs
     * @param[out] b_box_cis pointer to shared bounding box struct
     * @return 1 if ROI exists, 0 otherwise
     */
    int event_roi(int *x_count, int *y_count, int sum, Bbox *b_box_dvs, Bbox *b_box_cis);
    /**
     * counts number of events in 8-bit word
     * @param x input 8bit word
     * @return number of events
     */
    int pixel_count(uint8_t x);
    /**
     * converts input DVS sensor values to 8-bit grayscale frame(accumulation-aware)
     * @param is_flip if the DVS is flipped upside down using a mirror.
     */
    void convert2BitTo8Bit_count(bool is_flip = false);
    /**
     * converts input DVS sensor values to 8-bit grayscale frame(accumulation-aware)
     * @param is_flip if the DVS is flipped upside down using a mirror.
     */
    void convert2BitTo8Bit_count_accum(bool is_flip = false);
    /**
     * incremental algorithm to obtain ROI
     * @param[out] dvs ROI that fits inside DVS frame
     * @param[out] cis ROI that fits inside CIS frame
     * @return if there is detected roi, 1. else, 0.
     */
    int new_ROI(Bbox *dvs, Bbox *cis);
    /**
     * displays cropped ROI for DVS and sends bbox information through shared struct between classes
     * @param img_show whether to show DVS video.
     * @param is_update if 1, wakes up listener threads only if valid ROI bbox appears
     * @param is_flip if the DVS image is flipped using a mirror.
     * @param print_latency prints average algorithm and per-frame latency to terminal
     */
    void crop_new_ROI(int img_show = 1, int is_update = 0, bool is_flip = false, bool print_latency = false);
    ~DVS();
};

#endif // DVS_HPP
