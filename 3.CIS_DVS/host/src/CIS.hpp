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

    // Frame data
    cv::Mat frame;

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

    // mutex for PCIE transaction
    MutexManager *pcie_mutex;
    // mutex for opencv display
    MutexManager &display_mutex;
    // bounding box shared memory
    Bbox *bbox;
    // mutex for multithreading
    MutexManager *thread_mutex;
    // pointer to multithreading termination flag
    bool *terminate;
    // DVS to CIS relative frame size scale (0~1)
    float x_scale;
    float y_scale;
    // DVS frame offset inside CIS frame (in pixels)
    int x_offset;
    int y_offset;

public:
    /**
     * Constructor for non-multithreading purposes, except displaying along with DVS
     *
     * @param frame_h height of CIS frame in pixels
     * @param frame_w width of CIS frame in pixels
     * @param rdy_baseaddr (ZCU106 MMap IO) ready flag array base address
     * @param frame_baseaddr (ZCU106 MMap IO) base address of multiple frame buffers
     * @param buffer_num (ZCU106 MMap IO) number of multiple frame buffers
     * @param c2h_dev ("/dev/xdma_dvs0_c2h_0") c2h port alias of xdma driver
     * @param h2c_dev ("/dev/xdma_dvs0_h2c_0") h2c port alias of xdma driver
     * @param display_mutex mutex object for opencv display
     */
    CIS(
        int frame_h, int frame_w,
        uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
        int buffer_num,
        const char *c2h_dev, const char *h2c_dev,
        MutexManager &display_mutex);
    /**
     * Constructor for multithreading purposes
     *
     * @param frame_h height of CIS frame in pixels
     * @param frame_w width of CIS frame in pixels
     * @param rdy_baseaddr (ZCU106 MMap IO) ready flag array base address
     * @param frame_baseaddr (ZCU106 MMap IO) base address of multiple frame buffers
     * @param buffer_num (ZCU106 MMap IO) number of multiple frame buffers
     * @param c2h_dev ("/dev/xdma_dvs0_c2h_0") c2h port alias of xdma driver
     * @param h2c_dev ("/dev/xdma_dvs0_h2c_0") h2c port alias of xdma driver
     * @param display_mutex mutex object for opencv display
     * @param thread_mutex mutex object for DVS-CIS multithreading
     * @param bbox bounding box shared memory between CIS and DVS
     * @param terminate pointer to shared terminate flag for terminating multithreading properly
     */
    CIS(
        int frame_h, int frame_w,
        uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
        int buffer_num,
        const char *c2h_dev, const char *h2c_dev,
        MutexManager &display_mutex, MutexManager *thread_mutex, Bbox *bbox, bool *terminate);
    /**
     * calculate fps and display on frame
     */
    void calc_fps(double &fps, int &frameCount, double &startTime, cv::Mat &frame);
    /**
     * displays CIS frame to opencv video, multithreading-safe
     */
    void display_stream();
    /**
     @brief for use with DVS::save_png_stream
     * Saves CIS frames in PNG format at 60Hz, in unison with DVS::save_png_sream
     * @param output_folder_name path to folder to output CIS png images to
     */
    void save_png_stream(char *output_folder_name);
    /**
     * read frame safely from ZCU106 over PCI express using ready and done flags
     *
     * reads from MMAP-io:
     *   > buffer_rdy
     *   > buffer
     *
     * writes to MMAP-io :
     *   > buffer_done
     *
     * @param[out] frame frame to store CIS sensor data
     */
    void read_frame(cv::Mat &frame);
    /**
     * get CIS frame height
     * @return frame height
     */
    int get_frame_h();
    /**
     * get CIS frame width
     * @return frame width
     */
    int get_frame_w();
    /**
     * set DVS position relative to CIS
     * for DVS-CIS multithreading
     * to display DVS view range on top of CIS opencv video
     *
     * define :
     *   > DVS_view_frame : the box region inside CIS frame corresponding to DVS view range
     *
     * @param x_scale_ DVS_view_frame width / CIS frame width
     * @param y_scale_ DVS_view_frame height / CIS frame height
     * @param x_offset_ (distance between left edges of DVS_view_frame and CIS frame / CIS frame width)
     * @param y_offset_ (distance between top edges of DVS_view_frame and CIS frame / CIS frame height)
     */
    void set_DVS(float x_scale_, float y_scale_, float x_offset_, float y_offset_);
    /**
     * read shared ROI bounding box from shared memory to a struct pointer bbox.
     * multithreading-safe
     * @param[out] bbox pointer to output struct Bbox
     * @return true if ROI exists, false if otherwise
     */
    bool scale_bbox(Bbox *bbox);
    /**
     * Displays ROI data calculated from DVS object on CIS opencv video.
     *
     * prerequisites:
     *   > run set_DVS(...) to display DVS view frame
     *   > CIS_DVS multithreading
     */
    void crop_dvs_roi();
    /**
     *resizes dvs_frame to dvs_resize.
     *dvs_resize has width x height dimensions
     *@param dvs_frame input frame
     *@param[out] dvs_resize output frame
     *@param width width of dvs_reize
     *@param height height of dvs_resize
     */
    void resize_dvs(cv::Mat *dvs_frame, cv::Mat *dvs_resize, int width, int height);
    /**
     *initializes cv::Rect objects for clipping resized dvs frame to fit CIS frame
     * and cropping CIS frame to obtain region that needs to be overlayed.
     * @param[out] dvs_rect region on resized DVS frame to be cropped and overlayed on CIS frame
     * @param[out] cis_rect region on CIS frame to be cropped and overlayed with dvs_rect region
     * @param[out] dvs_width resizing target width for DVS frame, not width of dvs_rect
     * @param[out] dvs_height resizing target height for DVS frame, not height of dvs_rect
     */
    void set_roi(cv::Rect &dvs_rect, cv::Rect &cis_rect, int &dvs_width, int &dvs_height);
    /**
     *overlays DVS frame on CIS frame.
     *@param dvs_frame DVS frame pointer
     *@param dvs_rect region on resized DVS frame to be cropped and overlayed on CIS frame
     *@param cis_rect region on CIS frame to be cropped and overlayed with dvs_rect region
     *@param dvs_width resizing target width for DVS frame, not width of dvs_rect
     *@param dvs_height resizing target height for DVS frame, not height of dvs_rect
     *@param alpha overlay weight for DVS frame, default = 0.5
     *@param numRegions number of segmented regions for calibration grid, default : 10
     */
    bool overlay_dvs(cv::Mat *dvs_frame, cv::Rect &dvs_rect, cv::Rect &cis_rect, int dvs_width, int dvs_height, float alpha = 0.5, int numRegions = 10);
    /**
     * draws red grid on cv::Mat image.
     *@param image image to draw red grid on
     *@param numRegions number of segmented regions for red grid
     *@return true if esc pressed, false otherwise
     */
    void draw_grid(cv::Mat &image, int numRegions);
    void background_subtraction();
    ~CIS();
};

#endif // CIS_HPP