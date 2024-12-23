#ifndef NPU_HPP
#define NPU_HPP
#include "image.h"
#include "MutexManager.hpp"
#include "bbox.hpp"
#include "network.h"
#include "parser.h"

/**
 * @param im original CIS image
 * @param sized square cropped and resized CIS image
 * @param dets detection result information
 * @param nboxes number of bounding boxes
 */
typedef struct
{
    image im;
    image sized;
    detection *dets;
    int nboxes;
} frame_data;

class NPU
{
private:
    // darknet struct network
    network net;
    // darknet inference parameters
    float demo_thresh;
    int letter_box;
    char **demo_names;
    image **demo_alphabet;
    int demo_classes;
    int demo_ext_output;
    layer l;
    // mutex for opencv display
    MutexManager &display_mutex;
    // mutex for NPU input preprocessing
    MutexManager *pre_mutex;
    // mutex for running NPU
    MutexManager *run_mutex;
    // mutex for NPU output postprocessing
    MutexManager *post_mutex;
    // pointer to multithreading termination flag
    bool *terminate;
    // frame count shared between threads
    int frameCount;
    int skipFrameCount;
    // starting time shared between threads
    double startTime;
    // FPS
    double fps;
    double skipFps;
    // device
    char *npu_h2c_fname;
    int npu_h2c_fd;

public:
    /**
     * Constructor
     *
     * @param cfgfile config file for darknet network
     * @param weightfile original weightfile for network model (not sure if used)
     * @param letter_box_in darknet parameter
     * @param benchmark_layers darknet parameter
     * @param demo_thresh darknet parameter
     * @param demo_names class names?
     * @param demo_alphabet darknet parameter
     * @param demo_classes num of classes
     * @param demo_ext_output darknet parameter
     * @param display_mutex mutex for opencv display
     * @param terminate pointer to shared terminate flag for terminating multithreading properly
     */
    NPU(
        char *cfgfile, char *weightfile, int letter_box_in, int benchmark_layers, float demo_thresh, char **demo_names,
        image **demo_alphabet, int demo_classes, int demo_ext_output, MutexManager &display_mutex, bool *terminate);
    /**
     * crops CIS_frame using bounding box input, resizes it to match NPU input size, and changes format to struct frame_data.
     *
     * @param[out] frame struct frame_data output
     * @param bbox_cis pointer holding bounding box information passed from dvs to cis
     * @param CIS_frame input frame read from CIS object
     * @param is_update give false to not run img through NPU
     */
    void preprocess(frame_data &frame, Bbox *bbox_cis, cv::Mat *CIS_frame, bool is_update = true);
    /**
     * runs the NPU, and converts the resulting bounding boxes to CIS frame viewpoint, to before cropping and resizing happens.
     *
     * @param frame holds original CIS frame data and its resized version
     * @param bbox pointer holding bounding box information passed from dvs to cis
     * @param is_update give false to not run img through NPU
     */
    void run_NPU(frame_data &frame, Bbox *bbox, bool is_update = true);

    /**
     * postprocesses bounding boxes, and displays it on the original CIS opencv video.
     *
     * @param frame holds original CIS frame data and its resized version
     * @param bbox pointer holding bounding box information passed from dvs to cis
     * @param inv_frame_w 1.0/(float)(CIS frame width)
     * @param inv_frame_h 1.0/(float)(CIS frame height)
     * @param is_update give false to not run img through NPU
     */
    int postprocess(frame_data &frame, Bbox *bbox, float inv_frame_w, float inv_frame_h, bool is_update = true, bool show_dvs_view = true);
    ~NPU();
};
#endif