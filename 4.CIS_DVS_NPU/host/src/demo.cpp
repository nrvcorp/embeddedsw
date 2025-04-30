
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include "darknet.h"

#ifdef WIN32
#include <time.h>
#include "gettimeofday.h"
#else
#include <sys/time.h>
#endif

#ifdef OPENCV

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include "http_stream.h"
#include <pthread.h>
#include <signal.h>

#include <time.h>

#include <condition_variable>
#include <fstream>
using namespace cv;
using namespace std;

extern "C"
{
#include "dma_utils.h"
}
#define QUEUE_SIZE 64 // Number of frames to be processed concurrently

static char **demo_names;
static image **demo_alphabet;
static int demo_classes;

static network net;
static float demo_thresh = 0;
static int demo_ext_output = 0;
static int letter_box = 0;
static int frame_width = 0;
static int frame_height = 0;

static int frame_count = 0;
static double image_fetch_time = 0;
static int image_counter = 0;
static double npu_process_time = 0;

// function to pipeline CIS, DVS synchronization, preprocess, NPU and postprocess.
// There are 4 threads. (excluding stage for synchronization with DVS)
void CIS_NPU_thread(CIS *cis, NPU *npu, bool always_run_npu, bool full_frame_inference)
{
    if (full_frame_inference)
        always_run_npu = true;
    frame_data frame;
    // bounding box information from DVS thread
    Bbox bbox_cis;
    bbox_cis.lx = (cis->get_frame_w() - cis->get_frame_h()) / 2;
    bbox_cis.hx = (cis->get_frame_w() + cis->get_frame_h()) / 2;
    bbox_cis.ly = 0;
    bbox_cis.hy = cis->get_frame_h() - 1;
    bool roi_exist;
    // OpenCV cv::Mat CIS frame
    cv::Mat CIS_frame = cv::Mat::zeros(cis->get_frame_h(), cis->get_frame_w(), CV_8UC3);
    float inv_frame_h = 1.0 / cis->get_frame_h();
    float inv_frame_w = 1.0 / cis->get_frame_w();
    // while loop
    while (1)
    {
        // each stage is protected thru mutexes
        //---------------------------------------------------------------------------------
        // read CIS frame thru PCIE
        cis->read_frame(CIS_frame);
        //----------------------------------------------------------------------------------
        // receive CIS bounding box information from DVS thread
        if (!full_frame_inference)
            roi_exist = cis->scale_bbox(&bbox_cis);
        //----------------------------------------------------------------------------------
        // crop and resize image for loading into NPU
        npu->preprocess(frame, &bbox_cis, &CIS_frame, always_run_npu || roi_exist);
        //----------------------------------------------------------------------------------
        // run NPU on the ZCU106 board
        npu->run_NPU(frame, &bbox_cis, always_run_npu || roi_exist);
        //----------------------------------------------------------------------------------
        // postprocess to obtain and display bounding boxes
        if (npu->postprocess(frame, &bbox_cis, inv_frame_w, inv_frame_h, always_run_npu || roi_exist))
        {
            break;
        }
        //----------------------------------------------------------------------------------
    }
    // free all buffers
    CIS_frame.release();
}

void CIS_only_thread(CIS *cis, NPU *npu)
{

    frame_data frame;
    // bounding box information from DVS thread
    Bbox bbox_cis;
    int offset = (cis->get_frame_w() - cis->get_frame_h()) / 2;
    bbox_cis.lx = offset;
    bbox_cis.hx = offset + (cis->get_frame_h() - 1);
    bbox_cis.ly = 0;
    bbox_cis.hy = cis->get_frame_h() - 1;
    bool roi_exist;
    // OpenCV cv::Mat CIS frame
    cv::Mat CIS_frame = cv::Mat::zeros(cis->get_frame_h(), cis->get_frame_w(), CV_8UC3);
    float inv_frame_h = 1.0 / cis->get_frame_h();
    float inv_frame_w = 1.0 / cis->get_frame_w();
    // while loop
    while (1)
    {
        // each stage is protected thru mutexes
        //---------------------------------------------------------------------------------
        // read CIS frame thru PCIE
        cis->read_frame(CIS_frame);
        //----------------------------------------------------------------------------------
        // crop and resize image for loading into NPU
        npu->preprocess(frame, &bbox_cis, &CIS_frame, true);
        //----------------------------------------------------------------------------------
        // run NPU on the ZCU106 board
        npu->run_NPU(frame, &bbox_cis, true);
        //----------------------------------------------------------------------------------
        // postprocess to obtain and display bounding boxes
        if (npu->postprocess(frame, &bbox_cis, inv_frame_w, inv_frame_h, true, false))
        {
            break;
        }
        //----------------------------------------------------------------------------------
    }
    // free all buffers
    CIS_frame.release();
}

void demo(char *cfgfile, char *weightfile, float thresh, float hier_thresh, int cam_index, const char *filename, char **names, int classes, int avgframes,
          int frame_skip, char *prefix, char *out_filename, int mjpeg_port, int dontdraw_bbox, int json_port, int dont_show, int ext_output, int letter_box_in, int time_limit_sec, char *http_post_host,
          int benchmark, int benchmark_layers, char *json_file_output)
{
    demo_alphabet = load_alphabet();
    // mutex for opencv display
    MutexManager display_mutex;

    // mutex for shared Bbox struct
    MutexManager cis_dvs_mutex;
    Bbox bbox;

    // shared bool variable for terminating in unison
    bool terminate;
    std::vector<std::thread>
        threads;
    printf("Demo\n");
    //-------------------------------------
    // Inference options
    //--------------------------------------
    // always run NPU regardless of ROI detection
    bool always_run_npu = false;
    // always run NPU, at fixed(full) ROI size
    bool full_frame_inference = true;
    //---------------------------------------
    // NPU, CIS and DVS objects
    NPU *npu = new NPU(
        cfgfile, weightfile, letter_box_in, benchmark_layers, 0.5, names,
        demo_alphabet, classes, ext_output,
        display_mutex,
        &terminate);
    CIS *cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, display_mutex, &cis_dvs_mutex, &bbox, &terminate);
    DVS *dvs = NULL;

    // set DVS to CIS relative orientation
    cis->set_DVS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y);

    dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (2000 / 60), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, display_mutex, &bbox, &cis_dvs_mutex, &terminate);
    dvs->set_CIS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y, CIS_FRAME_W, CIS_FRAME_H, ROI_EVENT_SCORE, ROI_MIN_SCORE, ROI_LINE_WIDTH, CIS_ROI_MIN_SIZE, ROI_INFLATION);

    srand(2222222);

    // display window
    create_window_cv("Demo", 0, CIS_FRAME_W, CIS_FRAME_H);

    time_t start_time = time(NULL);

    // launch threads
    if (cis && npu)
    {
        if (dvs)
        {
            for (int i = 0; i < 4; i++)
            {
                threads.emplace_back([cis, npu, always_run_npu, full_frame_inference]()
                                     { CIS_NPU_thread(cis, npu, always_run_npu, full_frame_inference); });
            }
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                threads.emplace_back([cis, npu]()
                                     { CIS_only_thread(cis, npu); });
            }
        }
    }

    if (dvs)
    {
        threads.emplace_back([dvs]()
                             { dvs->crop_new_ROI(1, 1, true); });
    }

    // Wait for all threads to complete
    for (auto &t : threads)
    {
        if (t.joinable())
        {
            t.join();
        }
    }
    printf("Demo finished!\n");

    // cleanup
    delete cis;
    delete dvs;
    cis = NULL;
    dvs = NULL;

    delete npu;
    npu = NULL;
}

#else
void demo(char *cfgfile, char *weightfile, float thresh, float hier_thresh, int cam_index, const char *filename, char **names, int classes, int avgframes,
          int frame_skip, char *prefix, char *out_filename, int mjpeg_port, int dontdraw_bbox, int json_port, int dont_show, int ext_output, int letter_box_in, int time_limit_sec, char *http_post_host,
          int benchmark, int benchmark_layers, char *json_file_output)
{
    fprintf(stderr, "Demo needs OpenCV for webcam images.\n");
}
#endif
