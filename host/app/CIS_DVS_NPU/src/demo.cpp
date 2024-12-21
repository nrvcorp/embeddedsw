
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
void CIS_NPU_thread(CIS *cis, NPU *npu, float x_scale, float y_scale, int x_offset, int y_offset)
{

    frame_data frame;
    // bounding box information from DVS thread
    Bbox bbox_cis;

    // OpenCV cv::Mat CIS frame
    cv::Mat CIS_frame = cv::Mat::zeros(cis->get_frame_h(), cis->get_frame_w(), CV_8UC3);
    float inv_frame_h = 1.0 / cis->get_frame_h();
    float inv_frame_w = 1.0 / cis->get_frame_w();
    // while loop
    while (1)
    {
        //---------------------------------------------------------------------------------
        cis->read_frame(CIS_frame);
        //----------------------------------------------------------------------------------
        cis->scale_bbox(&bbox_cis, x_scale, y_scale, x_offset, y_offset);
        //----------------------------------------------------------------------------------
        npu->preprocess(frame, &bbox_cis, &CIS_frame);

        //----------------------------------------------------------------------------------

        // block contents
        npu->run_NPU(frame, &bbox_cis);

        //----------------------------------------------------------------------------------
        // postprocess code block

        if (npu->postprocess(frame, &bbox_cis, inv_frame_w, inv_frame_h))
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
    MutexManager display_mutex;
    MutexManager cis_dvs_mutex;
    Bbox bbox;
    bool terminate;
    std::vector<std::thread>
        threads;
    printf("Demo\n");
    NPU *npu = new NPU(
        cfgfile, weightfile, letter_box_in, benchmark_layers, 0.5, names,
        demo_alphabet, classes, ext_output,
        display_mutex,
        &terminate);
    CIS *cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, display_mutex, &cis_dvs_mutex, &bbox, &terminate);
    DVS *dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, false, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, display_mutex, ROI_THRESH, ROI_MIN_SIZE, ROI_INFLATION, &bbox, &cis_dvs_mutex, &terminate);

    srand(2222222);

    create_window_cv("Demo", 0, CIS_FRAME_W, CIS_FRAME_H);

    time_t start_time = time(NULL);

    if (cis && npu)
    {
        for (int i = 0; i < 4; i++)
        {
            threads.emplace_back([cis, npu]()
                                 { CIS_NPU_thread(cis, npu,  CIS_DVS_SCALE_X * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y); });
        }
    }

    if (dvs)
    {
        threads.emplace_back([dvs]()
                             { dvs->crop_coord(0); });
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
