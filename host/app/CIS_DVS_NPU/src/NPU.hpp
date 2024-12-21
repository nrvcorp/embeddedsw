#ifndef NPU_HPP
#define NPU_HPP
#include "image.h"
#include "MutexManager.hpp"
#include "bbox.hpp"
#include "network.h"
#include "parser.h"
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
    network net;
    float demo_thresh;
    int letter_box;
    char **demo_names;
    image **demo_alphabet;
    int demo_classes;
    int demo_ext_output;
    layer l;
    MutexManager &mutexManager;
    int break_trigger;
    MutexManager* pre_mutex;
    MutexManager* run_mutex;
    MutexManager* post_mutex;
    bool* terminate;

public:
    NPU(
        char *cfgfile, char *weightfile, int letter_box_in, int benchmark_layers, float demo_thresh, char **demo_names,
        image **demo_alphabet, int demo_classes, int demo_ext_output, MutexManager &mutexManager, bool *terminate);
    void preprocess(frame_data &frame, Bbox *bbox_cis, cv::Mat *CIS_frame);
    void run_NPU(frame_data &frame, Bbox *bbox);
    int postprocess(frame_data &frame, Bbox *bbox, float inv_frame_w, float inv_frame_h);
    ~NPU();
};
#endif