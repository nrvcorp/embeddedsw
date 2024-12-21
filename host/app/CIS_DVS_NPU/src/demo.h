#ifndef DEMO_H
#define DEMO_H
#include <thread>
#include <vector>
#include "NPU.hpp"
#include "CIS.hpp" // Include CIS class
#include "DVS.hpp" // Include DVS class
void CIS_NPU_thread(CIS *cis, NPU *npu,float x_scale, float y_scale, int x_offset, int y_offset);
#ifdef __cplusplus
extern "C"
{
#endif

    void demo(char *cfgfile, char *weightfile, float thresh, float hier_thresh, int cam_index, const char *filename, char **names, int classes, int avgframes,
              int frame_skip, char *prefix, char *out_filename, int mjpeg_port, int dontdraw_bbox, int json_port, int dont_show, int ext_output, int letter_box_in, int time_limit_sec, char *http_post_host, int benchmark, int benchmark_layers, char *json_file_output);
#ifdef __cplusplus
}
#endif

#endif
