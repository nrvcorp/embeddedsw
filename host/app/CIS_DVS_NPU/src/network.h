// Oh boy, why am I about to do this....
#ifndef NETWORK_H
#define NETWORK_H

/*
 * Necessary in C++ to get format macros out of inttypes.h
 */
#ifdef __cplusplus
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS 1
#endif
#endif
#include <inttypes.h>

#include "darknet.h"

#include <stdint.h>
#include "layer.h"
#include "dma_utils.h"

#include "image.h"
#include "data.h"
#include "tree.h"

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
#include <png.h>

#ifdef __cplusplus
extern "C"
{

#endif

    /*
    typedef enum {
        CONSTANT, STEP, EXP, POLY, STEPS, SIG, RANDOM
    } learning_rate_policy;

    typedef struct network{
        float *workspace;
        int n;
        int batch;
        uint64_t *seen;
        float epoch;
        int subdivisions;
        float momentum;
        float decay;
        layer *layers;
        int outputs;
        float *output;
        learning_rate_policy policy;

        float learning_rate;
        float gamma;
        float scale;
        float power;
        int time_steps;
        int step;
        int max_batches;
        float *scales;
        int   *steps;
        int num_steps;
        int burn_in;
        int cudnn_half;

        int adam;
        float B1;
        float B2;
        float eps;

        int inputs;
        int h, w, c;
        int max_crop;
        int min_crop;
        int flip; // horizontal flip 50% probability augmentaiont for classifier training (default = 1)
        float angle;
        float aspect;
        float exposure;
        float saturation;
        float hue;
        int small_object;

        int gpu_index;
        tree *hierarchy;

        #ifdef GPU
        float *input_state_gpu;

        float **input_gpu;
        float **truth_gpu;
        float **input16_gpu;
        float **output16_gpu;
        size_t *max_input16_size;
        size_t *max_output16_size;
        int wait_stream;
        #endif
    } network;


    typedef struct network_state {
        float *truth;
        float *input;
        float *delta;
        float *workspace;
        int train;
        int index;
        network net;
    } network_state;
    */

#ifdef GPU
    float train_networks(network *nets, int n, data d, int interval);
    void sync_nets(network *nets, int n, int interval);
    float train_network_datum_gpu(network net, float *x, float *y);
    float *network_predict_gpu(network net, float *input);
    float *network_predict_gpu_gl_texture(network net, uint32_t texture_id);
    float *get_network_output_gpu_layer(network net, int i);
    float *get_network_delta_gpu_layer(network net, int i);
    float *get_network_output_gpu(network net);
    void forward_network_gpu(network net, network_state state);
    void backward_network_gpu(network net, network_state state);
    void update_network_gpu(network net);
    void forward_backward_network_gpu(network net, float *x, float *y);
#endif

    float get_current_seq_subdivisions(network net);
    int get_sequence_value(network net);
    float get_current_rate(network net);
    int get_current_batch(network net);
    int64_t get_current_iteration(network net);
    // void free_network(network net); // darknet.h
    void compare_networks(network n1, network n2, data d);
    char *get_layer_string(LAYER_TYPE a);

    network make_network(int n);
    void forward_network(network net, network_state state);
    void backward_network(network net, network_state state);
    void update_network(network net);

    float train_network(network net, data d);
    float train_network_waitkey(network net, data d, int wait_key);
    float train_network_batch(network net, data d, int n);
    float train_network_sgd(network net, data d, int n);
    float train_network_datum(network net, float *x, float *y);

    matrix network_predict_data(network net, data test);
    // LIB_API float *network_predict(network net, float *input);
    // LIB_API float *network_predict_ptr(network *net, float *input);
    float network_accuracy(network net, data d);
    float *network_accuracies(network net, data d, int n);
    float network_accuracy_multi(network net, data d, int n);
    void top_predictions(network net, int n, int *index);
    float *get_network_output(network net);
    float *get_network_output_layer(network net, int i);
    float *get_network_delta_layer(network net, int i);
    float *get_network_delta(network net);
    int get_network_output_size_layer(network net, int i);
    int get_network_output_size(network net);
    image get_network_image(network net);
    image get_network_image_layer(network net, int i);
    int get_predicted_class_network(network net);
    void print_network(network net);
    void visualize_network(network net);
    int resize_network(network *net, int w, int h);
    // LIB_API void set_batch_network(network *net, int b);
    int get_network_input_size(network net);
    float get_network_cost(network net);
    // LIB_API layer* get_network_layer(network* net, int i);
    // LIB_API detection *get_network_boxes(network *net, int w, int h, float thresh, float hier, int *map, int relative, int *num, int letter);
    // LIB_API detection *make_network_boxes(network *net, float thresh, int *num);
    // LIB_API void free_detections(detection *dets, int n);
    // LIB_API void reset_rnn(network *net);
    // LIB_API network *load_network_custom(char *cfg, char *weights, int clear, int batch);
    // LIB_API network *load_network(char *cfg, char *weights, int clear);
    // LIB_API float *network_predict_image(network *net, image im);
    // LIB_API float validate_detector_map(char *datacfg, char *cfgfile, char *weightfile, float thresh_calc_avg_iou, const float iou_thresh, int map_points, int letter_box, network *existing_net);
    // LIB_API void train_detector(char *datacfg, char *cfgfile, char *weightfile, int *gpus, int ngpus, int clear, int dont_show, int calc_map, int mjpeg_port);
    // LIB_API int network_width(network *net);
    // LIB_API int network_height(network *net);
    // LIB_API void optimize_picture(network *net, image orig, int max_layer, float scale, float rate, float thresh, int norm);

    int get_network_nuisance(network net);
    int get_network_background(network net);
    // LIB_API void fuse_conv_batchnorm(network net);
    // LIB_API void calculate_binary_weights(network net);
    network combine_train_valid_networks(network net_train, network net_map);
    void copy_weights_net(network net_train, network *net_map);
    void free_network_recurrent_state(network net);
    void randomize_network_recurrent_state(network net);
    void remember_network_recurrent_state(network net);
    void restore_network_recurrent_state(network net);
    int is_ema_initialized(network net);
    void ema_update(network net, float ema_alpha);
    void ema_apply(network net);
    void reject_similar_weights(network net, float sim_threshold);

// #ifdef NPU
//  ANSI escape codes for text colors
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

    // #define DEBUG

#define DDR_BASEADDR 0x10000000

/******************* CIS Setting **********************************/
#define CIS_FRAME_W 1920
#define CIS_FRAME_H 1080
#define CIS_BUFFER_NUM 5
#define CIS_FRAME_SIZE (CIS_FRAME_W * CIS_FRAME_H * 3)
#define CIS_FRAME_RDY_BASEADDR (DDR_BASEADDR + 0x1000000)
#define CIS_FRAME_BASEADDR (DDR_BASEADDR + 0x10000000)

/******************* DVS Setting **********************************/
#define DVS_FRAME_W 960
#define DVS_FRAME_H 720
#define FRAME_HEADER_BYTES 8
#define DVS_BUFFER_NUM 100
#define DVS_FRAME_SIZE ((DVS_FRAME_W * DVS_FRAME_H * 2) / 8 + FRAME_HEADER_BYTES)
#define DVS_FRAME_RDY_BASEADDR (DDR_BASEADDR + 0x2000000)
#define DVS_FRAME_BASEADDR (DDR_BASEADDR + 0x30000000)

/******************* DISPLAY Setting ******************************/
#define DVS_FPS 2000
#define DISPLAY_FPS 60
#define SAVE_FPS 20
#define ROI_THRESH 7
#define ROI_INFLATION (float)(1.6)
#define ROI_MIN_SIZE 416
#define CIS_DVS_OFFSET_X 720
#define CIS_DVS_OFFSET_Y 330
#define CIS_DVS_SCALE_X 0.35
#define CIS_DVS_SCALE_Y 0.35
#define DMA_BUFFER_GRP_NUM (DVS_FPS / DISPLAY_FPS)
#define waitkey_delay (1000 / DISPLAY_FPS)

/******************* PCIE Setting ******************************/
#define H2C_DEVICE "/dev/xdma_zcu1060_h2c_0"
#define C2H_DEVICE "/dev/xdma_zcu1060_c2h_0"
#define REG_DEVICE "/dev/xdma_zcu1060_xvc"
#define USER_DEVICE "/dev/xdma_zcu1060_user"
#define MAP_SIZE (32 * 1024UL)

#define H2C_DEVICE_DVS "/dev/xdma_dvs0_h2c_0"
#define C2H_DEVICE_DVS "/dev/xdma_dvs0_c2h_0"
#define H2C_DEVICE_CIS "/dev/xdma_dvs0_h2c_1"
#define C2H_DEVICE_CIS "/dev/xdma_dvs0_c2h_1"

// #define YOLOv3_WGT_BASEADDR     0x840000000
#define YOLOv3_WGT_BASEADDR 0x100000000
#define YOLOv3_IFM_BUFFER_1 0x800000000
#define YOLOv3_IFM_BUFFER_2 0x880000000
#define YOLOv3_IFM_BUFFER_3 0x8a0000000
#define YOLOv3_CONCAT_BUFFER 0x810000000
#define YOLOv3_YOLO_OUT_1 0x900000000
#define YOLOv3_YOLO_OUT_2 0x920000000
// #define YOLOv3_YOLO_OUT_1_SCALE 0.2625377849
// #define YOLOv3_YOLO_OUT_2_SCALE 0.103072499
// #define YOLOv3_YOLO_OUT_1_BIAS  -7.566995453
// #define YOLOv3_YOLO_OUT_2_BIAS  -6.567660098
#define YOLOv3_YOLO_OUT_1_SCALE 0.1728981439
#define YOLOv3_YOLO_OUT_2_SCALE 0.1013921789
#define YOLOv3_YOLO_OUT_1_BIAS -7.610219864
#define YOLOv3_YOLO_OUT_2_BIAS -4.6513366209

#define AXILITE_NPU_START_ADDR 0x00000000
#define AXILITE_DATA_VSYNC 0
#define AXILITE_STATUS 4
#define AXILITE_LAYER_DONE 8
#define AXILITE_CONV_MODE 40
#define AXILITE_IFM_CASE 44
#define AXILITE_IFM_DTYPE 48
#define AXILITE_IFM_IS_PADDING 52
#define AXILITE_IFM_IS_MAXPOOL 56
#define AXILITE_IFM_IS_LEAKY_RELU 60
#define AXILITE_IS_CONCAT 64
#define AXILITE_IS_UPSAMPLE 68

#define AXILITE_IFM_BASEADDR 80
#define AXILITE_WEIGHT_BASEADDR 88
#define AXILITE_OFM_BASEADDR 96
#define AXILITE_IN_WIDTH 104
#define AXILITE_IN_HEIGHT 108
#define AXILITE_IN_CH 112
#define AXILITE_OUT_CH 116
#define AXILITE_POOL_STRIDE 120
#define AXILITE_CONV_STRIDE 124
#define AXILITE_SCALE_SHIFT 128
#define AXILITE_BIAS_SHIFT 132
#define AXILITE_SCALE_BIAS_1_NUM 136
#define AXILITE_SINGLE_SCALE 140
#define AXILITE_SINGLE_BIAS 144
#define AXILITE_CONCAT_BASEADDR 148
#define AXILITE_CONCAT_CH 156
#define AXILITE_BIAS_BASEADDR 160

    typedef enum
    {
        CONV1x1 = 0,
        CONV3x3 = 1
    } NPU_CONV_MODE;

    typedef enum
    {
        CASE_1x4 = 0,
        CASE_2x2 = 1
    } NPU_IFM_CASE;

    typedef enum
    {
        INT8 = 0,
        UINT8 = 1
    } NPU_DTYPE;

    typedef enum
    {
        BIAS_PER_CH = 0,
        BIAS_PER_LAYER = 1
    } NPU_IS_SINGLE_BIAS;
    // #endif

#ifdef __cplusplus
}
#endif

#endif
