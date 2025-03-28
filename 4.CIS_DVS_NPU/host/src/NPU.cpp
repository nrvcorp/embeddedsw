#include "NPU.hpp"
#include "image_opencv.h"

#include <fstream>
#include <vector>
#include <string>

NPU::NPU(
    char *cfgfile, char *weightfile, int letter_box_in, int benchmark_layers, float demo_thresh, char **demo_names,
    image **demo_alphabet, int demo_classes, int demo_ext_output, MutexManager &display_mutex, bool *terminate) : demo_thresh(demo_thresh),
                                                                                                                  demo_names(demo_names),
                                                                                                                  demo_alphabet(demo_alphabet),
                                                                                                                  demo_classes(demo_classes),
                                                                                                                  demo_ext_output(demo_ext_output),
                                                                                                                  display_mutex(display_mutex),
                                                                                                                  terminate(terminate)
{
    // darknet init code
    net = parse_network_cfg_custom(cfgfile, 1, 1); // set batch=1
    if (weightfile)
    {
        load_weights(&net, weightfile);
    }
    if (net.letter_box)
        letter_box = 1;
    else
        letter_box = letter_box_in;
    // detection layer l
    l = net.layers[net.n - 1];
    net.benchmark_layers = benchmark_layers;
    fuse_conv_batchnorm(net);
    calculate_binary_weights(net);
    parse_network_cfg_npu(&net, "");

    // set mutexes for each member function, for pipelining purposes
    pre_mutex = new MutexManager();
    pre_mutex->setReady(1);
    run_mutex = new MutexManager();
    run_mutex->setReady(1);
    post_mutex = new MutexManager();
    post_mutex->setReady(1);

    // set FPS counters
    frameCount = 0;
    skipFrameCount = 0;
    startTime = cv::getTickCount();
    skipFps = 0.0;
    fps = 0.0;

    // set pointer to h2c filename
    npu_h2c_fname = net.h2c_device;

    // set file descriptor to h2c
    npu_h2c_fd = net.h2c_fd;
}
void NPU::preprocess(frame_data &frame, Bbox *bbox_cis, cv::Mat *CIS_frame, bool is_update)
{
    // lock the pipeline
    pre_mutex->lock_pipeline();
    size_t in_c = 3;
    size_t in_h = 416;
    size_t in_w = 416;
    int in_bytes = in_w * in_h * ((in_c + 7) / 8) * 8;
    char *in_buffer = (char *)calloc(in_bytes, sizeof(char));
    // if no valid ROI, pass resizing
    if (is_update)
    {
        // crop frame using bbox
        cv::Point p1(bbox_cis->lx, bbox_cis->ly);
        cv::Point p2(bbox_cis->hx, bbox_cis->hy);
        cv::Mat img_roi = (*CIS_frame)(cv::Rect(p1, p2));

        // resize to 416x416
        if (img_roi.size().width != 416 || img_roi.size().height != 416)
        {
            cv::Mat img_resize;
            cv::resize(img_roi, img_resize, cv::Size(416, 416));
            frame.sized = mat_to_image(img_resize);
            for (int c_idx = 0; c_idx < in_c; c_idx++)
            {
                for (int pixel = 0; pixel < in_h * in_w; pixel++)
                {
                    int src = pixel * in_c + (in_c - 1 - c_idx);
                    int dest = pixel * 8 + c_idx;
                    in_buffer[dest] = img_resize.data[src];
                }
            }
            img_roi.release();
            img_resize.release();
        }
        else
        {
            frame.sized = mat_to_image(img_roi);
            for (int c_idx = 0; c_idx < in_c; c_idx++)
            {
                for (int pixel = 0; pixel < in_h * in_w; pixel++)
                {
                    int src = pixel * in_c + (in_c - 1 - c_idx);
                    int dest = pixel * 8 + c_idx;
                    in_buffer[dest] = img_roi.data[src];
                }
            }
            img_roi.release();
        }
    }
    frame.im = mat_to_image(*CIS_frame);
    // change to darknet format

    ////---------------------obtain NPU input from darknet resized image------------------------

    write_from_buffer(npu_h2c_fname, npu_h2c_fd, in_buffer,
                      in_bytes, YOLOv3_INPUT_IMAGE);
    free(in_buffer);
    // unlock the pipeline
    pre_mutex->unlock_pipeline();
}
void NPU::run_NPU(frame_data &frame, Bbox *bbox, bool is_update)
{
    // lock the pipeline
    run_mutex->lock_pipeline();

    // if no valid ROI, pass resizing
    if (is_update)
    {
        // run NPU
        network_predict(net, frame.sized.data);

        // decode bounding boxes
        frame.nboxes = 0;
        frame.dets = get_network_boxes(&net, bbox->hx - bbox->lx, bbox->hy - bbox->ly, demo_thresh, 0.5, 0, 1, &frame.nboxes, letter_box);
    }
    // unlock the pipeline
    run_mutex->unlock_pipeline();
}

int NPU::postprocess(frame_data &frame, Bbox *bbox, float inv_frame_w, float inv_frame_h, bool is_update, bool show_dvs_view)
{
    // lock the pipeline
    post_mutex->lock_pipeline();

    // if no valid ROI, skip running NPU
    if (is_update)
    {
        // non-max suppression
        if (l.nms_kind == DEFAULT_NMS)
            do_nms_sort(frame.dets, frame.nboxes, l.classes, .45);
        else
            diounms_sort(frame.dets, frame.nboxes, l.classes, .45, l.nms_kind, l.beta_nms);

        // draw bounding boxes
        draw_detections_CIS(frame.im, frame.dets, frame.nboxes, demo_thresh, demo_names, demo_alphabet, demo_classes, demo_ext_output,
                            (bbox->hx - bbox->lx) * inv_frame_w, (bbox->hy - bbox->ly) * inv_frame_h, bbox->lx * inv_frame_w, bbox->ly * inv_frame_h); // originally frame.im

        // draw square ROI on opencv video
        if (show_dvs_view)
            draw_box_width(frame.im, bbox->lx, bbox->ly, bbox->hx, bbox->hy, 10, 0.0, 0.0, 0.0);
    }
    else
    {
        skipFrameCount++;
    }
    // display image
    display_mutex.lock_display();
    show_image_cv(frame.im, "Demo");

    // cleanup
    if (is_update)
    {
        free_detections(frame.dets, frame.nboxes);
        free_image(frame.sized);
    }
    free_image(frame.im);

    // exit when ESC is pressed or other threads terminate
    if (*terminate || cv::waitKey(1) == 27)
    {
        *terminate = 1;

        // wake up other threads
        pre_mutex->terminate();
        run_mutex->terminate();
        post_mutex->terminate();

        // cleanup
        npu_h2c_fname = NULL;
        display_mutex.unlock_display();
        post_mutex->unlock_pipeline();
        return 1;
    }
    else
    {
        // cleanup
        npu_h2c_fname = NULL;
        display_mutex.unlock_display();
        post_mutex->unlock_pipeline();
        return 0;
    }

    // release memory
}
NPU::~NPU()
{
    free_ptrs((void **)demo_names, net.layers[net.n - 1].classes);
    free_alphabet(demo_alphabet);
    free_network(net);
    delete pre_mutex;
    delete run_mutex;
    delete post_mutex;
}