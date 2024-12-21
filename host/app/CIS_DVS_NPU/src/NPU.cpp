#include "NPU.hpp"
#include "image_opencv.h"
NPU::NPU(
    char *cfgfile, char *weightfile, int letter_box_in, int benchmark_layers, float demo_thresh, char **demo_names,
    image **demo_alphabet, int demo_classes, int demo_ext_output, MutexManager &mutexManager, bool* terminate) : demo_thresh(demo_thresh),
                                                                                                demo_names(demo_names),
                                                                                                demo_alphabet(demo_alphabet),
                                                                                                demo_classes(demo_classes),
                                                                                                demo_ext_output(demo_ext_output),
                                                                                                mutexManager(mutexManager),
                                                                                                terminate(terminate)
{
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
    break_trigger = 0;
    pre_mutex= new MutexManager();
    pre_mutex->setReady(1);
    run_mutex = new MutexManager();
    run_mutex->setReady(1);
    post_mutex = new MutexManager();
    post_mutex->setReady(1);
}
void NPU::preprocess(frame_data &frame, Bbox *bbox_cis, cv::Mat *CIS_frame)
{
    pre_mutex->lock_pipeline();
    cv::Point p1(bbox_cis->lx, bbox_cis->ly);
    cv::Point p2(bbox_cis->hx, bbox_cis->hy);
    cv::Mat img_roi = (*CIS_frame)(cv::Rect(p1, p2));

    // resize to 416x416

    cv::Mat img_resize;
    // printf("img_roi size = %d, %d\n", img_roi.size().width, img_roi.size().height);
    cv::resize(img_roi, img_resize, cv::Size(416, 416));
    // printf("img_resize size = %d, %d\n", img_resize.size().width, img_resize.size().height);
    // change to darknet format
    img_roi.release();
    frame.im = mat_to_image(*CIS_frame);
    frame.sized = mat_to_image(img_resize);
    img_resize.release();
    pre_mutex->unlock_pipeline();
}
void NPU::run_NPU(frame_data &frame, Bbox *bbox)
{
    // run NPU
    run_mutex->lock_pipeline();
    network_predict(net, frame.sized.data);

    // decode bounding boxes
    frame.nboxes = 0;
    frame.dets = get_network_boxes(&net, bbox->hx - bbox->lx, bbox->hy - bbox->ly, demo_thresh, 0.5, 0, 1, &frame.nboxes, letter_box);
    run_mutex->unlock_pipeline();
}
int NPU::postprocess(frame_data &frame, Bbox *bbox, float inv_frame_w, float inv_frame_h)
{
    post_mutex->lock_pipeline();
    // non-max suppression
    if (l.nms_kind == DEFAULT_NMS)
        do_nms_sort(frame.dets, frame.nboxes, l.classes, .45);
    else
        diounms_sort(frame.dets, frame.nboxes, l.classes, .45, l.nms_kind, l.beta_nms);

    // draw bounding boxes
    draw_detections_CIS(frame.im, frame.dets, frame.nboxes, demo_thresh, demo_names, demo_alphabet, demo_classes, demo_ext_output,
                        (bbox->hx - bbox->lx) * inv_frame_w, (bbox->hy - bbox->ly) * inv_frame_h, bbox->lx * inv_frame_w, bbox->ly * inv_frame_h); // originally frame.im

    // display image
    mutexManager.lock_display();
    show_image_cv(frame.im, "Demo");
    free_detections(frame.dets, frame.nboxes);
    free_image(frame.im);
    free_image(frame.sized);
    if (*terminate || cv::waitKey(1) == 27)
    {
        *terminate = 1;
        pre_mutex->terminate();
        run_mutex->terminate();
        post_mutex->terminate();
        mutexManager.unlock_display();
        post_mutex->unlock_pipeline();
        return 1;
    }
    else
    {
        mutexManager.unlock_display();
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