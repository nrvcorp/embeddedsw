#ifndef DVS_ROI_ALG_HPP_
#define DVS_ROI_ALG_HPP_

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
#include "bbox.hpp"

std::vector<Bbox> dvs_roi_cluster_tracker(
    const cv::Mat &frame,
    std::vector<std::vector<cv::Point>> &clusters,
    int max_dist = 10,
    int min_cluster_size = 20);

Bbox dvs_roi_average_based(
    const cv::Mat &frame,
    int roi_line_min_threshold);

Bbox dvs_roi_proposed(
    const cv::Mat &frame,
    int roi_event_score,         // Event Score for each events
    int row_score_threshold,     // Minimum number of events for a row to be considered active
    int roi_height_min_threshold // Minimum height of the ROI in rows
);

void draw_square_roi(
    Bbox *b_box, int x_min, int y_min,
    int x_max, int y_max, int width, int height);

#endif