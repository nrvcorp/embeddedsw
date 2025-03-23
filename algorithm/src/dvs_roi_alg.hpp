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

int dvs_roi_average_based(char *img_file_pth);

void draw_square_roi(Bbox *b_box, int x_min, int y_min,
                     int x_max, int y_max, int width, int height);

#endif