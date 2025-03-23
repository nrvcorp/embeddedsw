
#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
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

#include "dvs_roi_alg.hpp"

using namespace cv;
using namespace std;

enum Mode
{
    DVS_ROI_AVG_BASE = 1,
    DVS_ROI_PROPOSED
};

void handleMode(Mode mode);
Mode parseArguments(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    Mode mode = parseArguments(argc, argv);

    handleMode(mode);

    return 0;
}

void handleMode(Mode mode)
{
    char *img_file_pth = "./examples/irregular_shape/low_lighting/accumulated/pliers/frame_00010.png";

    switch (mode)
    {
    case DVS_ROI_AVG_BASE:
        printf("DVS ROI AVG mode\n");
        int status;
        status = dvs_roi_average_based(img_file_pth);
        break;
    case DVS_ROI_PROPOSED:
        printf("DVS ROI PROPOSED mode\n");

        break;
    default:
        fprintf(stderr, "Error: Unknown mode \n");
        exit(EXIT_FAILURE);
    }
}

Mode parseArguments(int argc, char *argv[])
{
    Mode mode = DVS_ROI_PROPOSED; // Default mode

    // Define command-line options
    struct option long_options[] = {
        {"roi_avg", no_argument, nullptr, 'a'},
        {"roi_proposed", no_argument, nullptr, 'p'},
        {nullptr, 0, nullptr, 0}};

    // Parse command-line arguments
    int opt;
    while ((opt = getopt_long(argc, argv, "ap", long_options, nullptr)) != -1)
    {
        switch (opt)
        {
        case 'a':
            // average based DVS ROI algorithm
            mode = DVS_ROI_AVG_BASE;
            break;
        case 'p':
            // proposed ROI algorithm
            mode = DVS_ROI_PROPOSED;
            break;
        default:
            fprintf(stderr, "Usage: %s [--roi_avg | --roi_proposed]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    return mode;
}
