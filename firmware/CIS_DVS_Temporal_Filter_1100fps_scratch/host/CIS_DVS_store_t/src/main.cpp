
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

#include "config.hpp"
#include "CIS.hpp" // Include CIS class
#include "DVS.hpp" // Include DVS class

using namespace cv;
using namespace std;

enum Mode
{
    CIS_DISPLAY = 1,
    DVS_DISPLAY,
    DVS_CHECK_FRAME_DROP,
    CIS_DVS_DISPLAY,
    DVS_STORE,
    DVS_ROI,
    CIS_DVS_BBOX
};

// Function declarations
void printBanner();
void handleMode(Mode mode);
Mode parseArguments(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    // Print application banner
    printBanner();

    // Parse command-line arguments to determine the mode
    Mode mode = parseArguments(argc, argv);

    // Handle the selected mode
    handleMode(mode);

    return 0;
}

void printBanner()
{
    printf(ANSI_COLOR_BLUE);
    printf("************************************************\n");
    printf("*******                              ***********\n");
    printf("*******  CIS, DVS Video Application  ***********\n");
    printf("*******                              ***********\n");
    printf("************************************************\n");
    printf(ANSI_COLOR_RESET);
}

void handleMode(Mode mode)
{
    MutexManager mutexManager; // Initialize mutex manager
    MutexCond mutexCond[2] = {MutexCond(), MutexCond()};
    // Create pointers for CIS and DVS objects
    CIS *cis = nullptr;
    DVS *dvs = nullptr;
    Bbox bbox;
    MutexCond *mutexCond2;
    // Declare the vector outside the switch block
    std::vector<std::thread>
        threads;

    switch (mode)
    {
    case CIS_DISPLAY:
        printf("CIS only display mode\n");
        cis = new CIS(
            CIS_FRAME_H, CIS_FRAME_W,
            CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR,
            CIS_BUFFER_NUM,
            C2H_DEVICE_CIS, H2C_DEVICE_CIS,
            mutexManager);
        cis->display_stream(); // Call CIS display stream
        delete cis;            // Cleanup
        cis = NULL;
        break;

    case DVS_DISPLAY:
        printf("DVS only display mode\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, false, 10, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);
        dvs->display_stream(); // Call DVS display stream
        delete dvs;            // Cleanup
        dvs = NULL;
        break;

    case DVS_CHECK_FRAME_DROP:
        printf("DVS only check mode\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);
        dvs->check_frame_drop();
        delete dvs; // Cleanup
        dvs = NULL;
        break;

    case CIS_DVS_DISPLAY:
        printf("CIS and DVS display mode\n");
        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, false, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);

        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis]()
                                 { cis->display_stream(); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->display_stream(); });
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete cis; // Cleanup
        delete dvs; // Cleanup
        cis = NULL;
        dvs = NULL;
        break;

    case DVS_STORE:
        printf("DVS store mode\n");

        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, false, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, mutexCond);
        // Start threads for CIS and DVS
        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->double_buf_bin_writer(); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->double_buf_reader(); });
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }

        delete dvs;
        dvs = NULL;
        break;

    case DVS_ROI:
        printf("DVS ROI mode\n ");
        mutexCond2 = new MutexCond();
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, false, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, ROI_THRESH, ROI_MIN_SIZE, ROI_INFLATION, &bbox, mutexCond2);
        dvs->crop_coord();
        dvs = NULL;
        mutexCond2 = NULL;
        delete mutexCond2;
        delete dvs;
        break;

    case CIS_DVS_BBOX:
        printf("CIS DVS BBOX mode\n ");
        mutexCond2 = new MutexCond();

        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager, mutexCond2, &bbox);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, false, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, ROI_THRESH, ROI_MIN_SIZE, ROI_INFLATION, &bbox, mutexCond2);

        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis]()
                                 { cis->crop_dvs_roi(CIS_DVS_SCALE_X * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->crop_coord(); });
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        delete mutexCond2;
        delete cis;
        delete dvs;
        dvs = NULL;
        cis = NULL;
        mutexCond2 = NULL;
        break;

    default:
        fprintf(stderr, "Error: Unknown mode\n");
        exit(EXIT_FAILURE);
    }
    delete &mutexCond[0];
    delete &mutexCond[1];
}

Mode parseArguments(int argc, char *argv[])
{
    Mode mode = DVS_DISPLAY; // Default mode

    // Define command-line options
    struct option long_options[] = {
        {"cis", no_argument, nullptr, 'c'},
        {"dvs", no_argument, nullptr, 'd'},
        {"check", no_argument, nullptr, 'x'},
        {"cis-dvs", no_argument, nullptr, 's'},
        {"write-dvs", no_argument, nullptr, 'w'},
        {"roi", no_argument, nullptr, 'r'},
        {"bbox", no_argument, nullptr, 'b'},
        {nullptr, 0, nullptr, 0}};

    // Parse command-line arguments
    int opt;
    while ((opt = getopt_long(argc, argv, "cdxswrb", long_options, nullptr)) != -1)
    {
        switch (opt)
        {
        case 'c':
            mode = CIS_DISPLAY;
            break;
        case 'd':
            mode = DVS_DISPLAY;
            break;
        case 'x':
            mode = DVS_CHECK_FRAME_DROP;
            break;
        case 's':
            mode = CIS_DVS_DISPLAY;
            break;
        case 'w':
            mode = DVS_STORE;
            break;
        case 'r':
            mode = DVS_ROI;
            break;
        case 'b':
            mode = CIS_DVS_BBOX;
            break;
        default:
            fprintf(stderr, "Usage: %s [--cis | --dvs | --check | --cis-dvs | --write-dvs | --roi | --bbox]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    return mode;
}
