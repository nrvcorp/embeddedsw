
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
    CIS_DVS_BBOX,
    CIS_DVS_OVERLAY,
    DVS_FPS_CHECK,
    CIS_DVS_DISPLAY_FPS,
    CIS_ONLY_ROI,
    DVS_BIN_TO_VID,
    DVS_BIN_TO_PNG,
    CIS_DVS_STORE_PNG
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
    // Create pointers for CIS and DVS objects
    CIS *cis = nullptr;
    DVS *dvs = nullptr;
    Bbox bbox;
    bool terminate;
    MutexManager bbox_mutex;
    // Declare the vector outside the switch block
    std::vector<std::thread>
        threads;
    cv::Mat frame_shared;
    cv::Rect dvs_rect, cis_rect;
    int dvs_width, dvs_height;
    char bin_file_name[100];
    char vid_file_name[100];
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
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / DISPLAY_FPS), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);
        dvs->display_stream(true); // Call DVS display stream
        delete dvs;                // Cleanup
        dvs = NULL;
        break;

    case DVS_CHECK_FRAME_DROP:
        printf("DVS only check mode\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);
        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->check_frame_drop(); });
            setThreadPriority(threads.back());
        }
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        delete dvs; // Cleanup
        dvs = NULL;
        break;

    case CIS_DVS_DISPLAY:
        printf("CIS and DVS display mode\n");
        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / (DISPLAY_FPS)), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);

        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis]()
                                 { cis->display_stream(); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->display_stream(true); });
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

        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, true);
        // Start threads for CIS and DVS
        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->double_buf_bin_writer(); });
        }
        setThreadPriority(threads.back()); // Set priority after thread creation
        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->double_buf_reader(); });
        }
        setThreadPriority(threads.back()); // Set priority after thread creation
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
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / DISPLAY_FPS), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, &bbox, &bbox_mutex, &terminate);
        dvs->set_DVS_ROI(ROI_EVENT_SCORE, ROI_MIN_SCORE, ROI_LINE_WIDTH, DVS_ROI_MIN_SIZE, 1.0);
        // run old algorithm
        // dvs->crop_coord(1,1,true, true);
        // run new algorithm
        dvs->crop_new_ROI(1, 1, true, true);
        dvs = NULL;
        delete dvs;
        break;

    case CIS_DVS_BBOX:
        printf("CIS DVS BBOX mode\n ");

        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager, &bbox_mutex, &bbox, &terminate);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / DISPLAY_FPS), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, &bbox, &bbox_mutex, &terminate);

        // set relative parameters between the two sensors
        cis->set_DVS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y);
        dvs->set_CIS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y, CIS_FRAME_W, CIS_FRAME_H, ROI_EVENT_SCORE, ROI_MIN_SCORE, ROI_LINE_WIDTH, CIS_ROI_MIN_SIZE, ROI_INFLATION);
        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis]()
                                 { cis->crop_dvs_roi(); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->crop_new_ROI(1, 1, true); });
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        delete cis;
        delete dvs;
        dvs = NULL;
        cis = NULL;
        break;
    case CIS_DVS_OVERLAY:
        printf("CIS DVS overlay mode for tuning\n");
        printf("when red grid shows up, use it to estimate scale between DVS and CIS images\n");
        printf("open up config.hpp\n");
        printf("if DVS image is larger sideways, try to shrink CIS_DVS_SCALE_X\n");
        printf("if DVS image is shorter vertically, increase CIS_DVS_SCALE_Y\n");
        printf("if grid has 10 regions horizontally, width of each region = 0.1\n");
        printf("if DVS image is shunted left from CIS image by 3 regions, increase CIS_DVS_OFFSET_X by 0.3\n");
        printf("if DVS image is shunted down from CIS image by 2 regions, decrease CIS_DVS_OFFSET_Y by 0.2\n");
        frame_shared = cv::Mat::zeros(DVS_FRAME_H, DVS_FRAME_W, CV_8UC1);
        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager, &bbox_mutex, &bbox, &terminate);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / DISPLAY_FPS), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, &bbox, &bbox_mutex, &terminate);

        // set relative parameters between the two sensors
        cis->set_DVS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y);
        dvs->set_CIS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y, CIS_FRAME_W, CIS_FRAME_H, ROI_EVENT_SCORE, ROI_MIN_SCORE, ROI_LINE_WIDTH, CIS_ROI_MIN_SIZE, ROI_INFLATION);
        // Start threads for CIS and DVS

        cis->set_roi(dvs_rect, cis_rect, dvs_width, dvs_height);
        while (true)
        {
            dvs->send_frame((cv::Mat *)(&frame_shared), true);
            bool retval = cis->overlay_dvs((cv::Mat *)(&frame_shared), dvs_rect, cis_rect, dvs_width, dvs_height, DVS_WEIGHT_ALPHA, CIS_DVS_CALIBRATION_GRID_NUM);
            if (retval)
                break;
        }

        delete cis;
        delete dvs;
        dvs = NULL;
        cis = NULL;
        break;
    case DVS_FPS_CHECK:
        printf("DVS FPS check mode\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager);
        dvs->fps_count();
        delete dvs; // Cleanup
        dvs = NULL;
        break;
    case CIS_DVS_DISPLAY_FPS:
        printf("CIS DVS display with fps check mode\n");
        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, (DVS_FPS / (DISPLAY_FPS * DISPLAY_DOWNSAMPLE_NUM)), DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, true, DISPLAY_DOWNSAMPLE_NUM);
        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis]()
                                 { cis->display_stream(); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs]()
                                 { dvs->double_buf_display_fps_writer(); });
            threads.emplace_back([dvs]()
                                 { dvs->double_buf_display_fps_reader(true); });
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
    case CIS_ONLY_ROI:
        printf("CIS only ROI through background subtraction and contour mode\n");
        cis = new CIS(
            CIS_FRAME_H, CIS_FRAME_W,
            CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR,
            CIS_BUFFER_NUM,
            C2H_DEVICE_CIS, H2C_DEVICE_CIS,
            mutexManager);
        cis->background_subtraction(); // Call CIS display stream
        delete cis;                    // Cleanup
        cis = NULL;
        break;
    case DVS_BIN_TO_VID:
        printf("convert the bin file from DVS_STORE mode to a grayscale video\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, true);
        cout << "Path to bin file:\n";
        cin.getline(bin_file_name, 100);
        cout << "Path to video file:\n";
        cin.getline(vid_file_name, 100);
        dvs->bin_to_vid((char *)bin_file_name, (char *)vid_file_name);
        delete dvs;
        dvs = NULL;
        break;
    case DVS_BIN_TO_PNG:
        printf("convert the bin file from DVS_STORE mode to a collection of PNGs\n");
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, true);
        cout << "Path to bin file:\n";
        cin.getline(bin_file_name, 100);
        cout << "Path to folder that will contain PNG images:\n";
        cin.getline(vid_file_name, 100);
        dvs->bin_to_png((char *)bin_file_name, (char *)vid_file_name);
        delete dvs;
        break;
    case CIS_DVS_STORE_PNG:
        printf("Save CIS and DVS images in PNG format, synchronized at 60FPS\n");
        cis = new CIS(CIS_FRAME_H, CIS_FRAME_W, CIS_FRAME_RDY_BASEADDR, CIS_FRAME_BASEADDR, CIS_BUFFER_NUM, C2H_DEVICE_CIS, H2C_DEVICE_CIS, mutexManager, &bbox_mutex, &bbox, &terminate);
        dvs = new DVS(DVS_FRAME_H, DVS_FRAME_W, true, 1, DVS_FRAME_RDY_BASEADDR, DVS_FRAME_BASEADDR, DVS_BUFFER_NUM, C2H_DEVICE_DVS, H2C_DEVICE_DVS, mutexManager, &bbox, &bbox_mutex, &terminate);
        // set relative parameters between the two sensors
        cis->set_DVS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y);
        dvs->set_CIS(CIS_DVS_SCALE_X * CIS_FRAME_W / DVS_FRAME_W, CIS_DVS_SCALE_Y * CIS_FRAME_H / DVS_FRAME_H, CIS_DVS_OFFSET_X, CIS_DVS_OFFSET_Y, CIS_FRAME_W, CIS_FRAME_H, ROI_EVENT_SCORE, ROI_MIN_SCORE, ROI_LINE_WIDTH, CIS_ROI_MIN_SIZE, ROI_INFLATION);
        cout << "Path to folder that will contain CIS images:\n";
        cin.getline(bin_file_name, 100);
        cout << "Path to folder that will contain DVS images:\n";
        cin.getline(vid_file_name, 100);
        // Start threads for CIS and DVS
        if (cis)
        {
            threads.emplace_back([cis, bin_file_name]()
                                 { cis->save_png_stream((char *)bin_file_name); });
        }

        if (dvs)
        {
            threads.emplace_back([dvs, vid_file_name]()
                                 { dvs->save_png_stream((char *)vid_file_name, true); });
        }

        // Wait for all threads to complete
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
        delete cis;
        delete dvs;
        dvs = NULL;
        cis = NULL;
        break;
    default:
        fprintf(stderr, "Error: Unknown mode\n");
        exit(EXIT_FAILURE);
    }
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
        {"overlay", no_argument, nullptr, 'o'},
        {"dvs-fps", no_argument, nullptr, 'f'},
        {"cis-dvs-fps", no_argument, nullptr, 'p'},
        {"cis-roi", no_argument, nullptr, 'i'},
        {"dvs-bin-to-vid", no_argument, nullptr, 'v'},
        {"dvs-bin-to-png", no_argument, nullptr, 'g'},
        {"cis-dvs-store-png", no_argument, nullptr, 't'},
        {nullptr, 0, nullptr, 0}};

    // Parse command-line arguments
    int opt;
    while ((opt = getopt_long(argc, argv, "cdxswrbofpivgt", long_options, nullptr)) != -1)
    {
        switch (opt)
        {
        case 'c':
            // cis only display
            mode = CIS_DISPLAY;
            break;
        case 'd':
            // dvs only display
            mode = DVS_DISPLAY;
            break;
        case 'x':
            // prints error whenever there is a frame drop
            // error can happen in other execution modes due to heavy host code
            mode = DVS_CHECK_FRAME_DROP;
            break;
        case 's':
            // shows both CIS and DVS display
            // to get DVS fps correctly, run ./main -p
            mode = CIS_DVS_DISPLAY;
            break;
        case 'w':
            // subdirectory bin_files must reside under CIS_DVS
            // store a raw file containing DVS frames using double buffering
            // also runs error checks like in ./main -x
            mode = DVS_STORE;
            break;
        case 'r':
            // runs DVS ROI detection
            // draws square bounding box around ROI
            mode = DVS_ROI;
            break;
        case 'b':
            // you should run ./main -o to calibrate between CIS and DVS first
            // shows ROI onto DVS and CIS perspective simultaneously
            mode = CIS_DVS_BBOX;
            break;
        case 'o':
            // shows red grid in order to calibrate between CIS and DVS viewpoint
            // modify config.hpp according to console messages
            mode = CIS_DVS_OVERLAY;
            break;
        case 'f':
            // prints DVS fps to the console
            // no error checking functionality
            mode = DVS_FPS_CHECK;
            break;
        case 'p':
            // displays CIS, DVS video streams
            // displays their FPS
            // runs error checks
            // since DVS is downsampled, the screen might be laggy or shaded less
            mode = CIS_DVS_DISPLAY_FPS;
            break;
        case 'i':
            // runs background subtraction on CIS images
            mode = CIS_ONLY_ROI;
            break;
        case 'v':
            // converts bin file to .avi video file
            // the path to bin file, and the path to video is required
            mode = DVS_BIN_TO_VID;
            break;
        case 'g':
            // converts bin file into a collection of .png files
            //  the path to bin file, and the path to png file folder is requred
            mode = DVS_BIN_TO_PNG;
            break;
        case 't':
            // captures CIS and DVS images in png format, synchronized at 60fps
            // the path to CIS, and the path to DVS image folders are required.
            mode = CIS_DVS_STORE_PNG;
            break;
        default:
            fprintf(stderr, "Usage: %s [--cis | --dvs | --check | --cis-dvs | --write-dvs | --roi | --bbox | --overlay | --dvs-fps | --cis-dvs-fps | --cis-roi | --dvs-bin-to-vid | --dvs-bin-to-png | --cis-dvs-store-png ]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    return mode;
}
