
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
    DVS_ROI_CLUSTERING = 1,
    DVS_ROI_AVG_BASE = 2,
    DVS_ROI_PROPOSED = 3,
    DVS_ROI_PROPOSED_MULTIOBJECT = 4
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
    const char *img_file_pth = "./examples/irregular_shape/low_lighting/accumulated/probe/frame_00095.png";
    // const char *img_file_pth = "./examples/irregular_shape/low_lighting/accumulated/glasses/frame_00005.png";

    cv::Mat frame = cv::imread(img_file_pth, cv::IMREAD_GRAYSCALE);
    if (frame.empty())
    {
        std::cerr << "Image not found!" << std::endl;
    }

    switch (mode)
    {
    case DVS_ROI_CLUSTERING:
    {
        printf("DVS ROI Clustering mode \n");

        auto start = std::chrono::high_resolution_clock::now();

        // Run algorithm...
        std::vector<std::vector<cv::Point>> clusters;
        std::vector<Bbox> rois = dvs_roi_cluster_tracker(
            frame,
            clusters,
            200,
            1000);

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "Algorithm took " << elapsed_ms << " ms" << std::endl;

        std::cout << clusters.size() << std::endl;
        // Convert grayscale to color for visualization
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
        // Assign white color to background pixels (128 in grayscale)
        frame.setTo(cv::Scalar(255, 255, 255), frame == 128);

        // Assign random colors to each cluster
        cv::RNG rng(12345);
        for (const auto &cluster : clusters)
        {
            if (cluster.size() < 20)
                continue; // skip small clusters
            cv::Scalar color(rng.uniform(50, 255), rng.uniform(50, 255), rng.uniform(50, 255));
            for (const auto &pt : cluster)
            {
                frame.at<cv::Vec3b>(pt.y, pt.x) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }

        // Draw all ROIs
        for (const auto &box : rois)
        {
            printf("bbox: (%d, %d), (%d, %d) \n", box.lx, box.hx, box.ly, box.hy);
            cv::rectangle(frame, cv::Point(box.lx, box.ly), cv::Point(box.hx, box.hy), cv::Scalar(255), 2);
        }
        cv::imshow("DVS Cluster Tracker ROI", frame);
        cv::waitKey(0);

        break;
    }

    case DVS_ROI_AVG_BASE:
    {
        printf("DVS ROI AVG mode\n");
        auto start = std::chrono::high_resolution_clock::now();

        Bbox bbox = dvs_roi_average_based(
            frame,
            10 /* roi_line_min_threshold */);

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "Algorithm took " << elapsed_ms << " ms" << std::endl;

        printf("x_min, x_max: %d, %d, y_min, y_max: %d, %d\n", bbox.lx, bbox.hx, bbox.ly, bbox.hy);
        // Draw the ROI rectangle on the image (in white)
        cv::Point p1(bbox.lx, bbox.ly);
        cv::Point p2(bbox.hx, bbox.hy);
        // Draw the ROI rectangle on the image (in white)
        cv::rectangle(frame, p1, p2, cv::Scalar(255), 2, cv::LINE_8);

        // Show the image in a window
        cv::imshow("DVS ROI", frame);
        cv::waitKey(0); // Wait for a key press to close the window
        cv::destroyAllWindows();
        break;
    }

    case DVS_ROI_PROPOSED:
    {
        printf("DVS ROI PROPOSED mode\n");
        auto start = std::chrono::high_resolution_clock::now();

        Bbox bbox = dvs_roi_proposed(
            frame,
            5,  /* roi_event_score */
            25, /* row_score_threshold */
            10 /* roi_height_min_threshold */);

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "Algorithm took " << elapsed_ms << " ms" << std::endl;

        printf("x_min, x_max: %d, %d, y_min, y_max: %d, %d\n", bbox.lx, bbox.hx, bbox.ly, bbox.hy);
        // Draw the ROI rectangle on the image (in white)
        cv::Point p1(bbox.lx, bbox.ly);
        cv::Point p2(bbox.hx, bbox.hy);
        // Draw the ROI rectangle on the image (in white)
        cv::rectangle(frame, p1, p2, cv::Scalar(255), 2, cv::LINE_8);

        // Show the image in a window
        cv::imshow("DVS ROI", frame);
        cv::waitKey(0); // Wait for a key press to close the window
        cv::destroyAllWindows();
        break;
    }

    case DVS_ROI_PROPOSED_MULTIOBJECT:
    {
        printf("DVS ROI PROPOSED MULTI mode\n");
        auto start = std::chrono::high_resolution_clock::now();

        auto rois = dvs_roi_proposed_multiobject(
            frame,
            5,  // roi_event_score
            20, // row_score_threshold
            5,  // roi_height_min_threshold
            10, // max_vertical_gap
            20, // min_roi_width
            20  // min_roi_height
        );

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "Algorithm took " << elapsed_ms << " ms" << std::endl;

        cv::Mat visual_out = cv::Mat(frame.size(), CV_8UC3);
        cv::cvtColor(frame, visual_out, cv::COLOR_GRAY2BGR);

        for (const auto &roi : rois)
        {
            cv::rectangle(visual_out,
                          cv::Point(roi.lx, roi.ly),
                          cv::Point(roi.hx, roi.hy),
                          cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("Multi ROI Visualization", visual_out);
        cv::waitKey(0);

        cv::destroyAllWindows();
        break;
    }

    default:
    {
        fprintf(stderr, "Error: Unknown mode \n");
        exit(EXIT_FAILURE);
    }
    }
}

Mode parseArguments(int argc, char *argv[])
{
    Mode mode = DVS_ROI_PROPOSED; // Default mode

    // Define command-line options
    struct option long_options[] = {
        {"roi_clustering", no_argument, nullptr, 'c'},
        {"roi_avg", no_argument, nullptr, 'a'},
        {"roi_proposed", no_argument, nullptr, 'p'},
        {"roi_proposed_multi", no_argument, nullptr, 'm'},
        {nullptr, 0, nullptr, 0}};

    // Parse command-line arguments
    int opt;
    while ((opt = getopt_long(argc, argv, "capm", long_options, nullptr)) != -1)
    {
        switch (opt)
        {
        case 'c':
            // clustering based DVS ROI algorithm
            mode = DVS_ROI_CLUSTERING;
            break;
        case 'a':
            // average based DVS ROI algorithm
            mode = DVS_ROI_AVG_BASE;
            break;
        case 'p':
            // proposed ROI algorithm
            mode = DVS_ROI_PROPOSED;
            break;
        case 'm':
            // proposed ROI multi algorithm
            mode = DVS_ROI_PROPOSED_MULTIOBJECT;
            break;
        default:
            fprintf(stderr, "Usage: %s [--roi_clustering | --roi_avg | --roi_proposed]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    return mode;
}
