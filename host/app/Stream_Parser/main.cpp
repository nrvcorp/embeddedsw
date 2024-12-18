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
#include "dma_utils.h"
#include "config.h"

using namespace cv;
using namespace std;

int main()
{
    printf(ANSI_COLOR_BLUE);
    printf("************************************************\r\n");
    printf("*******                              ***********\r\n");
    printf("*******  DVS Streaming Application   ***********\r\n");
    printf("*******                              ***********\r\n");
    printf("************************************************\r\n");
    printf(ANSI_COLOR_RESET);

    int frame_width = 960;
    int frame_height = 720;

    char *dma_buffer = (char *)malloc(AXI_STREAM_LEN * sizeof(char));
    /* check c2h functionality */
    int c2h_fd = open(C2H_DEVICE, O_RDWR | O_TRUNC); // streaming mode
    if (c2h_fd < 0)
    {
        fprintf(stderr, "unable to open device %s, %d.\r\n", C2H_DEVICE, c2h_fd);
        perror("open device");
        return -EINVAL;
    }

    // Create a VideoWriter object to save the output video
    VideoWriter video("output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(frame_width, frame_height));

    // Check if VideoWriter is successfully opened
    if (!video.isOpened())
    {
        cerr << "Error creating video file" << endl;
        return -1;
    }

    // Create a window called "Simulated Camera"
    namedWindow("Simulated Camera", WINDOW_NORMAL);

    int index = 0;
    uint32_t mask = 0;
    uint32_t pol = 0;
    uint8_t header = 0;
    uint32_t sFlag = 0;
    uint32_t posX = 0;
    uint32_t posY = 0;
    uint32_t grpAddr = 0;
    uint32_t mRefTimeStamp = 0;

    while (1)
    {
        Mat frame = Mat::zeros(frame_height, frame_width, CV_8UC1);

        if (index == 0)
        {
            read_to_buffer(C2H_DEVICE, c2h_fd, dma_buffer, AXI_STREAM_LEN * 4, 0);
        }

        header = dma_buffer[index] & 0x7c;

        if (dma_buffer[index] & 0x80) // Data packet
        {
            grpAddr = ((dma_buffer[index + 1] & 0xFC) >> 2) | ((dma_buffer[index] & 0x01) << 6);

            posY = grpAddr << 3;
            pol = dma_buffer[index + 1] & 0x01;
            mask = dma_buffer[index + 3];
            int32_t groupCount = 2;
            while (groupCount > 0)
            {
                if (groupCount == 2)
                {
                    grpAddr += (header >> 2);
                    posY = grpAddr << 3;
                    pol = (dma_buffer[index + 1] & 0x02 >> 1);
                    mask = dma_buffer[index + 2];
                }
                --groupCount;
            }
            continue;
        }
        // Normal Packet
        switch (header)
        {
        case (0x0C): // FRAMEEND
            break;
        case (0x04): // column packet
        {
            posX = (((dma_buffer[index + 2] & 0x07) << 8) | (dma_buffer[index + 3] & 0xFF));
        }
        break;
        case (0x08): // timestamp packet
            mRefTimeStamp = ((uint32_t)(dma_buffer[index + 1] & 0x3F) << 16) | ((uint32_t)(dma_buffer[index + 2] & 0xFF) << 8) |
                            ((uint32_t)(dma_buffer[index + 3] & 0xFF));
            break;
        default:
            break;
        }

        imshow("Simulated Camera", frame);

        if (waitKey(10) == 27) // Exit if ESC is pressed
            break;
    }

    // // Generate synthetic frames
    // for (int i = 0; i < 100; ++i)
    // {                                                               // Generate 100 frames
    //     Mat frame = Mat::zeros(frame_height, frame_width, CV_8UC3); // Create a black frame

    //     // Draw a rectangle
    //     rectangle(frame, Point(100, 100), Point(300, 300), Scalar(255, 0, 0), 2);

    //     // Draw a circle
    //     circle(frame, Point(frame_width - 150, frame_height - 150), 50, Scalar(0, 255, 0), 3);

    //     // Write text
    //     putText(frame, "Frame " + to_string(i), Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);

    //     // Display the frame
    //     imshow("Simulated Camera", frame);

    //     // Write frame to video file
    //     video.write(frame);

    //     // Wait for 100 milliseconds (0.1 second)
    //     // Adjust waitKey duration to control frame rate (1000 ms / desired FPS)
    //     if (waitKey(100) == 27) // Exit if ESC is pressed
    //         break;
    // }

    // Release VideoWriter and close OpenCV windows
    video.release();
    destroyAllWindows();

    return 0;
}