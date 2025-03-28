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
#include "dma_utils.hpp"
#include "config.hpp"
#include "image_util.hpp"

using namespace cv;
using namespace std;

// // Function to convert 2-bit image data to 8-bit
// void convert2BitTo8Bit(char *src, uint8_t *dst, int width, int height)
// {
//     int numPixels = width * height;
//     for (int i = 0; i < numPixels; ++i)
//     {
//         int byteIndex = i / 4;
//         int bitOffset = (3 - (i % 4)) * 2;
//         uint8_t pixel = (src[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
//         // dst[i] = pixel * 85;                                  // Map 2-bit value to 8-bit value (0, 85, 170, 255)
//         dst[i] = (pixel == 0) ? 128 : // no event
//                      (pixel == 1) ? 255: // on event
//                      (pixel == 2) ? 0: // off event
//                      0;
//     }
// }

// // Function to convert 2-bit image data to 8-bit
// void convert2BitTo8Bit_accum(char *src, uint8_t *dst, int width, int height)
// {
//     int numPixels = width * height;
//     for (int i = 0; i < numPixels; ++i)
//     {
//         int byteIndex = i / 4;
//         // int bitOffset = (3 - (i % 4)) * 2;
//         int bitOffset = (3-(i % 4)) * 2;
//         uint8_t pixel = (src[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
//         if (pixel == 1) {
//             dst[i] = 255;
//         } else if (pixel == 2) {
//             dst[i] = 0;
//         }
//     }
// }

// void convert2BitToBGR_accum(char *src, uint8_t *dst, int width, int height){
//     int numPixels = width * height;
//     for(int i = 0; i < numPixels; i++){
//         int byteIndex = i / 4;
//         int bitOffset = (3 - (i % 4)) * 2;
//         int pixel = (src[byteIndex] >> bitOffset) & 0x03;
//         if(pixel == 1){
//             // red
//             dst[i*3]   = (dst[i*3] < 40) ? 0: (dst[i*3] - 40);
//             dst[i*3+2] = (dst[i*3+2] + 40 > 255) ? 255: (dst[i*3+2] + 40);
//         } else
//         if(pixel == 2){
//             //blue
//             dst[i*3]   = (dst[i*3] + 40 > 255) ? 255: (dst[i*3] + 40);
//             dst[i*3+2] = (dst[i*3+2] < 40) ? 0: (dst[i*3+2] - 40);
//         }
//     }
// }

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
    int c2h_fd = open(C2H_DEVICE, O_RDONLY); // streaming mode
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

    //--------------------------------------------------------
    // Modes for Streaming code
    // Mode 0 (BGR scale mode)
    // Mode 1 (gray scale mode)
    // Mode 2 (frame num, timestamp dump mode)
    //--------------------------------------------------------
    int mode = 1;

    if (mode == 0)
    {
        while (1)
        {
            Mat frame = Mat::zeros(frame_height, frame_width, CV_8UC1);
            read_to_buffer(C2H_DEVICE, c2h_fd, dma_buffer, AXI_STREAM_LEN, 0);
            // convert 2 bit data array to 8bit data array
            convert2BitTo8Bit(dma_buffer + 8, frame.data, frame_width, frame_height);
            int frame_num = ((static_cast<unsigned char>(dma_buffer[7]) << 24) |
                             (static_cast<unsigned char>(dma_buffer[6]) << 16) |
                             (static_cast<unsigned char>(dma_buffer[5]) << 8) |
                             static_cast<unsigned char>(dma_buffer[4]));

            uint32_t timestamp = ((static_cast<unsigned char>(dma_buffer[3]) << 24) |
                                  (static_cast<unsigned char>(dma_buffer[2]) << 16) |
                                  (static_cast<unsigned char>(dma_buffer[1]) << 8) |
                                  static_cast<unsigned char>(dma_buffer[0]));
            cout << "frame_num value (decimal): " << std::dec << frame_num << "(hex): 0x" << std::hex << frame_num << "  ";
            cout << "timestamp value (decimal): " << std::dec << timestamp << "(hex): 0x" << std::hex << timestamp << endl;
            imshow("Simulated Camera", frame);

            if (waitKey(1) == 27) // Exit if ESC is pressed
                break;
        }
    }
    else if (mode == 1)
    {
        while (1)
        {
            Mat frame = Mat::zeros(frame_height, frame_width, CV_8UC1);
            for (int i = 0; i < DMA_BUFFER_GRP_NUM; i++)
            {
                read_to_buffer(C2H_DEVICE, c2h_fd, dma_buffer, AXI_STREAM_LEN, 0);
                if (i == 0)
                {
                    convert2BitTo8Bit(dma_buffer + 8, frame.data, frame_width, frame_height);
                }
                else
                {
                    convert2BitTo8Bit_accum(dma_buffer + 8, frame.data, frame_width, frame_height);
                }

                // convert2BitToBGR_accum(dma_buffer + 8, frame.data, frame_width, frame_height);
            }
            int frame_num = ((static_cast<unsigned char>(dma_buffer[7]) << 24) |
                             (static_cast<unsigned char>(dma_buffer[6]) << 16) |
                             (static_cast<unsigned char>(dma_buffer[5]) << 8) |
                             static_cast<unsigned char>(dma_buffer[4]));

            uint32_t timestamp = ((static_cast<unsigned char>(dma_buffer[3]) << 24) |
                                  (static_cast<unsigned char>(dma_buffer[2]) << 16) |
                                  (static_cast<unsigned char>(dma_buffer[1]) << 8) |
                                  static_cast<unsigned char>(dma_buffer[0]));
            cout << "frame_num value (decimal): " << std::dec << frame_num << "(hex): 0x" << std::hex << frame_num << "  ";
            cout << "timestamp value (decimal): " << std::dec << timestamp << "(hex): 0x" << std::hex << timestamp << endl;
            imshow("Simulated Camera", frame);

            if (waitKey(1) == 27) // Exit if ESC is pressed
                break;
        }
    }
    else if (mode == 2)
    {
        while (1)
        {
            read_to_buffer(C2H_DEVICE, c2h_fd, dma_buffer, AXI_STREAM_LEN, 0);
            int frame_num = ((static_cast<unsigned char>(dma_buffer[7]) << 24) |
                             (static_cast<unsigned char>(dma_buffer[6]) << 16) |
                             (static_cast<unsigned char>(dma_buffer[5]) << 8) |
                             static_cast<unsigned char>(dma_buffer[4]));

            int timestamp = ((static_cast<unsigned char>(dma_buffer[3]) << 24) |
                             (static_cast<unsigned char>(dma_buffer[2]) << 16) |
                             (static_cast<unsigned char>(dma_buffer[1]) << 8) |
                             static_cast<unsigned char>(dma_buffer[0]));
            cout << "frame_num value (decimal): " << std::dec << frame_num << "(hex): 0x" << std::hex << frame_num << "  ";
            cout << "timestamp value (decimal): " << std::dec << timestamp << "(hex): 0x" << std::hex << timestamp << endl;
        }
    }

    // Release VideoWriter and close OpenCV windows
    video.release();
    destroyAllWindows();

    return 0;
}