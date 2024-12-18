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

// Function to convert 2-bit image data to 8-bit
void convert2BitTo8Bit(char *src, uint8_t *dst, int width, int height)
{
    int numPixels = width * height;
    for (int i = 0; i < numPixels; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (3 - (i % 4)) * 2;
        uint8_t pixel = (src[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
        // dst[i] = pixel * 85;                                  // Map 2-bit value to 8-bit value (0, 85, 170, 255)
        dst[i] =    (pixel == 0) ? 85:  // no event
                    (pixel == 1) ? 255: // on event
                    (pixel == 3) ? 0:   // off event
                    170;
    }
}

// Function to convert 2-bit image data to 8-bit
void convert2BitTo8Bit_accum(char *src, uint8_t *dst, int width, int height)
{
    int numPixels = width * height;
    for (int i = 0; i < numPixels; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (3 - (i % 4)) * 2;
        uint8_t pixel = (src[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
        // dst[i] = pixel * 85;                                  // Map 2-bit value to 8-bit value (0, 85, 170, 255)
        if (pixel == 1) { // on event
            dst[i] = 255;
        } else if (pixel == 3) { // off event
            dst[i] = 0;
        }
    }
}

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

    // Create a window called "Simulated Camera"
    namedWindow("Simulated Camera", WINDOW_NORMAL);

    while (1)
    {
        read_to_buffer(C2H_DEVICE, c2h_fd, dma_buffer, AXI_STREAM_LEN, 0);
    }
    return 0;
}