#include <iostream>
#include <opencv2/opencv.hpp>

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
        dst[i] = (pixel == 0) ? 128 : // no event
                     (pixel == 1) ? 255: // on event
                     (pixel == 2) ? 0: // off event
                     0;
    }
}

// Function to convert 2-bit image data to 8-bit
void convert2BitTo8Bit_accum(char *src, uint8_t *dst, int width, int height)
{
    int numPixels = width * height;
    for (int i = 0; i < numPixels; ++i)
    {
        int byteIndex = i / 4;
        // int bitOffset = (3 - (i % 4)) * 2;
        int bitOffset = (3-(i % 4)) * 2;
        uint8_t pixel = (src[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
        if (pixel == 1) {
            dst[i] = 255;
        } else if (pixel == 2) {
            dst[i] = 0;
        }
    }
}

void convert2BitToBGR_accum(char *src, uint8_t *dst, int width, int height){
    int numPixels = width * height;
    for(int i = 0; i < numPixels; i++){
        int byteIndex = i / 4;
        int bitOffset = (3 - (i % 4)) * 2;
        int pixel = (src[byteIndex] >> bitOffset) & 0x03;
        if(pixel == 1){
            // red
            dst[i*3]   = (dst[i*3] < 40) ? 0: (dst[i*3] - 40);
            dst[i*3+2] = (dst[i*3+2] + 40 > 255) ? 255: (dst[i*3+2] + 40);
        } else
        if(pixel == 2){
            //blue
            dst[i*3]   = (dst[i*3] + 40 > 255) ? 255: (dst[i*3] + 40);
            dst[i*3+2] = (dst[i*3+2] < 40) ? 0: (dst[i*3+2] - 40);
        }
    }
}