#ifndef __IMAGE_UTIL_HPP
#define __IMGE_UTIL_HPP



void convert2BitTo8Bit(char *src, uint8_t *dst, int width, int height);
void convert2BitTo8Bit_accum(char *src, uint8_t *dst, int width, int height);
void convert2BitToBGR_accum(char *src, uint8_t *dst, int width, int height);

#endif