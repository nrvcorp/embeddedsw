#ifndef BBOX_HPP
#define BBOX_HPP
/**
 * struct to hold x and y coordinates of bounding box vertices.
 * @param lx x coordinate of leftmost edge
 * @param ly y coordinate of topmost edge
 * @param hx x coordinate of rightmost edge
 * @param hy y coordinate of bottommost edge
 */
typedef struct
{
    int lx;
    int ly;
    int hx;
    int hy;
} Bbox;
#endif
