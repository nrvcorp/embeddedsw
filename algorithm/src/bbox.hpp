#ifndef BBOX_HPP
#define BBOX_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>

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

typedef struct
{
    cv::Point2f centroid;
    Bbox bbox;
    std::vector<cv::Point> points;
} Cluster;

typedef struct
{
    int row;
    int left;
    int right;
} RowStreak;

typedef struct
{
    int last_row;
    std::vector<RowStreak> streaks;
} StreakCluster;

enum class ScanDirection
{
    Horizontal,
    Vertical,
    Diagonal45, // ↘
    Diagonal135 // ↙
};
typedef struct
{
    cv::Point start;
    cv::Point end;
} Streak;

#endif
