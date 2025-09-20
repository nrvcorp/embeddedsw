#include "bbox.hpp"
#include <optional>

double iou(const Bbox &a, const Bbox &b)
{
    int x1 = std::max(a.lx, b.lx);
    int y1 = std::max(a.ly, b.ly);
    int x2 = std::min(a.hx, b.hx); // ← 수정: a.hx
    int y2 = std::min(a.hy, b.hy);

    int inter_w = std::max(0, x2 - x1 + 1);
    int inter_h = std::max(0, y2 - y1 + 1);
    int inter = inter_w * inter_h;

    int area_a = (a.hx - a.lx + 1) * (a.hy - a.ly + 1);
    int area_b = (b.hx - b.lx + 1) * (b.hy - b.ly + 1);

    return inter == 0 ? 0.0
                      : static_cast<double>(inter) /
                            (area_a + area_b - inter);
}

double ciou(const Bbox &a, const Bbox &b)
{
    /* --- IoU --- */
    int x1 = std::max(a.lx, b.lx), y1 = std::max(a.ly, b.ly);
    int x2 = std::min(a.hx, b.hx), y2 = std::min(a.hy, b.hy);
    int inter = std::max(0, x2 - x1 + 1) * std::max(0, y2 - y1 + 1);

    int areaA = (a.hx - a.lx + 1) * (a.hy - a.ly + 1);
    int areaB = (b.hx - b.lx + 1) * (b.hy - b.ly + 1);
    double iou = inter / (double(areaA + areaB - inter) + 1e-9);

    /* --- 중심 거리 --- */
    double cxA = (a.lx + a.hx) * 0.5, cyA = (a.ly + a.hy) * 0.5;
    double cxB = (b.lx + b.hx) * 0.5, cyB = (b.ly + b.hy) * 0.5;
    double center_dist2 = (cxA - cxB) * (cxA - cxB) +
                          (cyA - cyB) * (cyA - cyB);

    /* --- 외접 박스 대각선² --- */
    int encL = std::min(a.lx, b.lx), encT = std::min(a.ly, b.ly);
    int encR = std::max(a.hx, b.hx), encB = std::max(a.hy, b.hy);
    double diag2 = (encR - encL) * (encR - encL) +
                   (encB - encT) * (encB - encT) + 1e-9;

    /* --- 비율 항 v (Aspect-Ratio) --- */
    double wA = a.hx - a.lx + 1, hA = a.hy - a.ly + 1;
    double wB = b.hx - b.lx + 1, hB = b.hy - b.ly + 1;
    double v = 4 / (M_PI * M_PI) *
               std::pow(std::atan(wB / hB) - std::atan(wA / hA), 2);
    double S = 1 - iou;
    double alpha = v / (S + v + 1e-9);

    /* --- CIoU --- */
    return iou - (center_dist2 / diag2) - alpha * v; // (-1, 1]
}

Bbox yolo2bbox(double xc, double yc, double w, double h,
               int width, int height)
{
    int bx = static_cast<int>((xc - w / 2.0) * width);
    int by = static_cast<int>((yc - h / 2.0) * height);
    int bw = static_cast<int>(w * width);
    int bh = static_cast<int>(h * height);
    return {bx, by, bx + bw, by + bh};
}

bool load_gt_one(const std::string &path,
                 int img_w, int img_h,
                 Bbox &bbox_out)
{
    std::ifstream ifs(path);
    if (!ifs)
        return false;

    int cls;
    double xc, yc, bw, bh;
    if (ifs >> cls >> xc >> yc >> bw >> bh)
    {
        bbox_out = yolo2bbox(xc, yc, bw, bh, img_w, img_h);
        return true;
    }
    return false;
}