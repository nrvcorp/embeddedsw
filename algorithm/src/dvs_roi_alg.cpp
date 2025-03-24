
#include "dvs_roi_alg.hpp"
#include "bbox.hpp"

std::vector<Bbox> dvs_roi_cluster_tracker(
    const cv::Mat &frame,
    std::vector<std::vector<cv::Point>> &clusters,
    int max_dist,
    int min_cluster_size)
{
    clusters.clear();
    std::vector<Cluster> tracked;
    std::vector<Bbox> bboxes;

    for (int y = 0; y < frame.rows; ++y)
    {
        for (int x = 0; x < frame.cols; ++x)
        {
            uchar pix = frame.at<uchar>(y, x);
            if (pix != 128)
            {
                cv::Point pt(x, y);
                bool matched = false;
                for (auto &cluster : tracked)
                {
                    float dx = cluster.centroid.x - x;
                    float dy = cluster.centroid.y - y;
                    if (std::sqrt(dx * dx + dy * dy) <= max_dist)
                    {
                        cluster.points.push_back(pt);
                        cluster.centroid.x = (cluster.centroid.x * (cluster.points.size() - 1) + x) / cluster.points.size();
                        cluster.centroid.y = (cluster.centroid.y * (cluster.points.size() - 1) + y) / cluster.points.size();
                        cluster.bbox.lx = std::min(cluster.bbox.lx, x);
                        cluster.bbox.hx = std::max(cluster.bbox.hx, x);
                        cluster.bbox.ly = std::min(cluster.bbox.ly, y);
                        cluster.bbox.hy = std::max(cluster.bbox.hy, y);
                        matched = true;
                        break;
                    }
                }
                if (!matched)
                {
                    Cluster new_cluster;
                    new_cluster.centroid = cv::Point2f(x, y);
                    new_cluster.points.push_back(pt);
                    new_cluster.bbox = {x, y, x, y};
                    tracked.push_back(new_cluster);
                }
            }
        }
    }

    for (const auto &cluster : tracked)
    {
        if (cluster.points.size() >= min_cluster_size)
        {
            clusters.push_back(cluster.points);
            bboxes.push_back(cluster.bbox);
        }
    }

    return bboxes;
}

Bbox dvs_roi_average_based(
    const cv::Mat &frame,
    int roi_line_min_threshold)
{
    int frame_h = 720, frame_w = 960;

    Bbox b_box_dvs = {0, 0, 0, 0};

    // buffers to count the number of events per column and row
    int *x_count = (int *)calloc(frame_w, sizeof(int));
    int *y_count = (int *)calloc(frame_h, sizeof(int));

    int sum = 0;
    // calculate distribution
    for (int h = 0; h < frame_h; h++)
    {
        for (int w = 0; w < frame_w; w++)
        {
            if (frame.at<uchar>(h, w) != 128)
            {
                x_count[w]++;
                y_count[h]++;
                sum++;
            }
        }
    }

    // check rows with number of events above average
    int x_avg = sum / frame_w;
    // to reduce noise, set minimum of events to 5
    if (x_avg < 5)
        x_avg = 5;
    int x_thresh_count = 0;
    int x_min = 0, x_max = 0;
    int y_avg = sum / frame_h;
    if (y_avg < 5)
        y_avg = 5;
    int y_thresh_count = 0;
    int y_min = 0, y_max = 0;

    for (int w = 0; w < frame_w; w++)
    {
        if (x_count[w] > x_avg)
        {
            x_thresh_count++;
            // check consecutive <roi_line_min_threshold> num of columns with numer of events above average
            if (x_thresh_count >= roi_line_min_threshold)
            {
                if (x_min == 0)
                {
                    // set as ROI left boundary
                    x_min = w - roi_line_min_threshold + 1;
                    x_max = w;
                }
                else if (w > x_max)
                {
                    x_max = w;
                }
            }
        }
        else
        {
            // if no consecutive <roi_line_min_threshold> columns exist, reset count
            x_thresh_count = 0;
        }
    }

    for (int h = 0; h < frame_h; h++)
    {
        if (y_count[h] > y_avg)
        {
            y_thresh_count++;
            // check consecutive <roi_line_min_threshold> num of rows with number of events above average
            if (y_thresh_count >= roi_line_min_threshold)
            {
                if (y_min == 0)
                {
                    // set as ROI top boundary
                    y_min = h - roi_line_min_threshold + 1;
                    y_max = h;
                }
                else if (h > y_max)
                {
                    // set as ROI bottom boundary
                    y_max = h;
                }
            }
        }
        else
        {
            // if no consecutive <roi_line_min_threshold> rows exist, reset count
            y_thresh_count = 0;
        }
    }

    if (x_min != 0 && y_min != 0)
    {
        // if valid ROI is detected
        draw_square_roi(&b_box_dvs, x_min, y_min, x_max, y_max, frame_w, frame_h);
    }

    //---------------------------------------------------------
    // Normalize x_count and y_count to 0–100 for visualization
    int x_count_max = *std::max_element(x_count, x_count + frame_w);
    int y_count_max = *std::max_element(y_count, y_count + frame_h);
    // Create blank white images (background = 255)
    cv::Mat x_plot(100, frame_w, CV_8UC1, cv::Scalar(255));
    cv::Mat y_plot(frame_h, 100, CV_8UC1, cv::Scalar(255));
    // Draw line plot for x_count (columns)
    for (int w = 1; w < frame_w; w++)
    {
        int y1 = 99 - static_cast<int>(100.0 * x_count[w - 1] / x_count_max);
        int y2 = 99 - static_cast<int>(100.0 * x_count[w] / x_count_max);
        cv::line(x_plot, cv::Point(w - 1, y1), cv::Point(w, y2), cv::Scalar(0), 1);
    }
    // Draw line plot for y_count (rows)
    for (int h = 1; h < frame_h; h++)
    {
        int x1 = static_cast<int>(100.0 * y_count[h - 1] / y_count_max);
        int x2 = static_cast<int>(100.0 * y_count[h] / y_count_max);
        cv::line(y_plot, cv::Point(x1, h - 1), cv::Point(x2, h), cv::Scalar(0), 1);
    }

    // Draw average threshold line on x_plot (horizontal line)
    int x_avg_level = 99 - static_cast<int>(100.0 * x_avg / x_count_max);
    cv::line(x_plot, cv::Point(0, x_avg_level), cv::Point(frame_w - 1, x_avg_level), cv::Scalar(128), 1, cv::LINE_8);

    // Draw average threshold line on y_plot (vertical line)
    int y_avg_level = static_cast<int>(100.0 * y_avg / y_count_max);
    cv::line(y_plot, cv::Point(y_avg_level, 0), cv::Point(y_avg_level, frame_h - 1), cv::Scalar(128), 1, cv::LINE_8);

    // Show the line plots
    cv::imshow("X Event Distribution (Line)", x_plot);
    cv::imshow("Y Event Distribution (Line)", y_plot);

    int check_w = 512;
    int check_h = 120;
    cv::Mat check_w_plot(frame_h, 100, CV_8UC1, cv::Scalar(255));
    for (int h = 0; h < frame_h; h++)
    {
        if (frame.at<uchar>(h, check_w) != 128)
        {
            cv::circle(check_w_plot, cv::Point(50, h), 2, cv::Scalar(50), -1);
        }
    }
    cv::Mat check_h_plot(100, frame_w, CV_8UC1, cv::Scalar(255));
    for (int w = 0; w < frame_w; w++)
    {
        if (frame.at<uchar>(w, check_h) != 128)
        {
            cv::circle(check_h_plot, cv::Point(w, 50), 2, cv::Scalar(50), -1);
        }
    }
    cv::imshow("vertical check", check_w_plot);
    cv::imshow("horizontal check", check_h_plot);
    cv::waitKey(0);

    //-----------------------------------------------------------------

    return b_box_dvs;
}

Bbox dvs_roi_proposed(
    const cv::Mat &frame,
    int roi_event_score,         // Event Score for each events
    int row_score_threshold,     // Minimum number of events for a row to be considered active
    int roi_height_min_threshold // Minimum height of the ROI in rows
)
{
    int frame_h = 720, frame_w = 960;
    Bbox b_box_dvs = {0, 0, 0, 0};

    // global (x_min, x_max, y_min, y_max)
    int global_x_min = frame_w, global_x_max = 0;
    int global_y_min = 0, global_y_max = -1;
    // candiate (x_min, x_max)
    int candidate_x_min = frame_w, candidate_x_max = 0;

    int roi_row_streak = 0;
    int row_start_idx = 0;
    for (int h = 0; h < frame_h; h++)
    {
        // local max score
        int local_max_score = 0;
        int local_max_left = frame_w, local_max_right = 0;
        // local current score
        int local_cur_score = 0;
        int local_cur_score_left = 0;

        // incremental algorithm
        for (int w = 0; w < frame_w; w++)
        {
            int cur_val;
            int pix_val = frame.data[row_start_idx + w];
            if (pix_val > 128 || pix_val < 128) // event present
            {
                cur_val = roi_event_score;
            }
            else
            {
                cur_val = -1;
            }

            // record local_cur_score (increment)
            local_cur_score = local_cur_score + cur_val;
            if (local_cur_score < 0)
            {
                local_cur_score = 0;
                local_cur_score_left = w;
            }

            // record max sequnce (score, left, right)
            if (local_cur_score > local_max_score)
            {
                local_max_score = local_cur_score;
                local_max_left = local_cur_score_left;
                local_max_right = w;
            }
        }

        if (local_max_score >= row_score_threshold)
        {
            roi_row_streak++;
            if (local_max_left < candidate_x_min)
                candidate_x_min = local_max_left;
            if (local_max_right > candidate_x_max)
                candidate_x_max = local_max_right;

            if (roi_row_streak >= roi_height_min_threshold)
            {
                if (global_y_min == 0)
                    global_y_min = h - roi_height_min_threshold + 1;
                global_y_max = h;

                if (candidate_x_min < global_x_min)
                    global_x_min = candidate_x_min;
                if (candidate_x_max > global_x_max)
                    global_x_max = candidate_x_max;
            }
        }
        else
        {
            // if roi_row_streak cannot reach
            // roi_height_min_threshold
            candidate_x_min = frame_w;
            candidate_x_max = 0;
            roi_row_streak = 0;
        }

        row_start_idx += frame_w;
    }

    if (global_y_max != -1)
    {
        // roi detected!!
        draw_square_roi(&b_box_dvs, global_x_min, global_y_min, global_x_max, global_y_max, frame_w, frame_h);
    }

    return b_box_dvs;
}

void draw_square_roi(Bbox *b_box, int x_min, int y_min,
                     int x_max, int y_max, int width, int height)
{
    // draw square ROI around given x and y coordinate ranges
    // while making sure ROI doesn't exceed the entire frame
    int roi_min_size = 10;
    float roi_inflation_ratio = 1.0;

    // move coordinates to inside the frame
    if (x_min < 0)
        x_min = 0;
    if (y_min < 0)
        y_min = 0;
    if (x_max >= width)
        x_max = width - 1;
    if (y_max >= height)
        y_max = height - 1;

    // determine square ROI size
    int inflated_size_x = (int)(roi_inflation_ratio * (x_max - x_min));
    if (inflated_size_x > height)
    {
        inflated_size_x = height;
    }
    else if (inflated_size_x < roi_min_size)
    {
        inflated_size_x = roi_min_size;
    }

    int inflated_size_y = (int)(roi_inflation_ratio * (y_max - y_min));
    if (inflated_size_y > height)
    {
        inflated_size_y = height;
    }
    else if (inflated_size_y < roi_min_size)
    {
        inflated_size_y = roi_min_size;
    }

    // give margin around x_max and x_min to determine ROI position
    int min_offset_x = (inflated_size_x - (x_max - x_min)) >> 1;

    // if ROI exceeds frame, move it inside
    if (x_min - min_offset_x < 0)
    {
        b_box->lx = 0;
        b_box->hx = inflated_size_x - 1;
    }
    else if (x_min + inflated_size_x - min_offset_x >= width)
    {
        b_box->hx = width;
        b_box->lx = width - inflated_size_x + 1;
    }
    else
    {
        b_box->lx = x_min - min_offset_x;
        b_box->hx = x_min - min_offset_x + inflated_size_x - 1;
    }
    // printf("DEBUG\n");

    // give margin around y_max and y_min to determine ROI position
    int min_offset_y = (inflated_size_y - (y_max - y_min)) >> 1;

    // if ROI exceeds frame, move it inside
    if (y_min - min_offset_y < 0)
    {
        b_box->ly = 0;
        b_box->hy = inflated_size_y - 1;
    }
    else if (y_min + inflated_size_y - min_offset_y >= height)
    {
        b_box->hy = height;
        b_box->ly = height - inflated_size_y + 1;
    }
    else
    {
        b_box->ly = y_min - min_offset_y;
        b_box->hy = y_min - min_offset_y + inflated_size_y - 1;
    }
}