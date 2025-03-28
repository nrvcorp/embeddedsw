
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

    int cell_size = 6;
    int img_height = 100;
    int img_width = frame_w;
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

    // Create a color image with white background
    cv::Mat check_h_plot_color(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw marked points (replacing gray circles from grayscale version)
    for (int check_range = 0; check_range < 50; check_range++)
    {
        for (int w = 0; w < frame_w; w++)
        {
            if (frame.at<uchar>(check_h + check_range, w) != 128)
            {
                // Set that pixel at (w, 50 + check_range) to dark gray
                check_h_plot_color.at<cv::Vec3b>(2 * check_range + 1, w + 1) = cv::Vec3b(50, 50, 50);
            }
        }
    }
    // Draw grid lines
    for (int x = 0; x <= check_h_plot_color.cols; x += 2)
    {
        cv::line(check_h_plot_color, cv::Point(x, 0), cv::Point(x, check_h_plot_color.rows), cv::Scalar(0, 0, 255), 1); // Red vertical lines
    }
    for (int y = 0; y <= check_h_plot_color.rows; y += 2)
    {
        cv::line(check_h_plot_color, cv::Point(0, y), cv::Point(check_h_plot_color.cols, y), cv::Scalar(0, 0, 255), 1); // Red horizontal lines
    }

    // Optional: Zoom for visibility
    cv::Mat zoomed;
    cv::resize(check_h_plot_color, zoomed, cv::Size(), cell_size, cell_size, cv::INTER_NEAREST);

    cv::imshow("Pixel Grid", zoomed);

    cv::imshow("vertical check", check_w_plot);
    // cv::imshow("horizontal check", check_h_plot);
    cv::waitKey(0);

    //-----------------------------------------------------------------

    return b_box_dvs;
}

Bbox dvs_roi_proposed(
    const cv::Mat &frame,
    const int roi_event_score,         // Event Score for each events
    const int row_score_threshold,     // Minimum number of events for a row to be considered active
    const int roi_height_min_threshold // Minimum height of the ROI in rows
)
{
    const int frame_h = 720, frame_w = 960;
    Bbox b_box_dvs = {0, 0, 0, 0};

    // //---------------------------------------------------
    // cv::Mat overlay;
    // cv::cvtColor(frame, overlay, cv::COLOR_GRAY2BGR); // Convert to BGR for colored drawing
    // int *y_score = (int *)calloc(frame_h, sizeof(int));
    // int plot_width = 200;
    // int axis_width = 60;                                                                  // extra margin for y-axis labels
    // cv::Mat y_plot(frame_h, plot_width + axis_width, CV_8UC3, cv::Scalar(255, 255, 255)); // color plot
    // //---------------------------------------------------
    // global (x_min, x_max, y_min, y_max)
    int global_x_min = frame_w, global_x_max = 0;
    int global_y_min = 0, global_y_max = -1;
    // candiate (x_min, x_max)
    int candidate_x_min = frame_w, candidate_x_max = 0;

    int roi_row_streak = 0;
    for (int h = 0; h < frame_h; h++)
    {
        // local max score
        int local_max_score = 0;
        int local_max_left = frame_w, local_max_right = 0;
        // local current score
        int local_cur_score = 0;
        int local_cur_left = 0;

        const uchar *row_ptr = frame.ptr<uchar>(h);
        // incremental algorithm
        for (int w = 0; w < frame_w; w += 4)
        {
            // record local_cur_score (increment)
            local_cur_score += (row_ptr[w] != 128) ? roi_event_score : -1;
            local_cur_score += (row_ptr[w + 1] != 128) ? roi_event_score : -1;
            local_cur_score += (row_ptr[w + 2] != 128) ? roi_event_score : -1;
            local_cur_score += (row_ptr[w + 3] != 128) ? roi_event_score : -1;

            if (local_cur_score < 0)
            {
                local_cur_score = 0;
                local_cur_left = w + 3;
            }

            // record max sequnce (score, left, right)
            if (local_cur_score > local_max_score)
            {
                local_max_score = local_cur_score;
                local_max_left = local_cur_left;
                local_max_right = w + 3;
            }
        }

        if (local_max_score >= row_score_threshold)
        {
            // //---------------------------------------------------
            // cv::Point start(local_max_left, h);
            // cv::Point end(local_max_right, h);
            // cv::arrowedLine(overlay, start, end, cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.2);
            // y_score[h] = local_max_score;
            // //---------------------------------------------------
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
    }

    if (global_y_max != -1)
    {
        // roi detected!!
        draw_square_roi(&b_box_dvs, global_x_min, global_y_min, global_x_max, global_y_max, frame_w, frame_h);
    }

    //---------------------------------------------------
    // cv::imshow("Proposed ROI Arrow Visualization", overlay);

    // int y_score_max = *std::max_element(y_score, y_score + frame_h);
    // for (int h = 1; h < frame_h; h++)
    // {
    //     int x1 = static_cast<int>(plot_width * y_score[h - 1] / (float)y_score_max);
    //     int x2 = static_cast<int>(plot_width * y_score[h] / (float)y_score_max);
    //     cv::line(y_plot,
    //              cv::Point(axis_width + x1, h - 1),
    //              cv::Point(axis_width + x2, h),
    //              cv::Scalar(0, 0, 0), 1);
    // }
    // for (int x = 0; x <= plot_width; x += 30)
    // {
    //     int score_val = static_cast<int>(y_score_max * (x / (float)plot_width));
    //     cv::line(y_plot,
    //              cv::Point(axis_width + x, 0),
    //              cv::Point(axis_width + x, frame_h),
    //              cv::Scalar(0, 0, 0), 1);
    //     cv::putText(y_plot, std::to_string(score_val),
    //                 cv::Point(axis_width + x - 10, frame_h - 10),
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
    // }
    // cv::imshow("Max Score", y_plot);
    // cv::waitKey(0);
    //---------------------------------------------------

    return b_box_dvs;
}

std::vector<Bbox> dvs_roi_proposed_multiobject(
    const cv::Mat &frame,
    const int roi_event_score,
    const int row_score_threshold,
    const int roi_height_min_threshold,
    const int max_vertical_gap,
    const int min_roi_width,
    const int min_roi_height)
{
    const int frame_h = frame.rows;
    const int frame_w = frame.cols;

    std::vector<RowStreak> all_streaks;

    // 1. Extract horizontal streaks from each row
    for (int h = 0; h < frame_h; h++)
    {
        const uchar *row_ptr = frame.ptr<uchar>(h);

        int score = 0;
        int streak_start = 0;
        for (int w = 0; w < frame_w; w++)
        {
            if (row_ptr[w] != 128)
                score += roi_event_score;
            else
                score--;

            if (score < 0)
            {
                score = 0;
                streak_start = w + 1;
            }
            else if (score >= row_score_threshold && w - streak_start > 5)
            {
                all_streaks.push_back({h, streak_start, w});
                score = 0;
                streak_start = w + 1;
            }
        }
    }

    //-------------------------------------------------------
    // Optional: visualize all streaks
    cv::Mat streak_vis;
    cv::cvtColor(frame, streak_vis, cv::COLOR_GRAY2BGR);
    for (const auto &streak : all_streaks)
    {
        cv::line(
            streak_vis,
            cv::Point(streak.left, streak.row),
            cv::Point(streak.right, streak.row),
            cv::Scalar(0, 0, 255),
            1);
    }
    cv::imshow("Streak Visualization", streak_vis);
    cv::waitKey(0);
    //-------------------------------------------------------

    // 2. Group streaks by horizontal overlap
    std::vector<std::vector<RowStreak>> clusters;

    for (const auto &streak : all_streaks)
    {
        std::vector<int> overlapping_clusters;

        for (int i = 0; i < clusters.size(); ++i)
        {
            const auto &cluster = clusters[i];
            if (cluster.empty())
                continue;

            int last_row = cluster.back().row;

            // Allow vertical continuation within max_vertical_gap
            if (streak.row - last_row > max_vertical_gap)
                continue;

            for (const auto &s : cluster)
            {
                if (streak.row > s.row + max_vertical_gap)
                    continue;

                if (streak.right >= s.left && streak.left <= s.right)
                {
                    overlapping_clusters.push_back(i);
                    break;
                }
            }
        }

        if (overlapping_clusters.empty())
        {
            clusters.push_back({streak});
        }
        else
        {
            int target_idx = overlapping_clusters[0];
            auto &target_cluster = clusters[target_idx];
            for (int j = 1; j < overlapping_clusters.size(); ++j)
            {
                int idx = overlapping_clusters[j];
                auto &src_cluster = clusters[idx];
                target_cluster.insert(
                    target_cluster.end(),
                    src_cluster.begin(),
                    src_cluster.end());
                src_cluster.clear(); // mark as empty
            }

            target_cluster.push_back(streak); // add to target

            clusters.erase(
                std::remove_if(clusters.begin(), clusters.end(),
                               [](const std::vector<RowStreak> &c)
                               { return c.empty(); }),
                clusters.end());
        }
    }

    // 3. Build bounding boxes from clusters
    std::vector<Bbox> rois;
    for (const auto &cluster : clusters)
    {
        if (cluster.size() >= roi_height_min_threshold)
        {
            int x_min = frame_w, x_max = 0;
            int y_min = cluster.front().row;
            int y_max = cluster.back().row;

            for (const auto &streak : cluster)
            {
                x_min = std::min(x_min, streak.left);
                x_max = std::max(x_max, streak.right);
            }

            int box_width = x_max - x_min + 1;
            int box_height = y_max - y_min + 1;

            if (box_width >= min_roi_width && box_height >= min_roi_height)
            {
                rois.push_back({x_min, y_min, x_max, y_max});
            }
        }
    }

    return rois;
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