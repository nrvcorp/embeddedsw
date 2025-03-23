
#include "dvs_roi_alg.hpp"
#include "bbox.hpp"

int dvs_roi_average_based(char *img_file_pth)
{
    cv::Mat frame = cv::imread(img_file_pth, cv::IMREAD_GRAYSCALE);
    if (frame.empty())
    {
        std::cerr << "Image not found!" << std::endl;
        return -1;
    }
    int frame_h = 720, frame_w = 960;

    // buffers to count the number of events per column and row
    int *x_count = (int *)calloc(frame_w, sizeof(int));
    int *y_count = (int *)calloc(frame_h, sizeof(int));

    int sum = 0;

    int x_avg = sum / frame_w;
    if (x_avg < 5)
        x_avg = 5;
    int x_thresh_count = 0;
    int x_min = 0, x_max = 0;

    for (int w = 0; w < frame_w; w++)
    {
        if (x_count[w] > x_avg)
        {
            x_thresh_count++;
            // check consecutive <roi_line_width> num of columns with numer of events above average
            if (x_thresh_count >= roi_line_width)
            {
                if (x_min == 0)
                {
                    // set as ROI left boundary
                    x_min = w - roi_line_width + 1;
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
            // if no consecutive <roi_line_width> columns exist, reset count
            x_thresh_count = 0;
        }
    }

    // check rows with number of events above average
    int y_avg = sum / frame_h;
    // to reduce noise, set minimum of events to 5
    if (y_avg < 5)
        y_avg = 5;
    int y_thresh_count = 0;
    int y_min = 0, y_max = 0;
    for (int h = 0; h < frame_h; h++)
    {
        if (y_count[h] > y_avg)
        {
            y_thresh_count++;
            // check consecutive <roi_line_width> num of rows with number of events above average
            if (y_thresh_count >= roi_line_width)
            {
                if (y_min == 0)
                {
                    // set as ROI top boundary
                    y_min = h - roi_line_width + 1;
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
            // if no consecutive <roi_line_width> rows exist, reset count
            y_thresh_count = 0;
        }
    }

    if (x_min != 0 && y_min != 0)
    {
        // if valid ROI is detected
        draw_square_roi()
    }

    return 0;
}

int dvs_roi_proposed(
    const char *image_path,      // Path to the DVS image file
    int row_score_threshold,     // Minimum number of events for a row to be considered active
    int roi_height_min_threshold // Minimum height of the ROI in rows
)
{
    cv::Mat frame = cv::imread(img_file_pth, cv::IMREAD_GRAYSCALE);
    if (frame.empty())
    {
        std::cerr << "Image not found!" << std::endl;
        return -1;
    }

    int global_x_min = frame_w, global_x_max = 0;
    int global_y_min = 0, global_y_max = -1;

    int roi_row_streak = 0;

    // candiate
    int candidate_x_min = frame_w, candidate_x_max = 0;

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
        draw_square_roi(dvs, global_x_min, global_y_min, global_x_max, global_y_max, frame_w, frame_h);
        return 1;
    }
    else
    {
        // roi not detected!!
        return 0;
    }
}

void draw_square_roi(Bbox *b_box, int x_min, int y_min,
                     int x_max, int y_max, int width, int height)
{
    // draw square ROI around given x and y coordinate ranges
    // while making sure ROI doesn't exceed the entire frame

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
    int inflated_size = (int)(roi_inflation_ratio * (x_max - x_min));
    if (inflated_size > height)
    {
        inflated_size = height;
    }
    else if (inflated_size < roi_min_size)
    {
        inflated_size = roi_min_size;
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
    int min_offset = (inflated_size - (x_max - x_min)) >> 1;

    // if ROI exceeds frame, move it inside
    if (x_min - min_offset < 0)
    {
        b_box->lx = 0;
        b_box->hx = inflated_size - 1;
    }
    else if (x_min + inflated_size - min_offset >= width)
    {
        b_box->hx = width;
        b_box->lx = width - inflated_size + 1;
    }
    else
    {
        b_box->lx = x_min - min_offset;
        b_box->hx = x_min - min_offset + inflated_size - 1;
    }

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