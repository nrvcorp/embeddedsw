
#include "bbox.hpp"
#include "DVS.hpp"
#include "PCIe.hpp"
#include "MutexManager.hpp"

DVS::DVS(
    int frame_h, int frame_w,
    bool is_header, int accum_num,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &mutexManager)
    : frame_h(frame_h), frame_w(frame_w), accum_num(accum_num),
      is_header(is_header), header_bytes(8),
      rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
      buffer_num(buffer_num),
      pcie(c2h_dev, h2c_dev),
      rd_ptr(0),
      mutexManager(mutexManager)
{

    frame_bytes = (is_header) ? (frame_h * frame_w) / 4 + 8 : (frame_h * frame_w) / 4;
    pixel_num = frame_h * frame_w;

    buffer_rdy_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    buffer_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    for (int i = 0; i < buffer_num; i++)
    {
        buffer_rdy_addr[i] = rdy_baseaddr + i;
        buffer_addr[i] = frame_baseaddr + ((frame_bytes)*i);
    }
    double_buffer = NULL;
    buffer = (char *)malloc(frame_bytes * sizeof(char));
    frame_start = (is_header) ? buffer + header_bytes : buffer;
    buffer_rdy = (char *)malloc(1 * sizeof(char));
    buffer_done = (char *)malloc(1 * sizeof(char));
    buffer_done[0] = 0x00;
}

DVS::DVS(
    int frame_h, int frame_w,
    bool is_header, int accum_num,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &mutexManager,
    MutexCond *mutexCond)
    : frame_h(frame_h), frame_w(frame_w), accum_num(accum_num),
      is_header(is_header), header_bytes(8),
      rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
      buffer_num(buffer_num),
      pcie(c2h_dev, h2c_dev),
      rd_ptr(0),
      mutexManager(mutexManager),
      mutexCond(mutexCond)
{

    frame_bytes = (is_header) ? (frame_h * frame_w) / 4 + 8 : (frame_h * frame_w) / 4;
    pixel_num = frame_h * frame_w;

    buffer_rdy_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    buffer_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    for (int i = 0; i < buffer_num; i++)
    {
        buffer_rdy_addr[i] = rdy_baseaddr + i;
        buffer_addr[i] = frame_baseaddr + ((frame_bytes)*i);
    }

    buffer = (char *)malloc(frame_bytes * sizeof(char));
    double_buffer = (char *)malloc(frame_bytes * sizeof(char));
    frame_start = (is_header) ? buffer + header_bytes : buffer;
    buffer_rdy = (char *)malloc(1 * sizeof(char));
    buffer_done = (char *)malloc(1 * sizeof(char));
    buffer_done[0] = 0x00;
}

DVS::DVS(
    int frame_h, int frame_w,
    bool is_header, int accum_num,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &mutexManager, int roi_thresh, int roi_min_size,
    float roi_inflation_ratio, Bbox *bbox, MutexCond *mutexCond = NULL)
    : frame_h(frame_h), frame_w(frame_w), accum_num(accum_num),
      is_header(is_header), header_bytes(8),
      rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
      buffer_num(buffer_num),
      pcie(c2h_dev, h2c_dev),
      rd_ptr(0),
      mutexManager(mutexManager),
      roi_thresh(roi_thresh),
      roi_min_size(roi_min_size),
      roi_inflation_ratio(roi_inflation_ratio),
      bbox(bbox),
      mutexCond(mutexCond)
{

    frame_bytes = (is_header) ? (frame_h * frame_w) / 4 + 8 : (frame_h * frame_w) / 4;
    pixel_num = frame_h * frame_w;

    buffer_rdy_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    buffer_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    for (int i = 0; i < buffer_num; i++)
    {
        buffer_rdy_addr[i] = rdy_baseaddr + i;
        buffer_addr[i] = frame_baseaddr + ((frame_bytes)*i);
    }
    double_buffer = NULL;
    buffer = (char *)malloc(frame_bytes * sizeof(char));
    frame_start = (is_header) ? buffer + header_bytes : buffer;
    buffer_rdy = (char *)malloc(1 * sizeof(char));
    buffer_done = (char *)malloc(1 * sizeof(char));
    buffer_done[0] = 0x00;
}

// Function to convert 2-bit image data to 8-bit
void DVS::convert2BitTo8Bit()
{
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        uint8_t pixel = (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
        // dst[i] = pixel * 85;                                  // Map 2-bit value to 8-bit value (0, 85, 170, 255)
        frame.data[i] = (pixel == 0) ? 128 : // no event
                            (pixel == 1) ? 255
                                         : // on event
                            (pixel == 2) ? 0
                                         : // off event
                            0;
    }
}

void DVS::convert2BitTo8Bit_accum()
{
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        uint8_t pixel = (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
        if (pixel == 1)
        {
            frame.data[i] = 255;
        }
        else if (pixel == 2)
        {
            frame.data[i] = 0;
        }
    }
}

void DVS::convert2BitToBGR_accum()
{
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        int pixel = (frame_start[byteIndex] >> bitOffset) & 0x03;
        if (pixel == 1)
        {
            // red
            frame.data[i * 3] = (frame.data[i * 3] < 40) ? 0 : (frame.data[i * 3] - 40);
            frame.data[i * 3 + 2] = (frame.data[i * 3 + 2] + 40 > 255) ? 255 : (frame.data[i * 3 + 2] + 40);
        }
        else if (pixel == 2)
        {
            // blue
            frame.data[i * 3] = (frame.data[i * 3] + 40 > 255) ? 255 : (frame.data[i * 3] + 40);
            frame.data[i * 3 + 2] = (frame.data[i * 3 + 2] < 40) ? 0 : (frame.data[i * 3 + 2] - 40);
        }
    }
}

void DVS::decode_header(const char *buffer, int &frame_num, uint32_t &timestamp)
{
    // Extract frame number from buffer
    frame_num = ((static_cast<unsigned char>(buffer[7]) << 24) |
                 (static_cast<unsigned char>(buffer[6]) << 16) |
                 (static_cast<unsigned char>(buffer[5]) << 8) |
                 static_cast<unsigned char>(buffer[4]));

    // Extract timestamp from buffer
    timestamp = ((static_cast<unsigned char>(buffer[3]) << 24) |
                 (static_cast<unsigned char>(buffer[2]) << 16) |
                 (static_cast<unsigned char>(buffer[1]) << 8) |
                 static_cast<unsigned char>(buffer[0]));
}

void DVS::display_stream()
{

    while (true)
    {
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);

        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            while (true)
            {
                pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);
                if ((buffer_rdy[0] & 0x01) == 1)
                {
                    break;
                }
            }
            pcie.c2h(buffer, frame_bytes, buffer_addr[rd_ptr]);
            pcie.h2c(buffer_done, 1, buffer_rdy_addr[rd_ptr]);
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }

            rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
        }

        mutexManager.lock_display();
        cv::imshow("DVS camera", frame);
        if (cv::waitKey(1) == 27)
        {
            mutexManager.unlock_display();
            frame.release();
            break;
        }
        mutexManager.unlock_display();
    }
}

void DVS::check_frame_drop()
{
    int prev_frame_num;
    int prev_timestamp;
    int check_init = 0;
    int error_num = 0;

    int frame_num;
    uint32_t timestamp;
    while (1)
    {
        if (check_init < 1000)
        {
            check_init++;
        }

        pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);

        if ((buffer_rdy[0] & 0x01) == 1)
        { // buffer ready
            pcie.c2h(buffer, frame_bytes, buffer_addr[rd_ptr]);
            pcie.h2c(buffer_done, 1, buffer_rdy_addr[rd_ptr]);

            decode_header(buffer, frame_num, timestamp);
            // std::cout << "frame_pointer value: " << std::dec << rd_ptr  << " ";
            // std::cout << "frame_num value (decimal): " << std::dec << frame_num << "(hex): 0x" << std::hex << frame_num << "  ";
            // std::cout << "timestamp value (decimal): " << std::dec << timestamp << "(hex): 0x" << std::hex << timestamp << std::endl;
            if ((prev_frame_num + 1 != frame_num) && (prev_frame_num != (frame_num + 255)))
            {
                if (!(check_init < 1000))
                {
                    error_num++;
                    std::cout << "ERROR NUM: " << std::dec << error_num << std::endl;
                }
            }

            prev_frame_num = frame_num;
            prev_timestamp = timestamp;

            rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
        }
    }
}

void *DVS::double_buf_reader()
// note : mutexCond needs to be an array like MutexCond[2]
{
    int prev_frame_num;
    int prev_timestamp;
    int check_init = 0;
    int error_num = 0;
    int double_buffer_idx = 0;
    int frame_num;
    uint32_t timestamp;
    char *dvs_buffer;
    while (1)
    {
        if (check_init < 1000)
        {
            check_init++;
        }
        if (double_buffer_idx == 0)
        {
            mutexCond[0].lock_single_writer();
            dvs_buffer = buffer;
        }
        else
        {
            mutexCond[1].lock_single_writer();
            dvs_buffer = double_buffer;
        }
        pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);

        if ((buffer_rdy[0] & 0x01) == 1)
        { // buffer ready
            pcie.c2h(dvs_buffer, frame_bytes, buffer_addr[rd_ptr]);
            pcie.h2c(buffer_done, 1, buffer_rdy_addr[rd_ptr]);

            decode_header(dvs_buffer, frame_num, timestamp);
            if ((prev_frame_num + 1 != frame_num) && (prev_frame_num != (frame_num + 255)))
            {
                if (!(check_init < 1000))
                {
                    error_num++;
                    std::cout << "ERROR NUM: " << std::dec << error_num << std::endl;
                }
            }

            prev_frame_num = frame_num;
            prev_timestamp = timestamp;

            rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
            if (double_buffer_idx == 0)
            {
                mutexCond[0].unlock_single_writer();
                double_buffer_idx = 1;
            }
            else
            {
                mutexCond[1].unlock_single_writer();
                double_buffer_idx = 0;
            }
        }
    }
}

void *DVS::double_buf_bin_writer()
{
    char *bin_name = NULL;

    time_t current_time;
    struct tm *local_time;

    time(&current_time);
    local_time = localtime(&current_time);
    if (asprintf(&bin_name, "./bin_files/data_%04d-%02d-%02d_%02d-%02d-%02d.bin",
                 local_time->tm_year + 1900,
                 local_time->tm_mon + 1,
                 local_time->tm_mday,
                 local_time->tm_hour,
                 local_time->tm_min,
                 local_time->tm_sec) == -1)
    {
        perror("Error creating bin file name");
        return nullptr;
    }

    std::ofstream file(bin_name, std::ios::binary | std::ios::app);
    if (!file.is_open())
    {
        perror("Failed to open file for writing.");
        return nullptr;
    }

    while (1)
    {
        mutexCond[0].lock_multiple_reader();
        file.write(buffer, frame_bytes);
        mutexCond[0].unlock_multiple_reader();

        mutexCond[1].lock_multiple_reader();
        file.write(double_buffer, frame_bytes);
        mutexCond[1].unlock_multiple_reader();
    }

    file.close();
    free(bin_name);
    return nullptr;
}

void DVS::crop_coord()
{
    while (true)
    {
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
        int *x_count = (int *)calloc(frame_w, sizeof(int));
        int *y_count = (int *)calloc(frame_h, sizeof(int));
        int sum = 0;
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            while (true)
            {
                pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);
                if ((buffer_rdy[0] & 0x01) == 1)
                {
                    break;
                }
            }
            pcie.c2h(buffer, frame_bytes, buffer_addr[rd_ptr]);
            pcie.h2c(buffer_done, 1, buffer_rdy_addr[rd_ptr]);
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }
            sum += event_accum(buffer + 8, x_count, y_count, frame_w, frame_h);
            rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
        }
        Bbox b_box;
        int is_roi = event_roi(x_count, y_count, frame_w, frame_h, sum, &b_box);
        free(x_count);
        free(y_count);
        x_count = NULL;
        y_count = NULL;
        cv::Point p1(b_box.lx, b_box.ly);
        cv::Point p2(b_box.hx, b_box.hy);
        mutexCond->lock_single_writer();
        if (is_roi)
        {
            bbox->lx = b_box.lx;
            bbox->ly = b_box.ly;
            bbox->hx = b_box.hx;
            bbox->hy = b_box.hy;
        }
        cv::rectangle(frame, p1, p2, cv::Scalar(255), 2, cv::LINE_8);
        mutexCond->unlock_single_writer(is_roi);
        mutexManager.lock_display();
        cv::imshow("DVS camera", frame);
        if (cv::waitKey(1) == 27)
        {
            printf("releasing frame\n");
            frame.release();
            mutexManager.unlock_display();
            printf("finished releasing");
            break;
        }
        mutexManager.unlock_display();
    }
}
int DVS::event_accum(char *src, int *x_count, int *y_count, int width, int height)
{
    int sum = 0;
    for (int h = 0; h < height; h++)
    {
        for (int byteIndex = 0; byteIndex < (width >> 2); ++byteIndex)
        {
            for (int bitOffset = 0; bitOffset < 8; bitOffset += 2)
            {
                uint8_t pixel = (src[h * (width >> 2) + byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
                if (pixel != 0)
                {
                    x_count[(byteIndex << 2) | (bitOffset >> 1)]++;
                    y_count[h]++;
                    sum++;
                }
            }
        }
    }
    return sum;
}

int DVS::event_roi(int *x_count, int *y_count, int width, int height, int sum, Bbox *b_box)
{
    int x_avg = sum / width;
    int x_thresh_count = 0;
    int x_min = 0, x_max = 0;
    for (int w = 0; w < width; w++)
    {
        if (x_count[w] > x_avg)
        {
            x_thresh_count++;
            if (x_thresh_count >= roi_thresh)
            {
                if (x_min == 0)
                {
                    x_min = w - roi_thresh + 1;
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
            x_thresh_count = 0;
        }
    }

    int y_avg = sum / height;
    int y_thresh_count = 0;
    int y_min = 0, y_max = 0;
    for (int h = 0; h < height; h++)
    {
        if (y_count[h] > y_avg)
        {
            y_thresh_count++;
            if (y_thresh_count >= roi_thresh)
            {
                if (y_min == 0)
                {
                    y_min = h - roi_thresh + 1;
                    y_max = h;
                }
                else if (h > y_max)
                {
                    y_max = h;
                }
            }
        }
        else
        {
            y_thresh_count = 0;
        }
    }
    int inflated_size = (int)(roi_inflation_ratio * (x_max - x_min));
    if (inflated_size > height)
    {
        inflated_size = height;
    }
    else if (inflated_size < roi_min_size)
    {
        inflated_size = roi_min_size;
    }
    int min_offset = (inflated_size - (x_max - x_min)) / 2;
    if (x_min - min_offset < 0)
    {
        b_box->lx = 0;
        b_box->hx = inflated_size - 1;
    }
    else if (y_min + inflated_size - min_offset >= width)
    {
        b_box->hx = width;
        b_box->lx = width - inflated_size + 1;
    }
    else
    {
        b_box->lx = x_min - min_offset;
        b_box->hx = x_min - min_offset + inflated_size - 1;
    }

    if (y_min - min_offset < 0)
    {
        b_box->ly = 0;
        b_box->hy = inflated_size - 1;
    }
    else if (y_min + inflated_size - min_offset >= height)
    {
        b_box->hy = height;
        b_box->ly = height - inflated_size + 1;
    }
    else
    {
        b_box->ly = y_min - min_offset;
        b_box->hy = y_min - min_offset + inflated_size - 1;
    }
    return (x_min != 0 || y_min != 0);
}

DVS::~DVS()
{
    printf("Deconstructor\n");
    if (double_buffer != NULL)
        free(double_buffer);
    free(buffer);
    free(buffer_rdy);
    free(buffer_done);
}