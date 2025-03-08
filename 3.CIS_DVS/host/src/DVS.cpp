
#include "bbox.hpp"
#include "DVS.hpp"
#include "PCIe.hpp"
#include "MutexManager.hpp"


DVS::DVS( //normal constructor
    int frame_h, int frame_w,
    bool is_header, int accum_num,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &display_mutex, bool double_buffering, int display_downsample_num)
    : frame_h(frame_h), frame_w(frame_w), accum_num(accum_num),
      is_header(is_header), header_bytes(8),
      rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
      buffer_num(buffer_num),
      pcie(c2h_dev, h2c_dev),
      rd_ptr(0),
      display_mutex(display_mutex),
      terminate(NULL),
      display_downsample_num(display_downsample_num)
{
    //set total frame bytes
    frame_bytes = (is_header) ? (frame_h * frame_w) / 4 + 8 : (frame_h * frame_w) / 4;

    //set total pixel num
    pixel_num = frame_h * frame_w;

    //pre-calculate buffer address for ZCU106 PCIE XDMA access
    buffer_rdy_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    buffer_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    for (int i = 0; i < buffer_num; i++)
    {
        buffer_rdy_addr[i] = rdy_baseaddr + i;
        buffer_addr[i] = frame_baseaddr + ((frame_bytes)*i);
    }
    buffer_rdy = (char *)malloc(1 * sizeof(char));
    buffer_done = (char *)malloc(1 * sizeof(char));
    buffer_done[0] = 0x00;
    
    //allocate buffers
    buffer = (char *)malloc(frame_bytes * sizeof(char));
    
    //set data pointer behind header
    frame_start = (is_header) ? buffer + header_bytes : buffer;
    
    //allocate mutex and buffer for double buffering 
    if(!double_buffering){
        dbuf_mutex[0] = NULL;
        dbuf_mutex[1] = NULL;
        double_buffer = NULL;
    }else{
        double_buffer = (char *)malloc(frame_bytes * sizeof(char));
        dbuf_mutex[0] = new MutexManager();
        dbuf_mutex[1] = new MutexManager();
        terminate = new bool;
    }

    //don't init CIS related params right now
    convert_cis = false;
}



DVS::DVS( // crop_roi constructor
    int frame_h, int frame_w,
    bool is_header, int accum_num,
    uintptr_t rdy_baseaddr, uintptr_t frame_baseaddr,
    int buffer_num,
    const char *c2h_dev, const char *h2c_dev,
    MutexManager &display_mutex, Bbox *bbox, MutexManager *thread_mutex,
    bool* terminate, int display_downsample_num)
    : frame_h(frame_h), frame_w(frame_w), accum_num(accum_num),
      is_header(is_header), header_bytes(8),
      rdy_baseaddr(rdy_baseaddr), frame_baseaddr(frame_baseaddr),
      buffer_num(buffer_num),
      pcie(c2h_dev, h2c_dev),
      rd_ptr(0),
      display_mutex(display_mutex),
      bbox(bbox),
      thread_mutex(thread_mutex),
      terminate(terminate),
      display_downsample_num(display_downsample_num)
{
    //set total frame bytes
    frame_bytes = (is_header) ? (frame_h * frame_w) / 4 + 8 : (frame_h * frame_w) / 4;
    //set total pixel num
    pixel_num = frame_h * frame_w;

    //pre-calculate buffer address for ZCU106 PCIE XDMA access
    buffer_rdy_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    buffer_addr = (uintptr_t *)malloc(buffer_num * sizeof(uintptr_t));
    for (int i = 0; i < buffer_num; i++)
    {
        buffer_rdy_addr[i] = rdy_baseaddr + i;
        buffer_addr[i] = frame_baseaddr + ((frame_bytes)*i);
    }
    buffer_rdy = (char *)malloc(1 * sizeof(char));
    buffer_done = (char *)malloc(1 * sizeof(char));
    buffer_done[0] = 0x00;
    
    //allocate buffer
    buffer = (char *)malloc(frame_bytes * sizeof(char));
    
    //set data pointer behind header
    frame_start = (is_header) ? buffer + header_bytes : buffer;

    //disable double buffering
    double_buffer = NULL;
    dbuf_mutex[0] = NULL;
    dbuf_mutex[1] = NULL;

    //don't init CIS related params right now
    convert_cis = false;
}


void DVS::convert2BitTo8Bit()
{
    //for each pixel
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        uint8_t pixel = (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits
        // dst[i] = pixel * 85;
        // Map 2-bit value to 8-bit value (0, 85, 170, 255)
        frame.data[i] = (pixel == 0) ? 128 : // no event
                            (pixel == 1) ? 255
                                         : // on event
                            (pixel == 2) ? 0
                                         : // off event
                            0;
    }
}

void DVS::convert2BitToBR(){
    //for BGR
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        int pixel = (frame_start[byteIndex] >> bitOffset) & 0x03;
        if (pixel == 1)
        {
            // for off events, bias current pixel closer to red
            frame.data[i * 3] = 255;
            frame.data[i * 3 + 1] = 0;
            frame.data[i * 3 + 2] = 0; 
        }
        else if (pixel == 2)
        {
            // for off events, bias current pixel closer to blue
            frame.data[i * 3] = 0;
            frame.data[i * 3 + 1] = 0;
            frame.data[i * 3 + 2] = 255;
        }else{
            frame.data[i * 3] = 255;
            frame.data[i * 3 + 1] = 255;
            frame.data[i * 3 + 2] = 255;
        }
    }
}

void DVS::convert2BitToBR_accum(){
    //for BGR
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        int pixel = (frame_start[byteIndex] >> bitOffset) & 0x03;
        if (pixel == 1)
        {
            // for off events, bias current pixel closer to red
            frame.data[i * 3] = 255;
            frame.data[i * 3 + 1] = 0;
            frame.data[i * 3 + 2] = 0; 
        }
        else if (pixel == 2)
        {
            // for off events, bias current pixel closer to blue
            frame.data[i * 3] = 0;
            frame.data[i * 3 + 1] = 0;
            frame.data[i * 3 + 2] = 255;
        }
    }
}
void DVS::convert2BitTo8Bit_accum()
{
    //for each pixel
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        uint8_t pixel = (frame_start[byteIndex] >> bitOffset) & 0x03; // Extract 2 bits

        //simply stack events on top of previous frames
        //can be replaced with an alternate image processing method
        if (pixel == 1)
        {
             //for on events, set grayscale pixel to 255
            frame.data[i] = 255;
        }
        else if (pixel == 2)
        {
            //for off events, set grayscale pixel to 0
            frame.data[i] = 0;
        }
    }
}


void DVS::convert2BitToBGR_accum()
{
    //for each pixel 
    for (int i = 0; i < pixel_num; ++i)
    {
        int byteIndex = i / 4;
        int bitOffset = (i % 4) * 2;
        int pixel = (frame_start[byteIndex] >> bitOffset) & 0x03;
        if (pixel == 1)
        {
            //for on events, bias current pixel closer to red
            frame.data[i * 3] = (frame.data[i * 3] < 40) ? 0 : (frame.data[i * 3] - 40);
            frame.data[i * 3 + 2] = (frame.data[i * 3 + 2] + 40 > 255) ? 255 : (frame.data[i * 3 + 2] + 40);
        }
        else if (pixel == 2)
        {
            // for off events, bias current pixel closer to blue
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

void DVS::read_frame(char* dvs_buffer){
    // wait for ready flag 
    // by polling through PCIE connection 
    while (true)
    {
        pcie.c2h(buffer_rdy, 1, buffer_rdy_addr[rd_ptr]);
        if ((buffer_rdy[0] & 0x01) == 1)
        {
            break;
        }
    }

    //read DVS frame through PCIE
    pcie.c2h(dvs_buffer, frame_bytes, buffer_addr[rd_ptr]);

    //set flag to DONE through PCIE
    pcie.h2c(buffer_done, 1, buffer_rdy_addr[rd_ptr]);

    //change the address for ready flag and DVS frame
    rd_ptr = (rd_ptr == buffer_num - 1) ? 0 : rd_ptr + 1;
}

void DVS::calc_fps(double &fps, int &frameCount, double &startTime, cv::Mat &frame) {
    frameCount+=accum_num * display_downsample_num;
    double end = cv::getTickCount();
    double elapsedTime = (end - startTime) / cv::getTickFrequency();
    // auto end = (std::chrono::high_resolution_clock::now());
    // auto elapsedTime = (std::chrono::duration_cast<std::chrono::microseconds>(end - startTime)) ;
    if (elapsedTime>= 1.0) {
        fps = frameCount/elapsedTime;
        frameCount = 0;
        startTime = end;
    }

    std::ostringstream oss;
    oss << "FPS: " << static_cast<int>(fps);
    cv::putText(frame, oss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
}


void DVS::display_stream(bool is_flip)
{
    // double fps = 0.0;
    // int frameCount = 0;
    // double startTime = cv::getTickCount();
    while (true)
    {
        //initialize cv::Mat frame
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);

        //stack frames using member functions
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            read_frame(buffer);
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }
        }


        //show image
        if(is_flip){
            cv::flip(frame,frame,0);
        }
        // calc_fps(fps, frameCount, startTime, frame);

        
        //display using mutex locking 
        display_mutex.lock_display();
        
        cv::imshow("DVS camera", frame);

        //press ESC to quit
        if (cv::waitKey(1) == 27)
        {
            display_mutex.unlock_display();

            //free frame 
            frame.release();
            break;
        }
        display_mutex.unlock_display();
    }
}

void DVS::double_buf_display_fps_writer()
// note : thread_mutex needs to be an array like thread_mutex[2]
{
    int prev_frame_num;
    int prev_timestamp;
    int check_init = 0;
    int error_num = 0;
    int double_buffer_idx = 0;
    int frame_num;
    uint32_t timestamp;
    char *dvs_buffer;
    //while reading raw sensor data, check frame num consistency too
    while (1)
    {
        if (check_init < 1000)
        {
            check_init++;
        }
        //maintain 2 buffers and 2 mutexes for double buffering
        if (double_buffer_idx == 0)
        {
            dbuf_mutex[0]->lock_single_writer();
            dvs_buffer = buffer;
        }
        else
        {
            dbuf_mutex[1]->lock_single_writer();
            dvs_buffer = double_buffer;
        }
        for(int i= 0; i<display_downsample_num; i++){
            //read raw data
            read_frame(dvs_buffer);

            //check frame num consistency
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
        }


        //alternate between the two per every loop
        if (double_buffer_idx == 0)
        {
            dbuf_mutex[0]->unlock_single_writer(1);
            double_buffer_idx = 1;
        }
        else
        {
            dbuf_mutex[1]->unlock_single_writer(1);
            double_buffer_idx = 0;
        }
        if(*terminate) break;
    }
}

void DVS::double_buf_display_fps_reader(bool is_flip)
{
    double fps = 0.0;
    int frameCount = 0;
    double startTime = cv::getTickCount();
    while (1)
    {
        
        //initialize cv::Mat frame
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
        
        //stack frames using member functions
        for (int frame_grp_num = 0; frame_grp_num < (accum_num / (2)); frame_grp_num++)
        {
            //lock mutex, write from buffer 0
            dbuf_mutex[0]->lock_multiple_reader();
           
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }
            dbuf_mutex[0]->unlock_multiple_reader();
            
            //lock mutex, write from buffer 1
            dbuf_mutex[1]->lock_multiple_reader();
            frame_start = (is_header) ? double_buffer + header_bytes : double_buffer;
            convert2BitTo8Bit_accum();
            frame_start = (is_header) ? buffer + header_bytes : buffer;
            dbuf_mutex[1]->unlock_multiple_reader();
        }


        //show image
        if(is_flip){
            cv::flip(frame,frame,0);
        }
        calc_fps(fps, frameCount, startTime, frame);

        
        //display using mutex locking 
        display_mutex.lock_display();
        
        cv::imshow("DVS camera", frame);

        if (*terminate || cv::waitKey(1) == 27)
        {
            *terminate = true;
            //cleanup
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        display_mutex.unlock_display();
        
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
    auto start = std::chrono::high_resolution_clock::now();

    while (1)
    {
        //wait some time after the sensor starts up
        if(check_init<1000){
            check_init++;
        }
        //get frame num and headers 
        read_frame(buffer);
        decode_header(buffer, frame_num, timestamp);
        // std::cout << "frame_pointer value: " << std::dec << rd_ptr  << " ";
        // std::cout << "frame_num value (decimal): " << std::dec << frame_num << "(hex): 0x" << std::hex << frame_num << "  ";
        // std::cout << "timestamp value (decimal): " << std::dec << timestamp << "(hex): 0x" << std::hex << timestamp << std::endl;

        //check if the frame num increments by 1 consistently
        if ((prev_frame_num + 1 != frame_num) && (prev_frame_num != (frame_num + 255)))
        {
            if (!(check_init < 1000))
            {
                //if frame num is inconsistent, print error message
                error_num++;
                std::cout << "ERROR NUM: " << std::dec << error_num << std::endl;
            }
        }

        prev_frame_num = frame_num;
        prev_timestamp = timestamp;
        // if(error_num > 1000) {
        //     break;
        // }

    }
    // std::cout << "ERROR NUM: " << std::dec << error_num << std::endl;
            
}
void DVS::fps_count(){
    int frame_count = 0;
    auto start = std::chrono::high_resolution_clock::now();

    while (1)
    {
        frame_count++;
        //read DVS frame
        read_frame(buffer);
        //whenever 4096 frames are read
        if(frame_count >= 4096){
            // Record the end time
            auto end = std::chrono::high_resolution_clock::now();

            // Calculate the elapsed time in microseconds
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            //calculate true fps
            std::cout << "FPS : " << ((4096*1000000.0)/(int)(elapsed.count())) << std::endl;
            start = end;
            frame_count = 0;
        }
    }
}
void *DVS::double_buf_reader()
// note : thread_mutex needs to be an array like thread_mutex[2]
{
    int prev_frame_num;
    int prev_timestamp;
    int check_init = 0;
    int error_num = 0;
    int double_buffer_idx = 0;
    int frame_num;
    uint32_t timestamp;
    char *dvs_buffer;

    //while reading raw sensor data, check frame num consistency too
    while (1)
    {
        if (check_init < 1000)
        {
            check_init++;
        }

        //maintain 2 buffers and 2 mutexes for double buffering
        if (double_buffer_idx == 0)
        {
            dbuf_mutex[0]->lock_single_writer();
            dvs_buffer = buffer;
        }
        else
        {
            dbuf_mutex[1]->lock_single_writer();
            dvs_buffer = double_buffer;
        }
        //read raw data
        read_frame(dvs_buffer);

        //check frame num consistency
        decode_header(dvs_buffer, frame_num, timestamp);
        if ((prev_frame_num + 1 != frame_num) && (prev_frame_num != (frame_num + 255)))
        {
            if (!(check_init < 1000))
            {
                error_num++;
                // std::cout << "ERROR NUM: " << std::dec << error_num << std::endl;
            }
        }

        prev_frame_num = frame_num;
        prev_timestamp = timestamp;

        //alternate between the two per every loop
        if (double_buffer_idx == 0)
        {
            dbuf_mutex[0]->unlock_single_writer(1);
            double_buffer_idx = 1;
        }
        else
        {
            dbuf_mutex[1]->unlock_single_writer(1);
            double_buffer_idx = 0;
        }
    }
}

void *DVS::double_buf_bin_writer()
{
    char *bin_name = NULL;

    time_t current_time;
    struct tm *local_time;

    //based on current time, determine the name of output file
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

    //open file descriptor to write bin file to 
    std::ofstream file(bin_name, std::ios::binary | std::ios::app);
    if (!file.is_open())
    {
        perror("Failed to open file for writing.");
        return nullptr;
    }

    while (1)
    {
        //lock mutex, write from buffer 0
        dbuf_mutex[0]->lock_multiple_reader();
        file.write(buffer, frame_bytes);
        dbuf_mutex[0]->unlock_multiple_reader();

        //lock mutex, write from buffer 1
        dbuf_mutex[1]->lock_multiple_reader();
        file.write(double_buffer, frame_bytes);
        dbuf_mutex[1]->unlock_multiple_reader();
    }

    file.close();
    free(bin_name);
    return nullptr;
}

void DVS::bin_to_vid(char* path_to_bin, char* output_vid_name)
{
    std::ifstream bin_file(path_to_bin, std::ios::binary);
    
    if (!bin_file) {
        std::cerr << "Error opening binary file!" << std::endl;
        return;
    }
    // std::ofstream vid_file(output_vid_name, std::ios_binary | std::ios_app);
    //  if (!vid_file.is_open())
    // {
    //     perror("Failed to open file for writing.");
    //     return nullptr;
    // }
    cv::VideoWriter videoWriter(output_vid_name, cv::VideoWriter::fourcc('M','J','P','G'),
                                10, cv::Size(frame_w, frame_h), false);
    while (bin_file.read(buffer, frame_bytes)) {
         std::streamsize bytesRead = bin_file.gcount();  // Get actual bytes read

        if (bytesRead < frame_bytes) {
            if (bin_file.eof()) {
                std::cout << "Reached end of file. Read only " << bytesRead << " bytes.\n";
            } else if (bin_file.fail() || bin_file.bad()) {
                std::cerr << "File read error occurred!\n";
            }
        } else {
            // std::cout << "Read " << bytesRead << " bytes successfully.\n";
            frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
            convert2BitTo8Bit();
            
            // If the format is BGR but needs conversion
            // cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

            videoWriter.write(frame);
        }
        
    }
    bin_file.close();
    videoWriter.release();
}


void DVS::crop_coord(int img_show, int is_update, bool is_flip)
{
    

    while (true)
    {
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
        //buffers to count the number of events per column and row
        int *x_count = (int *)calloc(frame_w, sizeof(int));
        int *y_count = (int *)calloc(frame_h, sizeof(int));

        //total number of events accumulated throughout several frames
        int sum = 0;

        //stack several frames and count their number of events 
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            read_frame(buffer);
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit();
            }
            else
            {
                convert2BitTo8Bit_accum();
            }
            sum += event_accum(x_count, y_count, is_flip);
        }
        Bbox b_box_dvs, b_box_cis;

        //calculate event ROI in the form of a bounding box
        int is_roi = event_roi(x_count, y_count, sum, &b_box_dvs, &b_box_cis);

        //cleanup
        free(x_count);
        free(y_count);
        x_count = NULL;
        y_count = NULL;

        //acquire mutex 
        thread_mutex->lock_single_writer();

        //if there are enough events to draw a ROI
        //write bounding box information in a shared struct variable
        if (is_roi)
        {
            bbox->lx = b_box_cis.lx;
            bbox->ly = b_box_cis.ly;
            bbox->hx = b_box_cis.hx;
            bbox->hy = b_box_cis.hy;
        }else if(!is_update){
            bbox->lx = -1;
            bbox->ly = -1;
            bbox->hx = -1;
            bbox->hy = -1;
        }
        thread_mutex->unlock_single_writer(1); //is_roi || is_update);

        //display DVS video and ROI if img_show == 1
        display_mutex.lock_display();
        if (img_show)
        {
            if(is_flip){
                cv::flip(frame,frame,0);
            }
            cv::Point p1(b_box_dvs.lx, b_box_dvs.ly);
            cv::Point p2(b_box_dvs.hx, b_box_dvs.hy);
            cv::rectangle(frame, p1, p2, cv::Scalar(255), 2, cv::LINE_8);
            cv::imshow("DVS camera", frame);
        }

        //if ESC pressed, exit... 
        //or some other thread detects ESC press, exit.
        if (*terminate || cv::waitKey(1) == 27)
        {
            *terminate = true;
            //wake up any waiting threads
            thread_mutex->terminate();
            //cleanup
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        display_mutex.unlock_display();
    }
}

void DVS::send_frame(cv::Mat* dest_frame, bool is_flip)
{
    frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC3);
    //stack several frames and count their number of events 
    for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
    {
        read_frame(buffer);
        if (frame_grp_num == 0)
        {
            convert2BitToBR();
        }
        else
        {
            convert2BitToBR_accum();
        }      
    }
    if(is_flip){
        cv::flip(frame, frame,0);
    }
    thread_mutex->lock_single_writer();
    //clone image to send to CIs
    *dest_frame = frame.clone();
    thread_mutex->unlock_single_writer(1);
}

int DVS::event_accum(int *x_count, int *y_count, bool is_flip)
{
    int sum = 0;
    for (int h = 0; h < frame_h; h++)
    {
        for (int byteIndex = 0; byteIndex < (frame_w >> 2); ++byteIndex)
        {
            uint8_t prev_pixel = (h==0)?0:(frame_start[(h-1)*(frame_w >> 2) + byteIndex]);
            uint8_t cur_pixel = frame_start[h * (frame_w >> 2) + byteIndex];
            uint8_t pixel_count_val = pixel_count(prev_pixel) + pixel_count(cur_pixel);
            if(pixel_count_val >= 2){
                for (int bitOffset = 0; bitOffset < 8; bitOffset += 2)
                {
                    uint8_t pixel = (cur_pixel >> bitOffset) & 0x03; // Extract 2 bits
                    if (pixel != 0)
                    {
                        //count events, events per column, and events per row
                        sum++;
                        x_count[(byteIndex << 2) | (bitOffset >> 1)]++;
                        if(is_flip){
                            y_count[frame_h-1-h]++;
                        }else{
                            y_count[h]++;
                        }   
                    }
                }
            }
        }
    }
    return sum;
}

int DVS::event_roi(int *x_count, int *y_count, int sum, Bbox *b_box_dvs, Bbox *b_box_cis)
{
    //check columns with number of events above average
    int x_avg = sum / frame_w;
    //to reduce noise, set minimum of events to 5
    if (x_avg < 5) x_avg = 5;
    int x_thresh_count = 0;
    int x_min = 0, x_max = 0;
    for (int w = 0; w < frame_w; w++)
    {
        if (x_count[w] > x_avg)
        {
            x_thresh_count++;
            //check consecutive <roi_line_width> num of columns with number of events above average
            if (x_thresh_count >= roi_line_width)
            {
                if (x_min == 0)
                {
                    //set as ROI left boundary
                    x_min = w - roi_line_width + 1;
                    x_max = w;
                }
                else if (w > x_max)
                {
                    //set as ROI right boundary
                    x_max = w;
                }
            }
        }
        else
        {
            //if no consecutive <roi_line_width> columns exist, reset count
            x_thresh_count = 0;
        }
    }
    //check rows with number of events above average
    int y_avg = sum / frame_h;
    //to reduce noise, set minimum of events to 5
    if (y_avg < 5) y_avg = 5;
    int y_thresh_count = 0;
    int y_min = 0, y_max = 0;
    for (int h = 0; h < frame_h; h++)
    {
        if (y_count[h] > y_avg)
        {
            y_thresh_count++;
            //check consecutive <roi_line_width> num of rows with number of events above average
            if (y_thresh_count >= roi_line_width)
            {
                if (y_min == 0)
                {
                    //set as ROI top boundary
                    y_min = h - roi_line_width + 1;
                    y_max = h;
                }
                else if (h > y_max)
                {
                    //set as ROI bottom boundary
                    y_max = h;
                }
            }
        }
        else
        {
            //if no consecutive <roi_line_width> rows exist, reset count
            y_thresh_count = 0;
        }
    }
    if(x_min !=0 && y_min != 0){
        //if valid ROI is detected
        draw_square_roi(b_box_dvs, x_min, y_min, x_max, y_max, frame_w, frame_h);
        if(convert_cis){
            //convert DVS coordinates to CIS
            x_min = (int)(cis_x_scale * x_min) + cis_x_offset;
            y_min = (int)(cis_y_scale * y_min) + cis_y_offset;
            x_max = (int)(cis_x_scale * x_max) + cis_x_offset;
            y_max = (int)(cis_y_scale * y_max) + cis_y_offset;
            //draw ROI for CIS
            draw_square_roi(b_box_cis, x_min, y_min, x_max, y_max, cis_frame_w, cis_frame_h);
        }
        //return if valid ROI detected
        return 1;
    }else{
        return 0;
    }

}
void DVS::set_CIS(float x_scale, float y_scale, float x_offset, float y_offset, int frame_w, int frame_h ,int roi_event_score_, int roi_min_score_, int roi_line_width_, int roi_min_size_,
    float roi_inflation_ratio_){
    //detect ROI for CIS instead of DVS
    convert_cis = true;

    //using the following parameters : 
    cis_x_scale = x_scale;
    cis_y_scale = y_scale;
    cis_x_offset = (int)(x_offset * frame_w);
    cis_y_offset = (int)(y_offset * frame_h);
    cis_frame_w = frame_w;
    cis_frame_h = frame_h;
    roi_event_score = roi_event_score_;
    roi_min_score = roi_min_score_;
    roi_line_width = roi_line_width_;
    roi_min_size = roi_min_size_;
    roi_inflation_ratio = roi_inflation_ratio_;
}
void DVS::set_DVS_ROI(int roi_event_score_, int roi_min_score_, int roi_line_width_, int roi_min_size_,float roi_inflation_ratio_){
    //detect ROI for DVS
    roi_event_score = roi_event_score_;
    roi_min_score = roi_min_score_;
    roi_line_width = roi_line_width_;
    roi_min_size = roi_min_size_;
    roi_inflation_ratio = roi_inflation_ratio_;
}
void DVS::draw_square_roi(Bbox* b_box, int x_min, int y_min, int x_max, int y_max, int width, int height){
    //draw square ROI around given x and y coordinate ranges
    //while making sure ROI doesn't exceed the entire frame 

    //move coordinates to inside the frame
    if(x_min < 0) x_min = 0;
    if(y_min < 0) y_min = 0;
    if(x_max >=width) x_max = width - 1 ;
    if(y_max >= height) y_max = height - 1 ;

    //determine square ROI size 
    int inflated_size = (int)(roi_inflation_ratio * (x_max-x_min));
    if (inflated_size > height)
    {
        inflated_size = height;
    }
    else if (inflated_size < roi_min_size)
    {
        inflated_size = roi_min_size;
    }

    int inflated_size_y = (int)(roi_inflation_ratio *(y_max-y_min));
    if (inflated_size_y > height)
    {
        inflated_size_y = height;
    }
    else if (inflated_size_y < roi_min_size)
    {
        inflated_size_y = roi_min_size;
    }

    //give margin around x_max and x_min to determine ROI position 
    int min_offset = (inflated_size - (x_max-x_min)) >> 1;

    //if ROI exceeds frame, move it inside
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
        b_box->lx = x_min- min_offset;
        b_box->hx = x_min - min_offset + inflated_size - 1;
    }

    //give margin around y_max and y_min to determine ROI position 
    int min_offset_y = (inflated_size_y-(y_max-y_min))>>1;

    //if ROI exceeds frame, move it inside
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
        b_box->hy = y_min- min_offset_y + inflated_size_y - 1;
    }
}

int DVS::pixel_count(uint8_t x){
    //calculate how many 2-bit events happen inside 8-bit word
    int cnt = 0;
    for(int i = 0; i < 4; i++){
        if(x && 0x03) cnt++;
        x = x >> 2;
    }
    return cnt;
}
void DVS::convert2BitTo8Bit_count(bool is_flip){
    for (int h = 0; h < frame_h; h++)
    {
        for (int byteIndex = 0; byteIndex < (frame_w >> 2); ++byteIndex)
        {
            //apply a simple spatial filter before written to frame.
            //8-bit word that includes current pixel U 8-bit word 1 row before.
            //if these 8 pixels contain less than 2 pixels, current pixel not written to frame.
            uint8_t prev_pixel = (h==0)?0:(frame_start[(h-1)*(frame_w >> 2) + byteIndex]);
            uint8_t cur_pixel = frame_start[h * (frame_w >> 2) + byteIndex];
            uint8_t pixel_count_val = pixel_count(prev_pixel) + pixel_count(cur_pixel);
            
            for (int bitOffset = 0; bitOffset < 8; bitOffset += 2)
            {
                uint8_t pixel = (cur_pixel >> bitOffset) & 0x03; // Extract 2 bits
                int frame_idx;
                //if DVS flipped, write to frame upside down
                if(is_flip){
                    frame_idx = (frame_h-h-1)*frame_w +  (byteIndex<<2)+(bitOffset>>1);
                }else{
                    frame_idx = h*frame_w + (byteIndex<<2)+(bitOffset>>1);
                }
                //write gray pixel values
                if (pixel_count_val >= 2 && pixel == 1)
                {
                    frame.data[frame_idx] = 224;
                }
                else if(pixel_count_val >= 2 && pixel == 2)
                {
                    frame.data[frame_idx] = 32;
                }
                else
                {
                    frame.data[frame_idx] = 128;
                }
            }
        }
    }
}

void DVS::convert2BitTo8Bit_count_accum(bool is_flip){
    for (int h = 0; h < frame_h; h++)
    {
        for (int byteIndex = 0; byteIndex < (frame_w >> 2); ++byteIndex)
        {
            //apply a simple spatial filter before written to frame.
            //8-bit word that includes current pixel U 8-bit word 1 row before.
            //if these 8 pixels contain less than 2 pixels, current pixel not written to frame.
            uint8_t prev_pixel = (h==0)?0:(frame_start[(h-1)*(frame_w >> 2) + byteIndex]);
            uint8_t cur_pixel = frame_start[h * (frame_w >> 2) + byteIndex];
            uint8_t pixel_count_val = pixel_count(prev_pixel) + pixel_count(cur_pixel);
            if(pixel_count_val >= 2){
                for (int bitOffset = 0; bitOffset < 8; bitOffset += 2)
                {
                    uint8_t pixel = (cur_pixel >> bitOffset) & 0x03; // Extract 2 bits
                    int frame_idx;
                    //if DVS flipped, write to frame upside down
                    if(is_flip){
                        frame_idx = (frame_h-h-1)*frame_w +  (byteIndex<<2)+(bitOffset>>1);
                    }else{
                        frame_idx = h*frame_w + (byteIndex<<2)+(bitOffset>>1);
                    }
                    //accumulate to gray pixels (functionality not used for now)
                    if(frame.data[frame_idx] > 128) {
                        if(pixel!=0) frame.data[frame_idx]+=1;
                    }else if(frame.data[frame_idx] < 128){
                        if(pixel!=0) frame.data[frame_idx]-=1;
                    }
                    //stack frames, overwriting pixels with no events
                    else if (pixel == 1)
                    {
                        frame.data[frame_idx] = 224;
                    }
                    else if(pixel == 2)
                    {
                        frame.data[frame_idx] = 32;
                    }
                }
            }
        }
    }
}
int DVS::new_ROI(Bbox* dvs, Bbox* cis){
    int row_idx = 0;
    //final roi values
    int global_x_min = frame_w, global_x_max = 0;
    int global_y_min = 0, global_y_max = -1;
    //height-wise line width threshold apply
    int height_count = 0;
    //calculate x-direction min, max for a few rows
    int candidate_x_min, candidate_x_max;
    candidate_x_min = frame_w;
    candidate_x_max = 0;
    //calculate max sum of consecutive partial sequence
    //where some values are -1 and others are roi_event_score(currently 5)
    for(int h = 0; h < frame_h; h++){
        int max_score = 0, cur_score = 0;
        int local_min = frame_w, local_max = 0;
        int cur_score_left_end = 0;
        //incremental algorithm
        for(int w = 0; w < frame_w; w++){
            //cur_val : current score of sequence element
            int cur_val;
            int pix_val = frame.data[row_idx + w];
            if(pix_val > 128){
                cur_val = roi_event_score;//2+width_count;//((pix_val-128)>>2)+width_count;
            }else if(pix_val < 128){
                cur_val = roi_event_score;//2+width_count;//((128-pix_val)>>2)+width_count;
            }else{
                cur_val = -1;
            }
            //cur_score : max value of partial sequence whose right end is w
            cur_score = cur_score + cur_val;
            //if cur_score < 0, 0(no elements in partial sequence) is larger, empty sequence corresponding to cur_score
            if(cur_score <0){
                cur_score = 0;
                //set the left end of partial sequence to current sequence element 
                cur_score_left_end = w;
            }
            //record maximum partial consecutive sequence
            //and its left & right ends
            if(cur_score>max_score){
                max_score = cur_score;
                local_min = cur_score_left_end;
                local_max = w;
            }
        }
        // printf("row %d min %d max %d max_score = %d\n", h,local_min, local_max, max_score );
        //if max score exceeds threshold
        if(max_score >= roi_min_score){
            height_count++;
            //union max & min with few adjacent rows
            if(local_min < candidate_x_min) candidate_x_min = local_min;
            if(local_max > candidate_x_max) candidate_x_max = local_max;
            //if consecutive rows' max score exceed roi_min_score
            //modify final ROI
            if(height_count >= roi_line_width){
                if(global_y_min == 0) global_y_min = h - roi_line_width + 1;
                global_y_max  = h;
                if(candidate_x_min < global_x_min) global_x_min = candidate_x_min;
                if(candidate_x_max > global_x_max) global_x_max = candidate_x_max;
            }
        } else{
            //if consecutive rows don't exist
            //initialize max, min and row count
            candidate_x_min = frame_w;
            candidate_x_max = 0;
            height_count = 0;
        }
        row_idx+=frame_w;
    }
    if(global_y_max != -1){
        //if valid ROI is detected
        // printf("%d %d %d %d\n", global_x_min, global_y_min, global_x_max, global_y_max);
        //draw dvs ROI
        draw_square_roi(dvs, global_x_min, global_y_min, global_x_max, global_y_max, frame_w, frame_h);
        if(convert_cis){
            //convert DVS coordinates to CIS
            int x_min = (int)(cis_x_scale * global_x_min) + cis_x_offset;
            int y_min = (int)(cis_y_scale * global_y_min) + cis_y_offset;
            int x_max = (int)(cis_x_scale * global_x_max) + cis_x_offset;
            int y_max = (int)(cis_y_scale * global_y_max) + cis_y_offset;
            //draw ROI for CIS
            draw_square_roi(cis, x_min, y_min, x_max, y_max, cis_frame_w, cis_frame_h);
        }
        //return if valid ROI detected
        return 1;
    }else{
        return 0;
    }

}
void DVS::crop_new_ROI(int img_show, int is_update, bool is_flip)
{   
    while(true){
        frame = cv::Mat::zeros(frame_h, frame_w, CV_8UC1);
        for (int frame_grp_num = 0; frame_grp_num < accum_num; frame_grp_num++)
        {
            read_frame(buffer);
            //generate DVS frame
            if (frame_grp_num == 0)
            {
                convert2BitTo8Bit_count(is_flip);
            }
            else
            {
                convert2BitTo8Bit_count_accum(is_flip);
            }
        }
        Bbox b_box_dvs, b_box_cis;

        //calculate event ROI in the form of a bounding box
        int is_roi = new_ROI(&b_box_dvs, &b_box_cis);
    
        //acquire mutex 
        thread_mutex->lock_single_writer();

        //if there are enough events to draw a ROI
        //write bounding box information in a shared struct variable
        if (is_roi)
        {
            bbox->lx = b_box_cis.lx;
            bbox->ly = b_box_cis.ly;
            bbox->hx = b_box_cis.hx;
            bbox->hy = b_box_cis.hy;
            // printf("%d %d %d %d\n", bbox->lx, bbox->ly, bbox->hx, bbox->hy);
        }else if(!is_update){
            bbox->lx = -1;
            bbox->ly = -1;
            bbox->hx = -1;
            bbox->hy = -1;
        }
        thread_mutex->unlock_single_writer(1); //is_roi || is_update);

        //display DVS video and ROI if img_show == 1
        display_mutex.lock_display();
        if (img_show)
        {
            if(is_roi){
                cv::Point p1(b_box_dvs.lx, b_box_dvs.ly);
                cv::Point p2(b_box_dvs.hx, b_box_dvs.hy);
                cv::rectangle(frame, p1, p2, cv::Scalar(255), 2, cv::LINE_8);
            }
            cv::imshow("DVS camera", frame);
        }

        //if ESC pressed, exit... 
        //or some other thread detects ESC press, exit.
        if (*terminate || cv::waitKey(1) == 27)
        {
            *terminate = true;
            //wake up any waiting threads
            thread_mutex->terminate();
            //cleanup
            frame.release();
            display_mutex.unlock_display();
            break;
        }
        frame.release();
        display_mutex.unlock_display();
    }
}
DVS::~DVS()
{
    if (double_buffer != NULL){
        free(double_buffer);
        delete dbuf_mutex[0];
        delete dbuf_mutex[1];
        if(terminate !=NULL){
            delete terminate;
        }
    }
    free(buffer);
    free(buffer_rdy);
    free(buffer_done);
}