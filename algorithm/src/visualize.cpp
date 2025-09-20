#include "visualize.hpp"

void visualize3D(const cv::Mat &acc)
{
    // Convert float32 image to 3D point cloud (x, y, z)
    std::vector<cv::Vec3f> cloud;
    for (int y = 0; y < acc.rows; ++y)
    {
        for (int x = 0; x < acc.cols; ++x)
        {
            float z = acc.at<float>(y, x);
            cloud.push_back(cv::Vec3f((float)x, (float)y, z));
        }
    }

    // Create viz cloud
    cv::viz::WCloud point_cloud(cloud, cv::viz::Color::white());

    // Set up window
    cv::viz::Viz3d window("3D Event Landscape");
    window.showWidget("EventCloud", point_cloud);
    window.spin(); // Shows window until user closes
}