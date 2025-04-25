#ifndef POINTCLOUD_FILTER_HPP
#define POINTCLOUD_FILTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>
#include <vector>
#include <map>

struct VerticalZone {
  float start;
  float end;
  int downsample;
};

class PointCloudFilter : public rclcpp::Node
{
public:
    PointCloudFilter(); // Constructor declaration
    ~PointCloudFilter() = default;

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    std::string input_topic_;
    std::string output_topic_;
    double filter_radius_;
    double filter_intensity_threshold_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
};

#endif // POINTCLOUD_FILTER_HPP