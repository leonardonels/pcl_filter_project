#include "pointcloud_filter_cpp/pointcloud_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

PointCloudFilter::PointCloudFilter() : Node("pointcloud_filter")
{
    // Declare parameters and load them
    this->declare_parameter<std::string>("input_topic", "input_point_cloud");
    this->declare_parameter<std::string>("output_topic", "filtered_point_cloud");
    this->declare_parameter<double>("filter_radius", 0.2);
    this->declare_parameter<double>("filter_intensity_threshold", 1.0);

    // Get parameters
    input_topic_ = this->get_parameter("input_topic").get_value<std::string>();
    output_topic_ = this->get_parameter("output_topic").get_value<std::string>();
    filter_radius_ = this->get_parameter("filter_radius").get_value<double>();
    filter_intensity_threshold_ = this->get_parameter("filter_intensity_threshold").get_value<double>();

    // Log the parameters
    RCLCPP_INFO(this->get_logger(), "Using parameters: %s, %s, %f, %f",
                input_topic_.c_str(), output_topic_.c_str(), filter_radius_, filter_intensity_threshold_);

    // Create subscribers and publishers
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, 10, std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
}

void PointCloudFilter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Filter the point cloud based on radius and intensity threshold
    // Implement your filtering logic here

    // For simplicity, just forward the message to the output topic
    point_cloud_publisher_->publish(*msg);
}
