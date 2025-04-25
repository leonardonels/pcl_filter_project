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
    PointCloudFilter();

private:
    // callbacks
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // methods
    void initialize();
    void load_parameters();

    // pubs and subs
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_input_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_output_pub;

    //topics
    std::string m_input_topic;
    std::string m_output_topic;

    std::vector<VerticalZone> m_vertical_zones;
};

#endif // POINTCLOUD_FILTER_HPP