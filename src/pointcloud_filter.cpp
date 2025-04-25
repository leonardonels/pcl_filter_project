#include "pointcloud_filter/pointcloud_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

void PointCloudFilter::load_parameters()
{
    // topics
    this->declare_parameter<std::string>("input_topic", "");
    m_input_topic = this->get_parameter("input_topic").get_value<std::string>();

    this->declare_parameter<std::string>("output_topic", "");
    m_output_topic = this->get_parameter("output_topic").get_value<std::string>();
}

void PointCloudFilter::initialize()
{
    // Load parameters
    this->load_parameters();

    rclcpp::QoS qos_rel(rclcpp::KeepLast(1));
    qos_rel.reliable();

    rclcpp::QoS qos_be(rclcpp::KeepLast(1));
    qos_be.best_effort();
    // Initialize pubs and subs
    m_input_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        m_input_topic, qos_rel,
        std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));
    m_output_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_output_topic, 10);
}


PointCloudFilter::PointCloudFilter() : Node("pointcloud_filter_node") 
{
    this->initialize();
}

void PointCloudFilter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Filter the point cloud based on radius and intensity threshold
    // Implement your filtering logic here

    // For simplicity, just forward the message to the output topic
    m_output_pub->publish(*msg);
}
