#include "pointcloud_filter/pointcloud_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h> 

void PointCloudFilter::load_parameters()
{
    this->declare_parameter<std::string>("input_topic", "");
    m_input_topic = this->get_parameter("input_topic").get_value<std::string>();

    this->declare_parameter<std::string>("output_topic", "");
    m_output_topic = this->get_parameter("output_topic").get_value<std::string>();

    this->declare_parameter<double>("rotation_angle", 0.0);
    m_rotation_angle = this->get_parameter("rotation_angle").as_double();

    this->declare_parameter<double>("traslation", 0.0);
    m_traslation = this->get_parameter("traslation").as_double();

    this->declare_parameter<std::vector<std::string>>("vertical_zones");
    auto vertical_zones = this->get_parameter("vertical_zones").get_value<std::vector<std::string>>();

    for (const auto& zone_str : vertical_zones)
    {
        try
        {
            // You need to manually parse the string into start, end, and downsample values
            std::map<std::string, std::string> zone_map;
            // For example:
            // "start: 0.0, end: 0.33, downsample: 3"
            size_t start_pos = zone_str.find("start: ");
            size_t end_pos = zone_str.find(", end: ");
            size_t downsample_pos = zone_str.find(", downsample: ");

            if (start_pos != std::string::npos && end_pos != std::string::npos && downsample_pos != std::string::npos)
            {
                VerticalZone vzone;
                vzone.start = std::stod(zone_str.substr(start_pos + 7, end_pos - start_pos - 7));
                vzone.end = std::stod(zone_str.substr(end_pos + 7, downsample_pos - end_pos - 7));
                vzone.downsample = std::stoi(zone_str.substr(downsample_pos + 14));

                m_vertical_zones.push_back(vzone);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to parse vertical zone: %s", zone_str.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error parsing vertical zone: %s", e.what());
        }
    }

    for (const auto& zone : m_vertical_zones)
    {
        RCLCPP_INFO(this->get_logger(), "Zone - Start: %f, End: %f, Downsample: %d", zone.start, zone.end, zone.downsample);
    }
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
    auto start_time = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);  // Use pcl_conversions to convert PointCloud2 to pcl::PointCloud

    // Apply rotation compensation if enabled
    if (m_rotation_angle || m_traslation)
    {
        RCLCPP_INFO(this->get_logger(), "Rotation angle: %f", static_cast<float>(m_rotation_angle));
        RCLCPP_INFO(this->get_logger(), "Traslation: %f", static_cast<float>(m_traslation));
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(static_cast<float>(m_rotation_angle), Eigen::Vector3f::UnitY()));
        transform.translation() << 0.0, 0.0, static_cast<float>(m_traslation);
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }


    std::vector<int> selected_indices;
    for (const auto& zone : m_vertical_zones)
    {
        int start_row = static_cast<int>(zone.start * cloud->height);
        int end_row = static_cast<int>(zone.end * cloud->height);
        end_row = std::min(end_row, static_cast<int>(cloud->height));
        int step = std::max(zone.downsample, 1);
        
        for (int i = start_row; i < end_row; i += step)
        {
            for (int j = 0; j < cloud->width; ++j)
            {
                int index = i * cloud->width + j;
                selected_indices.push_back(index);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int idx : selected_indices)
    {
        inliers->indices.push_back(idx);
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*filtered_cloud);

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);  // Use pcl_conversions to convert back to PointCloud2
    filtered_msg.header = msg->header;

    m_output_pub->publish(filtered_msg);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> total_duration = end_time - start_time;
    RCLCPP_INFO(this->get_logger(), "Total callback duration: %f seconds", total_duration.count());
}