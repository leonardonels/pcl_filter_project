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

    this->declare_parameter<double>("y_rotation_angle", 0.0);
    m_y_rotation_angle = this->get_parameter("y_rotation_angle").as_double();

    this->declare_parameter<double>("x_traslation", 0.0);
    m_x_traslation = this->get_parameter("x_traslation").as_double();

    this->declare_parameter<double>("y_traslation", 0.0);
    m_y_traslation = this->get_parameter("y_traslation").as_double();

    this->declare_parameter<double>("z_traslation", 0.0);
    m_z_traslation = this->get_parameter("z_traslation").as_double();

    this->declare_parameter<bool>("intensity", false);
    m_intensity = this->get_parameter("intensity").get_value<bool>();

    this->declare_parameter<bool>("gradient", false);
    m_gradient = this->get_parameter("gradient").get_value<bool>();

    this->declare_parameter<std::vector<std::string>>("angle_zones", std::vector<std::string>{});
    auto angle_zones = this->get_parameter("angle_zones").as_string_array();

    
    if(m_gradient){
        try{
            
            for (const auto& zone_str : angle_zones) {
                AngleZone zone;
                std::sscanf(zone_str.c_str(), "start: %lf, end: %lf, downsample: %d", 
                &zone.start, &zone.end, &zone.downsample);
                m_angle_zones.push_back(zone);
                RCLCPP_INFO(this->get_logger(), "Zone - Start: %f, End: %f, Downsample: %d", zone.start, zone.end, zone.downsample);
            }
        }catch (const std::exception& e){
            RCLCPP_ERROR(this->get_logger(), "Error parsing zone: %s", e.what());
        }
    }

    if (m_y_rotation_angle || m_x_traslation || m_y_traslation || m_z_traslation){
        RCLCPP_INFO(this->get_logger(), "Rotation angle: %f rads", static_cast<float>(m_y_rotation_angle));
        RCLCPP_INFO(this->get_logger(), "Translation: %fm, %fm, %fm", static_cast<float>(m_x_traslation), static_cast<float>(m_y_traslation), static_cast<float>(m_z_traslation));
    }
}

void PointCloudFilter::initialize()
{
    // Load parameters
    this->load_parameters();

    rclcpp::QoS qos_rel(rclcpp::KeepLast(1));
    qos_rel.reliable();

    // Initialize publishers and subscribers
    if (m_intensity) {
        m_input_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_input_topic, qos_rel,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->pointcloud_callback<pcl::PointXYZI>(msg);
            });
    } else {
        m_input_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_input_topic, qos_rel,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->pointcloud_callback<pcl::PointXYZ>(msg);
            });
    }

    m_output_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_output_topic, 10);
}

PointCloudFilter::PointCloudFilter() : Node("pointcloud_filter_node") 
{
    this->initialize();
}

template<typename T>
void PointCloudFilter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
    pcl::fromROSMsg(*msg, *cloud);

    // Apply transformation
    if (m_y_rotation_angle || m_x_traslation || m_y_traslation || m_z_traslation) {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        if (m_y_rotation_angle){
            transform.rotate(Eigen::AngleAxisf(static_cast<float>(m_y_rotation_angle), Eigen::Vector3f::UnitY()));
        }
        transform.translation() << static_cast<float>(m_x_traslation), 
                                   static_cast<float>(m_y_traslation), 
                                   static_cast<float>(m_z_traslation);
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }

    // Apply angle zone downsampling if enabled
    if (m_gradient && cloud->isOrganized() && !m_angle_zones.empty()) {
        typename pcl::PointCloud<T>::Ptr filtered_cloud(new pcl::PointCloud<T>);
        int height = cloud->height;
        int width = cloud->width;
    
        for (const auto& zone : m_angle_zones) {
            int start_row = static_cast<int>(zone.start * height);
            int end_row = static_cast<int>(zone.end * height);
    
            for (int r = start_row; r < end_row; ++r) {
                // Only keep the row if it satisfies the downsampling condition
                if ((r - start_row) % zone.downsample == 0) {
                    for (int c = 0; c < width; ++c) {
                        filtered_cloud->points.push_back(cloud->at(c, r));
                    }
                }
            }
        }
    
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = false;
        cloud = filtered_cloud;
    }

    // Convert back to ROS message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header = msg->header;

    m_output_pub->publish(output_msg);
}