#include "pointcloud_filter/pointcloud_filter.hpp"
#include <unistd.h>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing lap_counter process.\n";
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, handleSignal);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
