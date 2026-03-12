#include "ame_ros2/lifecycle_manager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ame_ros2::AmeLifecycleManager>();
    // Trigger startup after a short delay for other nodes to register their services
    std::thread([node]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        node->startup();
    }).detach();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
