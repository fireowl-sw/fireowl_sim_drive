#include "carla_lidar_mapping_ros2/lidar_mapping.h"
#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarMapping>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
