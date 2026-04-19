// system
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>
#include <thread>
#include <memory>

// eigen
#include <Eigen/Geometry>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

// ros2
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const double pi = M_PI;


struct Point {
  double x, y, z;

  Point() : x(0), y(0), z(0) {}

  Point(double setX, double setY, double setZ) : x(setX), y(setY), z(setZ) {}

  void Print() {
    std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
  }
};

struct Rotate {
  double roll, pitch, yaw;

  Rotate() : roll(0), pitch(0), yaw(0) {}

  Rotate(double setRoll, double setPitch, double setYaw)
      : roll(setRoll), pitch(setPitch), yaw(setYaw) {}

  void Print() {
    std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << std::endl;
  }
};

struct Pose {

  Point position;
  Rotate rotation;

  Pose() : position(Point(0, 0, 0)), rotation(Rotate(0, 0, 0)) {}

  Pose(Point setPos, Rotate setRotation)
      : position(setPos), rotation(setRotation) {}

  Pose operator-(const Pose &p) {
    Pose result(Point(position.x - p.position.x, position.y - p.position.y,
                      position.z - p.position.z),
                Rotate(rotation.yaw - p.rotation.yaw,
                       rotation.pitch - p.rotation.pitch,
                       rotation.roll - p.rotation.roll));
    return result;
  }
};

class LidarMapping : public rclcpp::Node {
public:
  LidarMapping();
  ~LidarMapping();

private:
  // ros2 subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;

  // ros2 publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_map_pub_;

  // ros2 timer
  rclcpp::TimerBase::SharedPtr timer_;

  // vars
  bool new_scan_;
  bool new_scan_available_;  // Flag to indicate new point cloud data available
  double vehicle_radius_;
  int cloud_resolution_;
  size_t max_map_points_;  // Maximum number of points in the map
  std::chrono::time_point<std::chrono::system_clock> prev_scan_time_;
  Pose current_pose_;
  std::vector<Point> scan_poses_;
  PointCloudT::Ptr scaned_cloud_ptr_;
  PointCloudT pcl_cloud_; // point cloud map before filtering
  sensor_msgs::msg::PointCloud2 saved_map_;

  // Thread safety
  std::mutex cloud_mutex_;
  std::mutex pose_mutex_;


  // callback functions
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void localizationCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // core method
  void scanTransformation(const PointCloudT::Ptr scan_cloud);

  // methods
  Eigen::Matrix4d transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt);
  Eigen::Quaternionf getQuaternion(float theta);
  Pose getPose(Eigen::Matrix4d matrix);
  double getDistance(Point p1, Point p2);
  double minDistance(Point p1, std::vector<Point> points);
  void print4x4Matrix(const Eigen::Matrix4d &matrix);
  void print4x4Matrixf(const Eigen::Matrix4f &matrix);
  PointCloudT::Ptr downsamplePointCloud(const PointCloudT& pcl_cloud);
  void savePointCloudMap(const PointCloudT::Ptr cloud_filtered);
};
