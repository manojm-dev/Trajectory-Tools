#ifndef TRAJECTORY_TOOLS__TRAJECTORY_PUBLISHER_SAVER_HPP_
#define TRAJECTORY_TOOLS__TRAJECTORY_PUBLISHER_SAVER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <trajectory_tools/srv/save_trajectory.hpp>

using namespace std::chrono_literals;

class TrajectoryPubSaver : public rclcpp::Node {
public:
  TrajectoryPubSaver();

private:
  struct TrajectoryPoint {
    geometry_msgs::msg::Point point;
    rclcpp::Time timestamp;
  };

  std::vector<TrajectoryPoint> trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<trajectory_tools::srv::SaveTrajectory>::SharedPtr save_trajectory_srv_;
  std::string trajectory_folder_path_;
  std::string odom_topic_;
  std::string marker_topic_;

  void load_parameters();
  void setup_subscriptions();
  void setup_publishers();
  void setup_services();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_trajectory();
  void save_trajectory_callback(
      const std::shared_ptr<trajectory_tools::srv::SaveTrajectory::Request> request,
      std::shared_ptr<trajectory_tools::srv::SaveTrajectory::Response> response);
};

#endif  // TRAJECTORY_TOOLS__TRAJECTORY_PUBLISHER_SAVER_HPP_