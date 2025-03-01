#ifndef TRAJECTORY_TOOLS__TRAJECTORY_READER_PUBLISHER_HPP_
#define TRAJECTORY_TOOLS__TRAJECTORY_READER_PUBLISHER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class TrajectoryReaderPublisher : public rclcpp::Node
{
public:
  explicit TrajectoryReaderPublisher();

private:
  struct TrajectoryPoint
  {
    geometry_msgs::msg::Point point;
    double timestamp; 
  };

  std::vector<TrajectoryPoint> trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string odom_topic_;
  std::string marker_topic_;
  std::string trajectory_folder_path_;
  std::string trajectory_file_name_;

  void load_parameters();
  void setup_publishers();
  void read_trajectory_from_file();
  void publish_trajectory();
};

#endif  // TRAJECTORY_TOOLS__TRAJECTORY_READER_PUBLISHER_HPP_