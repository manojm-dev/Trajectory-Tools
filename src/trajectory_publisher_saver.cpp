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

// Main class for the Trajectory Publisher and Saver node
class TrajectoryPubSaver : public rclcpp::Node {
public:
  // Constructor to initialize the node
  TrajectoryPubSaver()
      : Node("trajectory_publisher_saver") {
    load_parameters();
    setup_subscriptions();
    setup_publishers();
    setup_services();
  }

private:
  // Structure to store trajectory points with position and timestamp
  struct TrajectoryPoint {
    geometry_msgs::msg::Point point;
    rclcpp::Time timestamp;
  };

  std::vector<TrajectoryPoint> trajectory_;  // Vector to store trajectory points
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  // Odometry subscription
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;  // Marker publisher
  rclcpp::Service<trajectory_tools::srv::SaveTrajectory>::SharedPtr save_trajectory_srv_;  // Service to save trajectory
  std::string trajectory_folder_path_;  // Path to save trajectory files
  std::string odom_topic_;  // Topic name for odometry data
  std::string marker_topic_;  // Topic name for marker data

  // Load parameters from the parameter server
  void load_parameters() {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("marker_topic", "/trajectory_marker");
    this->declare_parameter<std::string>("trajectory_folder_path", "/tmp/");
    
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    trajectory_folder_path_ = this->get_parameter("trajectory_folder_path").as_string();
  }

  // Setup subscriptions for the node
  void setup_subscriptions() {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&TrajectoryPubSaver::odom_callback, this, std::placeholders::_1));
  }

  // Setup publishers for the node
  void setup_publishers() {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  }

  // Setup services for the node
  void setup_services() {
    save_trajectory_srv_ = this->create_service<trajectory_tools::srv::SaveTrajectory>(
        "save_trajectory",
        std::bind(&TrajectoryPubSaver::save_trajectory_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  // Callback function for odometry data
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    TrajectoryPoint tp;
    tp.point.x = msg->pose.pose.position.x;
    tp.point.y = msg->pose.pose.position.y;
    tp.point.z = 0;
    tp.timestamp = this->now();

    trajectory_.push_back(tp);
    publish_trajectory();
  }

  // Publish the trajectory as a marker array
  void publish_trajectory() {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = odom_topic_; 
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.b = 1.0;

    for (const auto &tp : trajectory_) {
      marker.points.push_back(tp.point);
    }

    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
  }

  // Callback function to save the trajectory to a file
  void save_trajectory_callback(
      const std::shared_ptr<trajectory_tools::srv::SaveTrajectory::Request> request,
      std::shared_ptr<trajectory_tools::srv::SaveTrajectory::Response> response) {
    
    std::string trajectory_file_path = trajectory_folder_path_ + request->filename + ".json";
    double duration_sec = request->duration;
    rclcpp::Time current_time = this->now();

    // Filter trajectory points based on the specified duration
    std::vector<TrajectoryPoint> filtered_trajectory;
    for (auto it = trajectory_.rbegin(); it != trajectory_.rend(); ++it) {
      if ((current_time - it->timestamp).seconds() <= duration_sec) {
        filtered_trajectory.push_back(*it);
      } else {
        break;
      }
    }

    if (filtered_trajectory.empty()) {
      response->success = false;
      response->message = "No trajectory points found within the specified duration.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    std::reverse(filtered_trajectory.begin(), filtered_trajectory.end());

    // Save the filtered trajectory to a JSON file
    std::ofstream file(trajectory_file_path);
    if (!file.is_open()) {
      response->success = false;
      response->message = "Failed to open file: " + trajectory_file_path;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    nlohmann::json json_trajectory;
    for (const auto &tp : filtered_trajectory) {
      json_trajectory["trajectory"].push_back({
          {"x", tp.point.x},
          {"y", tp.point.y},
          {"z", tp.point.z},
          {"timestamp", tp.timestamp.seconds()}});
    }

    file << json_trajectory.dump(4); // 4 space indentation
    file.close();

    response->success = true;
    response->message = "Trajectory saved to " + trajectory_file_path;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPubSaver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
