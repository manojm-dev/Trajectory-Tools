#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

// Main class for the Trajectory Reader and Publisher node
class TrajectoryReaderPublisher : public rclcpp::Node
{
public:
  // Constructor to initialize the node
  explicit TrajectoryReaderPublisher()
      : Node("trajectory_reader_publisher")
  {
    load_parameters();
    setup_publishers();
    read_trajectory_from_file();
  }

private:
  // Structure to store trajectory points with position and timestamp
  struct TrajectoryPoint
  {
    geometry_msgs::msg::Point point;
    double timestamp; 
  };

  std::vector<TrajectoryPoint> trajectory_;  // Vector to store trajectory points
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;  // Marker publisher
  std::string odom_topic_;  // Topic name for odometry data
  std::string marker_topic_;  // Topic name for marker data
  std::string trajectory_folder_path_;  // Path to save trajectory files
  std::string trajectory_file_name_;  // Name of the trajectory file

  // Load parameters from the parameter server
  void load_parameters()
  {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("marker_topic", "/trajectory_marker");
    this->declare_parameter<std::string>("trajectory_folder_path", "/tmp/");
    this->declare_parameter<std::string>("trajectory_file_name", "trajectory");

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    trajectory_folder_path_ = this->get_parameter("trajectory_folder_path").as_string();
    trajectory_file_name_ = this->get_parameter("trajectory_file_name").as_string();
  }

  // Setup publishers for the node
  void setup_publishers()
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  }

  // Read the trajectory from a JSON file
  void read_trajectory_from_file()
  {
    std::string trajectory_file_path = trajectory_folder_path_ + trajectory_file_name_ + ".json";

    std::ifstream file(trajectory_file_path);
    if (!file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", trajectory_file_path.c_str());
      return;
    }

    nlohmann::json json_trajectory;
    file >> json_trajectory;
    file.close();

    for (const auto &point_data : json_trajectory["trajectory"])
    {
      TrajectoryPoint tp;
      tp.point.x = point_data["x"];
      tp.point.y = point_data["y"];
      tp.point.z = point_data["z"];
      tp.timestamp = point_data["timestamp"]; 

      trajectory_.push_back(tp);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points from %s", trajectory_.size(), trajectory_file_path.c_str());
    publish_trajectory();
  }

  // Publish the trajectory as a marker array
  void publish_trajectory()
  {
    if (trajectory_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory is empty, nothing to publish.");
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = odom_topic_; 
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.ns = "trajectory_marker";

    for (size_t i = 0; i < trajectory_.size(); ++i)
    {
      marker.points.push_back(trajectory_[i].point);
      marker.id = i;
      marker_array.markers.push_back(marker);
      marker_pub_->publish(marker_array);

      RCLCPP_INFO(this->get_logger(), "Published point at time: %f", trajectory_[i].timestamp);

      if (i < trajectory_.size() - 1)
      {
        double diff_time = trajectory_[i + 1].timestamp - trajectory_[i].timestamp;
        std::chrono::duration<double> delay(diff_time);
        std::this_thread::sleep_for(delay);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Completed publishing trajectory.");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryReaderPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
