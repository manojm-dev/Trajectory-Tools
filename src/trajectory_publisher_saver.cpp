#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <trajectory_tools/SaveTrajectory.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>

// Main class for the Trajectory Publisher and Saver node
class TrajectoryPubSaver {
public:
  // Constructor to initialize the node
  TrajectoryPubSaver(ros::NodeHandle& nh) : nh_(nh) {
    ROS_INFO("Initializing TrajectoryPubSaver node...");
    load_parameters();
    setup_subscriptions();
    setup_publishers();
    setup_services();
    ROS_INFO("TrajectoryPubSaver node initialized.");
  }

private:
  // Structure to store trajectory points with position and timestamp
  struct TrajectoryPoint {
    geometry_msgs::Point point;
    ros::Time timestamp;
  };

  std::vector<TrajectoryPoint> trajectory_;  // Vector to store trajectory points
  ros::Subscriber odom_sub_;  // Odometry subscription
  ros::Publisher marker_pub_;  // Marker publisher
  ros::ServiceServer save_trajectory_srv_;  // Service to save trajectory
  std::string trajectory_folder_path_;  // Path to save trajectory files
  std::string odom_topic_;  // Topic name for odometry data
  std::string marker_topic_;  // Topic name for marker data

  ros::NodeHandle& nh_;

  // Load parameters from the parameter server
  void load_parameters() {
    nh_.param<std::string>("odom_topic", odom_topic_, "odom");
    nh_.param<std::string>("marker_topic", marker_topic_, "/trajectory_marker");
    nh_.param<std::string>("trajectory_folder_path", trajectory_folder_path_, "/tmp/");
    ROS_INFO("Parameters loaded: odom_topic=%s, marker_topic=%s, trajectory_folder_path=%s",
             odom_topic_.c_str(), marker_topic_.c_str(), trajectory_folder_path_.c_str());
  }

  // Setup subscriptions for the node
  void setup_subscriptions() {
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        odom_topic_, 10, &TrajectoryPubSaver::odom_callback, this);
    ROS_INFO("Subscribed to odometry topic: %s", odom_topic_.c_str());
  }

  // Setup publishers for the node
  void setup_publishers() {
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
    ROS_INFO("Publisher set up for marker topic: %s", marker_topic_.c_str());
  }

  // Setup services for the node
  void setup_services() {
    save_trajectory_srv_ = nh_.advertiseService("save_trajectory", &TrajectoryPubSaver::save_trajectory_callback, this);
    ROS_INFO("Service server set up for saving trajectory.");
  }

  // Callback function for odometry data
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_DEBUG("Received odometry data.");
    TrajectoryPoint tp;
    tp.point.x = msg->pose.pose.position.x;
    tp.point.y = msg->pose.pose.position.y;
    tp.point.z = 0;
    tp.timestamp = ros::Time::now();

    trajectory_.push_back(tp);
    ROS_DEBUG("Trajectory point added: x=%f, y=%f, timestamp=%f", tp.point.x, tp.point.y, tp.timestamp.toSec());
    publish_trajectory();
  }

  // Publish the trajectory as a marker array
  void publish_trajectory() {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = odom_topic_; 
    marker.header.stamp = ros::Time::now();            
    marker.ns = "trajectory_marker";
    marker.id = 0;     
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    // Set a valid orientation to avoid warnings
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Marker styling
    marker.scale.x = 0.05;           
    marker.color.a = 1.0;            
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    for (const auto &tp : trajectory_) {
      marker.points.push_back(tp.point);
    }

    marker_array.markers.push_back(marker);
    marker_pub_.publish(marker_array);
    ROS_DEBUG("Published trajectory marker array with %lu points.", marker.points.size());
  }

  // Callback function to save the trajectory to a file
  bool save_trajectory_callback(trajectory_tools::SaveTrajectory::Request& request, trajectory_tools::SaveTrajectory::Response& response) {
    ROS_INFO("Received request to save trajectory to file: %s", request.filename.c_str());
    std::string trajectory_file_path = trajectory_folder_path_ + request.filename + ".json";
    double duration_sec = request.duration;
    ros::Time current_time = ros::Time::now();

    // Filter trajectory points based on the specified duration
    std::vector<TrajectoryPoint> filtered_trajectory;
    for (auto it = trajectory_.rbegin(); it != trajectory_.rend(); ++it) {
      if ((current_time - it->timestamp).toSec() <= duration_sec) {
        filtered_trajectory.push_back(*it);
      } else {
        break;
      }
    }

    if (filtered_trajectory.empty()) {
      response.success = false;
      response.message = "No trajectory points found within the specified duration.";
      ROS_WARN("%s", response.message.c_str());
      return true;
    }

    std::reverse(filtered_trajectory.begin(), filtered_trajectory.end());

    // Save the filtered trajectory to a JSON file
    std::ofstream file(trajectory_file_path);
    if (!file.is_open()) {
      response.success = false;
      response.message = "Failed to open file: " + trajectory_file_path;
      ROS_ERROR("%s", response.message.c_str());
      return true;
    }

    nlohmann::json json_trajectory;
    for (const auto &tp : filtered_trajectory) {
      json_trajectory["trajectory"].push_back({
          {"x", tp.point.x},
          {"y", tp.point.y},
          {"z", tp.point.z},
          {"timestamp", tp.timestamp.toSec()}});
    }

    file << json_trajectory.dump(4); // 4 space indentation
    file.close();

    response.success = true;
    response.message = "Trajectory saved to " + trajectory_file_path;
    ROS_INFO("%s", response.message.c_str());
    return true;
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "trajectory_publisher_saver");
  ros::NodeHandle nh;
  TrajectoryPubSaver node(nh);
  ros::spin();
  return 0;
}