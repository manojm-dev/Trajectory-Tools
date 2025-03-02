#ifndef TRAJECTORY_PUBLISHER_SAVER_H
#define TRAJECTORY_PUBLISHER_SAVER_H

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
  TrajectoryPubSaver(ros::NodeHandle& nh);

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
  void load_parameters();

  // Setup subscriptions for the node
  void setup_subscriptions();

  // Setup publishers for the node
  void setup_publishers();

  // Setup services for the node
  void setup_services();

  // Callback function for odometry data
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

  // Publish the trajectory as a marker array
  void publish_trajectory();

  // Callback function to save the trajectory to a file
  bool save_trajectory_callback(trajectory_tools::SaveTrajectory::Request& request, trajectory_tools::SaveTrajectory::Response& response);
};

#endif // TRAJECTORY_PUBLISHER_SAVER_H