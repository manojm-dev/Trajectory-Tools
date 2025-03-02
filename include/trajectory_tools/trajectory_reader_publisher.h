#ifndef TRAJECTORY_READER_PUBLISHER_H
#define TRAJECTORY_READER_PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

// Main class for the Trajectory Reader and Publisher node
class TrajectoryReaderPublisher {
public:
  // Constructor to initialize the node
  TrajectoryReaderPublisher();

private:
  // Structure to store trajectory points with position and timestamp
  struct TrajectoryPoint {
    geometry_msgs::Point point;
    double timestamp; 
  };

  std::vector<TrajectoryPoint> trajectory_;  // Vector to store trajectory points
  ros::Publisher marker_pub_;  // Marker publisher
  std::string odom_topic_;  // Topic name for odometry data
  std::string marker_topic_;  // Topic name for marker data
  std::string trajectory_folder_path_;  // Path to save trajectory files
  std::string trajectory_file_name_;  // Name of the trajectory file

  // Load parameters from the parameter server
  void load_parameters();

  // Setup publishers for the node
  void setup_publishers();

  // Read the trajectory from a JSON file
  void read_trajectory_from_file();

  // Publish the trajectory as a marker array
  void publish_trajectory();
};

#endif // TRAJECTORY_READER_PUBLISHER_H