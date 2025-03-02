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
class TrajectoryReaderPublisher
{
public:
  // Constructor to initialize the node
  TrajectoryReaderPublisher()
  {
    ROS_INFO("Initializing TrajectoryReaderPublisher node...");
    load_parameters();
    setup_publishers();
    read_trajectory_from_file();
  }

private:
  // Structure to store trajectory points with position and timestamp
  struct TrajectoryPoint
  {
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
  void load_parameters()
  {
    ros::NodeHandle nh("~");
    nh.param<std::string>("odom_topic", odom_topic_, "odom");
    nh.param<std::string>("marker_topic", marker_topic_, "/trajectory_marker");
    nh.param<std::string>("trajectory_folder_path", trajectory_folder_path_, "/tmp/");
    nh.param<std::string>("trajectory_file_name", trajectory_file_name_, "test");

    ROS_INFO("Parameters loaded: odom_topic=%s, marker_topic=%s, trajectory_folder_path=%s, trajectory_file_name=%s",
             odom_topic_.c_str(), marker_topic_.c_str(), trajectory_folder_path_.c_str(), trajectory_file_name_.c_str());
  }

  // Setup publishers for the node
  void setup_publishers()
  {
    ros::NodeHandle nh;
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
    ROS_INFO("Publisher set up for topic: %s", marker_topic_.c_str());
  }

  // Read the trajectory from a JSON file
  void read_trajectory_from_file()
  {
    std::string trajectory_file_path = trajectory_folder_path_ + trajectory_file_name_ + ".json";

    std::ifstream file(trajectory_file_path);
    if (!file.is_open())
    {
      ROS_ERROR("Failed to open file: %s", trajectory_file_path.c_str());
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

    ROS_INFO("Loaded %zu trajectory points from %s", trajectory_.size(), trajectory_file_path.c_str());
    publish_trajectory();
  }

  // Publish the trajectory as a marker array
  void publish_trajectory()
  {
    if (trajectory_.empty())
    {
      ROS_WARN("Trajectory is empty, nothing to publish.");
      return;
    }
  
    visualization_msgs::Marker marker;
    marker.header.frame_id = odom_topic_;
    marker.header.stamp = ros::Time::now();            
    marker.ns = "trajectory_marker";
    marker.id = 0;                                     
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
  
    // Set a valid (identity) orientation to avoid warnings
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0; 
  
    // Marker styling
    marker.scale.x = 0.05;           
    marker.color.a = 1.0;            
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  
    // Publish each point in "real time"
    for (size_t i = 0; i < trajectory_.size(); ++i)
    {
      // Add the new point to the line strip
      marker.points.push_back(trajectory_[i].point);
  
      // Update timestamp each iteration (optional but recommended)
      marker.header.stamp = ros::Time::now();
  
      // Publish the updated marker
      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.push_back(marker);
      marker_pub_.publish(marker_array);
  
      ROS_INFO("Published point at time: %f", trajectory_[i].timestamp);
  
      // Sleep for the difference in time between this and the next point
      if (i < trajectory_.size() - 1)
      {
        double diff_time = trajectory_[i + 1].timestamp - trajectory_[i].timestamp;
        std::chrono::duration<double> delay(diff_time);
        std::this_thread::sleep_for(delay);
      }
    }
  
    ROS_INFO("Completed publishing trajectory.");
  }  
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trajectory_reader_publisher");
  ROS_INFO("Starting TrajectoryReaderPublisher node...");
  TrajectoryReaderPublisher node;
  ros::spin();
  ROS_INFO("Shutting down TrajectoryReaderPublisher node...");
  return 0;
}