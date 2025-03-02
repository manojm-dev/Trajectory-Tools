cmake_minimum_required(VERSION 3.0.2)
project(trajectory_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
  message_generation
)

find_package(nlohmann_json REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveTrajectory.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs visualization_msgs geometry_msgs std_srvs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)

target_link_libraries(trajectory_publisher_saver
  ${catkin_LIBRARIES}
  nlohmann_json::nlohmann_json
)

add_dependencies(trajectory_publisher_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)

target_link_libraries(trajectory_reader_publisher
  ${catkin_LIBRARIES}
  nlohmann_json::nlohmann_json
)

add_dependencies(trajectory_reader_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install executables 
install(
  TARGETS trajectory_publisher_saver trajectory_reader_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install rviz configuration folder
install(
  DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)