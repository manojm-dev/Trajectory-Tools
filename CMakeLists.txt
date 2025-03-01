cmake_minimum_required(VERSION 3.8)
project(trajectory_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)  
find_package(nlohmann_json REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
)

# Get type support for ROS messages 
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Define common dependencies
set(COMMON_DEPENDENCIES
  rclcpp
  visualization_msgs
  nav_msgs
  geometry_msgs
  nlohmann_json
  rosidl_default_runtime
)

# Create executables
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)

# Link common dependencies for both executables
ament_target_dependencies(trajectory_publisher_saver ${COMMON_DEPENDENCIES})
ament_target_dependencies(trajectory_reader_publisher ${COMMON_DEPENDENCIES})

# Link typesupport for both executables
target_link_libraries(trajectory_publisher_saver ${cpp_typesupport_target})
target_link_libraries(trajectory_reader_publisher ${cpp_typesupport_target})

# Install executables
install(
  TARGETS trajectory_publisher_saver trajectory_reader_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Export package dependencies
ament_export_dependencies(${COMMON_DEPENDENCIES})

ament_package()
