cmake_minimum_required(VERSION 3.5)
project(map_manager)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckPosCollision.srv"
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add the shared library
add_library(raycaster SHARED include/map_manager/raycast.cpp)
target_link_libraries(raycaster ${PCL_LIBRARIES})
install(TARGETS raycaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Add the executable
add_executable(occupancy_map_node src/occupancy_map_node.cpp include/map_manager/occupancyMap.cpp)

# Link libraries
target_link_libraries(occupancy_map_node ${PCL_LIBRARIES} raycaster)

# Specify libraries to link a library or executable target against
ament_target_dependencies(occupancy_map_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  Eigen3
  cv_bridge
  image_transport
)

# Link the generated service files
rosidl_target_interfaces(occupancy_map_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets and directories
install(TARGETS
  occupancy_map_node
  raycaster
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()

