cmake_minimum_required(VERSION 3.8)
project(lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS 
  common 
  filters 
  segmentation 
  kdtree 
  surface
  visualization 
  conversions
)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(lidar_component SHARED
  src/lidar_node.cpp
)
add_library(DBSCAN_component SHARED
  src/dbscan.cpp
)

ament_target_dependencies(lidar_component
  rclcpp
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
  pcl_conversions
  rclcpp_components
)

ament_target_dependencies(DBSCAN_component
  rclcpp
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
  pcl_conversions
  rclcpp_components
  Eigen3
)

target_link_libraries(lidar_component
  ${PCL_COMMON_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
  ${PCL_KDTREE_LIBRARIES}
  ${PCL_SURFACE_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${PCL_CONVERSIONS_LIBRARIES}
)

target_link_libraries(DBSCAN_component
  ${PCL_COMMON_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
  ${PCL_KDTREE_LIBRARIES}
  ${PCL_SURFACE_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${PCL_CONVERSIONS_LIBRARIES}
)

rclcpp_components_register_nodes(lidar_component "RosPointCloudBagPublisher")
rclcpp_components_register_nodes(DBSCAN_component "DBSCAN_component")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS lidar_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS DBSCAN_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/&{PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
