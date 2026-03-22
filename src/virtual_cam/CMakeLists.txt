cmake_minimum_required(VERSION 3.8)
project(virtual_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
include_directories($ENV{ONNXRUNTIME_DIR}/include)
link_directories($ENV{ONNXRUNTIME_DIR}/lib)

add_library(virtual_cam_component SHARED
  src/virtual_cam.cpp
)
ament_target_dependencies(virtual_cam_component rclcpp cv_bridge sensor_msgs image_transport tf2_ros rclcpp_components)
target_link_libraries(virtual_cam_component ${OpenCV_LIBS})
rclcpp_components_register_nodes(virtual_cam_component "VirtualCamNode")

add_library(cone_detector_component SHARED
  src/cone_detector.cpp
)
ament_target_dependencies(cone_detector_component rclcpp sensor_msgs cv_bridge image_transport rclcpp_components tf2_ros tf2)
target_link_libraries(cone_detector_component ${OpenCV_LIBS} libonnxruntime.so)
rclcpp_components_register_nodes(cone_detector_component "ImageSubscriberComponent")

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

install(TARGETS virtual_cam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS cone_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
