cmake_minimum_required(VERSION 3.8)
project(cam_pos_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)



# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/GetCameraPose.srv"
#   DEPENDENCIES geometry_msgs
# )


add_executable(robot_position
  src/robot_position_node.cpp 
  src/image_processor.cpp
  src/aruco_detector.cpp
  src/pose_calculator.cpp
)
# add_executable(camera_position src/camera_position_service.cpp)
add_executable(report_img src/report_img.cpp)


target_include_directories(robot_position PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)

# target_include_directories(camera_position PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
# )

target_include_directories(report_img PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)


ament_target_dependencies(robot_position rclcpp sensor_msgs OpenCV cv_bridge)
# ament_target_dependencies(camera_position rclcpp geometry_msgs)
ament_target_dependencies(report_img OpenCV)


install(TARGETS
  robot_position
  # camera_position
  report_img
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
