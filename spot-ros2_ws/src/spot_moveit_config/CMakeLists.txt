cmake_minimum_required(VERSION 3.22)
project(spot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(spot_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_ros REQUIRED)

# Build servo_pose_tracking_spot executable
add_executable(servo_pose_tracking_spot src/servo_pose_tracking_spot.cpp)
ament_target_dependencies(servo_pose_tracking_spot
  rclcpp
  std_msgs
  geometry_msgs
  moveit_servo
  moveit_ros_planning
  tf2_ros
)

install(TARGETS servo_pose_tracking_spot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/continuos_to_spot.py
  scripts/spot_pose_tracker.py
  scripts/target_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
