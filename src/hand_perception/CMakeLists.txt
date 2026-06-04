cmake_minimum_required(VERSION 3.8)
project(hand_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(hand_perception_msgs REQUIRED)

# Install the installable Python package (hand_perception/ subdir)
ament_python_install_package(${PROJECT_NAME})

# Install node executables — stripped of .py suffix following repo convention
install(PROGRAMS
  scripts/hand_landmark_node.py
  RENAME hand_landmark_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rs485_hand_node.py
  RENAME rs485_hand_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
