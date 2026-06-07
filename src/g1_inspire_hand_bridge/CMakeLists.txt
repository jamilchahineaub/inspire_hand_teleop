cmake_minimum_required(VERSION 3.8)
project(g1_inspire_hand_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(hand_perception_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# This package's own state message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandState.msg"
  DEPENDENCIES std_msgs
)

# ── Bridge node ────────────────────────────────────────────────────────────
add_executable(g1_inspire_hand_bridge_node src/g1_inspire_hand_bridge_node.cpp)
ament_target_dependencies(g1_inspire_hand_bridge_node
  rclcpp std_msgs unitree_go hand_perception_msgs)
# Link against this package's generated HandState message
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(g1_inspire_hand_bridge_node ${cpp_typesupport_target})

# ── Test publisher ─────────────────────────────────────────────────────────
add_executable(test_hand_cmd_publisher src/test_hand_cmd_publisher.cpp)
ament_target_dependencies(test_hand_cmd_publisher
  rclcpp hand_perception_msgs)

install(TARGETS
  g1_inspire_hand_bridge_node
  test_hand_cmd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
