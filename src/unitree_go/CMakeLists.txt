cmake_minimum_required(VERSION 3.5)
project(unitree_go)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Vendored subset of the Unitree unitree_go messages.
# The upstream package also generates Connext DDS IDL (rosidl_generator_dds_idl);
# that is dropped here because we use CycloneDDS (rmw_cyclonedds_cpp), which needs
# only the standard ROS 2 message types. We vendor just the Motor* messages the
# Inspire hand bridge uses.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/MotorCmds.msg"
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()
