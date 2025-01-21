cmake_minimum_required(VERSION 3.5)
project(colibus_simulator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/)


ament_export_dependencies(rosidl_default_runtime)
rosidl_generate_interfaces(${PROJECT_NAME} 
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/Beep.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/BlkLights.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/CarInterfaceCommand.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/CarInterfaceConditions.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/CarInterfaceStatus.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/CarStatus.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/ClimaStatus.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/ControlRefs.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/Forces.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/LCDText.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/PID.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/SpeedAccelData.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/SpeedControlVars.msg"

  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:srv/GetBlStatus.srv"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:srv/GetControlMode.srv"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:srv/GetRunMode.srv"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:srv/PIDParams.srv"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:srv/SetControlMode.srv"
  
  "/home/toto/ws/ros2_ws/src/colibus_simulator/ros_zoe_msgs:msg/SteeringData.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/icars_msgs:msg/CarId.msg"
  "/home/toto/ws/ros2_ws/src/colibus_simulator/icars_msgs:msg/LogErr.msg"
  DEPENDENCIES std_msgs

)

# Build the package
ament_package()