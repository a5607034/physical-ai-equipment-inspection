cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

# ROS 2 Jazzy / ament_cmake
find_package(ament_cmake REQUIRED)

# Gazebo simulation launch
install(
  FILES
    launch/my_robot_gazebo.launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

# ROS 2 - Gazebo bridge configuration
install(
  FILES
    config/gazebo_bridge.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Gazebo equipment models
install(
  FILES
    models/equipment_A.sdf
    models/equipment_B.sdf
  DESTINATION share/${PROJECT_NAME}/models
)

# Gazebo simulation world
install(
  FILES
    worlds/test_world.sdf
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
