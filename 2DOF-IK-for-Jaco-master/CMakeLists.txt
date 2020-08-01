cmake_minimum_required(VERSION 2.8.3)
project(move_kinova)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  std_srvs
)


catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs trajectory_msgs std_srvs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(Yeah_virtual_robot_demo src/Yeah_virtual_robot_demo.cpp)
target_link_libraries(
  Yeah_virtual_robot_demo 
  ${catkin_LIBRARIES}
  # syntax: target_link_libraries(<executableTargetName> <lib>)
)

add_executable(teleop_key src/teleop_key.cpp)
target_link_libraries(
  teleop_key 
  ${catkin_LIBRARIES}
  # syntax: target_link_libraries(<executableTargetName> <lib>)
)

add_executable(Simple_IK_virtual_robot src/Simple_IK_virtual_robot.cpp)
target_link_libraries(
  Simple_IK_virtual_robot 
  ${catkin_LIBRARIES}
  # syntax: target_link_libraries(<executableTargetName> <lib>)
)


add_executable(test src/test.cpp)
target_link_libraries(
  test
  ${catkin_LIBRARIES}
  # syntax: target_link_libraries(<executableTargetName> <lib>)
)

add_executable(template src/template.cpp)
target_link_libraries(
  template
  ${catkin_LIBRARIES}
  # syntax: target_link_libraries(<executableTargetName> <lib>)
)
