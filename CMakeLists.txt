cmake_minimum_required(VERSION 2.8.3)
project(hear_navio_fc_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hear_hardware_interface
  hear_architecture
  hear_ros_bridge
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    hear_ros_bridge
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_flight_controller_node example_node/example_node.cpp)
target_link_libraries(example_flight_controller_node ${catkin_LIBRARIES})

add_executable(global2inertial_node global2inertial_node/global2inertial_node.cpp)
target_link_libraries(global2inertial_node ${catkin_LIBRARIES})

add_executable(providers_node providers_node/providers_node.cpp)
target_link_libraries(providers_node ${catkin_LIBRARIES})

add_executable(waypoint_reference_node waypoint_reference_node/waypoint_reference_node.cpp)
target_link_libraries(waypoint_reference_node ${catkin_LIBRARIES})