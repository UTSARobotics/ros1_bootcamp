cmake_minimum_required(VERSION 3.0.2)
project(smb_highlevel_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(laser_scan_subscriber src/laserscan_min_distance_subscriber.cpp)
target_link_libraries(laser_scan_subscriber ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/smb_highlevel_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})




