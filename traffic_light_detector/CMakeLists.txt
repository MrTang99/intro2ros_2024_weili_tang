cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  message_filters
  message_generation
)

add_message_files(
  FILES
  TrafficLightStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge message_filters message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(traffic_light_detector_node src/traffic_light_detector_node.cpp)
add_executable(traffic_light_controller_node src/traffic_light_controller.cpp)

target_link_libraries(traffic_light_detector_node
  ${catkin_LIBRARIES}
)

target_link_libraries(traffic_light_controller_node
  ${catkin_LIBRARIES}
)

add_dependencies(traffic_light_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(traffic_light_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

