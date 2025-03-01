cmake_minimum_required(VERSION 3.0.2)
project(semantic_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  message_filters
  image_transport
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  TrafficLightStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs cv_bridge message_filters image_transport std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(semantic_camera_node src/semantic_camera_node.cpp)
add_executable(traffic_light_controller src/traffic_light_controller.cpp)

target_link_libraries(semantic_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(traffic_light_controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(semantic_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(traffic_light_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
