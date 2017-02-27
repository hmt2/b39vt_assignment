cmake_minimum_required(VERSION 2.8.3)
project(b39vt_assignment)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(image_subscriber src/image_subscriber.cpp src/image_processing.cpp)
add_dependencies(image_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_subscriber ${catkin_LIBRARIES})

add_executable(object_avoidance src/object_avoidance.cpp)
add_dependencies(object_avoidance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_avoidance ${catkin_LIBRARIES})

