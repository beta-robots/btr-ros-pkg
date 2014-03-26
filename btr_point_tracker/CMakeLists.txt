cmake_minimum_required(VERSION 2.8.3)
project(btr_point_tracker)

## Find catkin macros and libraries
find_package(catkin REQUIRED 
            COMPONENTS roscpp rospy dynamic_reconfigure message_generation cv_bridge image_transport sensor_msgs geometry_msgs)

## System dependencies are found with CMake's conventions
find_package(point_tracker REQUIRED)

#SRCS files
SET(SRCS 
      src/point_tracker_node.cpp
)

## Add messages to be generated
add_message_files(FILES pointTracks.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

#dynamic reconfigure cfg file
generate_dynamic_reconfigure_options(cfg/trackerParams.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      CATKIN_DEPENDS roscpp rospy dynamic_reconfigure message_runtime cv_bridge image_transport sensor_msgs geometry_msgs
      DEPENDS point_tracker)

## Specify additional locations of header files
include_directories(${point_tracker_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(btr_point_tracker_node ${SRCS} src/point_tracker_main.cpp)
add_dependencies(btr_point_tracker_node ${PROJECT_NAME}_EXPORTED_TARGETS)
target_link_libraries(btr_point_tracker_node ${point_tracker_LIBRARIES} ${catkin_LIBRARIES})
