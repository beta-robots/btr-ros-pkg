cmake_minimum_required(VERSION 2.8.3)
project(btr_bfly_camera)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs image_transport cv_bridge dynamic_reconfigure)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(bfly_camera REQUIRED)
SET (ptgrey_INCLUDE_DIRS "/usr/include/flycapture")
SET (ptgrey_LIBRARIES "flycapture")

#dynamic reconfigure cfg file
generate_dynamic_reconfigure_options(cfg/bflyParams.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      CATKIN_DEPENDS roscpp rospy sensor_msgs image_transport cv_bridge dynamic_reconfigure
      DEPENDS OpenCV bfly_camera
)

## Specify additional locations of header files
include_directories(${OpenCV_INCLUDE_DIRS} ${bfly_camera_INCLUDE_DIRS} ${ptgrey_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(btr_bfly_camera_node src/btr_bfly_camera_node.cpp)
add_dependencies(btr_bfly_camera_node ${PROJECT_NAME}_EXPORTED_TARGETS)
target_link_libraries(btr_bfly_camera_node  ${OpenCV_LIBRARIES} ${bfly_camera_LIBRARIES} ${ptgrey_LIBRARIES} ${catkin_LIBRARIES})
