cmake_minimum_required(VERSION 2.8.3)
project(concatenate_clouds)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  pcl_ros
  pcl_conversions
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL 1.7.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost COMPONENTS format)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(concatenate_clouds src/concatenate_clouds.cpp)
target_link_libraries(concatenate_clouds ${catkin_LIBRARIES} ${PCL_LIBRARIES})