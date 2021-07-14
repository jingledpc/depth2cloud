cmake_minimum_required(VERSION 3.0.2)
project(depth2cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  pcl_conversions
  cv_bridge
)

find_package(PCL REQUIRED QUIET)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES depth2cloud
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(
	# include
	${PCL_LIBRARY_DIRS}
)

add_executable(depth2cloud_node src/depth2cloud_rate_node.cpp)
add_dependencies(depth2cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth2cloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(save_cloud src/save_cloud.cpp)
add_dependencies(save_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
