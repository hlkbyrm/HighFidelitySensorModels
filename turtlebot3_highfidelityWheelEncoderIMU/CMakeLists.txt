################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_description_poisson)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  rosconsole
  diagnostic_updater
  std_msgs
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES
  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  rosconsole
  std_msgs
)

###########
## Build ##
###########

add_library(noisy_imu_sensor plugins/noisy_imu_sensor.cpp)
target_link_libraries(noisy_imu_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(noisy_diff_drive SHARED plugins/noisy_diff_drive.cpp)
target_link_libraries(noisy_diff_drive gazebo_ros_utils ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES}) 

add_library(gazebo_ros_utils SHARED plugins/gazebo_ros_utils.cpp)
target_link_libraries(gazebo_ros_utils ${OpenCV_LIBS} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})



##########
## Test ##
##########

#############
## Install ##

