cmake_minimum_required(VERSION 2.8.3)
project(stewart_platform)

find_package(catkin REQUIRED COMPONENTS
  roslib
  genmsg
  roscpp
  rospy
  std_msgs
  control_msgs
  geometry_msgs
)

find_package(cmake_modules REQUIRED)

generate_messages(
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    control_msgs
    geometry_msgs
  DEPENDS
    Eigen
  INCLUDE_DIRS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


#############
## Install ##
#############


#############
## Testing ##
#############

