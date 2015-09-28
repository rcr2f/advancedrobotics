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
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED regex)

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
    include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(stewart_ik_test
  src/tests/ik_test.cpp
)
target_link_libraries(stewart_ik_test
  ${catkin_LIBRARIES}
)

add_executable(stewart_accel_test
  src/tests/accel_test.cpp
)
target_link_libraries(stewart_accel_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

