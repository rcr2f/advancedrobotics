/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 *             Rebecca Curtis ( recu2236@colorado.edu )
 * Desc      : 
 * Copyright 2015 Andy McEvoy
 */


#include "MotorControl.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stewart_platform_main");

  ROS_INFO_STREAM_NAMED("main","\033[1;32m" << "Stabilizing the stewart platform...eventually" << "\033[0m");
  
  stewart_platform::MotorControl controller;
  ros::spin();



  return 0;
}
