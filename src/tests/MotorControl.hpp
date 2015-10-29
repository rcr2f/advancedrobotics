/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 *             Rebecca Curtis ( recu2236@colorado.edu )
 * Desc      : 
 * Copyright 2015 Andy McEvoy
 */

#include <string>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "InverseKinematics.hpp"

namespace stewart_platform
{

class MotorControl
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber servo_sub_;
  ros::Subscriber orientation_sub_;
  ros::Publisher servo_pub_;
  ros::Publisher servo_pubs_[6];

  double servo_goals_[6];
  double servo_positions_[6];
  double servo_errors_[6];
  Eigen::Vector3d base_orientation_;

  std::size_t id_;
  trajectory_msgs::JointTrajectory trajectory_msg_;

  Eigen::Vector3d arm_loc_on_base_[6];
  Eigen::Vector3d calc_arm_loc_on_base_[6];
  Eigen::Vector3d arm_loc_on_platform_[6];
  double arm_lengths_[6];
  double calc_arm_lengths_[6];
  double min_platform_height_;
  double max_platform_height_;
  double platform_height_;

  double servoMinPos = -1.4;
  double servoMaxPos = 1.5708;
  double servoNeutralPos = 0.0;


public:

  MotorControl()
    : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("MotorControl_Setup","\033[1;32m" << "Initializing Servo Motors.." << "\033[0m");
    
    initialize();

    // subscribe to controller topic to monitor servo states
    servo_sub_ = nh_.subscribe("/stewart_controller/state", 10, &IKTester::servoCallback, this);
    
    // subscribe to accelerometer topic for base orientation
    orientation_sub_ = nh_.subscribe("/stewart_platform/orientation", 10, &IKTester::orientationCallback, this);

    // publish motor commands
    id_=0;
    servo_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/stewart_controller/command",10);

    std::stringstream pub_topic;
    for (std::size_t i = 0; i < 6; i++)
    {
      pub_topic.str("");
      pub_topic << "/servo_" << i + 1 << "/command";
      servo_pubs_[i] = nh_.advertise<std_msgs::Float64>(pub_topic.str(),10);      
      ROS_DEBUG_STREAM_NAMED("MotorControl_Setup","Setting up publisher: " << pub_topic.str());      
    }
    ros::Duration(1.0).sleep();

    /***** Bring Platform to neutral position *****/
    ROS_INFO_STREAM_NAMED("MotorControl_Setup","Moving to neutral position...");
    for (std::size_t i = 0; i < 6; i++)
    {
      sendSingleServoCommand(i, servoNeutralPos);
    }
    ros::Duration(2.0).sleep();
  }

  void initialize()
  {
    // set min and max platform heights
    min_platform_height_ = 0.3302;
    max_platform_height_ = 0.4064;
    platform_height_ = (max_platform_height_ + min_platform_height_ ) / 2.0;

    // arm locations on the base in the base coordinate system
    arm_loc_on_base_[0][0] = -0.114100;
    arm_loc_on_base_[0][1] = 0.105251;
    arm_loc_on_base_[0][2] = 0.088900;

    arm_loc_on_base_[1][0] = 0.114100;
    arm_loc_on_base_[1][1] = 0.105251;
    arm_loc_on_base_[1][2] = 0.088900;

    arm_loc_on_base_[2][0] = 0.148200;
    arm_loc_on_base_[2][1] = 0.046188;
    arm_loc_on_base_[2][2] = 0.088900;

    arm_loc_on_base_[3][0] = 0.034100;
    arm_loc_on_base_[3][1] = -0.151439;
    arm_loc_on_base_[3][2] = 0.088900;

    arm_loc_on_base_[4][0] = -0.034100;
    arm_loc_on_base_[4][1] = -0.151439;
    arm_loc_on_base_[4][2] = 0.088900;

    arm_loc_on_base_[5][0] = -0.148200;
    arm_loc_on_base_[5][1] = 0.046188;
    arm_loc_on_base_[5][2] = 0.088900;

    // arm locations on the platform in the platform coordinate system
    arm_loc_on_platform_[0][0] = -0.038100;
    arm_loc_on_platform_[0][1] = 0.104902;
    arm_loc_on_platform_[0][2] = platform_height_;

    arm_loc_on_platform_[1][0] = 0.038100;
    arm_loc_on_platform_[1][1] = 0.104902;
    arm_loc_on_platform_[1][2] = platform_height_;
    
    arm_loc_on_platform_[2][0] = 0.109898;
    arm_loc_on_platform_[2][1] = -0.019455;
    arm_loc_on_platform_[2][2] = platform_height_;

    arm_loc_on_platform_[3][0] = 0.071798;
    arm_loc_on_platform_[3][1] = -0.085447;
    arm_loc_on_platform_[3][2] = platform_height_;

    arm_loc_on_platform_[4][0] = -0.071798;
    arm_loc_on_platform_[4][1] = -0.085447;
    arm_loc_on_platform_[4][2] = platform_height_;

    arm_loc_on_platform_[5][0] = -0.109898;
    arm_loc_on_platform_[5][1] = -0.019455;
    arm_loc_on_platform_[5][2] = platform_height_;

    // set arm lengths
    arm_lengths_[0] = 10.95;
    arm_lengths_[1] = 10.95;
    arm_lengths_[2] = 10.95;
    arm_lengths_[3] = 10.95;
    arm_lengths_[4] = 10.95;
    arm_lengths_[5] = 10.95;

  }

  void servoCallback(const control_msgs::FollowJointTrajectoryFeedback msg) 
  {
    for (std::size_t i = 0; i < 6; i++)
    {
      servo_goals_[i] = msg.desired.positions[i];
      servo_positions_[i] = msg.actual.positions[i];
      servo_errors_[i] = msg.error.positions[i];
    }

  }

  void orientationCallback(const geometry_msgs::Vector3 msg)
  {
    base_orientation_[0] = degToRad(msg.x);
    base_orientation_[1] = degToRad(msg.y);
    base_orientation_[2] = degToRad(msg.z);
    ROS_DEBUG_STREAM_NAMED("MotorControl_Setup","orientation: " << base_orientation_[0] << ", " << 
                           base_orientation_[1] << ", " << base_orientation_[2]);

    calculateArmLengths();

  }

  double degToRad(double degrees)
  {
    return degrees*3.14159/180;
  }

  bool isInServoRange(double radians)
  {
    return !(radians > servoMaxPos || radians < servoMinPos);
  }


  bool sendServoCommand(const double angles[6])
  {
    // create message header
    ROS_DEBUG_STREAM_NAMED("MotorControl_Setup","sending command to servo...");
    trajectory_msg_.header.seq = id_++;
    trajectory_msg_.header.stamp = ros::Time::now();
    trajectory_msg_.header.frame_id = "/world";
    
    std::vector<std::string> joint_names;
    joint_names.push_back("one");
    joint_names.push_back("two");
    joint_names.push_back("three");
    joint_names.push_back("four");
    joint_names.push_back("five");
    joint_names.push_back("six");
    
    trajectory_msg_.joint_names = joint_names;

    trajectory_msgs::JointTrajectoryPoint joint_trajectory_point_msg;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    std::vector<double> effort;

    for (std::size_t i = 0; i < 6; i++)
    {
      positions.push_back(angles[i]);
      velocities.push_back(0.1);
      accelerations.push_back(0);
      effort.push_back(0);
    }

    ros::Duration time_from_start = ros::Duration(2.0);

    joint_trajectory_point_msg.positions = positions;
    joint_trajectory_point_msg.velocities = velocities;
    joint_trajectory_point_msg.accelerations = accelerations;
    joint_trajectory_point_msg.effort = effort;
    joint_trajectory_point_msg.time_from_start = time_from_start;

    points.push_back(joint_trajectory_point_msg);

    trajectory_msg_.points = points;

    // publish message
    servo_pub_.publish(trajectory_msg_);
    ros::spinOnce();

    return true;
  }

  bool sendSingleServoCommand(int servo_id, double angle)
  {
    std_msgs::Float64 command;
    command.data = angle;

    servo_pubs_[servo_id].publish(command);
    ros::spinOnce();

    return true;
  }


}; // end class MotorControl
}
