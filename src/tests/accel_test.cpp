/*
 * Copyright 2015 Andy McEvoy
 * Authors : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc    : A simple test for the Sparkfun 9DOF Razor IMU
 */

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>

#include <boost/regex.hpp>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

#include <Eigen/Core>

namespace stewart_platform
{

class AccelTester
{
private:
  ros::NodeHandle nh_;

  FILE* serial_port_;
  struct termios::termios serial_config_, serial_config_old_;
  char read_buff_[1024];
  Eigen::Vector3d base_orientation_;
  Eigen::Vector3d level_readings_;

  ros::Publisher orientation_pub_;

  geometry_msgs::Vector3 base_orientation_msg_;

public:

  AccelTester()
    : nh_("~")
  {

    openSerialPort();
    if (!setSerialConfig())
    {
      ROS_ERROR_STREAM_NAMED("accel_test","did not set serial port parameters correctly. exiting.");
      exit(0);
    }

    orientation_pub_ = nh_.advertise<geometry_msgs::Vector3>("/stewart_platform/orientation",10);

    ROS_INFO_STREAM_NAMED("accel_test","leveling accelerometer readings...");
    // throw away first few readings while they settle
    for (std::size_t i = 0; i < 150; i++)
    {
      readSerialPort();
    }

    level_readings_ = Eigen::Vector3d::Zero();
    ROS_INFO_STREAM_NAMED("accel_test","begin leveling...");
    for (std::size_t i = 0; i < 50; i++)
    {
      readSerialPort();
      level_readings_[0] += base_orientation_[0];
      level_readings_[1] += base_orientation_[1];
      level_readings_[2] += base_orientation_[2];
    }
    level_readings_ /= 50.0;
    ROS_DEBUG_STREAM_NAMED("accel_test","subtracting offsets: " << level_readings_[0] << ", " << 
                           level_readings_[1] << ", " << level_readings_[2]);

    ROS_INFO_STREAM_NAMED("accel_test","starting main loop...");
    while (ros::ok())
    {
      readSerialPort();
      publishOrientationMsg(base_orientation_);
    }

    closeSerialPort();
  }

  bool openSerialPort()
  {
    ROS_DEBUG_STREAM_NAMED("accel_test","opening serial port...");

    const char* device = "/dev/razor";
    serial_port_ = fopen(device, "r+");

    ROS_DEBUG_STREAM_NAMED("accel_test","serial_port_ = " << serial_port_);

    if (serial_port_ == NULL)
    {
      ROS_WARN_STREAM_NAMED("accel_test","Serial port did not open properly. Exiting...");
      return false;
    }
    else
      return true;
  }

  bool closeSerialPort()
  {
    ROS_DEBUG_STREAM_NAMED("accel_test","closing serial port...");
    fclose(serial_port_);
  }

  bool setSerialConfig()
  {
    ROS_DEBUG_STREAM_NAMED("accel_test","setting serial interface parameters... ");

    // check that the file descriptor is TTY device
    if (!isatty(fileno(serial_port_)))
    {
      ROS_ERROR_STREAM_NAMED("accel_test","device is not a TTY device.");
      return false;
    }
    
    // get current configuration
    if (tcgetattr(fileno(serial_port_), &serial_config_) < 0)
    {
      ROS_ERROR_STREAM_NAMED("accel_test","could not get current serial config.");
      return false;
    }
    memset(&serial_config_, 0, sizeof serial_config_); // clear for new settings

    // set baud rate
    if (cfsetispeed(&serial_config_, (speed_t)B57600) < 0 )
    {
      ROS_ERROR_STREAM_NAMED("accel_test","could not set input baud rate");
      return false;
    }

    if (cfsetospeed(&serial_config_, B57600) < 0 )
    {
      ROS_ERROR_STREAM_NAMED("accel_test","could not set output baud rate");
      return false;
    }

    serial_config_.c_cflag |= CRTSCTS | CS8 | CLOCAL | CREAD;
    serial_config_.c_iflag = IGNPAR | ICRNL;
    serial_config_.c_oflag = 0;
    serial_config_.c_lflag = ICANON;
    
    cfmakeraw(&serial_config_);

    tcflush(fileno(serial_port_), TCIFLUSH);
    if (tcsetattr(fileno(serial_port_), TCSANOW, &serial_config_) != 0 )
    {
      ROS_WARN_STREAM_NAMED("accel_test","serial parameters not set properly");
      return false;
    }

    return true;
  }

  bool readSerialPort()
  {
    //ROS_DEBUG_STREAM_NAMED("accel_test","readSerialPort()");

    memset(&read_buff_[0], 0, sizeof(read_buff_));
    fscanf(serial_port_, "%1023s", read_buff_);
    //ROS_DEBUG_STREAM_NAMED("accel_test", "read_buff_ = " << read_buff_);

    // pull out roll pitch and yaw
    std::string s = read_buff_;
    //ROS_DEBUG_STREAM_NAMED("accel_test","string = " << s);

    boost::regex re("-?\\d+\\.\\d+");
    boost::match_results<std::string::const_iterator> matches;

    std::string::const_iterator search_start, search_end;
    search_start = s.begin();
    search_end = s.end();

    /***** DEV NOTE *****/
    // serial is printed as Yaw, Pitch, Roll
    int count = 0;
    while (boost::regex_search(search_start, search_end, matches, re))
    {
      //ROS_DEBUG_STREAM_NAMED("accel_test","match[" << count << "] = " << matches[0]);      
      if (count > 3)
      {
        ROS_WARN_STREAM_NAMED("accel_test","got more than 3 values from accel...");
        break;
      }
      std::string value = matches[0];
      base_orientation_[count] = atof(value.c_str());
      search_start = matches[1].second;
      count += 1;
    }
  }

  void publishOrientationMsg(const Eigen::Vector3d orientation)
  {
    base_orientation_msg_.z = orientation[0] - level_readings_[0];
    base_orientation_msg_.y = orientation[1] - level_readings_[1];
    base_orientation_msg_.x = orientation[2] - level_readings_[2];

    ROS_DEBUG_STREAM_NAMED("accell_test","orientation_ = " << base_orientation_msg_.x << ", " <<
                           base_orientation_msg_.y << ", " << base_orientation_msg_.z);

    orientation_pub_.publish(base_orientation_msg_);
  }

}; // end class AccelTester

} // end namespace stewart_platform

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stewart_platform_accel_test");

  ROS_INFO_STREAM_NAMED("accel_test", "\033[1;32m" << "Starting stewart platform accel test" << "\033[0m");

  stewart_platform::AccelTester tester;
  ros::spin();
}

