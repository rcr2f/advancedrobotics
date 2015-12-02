# Advanced Robotics Fall 2015
## Stewart Platform Fork

Code repository and issue tracker for the Fall 2015 of "Advanced Robotics" (Independent Study group)

http://correll.cs.colorado.edu/?page_id=3748

This repo contains the files necessary to build and control a Stewart platform.

### Outline of this README

1. Getting Started
2. Setting up the main Stewart Platform Workspace
3. Setting up the IMU workspace
4. Running the Stewart Platform
5. Windows Dynamixel Manager

# Getting Started

This code uses:

- Ubuntu 14.04

- ROS Indigo


Install ROS indigo if you haven't already: http://wiki.ros.org/indigo/Installation/Ubuntu

Setup environment variables:
```
source /opt/ros/indigo/setup.bash
```

Additional dependencies to install:
```
sudo apt-get install ros-indigo-rosserial-arduino ros-indigo-rosserial ros-indigo-rosserial-server python-catkin-tools
```

Install Dynamixel Dependencies:
```
sudo apt-get install ros-indigo-dynamixel-motor
```

The inverse kinematics calculations for the stewart platform are [here](http://www.instructables.com/id/Stewart-Platform/?ALLSTEPS).


# Setting up the main Stewart Platform workspace

Setup the workspace:
```
mkdir -p ~/stewart_ws/src
cd ~/stewart_ws/src
catkin_init_workspace
```


### Clone this repo into the catkin workspace

```
cd ~/stewart_ws/src
git clone <https://address>
mv advancedrobotics/ stewart_platform/
```

### Build this workspace

```
cd ~/stewart_ws
mkdir src/stewart_platform/include
catkin build
source ~/stewart_ws/devel/setup.bash
sudo cp ~/stewart_ws/src/stewart_platform/dev_rules/*.rules /etc/udev/rules.d/
sudo udevadm trigger
roscd stewart_platform/
```

Make sure to unplug and replug the usb devices after running udevadm trigger.



# Setting up the IMU Workspace

A [Razor](https://github.com/ptrbrtz/razor-9dof-ahrs/wiki/Tutorial) tutorial. Use this tutorial to calibrate the Sparkfun Razor 9DOF IMU.

**NOTE** Yaw will not be correct if the magnetometer is not set up properly.


#### Razor Workspace:

Create a separate workspace for the IMU using similar instuctions. Substitute the command "catkin_make" for "catkin build". Use this [source](https://github.com/KristofRobot/razor_imu_9dof) or another similar one. A few more things will need to be done so that the IMU is calibrated and communicating with the other ros nodes. 

Modify my_razor.yaml (calibration may vary on your IMU):

```
## USB port
port: /dev/razor

##### Calibration ####
### accelerometer
accel_x_min: -289.0
accel_x_max: 292.0
accel_y_min: -263.0
accel_y_max: 316.0
accel_z_min: -279.0
accel_z_max: 232.0

### magnetometer — should be calibrated when IMU is mounted to stewart platform
# standard calibration
magn_x_min: -600.0
magn_x_max: 600.0
magn_y_min: -600.0
magn_y_max: 600.0
magn_z_min: -600.0
magn_z_max: 600.0

# AHRS to robot calibration
imu_yaw_calibration: 0.0

### gyroscope
gyro_average_offset_x: -40.71
gyro_average_offset_y: 17.44
gyro_average_offset_z: -2.55
```

Modify the publishers in imu_node.py:
```
pub = rospy.Publisher('/stewart_platform/imu', Imu, queue_size=1)
...
diag_pub = rospy.Publisher('/stewart_platform/imu/diagnostics', DiagnosticArray, queue_size=1)

```

Modify the subscriber in display_3D_visualization.py:
```
sub = rospy.Subscriber('/stewart_platform/imu', Imu, processIMU_message)
```

With roscore running, launch the IMU node:
```
roslaunch razor_imu_9dof razor-pub-and-display.launch
```


# Running the Stewart Platform

### Setup the environment variables to be sourced on startup


```
gedit ~/.bashrc
```

Add these 3 lines at the bottom. Make sure the workspace names are correct.
```
source /opt/ros/indigo/setup.bash
source ~/stewart_ws/devel/setup.bash
source ~/razor_ws/devel/setup.bash
```

Make sure roscore is running. If not, run this command in a separate terminal:
```
roscore
```

In a new terminal, bring up the dynamixel servos:
```
roslaunch stewart_platform controller_manager.launch
```

Then, you can start the main node:
```
rosrun stewart_platform stewart.py
```

That script accepts different options, and you may first want to test that the servos are working well by passing the `test` argument:
```
rosrun stewart_platform stewart.py test
```
You should see the motors move to the min range, max range, and then a neutral position when running this test. If they don't move, but you don't have any errors, try restarting everything. If you have errors, good luck!

If you want to manually control the pose from the keyboard, launch the following node:
```
rosrun stewart_platform controller.py
```





# Windows Dynamixel Manager
download the [RoboPlus software](http://en.robotis.com/BlueAD/board.php?bbs_id=downloads&mode=view&bbs_no=1132559&page=1&key=&keyword=&sort=&scate=SOFTWARE) from dynamixel

see [this page](http://learn.trossenrobotics.com/34-blog/140-ftdi-2-12-00-notice-robotis-usb2dynamixel-cm-530-and-ln-101-october-2014.html) for driver installation problems

Use the Dynamixel wizard to set up the servos. 2, 4, 6 need to be in the reverse direction mode. The files in this package assume the communication baud rate is (0x34) 57600.

