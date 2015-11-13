#! /usr/bin/env python
from __future__ import absolute_import, division, print_function

import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import (euler_from_quaternion,
                                quaternion_from_euler)

from utils import dispatch, _Getch


class Controller(object):
    """A keyboard-controlled pose publisher."""
    def __init__(self,
                 initial_pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                 log_level=rospy.INFO):
        self.nh = rospy.init_node("controller")
        self.pub = rospy.Publisher("/stewart_platform/pose",
                                   Pose,
                                   queue_size=10)
        self.pose = initial_pose

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, new_pose):
        rospy.logdebug("Setting pose: {}".format(new_pose))
        print(new_pose, type(new_pose))
        self._pose = new_pose
        self.pub.publish(new_pose)

    def start(self):
        rospy.loginfo(
            """\n\nCONTROLLER KEYMAP (HELP):
            wasd : control XY
            12 : control Z
            ijkl : control roll & pitch
            34 : control yaw

            q : quit\n""")
        while True:
            pose = self.pose

            p = pose.position
            # https://github.com/ros/geometry/issues/109
            o = euler_from_quaternion((pose.orientation.x,
                                       pose.orientation.y,
                                       pose.orientation.z,
                                       pose.orientation.w))
            rospy.loginfo(
                "Current position (mm):\n{}".format(p))
            rospy.loginfo(
                "Current orientation (deg): {}".format(np.rad2deg(o)))

            gchr = _Getch()
            user_input = gchr()
            if user_input == 'q':
                break
            p, o = dispatch(user_input, p, o)
            pose = Pose(position=Point(*p),
                        orientation=Quaternion(*quaternion_from_euler(*o)))
            self.pose = pose


if __name__ == '__main__':
    c = Controller(initial_pose=Pose(Point(0, 0, 0.275),
                                     Quaternion(0, 0, 0, 1)),
                   log_level=rospy.DEBUG)
    c.start()
