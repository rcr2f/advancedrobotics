#! /usr/bin/env python
from __future__ import absolute_import, division, print_function

import math
import numpy as np
from scipy.optimize import fmin_bfgs

import rospy
import tf
from tf import transformations
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu

from pid import PID


class Stewart(object):
    """A Stewart platform actuated via servos.

    The inverse kinematics are based off
    https://web.archive.org/web/20130506134518/http://www.wokinghamu3a.org.uk/Maths%20of%20the%20Stewart%20Platform%20v5.pdf
    with some corrections available in
    http://www.memememememememe.me/controlling-stewart-platform/ and
    also derived using sympy, which was used to generate the code in
    _rotation_matrix and _calc_angles.

    """
    def __init__(self, base_coordinates, top_coordinates, servo_arm_length,
                 leg_length, beta):
        self.p = np.asarray(top_coordinates)
        self.b = np.asarray(base_coordinates)
        if self.b.shape != (3, 6) or self.p.shape != (3, 6):
            raise ValueError("There should be 6 sets of coordinates in 3D")

        self.a = servo_arm_length
        self.s = leg_length
        self.beta = np.asarray(beta)
        if len(self.beta) != 6:
            raise ValueError("There must be 6 angles in beta")

    @staticmethod
    def _rotation_matrix(orientation):
        r, p, y = tf.transformations.euler_from_quaternion(orientation)
        cphi, ctheta, cpsi = math.cos(r), math.cos(p), math.cos(y)
        sphi, stheta, spsi = math.sin(r), math.sin(p), math.sin(y)
        return np.array(
            [[cpsi*ctheta, sphi*stheta*cpsi - spsi*cphi, sphi*spsi + stheta*cphi*cpsi],
             [spsi*ctheta, sphi*spsi*stheta + cphi*cpsi, -sphi*cpsi + spsi*stheta*cphi],
             [-stheta, sphi*ctheta, cphi*ctheta]])

    def _calc_legs(self, pose):
        # pos = [pose.position.x, pose.position.y, pose.position.z]
        # orient = [pose.orientation.x, pose.orientation.y,
        #           pose.orientation.z, pose.orientation.w]
        pos = pose.position
        orient = pose.orientation
        translation = np.array(pos)[:, np.newaxis]
        rotation = self._rotation_matrix(orient)
        # Must be possible to use tf for affine transformations. Maybe
        # using PyKDL (?)
        # http://wiki.ros.org/kdl/Tutorials/Frame%20transformations%20%28Python%29
        return translation + rotation.dot(self.p) - self.b

    def _calc_angles(self, legs):
        alpha = [0] * 6
        a, s = self.a, self.s
        for i, beta in enumerate(self.beta):
            l1, l2, l3 = legs[:, i]
            A = 2*a*l1*math.cos(beta) + 2*a*l2*math.sin(beta)
            B = 2*a*l3
            C = -a**2 - l1**2 - l2**2 - l3**2 + s**2
            alpha[i] = -math.asin(C/math.sqrt(A**2 + B**2)) - math.atan2(A, B)
        return alpha

    def ik(self, pose):
        """Compute servo angles necessary for a given pose message"""
        legs = self._calc_legs(pose)
        servo_angles = self._calc_angles(legs)
        return servo_angles

    def fk(self, angles):
        angles = np.asarray(angles)

        def func(pose):
            p, o = pose[:3], pose[3:]
            pose = Pose(position=p, orientation=o)
            diff = np.linalg.norm(self.ik(pose) - angles)
            return diff
        x0 = [0, 0, 0.275, 0, 0, 0, 1]
        return fmin_bfgs(func, x0)


class StewartNode(object):
    """ROS node that listens to pose requests on "/stewart_platform/pose",
    computes the inverse kinematics of the given Stewart platform, and
    sends servo commands to move to that pose.

    """
    def __init__(self, platform, log_level=rospy.INFO):
        self.platform = platform
        # Quaternion, xyzw
        self.desired_orientation = np.array([0, 0, 0, 1], dtype=np.float64)
        self.desired_position = np.array([0, 0, 0.275], dtype=np.float64)
        self.nh = rospy.init_node("stewart", log_level=log_level)

        rospy.logdebug("Setting up servo publishers...")
        self.servo_publishers = [rospy.Publisher("/servo_{}/command".format(i),
                                                 Float64,
                                                 queue_size=10)
                                 for i in range(1, 7)]

        rospy.logdebug("Setting up pose listener...")
        self.pose_listener = rospy.Subscriber("/stewart_platform/pose",
                                              Pose,
                                              self.listen_pose)
        rospy.logdebug("Setting up AHRS listener...")
        self.ahrs_listener = rospy.Subscriber("/stewart_platform/imu",
                                              Imu,
                                              self.listen_imu)

        self.rp_pid = PID(1.0, 0.005, 0.0, 0.3)
        self.NN, self.t0 = 0, rospy.get_time()

    def listen_pose(self, pose):
        self.desired_orientation[:] = [pose.orientation.x,
                                       pose.orientation.y,
                                       pose.orientation.z,
                                       pose.orientation.w]
        self.desired_position[:] = [pose.position.x,
                                    pose.position.y,
                                    pose.position.z]

        rospy.logdebug(
            "Desired orientation (deg): {}".format(
                np.rad2deg(transformations.euler_from_quaternion(
                    self.desired_orientation))))
        rospy.logdebug("Desired position: {}".format(
            self.desired_position))

    def listen_imu(self, imu):
        # node can sleep for 16 out of 20 ms without dropping any messages
        # rospy.sleep(16.0/1000.0)
        # self.NN += 1
        # if rospy.get_time() - self.t0 > 1:
        #     rospy.logfatal("Published {} messages in 1 s, {} Hz".format(self.NN, self.NN))
        #     self.NN, self.t0 = 0, rospy.get_time()
        current_orientation = np.array([imu.orientation.x,
                                        imu.orientation.y,
                                        imu.orientation.z,
                                        imu.orientation.w])
        current_angular_speed = np.array([imu.angular_velocity.x,
                                          imu.angular_velocity.y,
                                          0.0])
                                          
        current_rpy = list(transformations.euler_from_quaternion(current_orientation))
        # No care about yaw, but must be close to zero
        current_rpy[-1] = 0
        desired_rpy = list(transformations.euler_from_quaternion(self.desired_orientation))
        desired_rpy[-1] = 0
        corrected = self.rp_pid(desired_rpy, current_rpy, current_angular_speed)
        corrected_orientation = transformations.quaternion_multiply(
            transformations.quaternion_from_euler(*corrected),
            transformations.quaternion_from_euler(*current_rpy))

        setpoint_pose = Pose(self.desired_position, corrected_orientation)

        servo_angles = self.platform.ik(setpoint_pose)
        rospy.logdebug(
            "Servo angles (deg): {}".format(np.rad2deg(servo_angles)))
        self.publish_servo_angles(servo_angles)

    def listen_imu_quaternion(self, imu):
        """Doesn't work b/c desired yaw is too far from possible yaw"""
        current_orientation = np.array([imu.orientation.x,
                                        imu.orientation.y,
                                        imu.orientation.z,
                                        imu.orientation.w])
        # current_angular_speed = np.array([imu.angular_velocity.x,
        #                                   imu.angular_velocity.y,
        #                                   imu.angular_velocity.z])
        diff_orientation = transformations.quaternion_multiply(
            self.desired_orientation,
            # Unit quaternion => inverse = conjugate / norm = congugate
            transformations.quaternion_conjugate(current_orientation))

        assert np.allclose(
            transformations.quaternion_multiply(diff_orientation,
                                                current_orientation),
            self.desired_orientation)

        # diff_r, diff_p, diff_y = transformations.euler_from_quaternion(
        #     diff_orientation)
        # rospy.loginfo("Orientation error (quaternion): {}".format(diff_orientation))
        # rospy.loginfo(
        #     "Orientation error (deg): {}".format(
        #         np.rad2deg([diff_r, diff_p, diff_y]))
        # )
        # out = self.pitch_controller(diff_p)

        corrected_orientation = transformations.quaternion_multiply(
            quaternion_power(diff_orientation, 1.5),
            self.desired_orientation)
        rospy.loginfo(
            "Desired orientation (deg): {}".format(
                np.rad2deg(transformations.euler_from_quaternion(
                    self.desired_orientation))))
        rospy.loginfo(
            "Corrected orientation (deg): {}".format(
                np.rad2deg(transformations.euler_from_quaternion(
                    corrected_orientation))))

        rospy.loginfo("Desired position: {}".format(
            self.desired_position))

        setpoint_pose = Pose(self.desired_position, corrected_orientation)

        servo_angles = self.platform.ik(setpoint_pose)
        rospy.logdebug(
            "Servo angles (deg): {}".format(np.rad2deg(servo_angles)))
        self.publish_servo_angles(servo_angles)

    def publish_servo_angles(self, angles):
        for angle, pub in zip(angles, self.servo_publishers):
            pub.publish(angle)

    def test(self):
        rospy.loginfo("\033[1;33mTesting range of motion\033[0m")
        rospy.loginfo("Moving to neutral height...")
        self.publish_servo_angles([0] * 6)
        rospy.sleep(2)
        rospy.loginfo("Moving to lowest height...")
        self.publish_servo_angles([-1.4] * 6)
        rospy.sleep(2)
        rospy.loginfo("Moving to highest height...")
        self.publish_servo_angles([1.5708] * 6)
        rospy.sleep(2)
        rospy.loginfo("Moving to neutral height...")
        self.publish_servo_angles([0] * 6)


def quaternion_exp(q):
    xyz, w = q[:3], q[3]
    nrm = np.linalg.norm(xyz)
    new_w = math.exp(w) * math.cos(nrm)
    new_xyz = np.array(xyz) * (math.sin(nrm) / nrm * math.exp(w))
    return list(new_xyz) + [new_w]


def quaternion_power(q, alpha):
    xyz, w = q[:3], q[3]
    q_nrm = np.linalg.norm(q)
    theta = math.acos(w / q_nrm)
    new_w = q_nrm**alpha * math.cos(alpha * theta)
    new_xyz = q_nrm**alpha * math.sin(alpha * theta) * np.array(xyz) / (q_nrm * math.sin(theta))
    return list(new_xyz) + [new_w]


if __name__ == '__main__':
    import sys

    # Let's instantiate our Stewart platform
    base_coordinates = np.array(
        [[105.25, 105.25, 11.55, -116.80, -116.80, 11.50],
         [74.1, -74.1, -128.2, -54.1, 54.1, 128.2],
         [0, 0, 0, 0, 0, 0]]) / 1000
    top_coordinates = np.array(
        [[104.9, 104.9, -19.46, -85.45, -85.45, -19.46],
         [38.1, -28.1, -109.9, -71.8, 71.8, 109.9],
         [0, 0, 0, 0, 0, 0]]) / 1000
    servo_arm_length = 40 / 1000
    leg_length = 275 / 1000
    betas = np.pi * np.array([1/2, 1/2, -1/6, -1/6, 1/6, 1/6])
    stewart = Stewart(base_coordinates, top_coordinates,
                      servo_arm_length, leg_length, betas)
    # And the ROS node
    node = StewartNode(stewart, log_level=rospy.INFO)

    # TODO: use ROS parameters
    args = sys.argv
    args = [arg for arg in args if not arg.startswith('__')]
    if len(args) == 1:
        # No args, just keep node alive
        rospy.spin()
    elif args[1] == 'test':
        # Go up and down
        node.test()
