import rospy
from std_msgs.msg import Float64
import numpy as np


class PID(object):
    def __init__(self, p, i, d, anti_windup):
        self.pub_p = rospy.Publisher("/stewart_platform/pid/p",
                                     Float64,
                                     queue_size=1)
        self.pub_i = rospy.Publisher("/stewart_platform/pid/i",
                                     Float64,
                                     queue_size=1)
        self.pub_d = rospy.Publisher("/stewart_platform/pid/d",
                                     Float64,
                                     queue_size=1)
        self.p, self.i, self.d = p, i, d
        self.anti_windup = anti_windup / i
        self.prev_err = 0.0
        self.i_err = 0.0

    def __call__(self, setpoint, state):
        err = np.array(setpoint) - np.array(state)
        self.i_err += err
        self.i_err = np.clip(self.i_err, -self.anti_windup, self.anti_windup)
        P = self.p * err
        I = self.i * self.i_err
        D = self.d * (err - self.prev_err)
        self.pub_p.publish(P[0]);self.pub_i.publish(I[0]);self.pub_d.publish(D[0]);
        correction = (P + I + D)
        self.prev_err = err
        return correction
