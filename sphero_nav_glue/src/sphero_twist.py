#!/usr/bin/env python
from __future__ import division

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from tracking.msg import TaggedPose2D
import tf.transformations

def yaw_rot_matrix(a):
    return np.array(((math.cos(a), -math.sin(a), 0.0, 0.0),
                     (math.sin(a), math.cos(a), 0.0, 0.0),
                     (0.0, 0.0, 1.0, 0.0),
                     (0.0, 0.0, 0.0, 1.0)), dtype=np.float64)


class SpheroTwist:
    def receive(self, msg):
        x = msg.linear.x * self.linear_ratio
        if x < 20 and abs(msg.angular.z) > 0:
            rospy.logwarn("in_place_rotation!!")
	    x = 20
        current_time = rospy.Time.now()
        z = (self.last_rot+self.get_passed_seconds(current_time)*self.last_rot_speed)%(2*np.pi)
        self.last_rot=z
        self.last_tsp=current_time
        self.last_rot_speed = msg.angular.z
        output = Twist()
        linear = np.matmul(yaw_rot_matrix(z), np.array((1, 0, 0, 1))) * x
        output.linear.x = linear[0]
        output.linear.y = linear[1]
        output.linear.z = linear[2]
        self.vel_pub.publish(output)

    def __init__(self):
        self.vel_pub = rospy.Publisher("/sphero/cmd_vel", Twist, queue_size=1)
        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.receive)
        # self.angular_ratio = 1 / 0.784 * 180 / math.pi
        self.linear_ratio = 255.0 / 2.0
        self.last_rot = 0
        self.last_rot_speed = 0
        self.last_tsp = rospy.Time.now()
        rospy.spin()

    def get_passed_seconds(self,stamp):
        return (stamp.secs+stamp.nsecs*1e-9)-(self.last_tsp.secs+self.last_tsp.nsecs*1e-9)


if __name__ == '__main__':
    try:
        rospy.init_node("sphero_twist")
        s = SpheroTwist()
    except rospy.ROSInterruptException:
        pass

        # 1 m/sec -> 255/2 unit/sec
