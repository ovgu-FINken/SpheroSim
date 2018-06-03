#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
from __future__ import division

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from tracking.msg import TaggedPose2D
import tf.transformations
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actionlib

# gibt yaw-winkel (um die z-Achse) eines Vektors zurück
def yaw_angle(p):
    return np.arctan2(p[1], p[0])


class SpheroGoalPub:
    def __init__(self, pub_topic="/move_base_simple/goal", pose_topic="/sphero/red_filtered"):
        # Bereite Schnittstelle für move_base vor (ActionLib)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Abonniere Nachrichten des Trackers
        rospy.Subscriber(pose_topic, TaggedPose2D, self.receive_pose)
        # Lies Liste der Punkte mit Richtungen ein
        self.points = np.genfromtxt(os.path.dirname(os.path.realpath(__file__))+'/points.csv', delimiter=' ')        # x,y,dx,dy
        self.index = 0
        # Erstes Ziel abschicken
        self.send_goal()
        # Warte auf ROS Ereignisse
        rospy.spin()

    # Ziel zusammensetzen und abschicken
    def send_goal(self, index=0):
        goal = MoveBaseGoal()
        goal.target_pose.header = Header()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.seq = 0
        goal.target_pose.pose = Pose()
        goal.target_pose.pose.position = Point()
        goal.target_pose.pose.position.x = self.points[index][0]
        goal.target_pose.pose.position.y = self.points[index][1]
        direction = [self.points[index][2], self.points[index][3]]
        rotation = tf.transformations.quaternion_about_axis(yaw_angle(direction), [0, 0, 1])
        goal.target_pose.pose.orientation = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])
        self.client.send_goal(goal)
        rospy.logwarn(str(goal))

    # Aktuelle Position empfangen und ggfs. neues Ziel senden
    def receive_pose(self, msg=TaggedPose2D):
        current_pos = np.array([msg.x, msg.y])
        current_goal = np.array([self.points[self.index][0], self.points[self.index][1]])
        if np.linalg.norm(current_goal - current_pos) <= 0.4:
            self.index = (self.index + 1) % len(self.points)
            self.send_goal(self.index)


if __name__ == '__main__':
    rospy.init_node('sphero_goal_pub')
    sphero_goal_pub = SpheroGoalPub()
