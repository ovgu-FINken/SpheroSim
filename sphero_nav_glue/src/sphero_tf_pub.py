#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
from __future__ import division

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from tracking.msg import TaggedPose2D
import tf.transformations


# berechnet yaw-winkel, d.h. um die Z-Achse eines Vektors
def yaw_angle(p):
    return np.arctan2(p[1], p[0])

# liest den Translationsanteil aus einer affinen Matrix aus (3D: 4x4 Matrix)
def vector_from_translation_matrix(a):
    return np.array([a[0, 3], a[1, 3], a[2, 3]])


class SpheroTFPub:
    def pose_sanity_check(self, pose_a, pose_b):
        # sphero can only move at a maximum speed of 2 m/s, which is 2e-9 m/ns
        # maximum_length = abs(pose_a.header.stamp.secs + (1e-9) * pose_a.header.stamp.nsecs - pose_a.header.stamp.secs - (1e-9) * pose_b.header.stamp.nsecs) * 2
        minimum_length = 0.074 # kleinste Distanz, ab der wir eine neue Richtung berechnen in m
        a = np.array([pose_a.x, pose_a.y])
        b = np.array([pose_b.x, pose_b.y])
        length = abs(np.linalg.norm(a - b))
        return length >= minimum_length

    def calculate_transforms(self):
        # A map -> base_link
        a_offset = self.current_position  # -(0,0,0)
        a_angle = yaw_angle(self.current_direction)

        self.a_rotation = tf.transformations.quaternion_about_axis(a_angle, (0, 0, 1))
        self.a_transform = a_offset

    def send_transforms(self):
        # publish transforms
        time = rospy.Time(self.current_pose.header.stamp.secs, self.current_pose.header.stamp.nsecs)
        rospy.loginfo("sending transform at " + str(time))
        self.br.sendTransform(self.a_transform, self.a_rotation, time, "base_link", "map")
        self.br.sendTransform(self.b_transform, self.b_rotation, time, "odom", "map")
        rospy.loginfo("A tr: " + str(self.a_transform) + " rot: " + str(self.a_rotation))
        rospy.loginfo("B tr: " + str(self.b_transform) + " rot: " + str(self.b_rotation))

    def receive_first(self, pose):
        # Erster Zustand, in dem die empfangene Pose einfach abgelegt wird
        self.current_pose = pose
        rospy.loginfo("TF Pub reached second state")
        self.state = "no_previous"

    def receive_second(self, pose):
        # Zweiter Zustand, in dem die Anfangspose ausgerechnet wird
        self.go_forward()
        above_minimum = self.pose_sanity_check(self.current_pose, pose)
        if above_minimum: # genug Abstand für Richtungsberechnung
            self.previous_pose = self.current_pose
            self.current_pose = pose
            self.initial_position = np.array([self.previous_pose.x, self.previous_pose.y, 0.037])
            self.initial_direction = self.calc_direction(self.previous_pose, self.current_pose)

            # B map -> odom
            b_angle = yaw_angle(self.initial_direction)
            b_offset = self.initial_position  # -(0,0,0)

            self.b_rotation = tf.transformations.quaternion_about_axis(b_angle, (0, 0, 1))
            self.b_transform = b_offset

            self.current_position = self.initial_position
            self.current_position = np.array([self.current_pose.x, self.current_pose.y, 0.037])
            rospy.loginfo("TF Pub reached third state")

            self.calculate_transforms()
            self.send_transforms()
            self.stop_moving()

            self.state = "operative"

    def receive_further(self, pose):
        # letzter Zustand, in dem die jeweils aktuelle Pose ausgerechnet wird
        above_minimum = self.pose_sanity_check(self.previous_pose, pose)
        if above_minimum:  # update direction
            rospy.loginfo("above_minimum")
            self.current_direction = self.calc_direction(self.previous_pose, pose)
            self.previous_pose = self.current_pose
            self.current_pose = pose
        else:  # only update position
            rospy.loginfo("below_minimum")
            self.current_pose = pose
        self.current_position = np.array([self.current_pose.x, self.current_pose.y, 0.037])
        self.calculate_transforms()
        self.send_transforms()

    def calc_direction(self, pose_a, pose_b):
        # berechnet die Richtung zwischen zwei Punkten
        vec_a = np.array([pose_a.x, pose_a.y, 0.037])
        vec_b = np.array([pose_b.x, pose_b.y, 0.037])
        # rospy.loginfo("A:" + str(vec_a) + " B: " + str(vec_b))
        direction = (vec_b - vec_a) / np.linalg.norm(vec_b - vec_a)
        # rospy.loginfo("dir: " + str(direction) + "timestamp nsecs: " + str(self.current_pose.header.stamp.nsecs))
        return direction

    def receive_pose(self, pose):
        self.logpos.write(str(pose.header.stamp.secs) + ";" + str(pose.header.stamp.nsecs) + ";"
                          + str(pose.x) + ";" + str(pose.y) + "\n")
        self.states[self.state](pose) # ruft eine Funktion aus dem dictionary self.state auf

    def go_forward(self):
        rospy.loginfo("TF Pub sent go_forward command")
        twist = Twist()
        twist.linear.x = 35
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.vel_pub.publish(twist)

    def stop_moving(self):
        rospy.loginfo("TF Pub sent stop_moving command")
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.vel_pub.publish(twist)

    def receive_cmd_vel(self, msg):
        # Empfängt cmd_vel Nachrichten und loggt diese (Datei kann im .ros Ordner gefunden werden)
        time = rospy.Time.now()
        self.logvel.write(str(time.secs) + ";" + str(time.nsecs) + ";"
                          + str(msg.linear.x) + ";" + str(msg.linear.y) + ";" + str(msg.linear.z) + ";"
                          + str(msg.angular.x) + ";" + str(msg.angular.y) + ";" + str(msg.angular.z) + "\n")

    def __init__(self, pose_topic="/sphero/red_filtered", vel_topic="/sphero/cmd_vel"):

        # interne Variablen, Publisher, Subscriber
        self.color = pose_topic
        self.previous_pose = None
        self.current_pose = None
        self.initial_direction = np.array([42.0, 1337.0, 1234.0])
        self.initial_position = np.array([42.0, 1337.0, 1234.0])
        self.current_direction = np.array([42.0, 1337.0, 1234.0])
        self.current_position = np.array([42.0, 1337.0, 1234.0])
        rospy.Subscriber(pose_topic, TaggedPose2D, self.receive_pose)
        self.vel_sub = rospy.Subscriber(vel_topic, Twist, self.receive_cmd_vel)
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.a_transform = np.array([0, 0, 0])
        self.a_rotation = np.array([1, 0, 0])
        self.b_transform = None
        self.b_rotation = None
        self.c_transform = None
        self.c_rotation = None
        self.b_matrix = None

        # dict mit aufzurufenden Funktionen der state machine
        self.states = {"no_current": self.receive_first,
                       "no_previous": self.receive_second,
                       "operative": self.receive_further}

        # initiiert state machine
        self.state = "no_current"
        rospy.loginfo("TF Pub reached first state")

        # öffnet logdateien (in .ros)
        self.logpos = open("logpos.csv", "w")
        self.logpos.write("secs;nsecs;x(m);y(m)\n")
        self.logvel = open("logvel.csv", "w")
        self.logvel.write("secs;nsecs;linear.x;linear.y;linear.z;angular.x;angular.y;angular.z\n")
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('sphero_tf_pub')
    rospy.logwarn("waiting 35s...")
    rospy.sleep(35)  # wait for sphero to connect
    sphero_tf_pub_red = SpheroTFPub()
