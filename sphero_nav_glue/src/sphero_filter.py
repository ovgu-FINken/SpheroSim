#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
import numpy as np
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from tracking.msg import TaggedPose2D
import std_msgs.msg
import tf.transformations

class SpheroFilter:
	def filter(self):
		'''plx=positions_x_from_last_10_timestamps(die Länge der List kann größer oder kleiner 10 sein, je nach dem ob man mehr Sicherheit oder Effizienz haben will);necs=time_of_positions_from_last_10_timestamps_in_nano_second(Nach der Beobachtung kann man feststellen, dass die Aufnahmefrequenz 10HZ ist. Die Datei enthalten Informationen von der Bewegung des Spheros innerhalb einer Sekunde)'''

		l = len(self.plx)

		self.ct = []
		# ct=corrected timestamp
		for i in range(l):
			self.ct.append(self.nsecs[i] + self.secs[i] * (10 ** 9))
		self.cd = [self.plx, self.ply, self.ct]
		# cd=cleaned_data

		for i in range(1, l):
			if self.plx[i] <= 0.03 or self.plx[i] >= 3.97 or self.ply[i] <= 0.03 or self.ply[i] >= 2.97:
			# Positionen ausser Arena wegschmeissen
				self.cd[0][i] = self.cd[0][i - 1]
				self.cd[1][i] = self.cd[1][i - 1]
			# Die Geschwindigkeit soll kleiner 2m/s sein
			if ((self.plx[i] - self.plx[i - 1]) ** 2 + (self.ply[i] - self.ply[i - 1]) ** 2) ** 0.5 / (self.ct[i] - self.ct[i - 1] + 1 * 10 ** -6) > 2:
				self.cd[0][i] = self.cd[0][i - 1]
				self.cd[1][i] = self.cd[1][i - 1]
		# multipl punkte in einem zeitpunkt filtern
			if self.ct[i]==self.ct[i-1] and i>=2:
				if max(((self.cd[0][i]-self.cd[0][i-2])**2+(self.cd[1][i]-self.cd[1][i-2])**2),((self.cd[0][i-1]-self.cd[0][i-2])**2+(self.cd[1][i-1]-self.cd[1][i-2])**2))==((self.cd[0][i-1]-self.cd[0][i-2])**2+(self.cd[1][i-1]-self.cd[1][i-2])**2):
					self.cd[0][i] = self.cd[0][i-1]
					self.cd[1][i] = self.cd[1][i-1]
				else:
					self.cd[0][i-1] = self.cd[0][i]
				self.cd[1][i-1] = self.cd[1][i]
		position=[self.cd[0][-1], self.cd[1][-1]]
		return position

	''' # Oscillation
		# Die Zeilen wurden am Ende ausgelassen, weil die in niedriger Geschwindigkeit nicht viel Sinn macht.
				def sa(lst):
					result = 0
					result = (lst[-1] - lst[0]) / len(lst)
					return result

				# sa=Summe der Aenderung einer List
				def sab(lst):
					result = 0
					for i in range(1, len(lst)):
						result += abs(lst[i - 1] - lst[i])
					return result
					# sab=Summe der (Aenderung in betrag) einer list
			if sab(cd[0]) / (sa(cd[0]) + 1 * 10 ** -6) >= self.odd or sab(cd[1]) / (sa(cd[1]) + 1 * 10 ** -6) >= self.odd:
				position = [cd[0][0], cd[1][0]]
			else:
				position = [cd[0][-1], cd[1][-1]]'''
		

	def receive_pose(self, msg):
		#rospy.logwarn("got message")
		self.plx.append(msg.x)
		self.ply.append(msg.y)
		self.nsecs.append(msg.header.stamp.nsecs)
		self.secs.append(msg.header.stamp.secs)
		if len(self.plx) > self.count:
			self.plx.pop(0)
			self.ply.pop(0)
			self.nsecs.pop(0)
			self.secs.pop(0)
			self.position = self.filter()
		else:
			self.position = [msg.x, msg.y]
		#rospy.logwarn("trying to publish")
		self.publish()

	def publish(self):
		msg = TaggedPose2D()
		msg.x = self.position[0]
		msg.y = self.position[1]
		msg.quality = 10.0
		msg.id = str(0)
		msg.theta = 0.0
		msg.header=std_msgs.msg.Header()
		msg.header.stamp = rospy.Time.now()
		self.pub.publish(msg)
	# get pose from "/sphero/red" , and then send the filtered data to "/sphero/red_filtered". Data type:TaggedPose2D
	def __init__(self, pose_topic="/sphero/red", pub_topic="/sphero/red_filtered", count=10):
		rospy.Subscriber(pose_topic, TaggedPose2D, self.receive_pose)
		self.pub = rospy.Publisher(pub_topic, TaggedPose2D, queue_size=1)
		self.odd = 1.2
		self.count = count
		self.plx = []
		self.ply = []
		self.nsecs = []
		self.secs = []
		self.position = [0, 0]



if __name__ == '__main__':
    rospy.init_node('sphero_filter')
    rospy.logwarn("waiting 40s...")
    rospy.sleep(40)  # wait for sphero to connect
    sphero_filter_red = SpheroFilter()
    rospy.spin()
