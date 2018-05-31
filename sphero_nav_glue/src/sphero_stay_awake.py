#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
import rospy

from std_msgs.msg import ColorRGBA

colorBindings = {
    "red": (1, 0, 0),
    "green": (0, 1, 0),
    "blue": (0, 0, 1),
    "turquoise": (0, 1, 1),
    "violet": (1, 0, 1),
    "yellow": (1, 1, 0),
}


class StayAwake:
    # Timeout zurücksetzen
    def receive(self, msg):
        self.secs = rospy.get_time()
        self.msg = msg

    # Farbnachricht schicken
    def poke(self, msg):
        if rospy.get_time() - self.secs > 20:
            self.pub.publish(self.msg)

    
    def __init__(self):
        self.topic_name = rospy.get_param("~topic_name", "sphero/set_color")
        self.color = rospy.get_param("~color", "red")
        self.pub = rospy.Publisher(self.topic_name, ColorRGBA, queue_size=1)
        self.sub = rospy.Subscriber(self.topic_name, ColorRGBA, self.receive)
        self.secs = rospy.get_time()
        self.msg = ColorRGBA()
        self.msg.r = colorBindings[self.color][0]
        self.msg.g = colorBindings[self.color][1]
        self.msg.b = colorBindings[self.color][2]
        timer = rospy.Timer(rospy.Duration(5), self.poke)
        rospy.spin()


if __name__ == '__main__':    
    try:
	rospy.init_node("sphero_stay_awake")
    	rospy.logwarn("waiting 30s...")
	rospy.sleep(30) #wait for sphero to connect
        s = StayAwake()
    except rospy.ROSInterruptException:
        pass
