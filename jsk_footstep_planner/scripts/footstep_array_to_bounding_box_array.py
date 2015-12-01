#!/usr/bin/env python

import rospy
from jsk_footstep_msgs.msg import Footstep, FootstepArray
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

def callback(msg):
    global box_array
    box_array = BoundingBoxArray()
    box_array.header = msg.header
    for footstep in msg.footsteps:
        box = BoundingBox()
        box.header = msg.header
        box.pose = footstep.pose
        box.dimensions = footstep.dimensions
        box.pose.position.z += (z_max + z_min) / 2.0
        box.dimensions.z = z_max - z_min
        box_array.boxes.append(box)
    pub.publish(box_array)

if __name__ == "__main__":
    rospy.init_node("footstep_array_to_bounding_box")
    z_max = rospy.get_param('~z_max',0.0005)
    z_min = rospy.get_param('~z_min',-0.0005)
    publish_once = rospy.get_param('~publish_once', True)
    publish_rate = rospy.get_param('~publish_rate', 30)
    pub = rospy.Publisher("~output", BoundingBoxArray)
    sub = rospy.Subscriber("~input", FootstepArray, callback)
    rate = rospy.Rate(publish_rate)
    if not publish_once :
        while not rospy.is_shutdown():
            try :
                pub.publish(box_array)
            except NameError :
                pass
            finally :
                rate.sleep()
    else :
        rospy.spin()
