#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import ClusterPointIndices
from

def ClusterIndicesCallback(msg):
  


def main():
    global foot_obs_pub
    cpi_sub = rospy.Subscriber("~/input",ClusterPointIndices, ClusterIndicesCallback)
    foot_obs_pub = rospy.Publisher()
    point_threshold = rospy.get_param('~point_threshold', 10)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("obstacle_perception")
    main()
