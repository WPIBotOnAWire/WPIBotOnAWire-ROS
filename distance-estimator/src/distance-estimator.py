#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt16

pubDistance = rospy.Publisher('distance', UInt16, queue_size=10)

def RangefinderFrontMB_CallBack(msg):

    distance = msg.data
    rospy.loginfo("Front(MB): %i cm", distance)
    pubDistance.publish(distance)


def main():
    rospy.init_node('distance_estimator')

    rospy.Subscriber("/rangefinder/front/MB", UInt16, RangefinderFrontMB_CallBack)

    rospy.spin()

if __name__ == '__main__':
    main()
    