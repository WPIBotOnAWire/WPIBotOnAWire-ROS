#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import BatteryState
import threading


# Publishers
flashLightPub = rospy.Publisher('/deterrents/led', Bool, queue_size=10)
soundPub = rospy.Publisher('/play_sound', Int32, queue_size=10)


def main():
    while(1):
        rospy.init_node('BOAW_SM')
        soundPub.publish(4001)
        rospy.loginfo("done")
        # flashLightPub.publish(True)
        # rospy.sleep(0.26)
        # flashLightPub.publish(False)
        # rospy.sleep(0.26)
    rospy.spin()


if __name__ == '__main__':
    main()

