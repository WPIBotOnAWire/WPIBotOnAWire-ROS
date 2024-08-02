#!/usr/bin/env python

import rospy

from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from std_msgs.msg import String

def Front_Distance_CallBack(msg):
    pass

def Command_CallBack(msg):
    command = msg.data
    rospy.loginfo("Command: ")
    rospy.loginfo(command)


def main():
    rospy.init_node('wire_bot')

    rospy.Subscriber("/distance/fore", UInt16, Front_Distance_CallBack)
    rospy.Subscriber("status", String, Command_CallBack)

    rospy.spin()

if __name__ == '__main__':
    main()
