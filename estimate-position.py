#!/usr/bin/env python

import rospy

from std_msgs.msg import Int16

def EncoderCallback(msg):
    
    rospy.loginfo("Speed -> %i cm/s", speed)

def main():
    rospy.init_node('est-pos')
    rospy.Subscriber('robot_speed', Int16, EncoderCallback)

    rospy.spin()

if __name__ == '__main__':
    main()
    
