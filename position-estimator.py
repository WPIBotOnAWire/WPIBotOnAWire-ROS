#!/usr/bin/env python

import rospy

from std_msgs.msg import Int16

class PositionEstimator:
    def __init__(self):
        self.x = 0
        self.s = 0

    def handleEncoder(self, delta):
        self.s += delta
        rospy.loginfo("S = %f", self.s)

estimator = PositionEstimator()

def EncoderCallback(msg):
    delta = msg.data
    estimator.handleEncoder(delta)
    rospy.loginfo("Delta -> %i meter", delta)

def main():
    rospy.init_node('pos-est')
    rospy.Subscriber('/encoder/meters_per_interval', Int16, EncoderCallback)

    rospy.spin()

if __name__ == '__main__':
    main()
    
