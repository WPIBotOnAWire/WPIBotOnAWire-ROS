#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt16

KAPPA = 0.1

class distance_estimator:
    def __init__(self, mb_topic, tf_topic, pub_topic):
        self.distance_estimate = 1000.0
        self.distance_sub_mb = rospy.Subscriber(mb_topic, UInt16, self.mb_callback)
        self.distance_sub_tf = rospy.Subscriber(tf_topic, UInt16, self.tf_callback)
        self.distance_pub = rospy.Publisher(pub_topic, UInt16, queue_size=10)

    def mb_callback(self, mb_msg):
        distance = mb_msg.data - 10 # subtract 10 cm for offset b/w the two sensors
        if distance < self.distance_estimate: # if MB reads closer, take that
            self.distance_estimate = distance
        if distance > self.distance_estimate: # if farther, 'fuse' by weighting
            self.distance_estimate = KAPPA * distance + (1 - KAPPA) * self.distance_estimate
        self.distance_pub.publish(round(self.distance_estimate))

    def tf_callback(self, tf_msg):
        distance = tf_msg.data
        if distance < self.distance_estimate: # if TF reads closer, take that
            self.distance_estimate = distance
        self.distance_pub.publish(round(self.distance_estimate))

        
def main():
    rospy.init_node('distance_estimator')

    fore_distance_estimator = distance_estimator('/rangefinder/fore/MB', '/rangefinder/fore/TF', '/distance/fore')
    aft_distance_estimator = distance_estimator('/rangefinder/aft/MB', '/rangefinder/aft/TF', '/distance/aft')

    rospy.spin()

if __name__ == '__main__':
    main()
