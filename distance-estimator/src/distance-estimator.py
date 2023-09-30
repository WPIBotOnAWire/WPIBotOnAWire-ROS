#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt16

pubFrontDistance = rospy.Publisher('/distance/front', UInt16, queue_size=10)
pubAftDistance = rospy.Publisher('/distance/aft', UInt16, queue_size=10)

pubFront = False
pubAft = False
frontMBDistance = 0
aftMBDistance = 0
frontTFDistance = 0
aftTFDistance = 0

# rosrun distance-estimator distance-estimator.py
# tf faster than mb

def estimate_distance(distanceMB, distanceTF):

    if distanceMB > 200: estimatedDistance = distanceMB
    elif distanceMB < 100: estimatedDistance = distanceTF
    else: estimatedDistance = (distanceMB + distanceTF) / 2

    return estimatedDistance

def RangefinderFrontMB_CallBack(msg):

    global frontMBDistance
    global pubFront

    frontMBDistance = msg.data
    rospy.loginfo("Front(MB): %i cm", frontMBDistance)

    pubFront = True

def RangefinderFrontTF_CallBack(msg):

    global frontTFDistance
    global frontMBDistance
    global pubFront

    frontTFDistance = msg.data

    if pubFront:

        pubFront = False

        frontDist = estimate_distance(frontMBDistance, frontTFDistance)

        rospy.loginfo("Front(TF): %i cm", frontTFDistance)
        rospy.loginfo("Front Dist: %i cm", frontDist)
        pubFrontDistance.publish(frontDist)

def RangefinderAftMB_CallBack(msg):

    aftMBDistance = msg.data
    # rospy.loginfo("Aft(MB): %i cm", aftMBDistance)

def RangefinderAftTF_CallBack(msg):

    aftTFDistance = msg.data
    # rospy.loginfo("Aft(TF): %i cm", aftTFDistance)

def main():
    rospy.init_node('distance_estimator')

    rospy.Subscriber("/rangefinder/front/MB", UInt16, RangefinderFrontMB_CallBack)
    rospy.Subscriber("/rangefinder/aft/MB", UInt16, RangefinderAftMB_CallBack)
    rospy.Subscriber("/rangefinder/fore/TF", UInt16, RangefinderFrontTF_CallBack)
    rospy.Subscriber("/rangefinder/aft/TF", UInt16, RangefinderAftTF_CallBack)

    rospy.spin()

if __name__ == '__main__':
    main()