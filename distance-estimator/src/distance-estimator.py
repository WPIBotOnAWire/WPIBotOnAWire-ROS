#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt16

pubFrontDistance = rospy.Publisher('/distance/fore', UInt16, queue_size=10)
pubAftDistance = rospy.Publisher('/distance/aft', UInt16, queue_size=10)

newMBForeReading, newMBAftReading = False
newTFForeReading, newTFAftReading = False

frontMBDistance = 0
aftMBDistance = 0
frontTFDistance = 0
aftTFDistance = 0

# rosrun distance-estimator distance-estimator.py
# tf faster than mb

def estimate_distance(distanceMB, distanceTF, isFore):

    estimatedDistance = 0
    if distanceMB > 200: estimatedDistance = distanceMB
    elif distanceMB < 100: estimatedDistance = distanceTF
    else: estimatedDistance = (distanceMB + distanceTF) / 2
    
    #Set both the flags that there is new data to false, resetting the data timing
    if isFore:
        newMBForeReading = False
        newTFForeReading = False
    else:
        newMBAftReading = False
        newTFAftReading = False

    return int(estimatedDistance)

def RangefinderFrontMB_CallBack(msg):

    newMBForeReading = True
    frontMBDistance = msg.data
        
    rospy.loginfo("Front(MB): %i cm", frontMBDistance)

    if newTFForeReading:
        estDist = estimate_distance(frontMBDistance, frontTFDistance, True)
        pubFrontDistance.publish(estDist)



def RangefinderFrontTF_CallBack(msg):

    newTFForeReading = True
    frontTFDistance = msg.data
    rospy.loginfo("Fore (TF): %i(cm)", frontTFDistance)
    if newMBForeReading:

        estDist = estimate_distance(frontMBDistance, frontTFDistance, True)
        pubFrontDistance.publish(estDist)


def RangefinderAftMB_CallBack(msg):

    newMBAftReading = True
    aftMBDistance = msg.data
    rospy.loginfo("Aft(MB): %i cm", aftMBDistance)
    if newTFAftReading:
        estDist = estimate_distance(aftMBDistance, aftTFDistance, False)
        pubAftDistance.publish(estDist)

def RangefinderAftTF_CallBack(msg):

    newTFAftReading
    aftTFDistance = msg.data
    rospy.loginfo("Aft(TF): %i cm", aftTFDistance)
    if newMBAftReading:
        estDist = estimate_distance(aftMBDistance, aftTFDistance, False)
        pubAftDistance.publish(estDist)

def main():
    rospy.init_node('distance_estimator')

    rospy.Subscriber("/rangefinder/fore/MB", UInt16, RangefinderFrontMB_CallBack)
    rospy.Subscriber("/rangefinder/aft/MB", UInt16, RangefinderAftMB_CallBack)
    rospy.Subscriber("/rangefinder/fore/------", UInt16, RangefinderFrontTF_CallBack)
    rospy.Subscriber("/rangefinder/aft/TF", UInt16, RangefinderAftTF_CallBack)

    rospy.spin()

if __name__ == '__main__':
    main()