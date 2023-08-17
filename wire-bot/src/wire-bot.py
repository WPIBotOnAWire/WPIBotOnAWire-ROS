#!/usr/bin/env python

from enum import Enum

import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
#from sensor_msgs.msg import BatteryState

class states(Enum):
    ROBOT_IDLE = 0, 
    ROBOT_PATROL_FWD = 1
    ROBOT_APPROACH_FWD = 2
    ROBOT_DETERRENT_FWD = 3

state = states.ROBOT_IDLE

pubTargetSpeed = rospy.Publisher('target_speed', Float32, queue_size=10)

def RangefinderFrontMB_CallBack(msg):
    global state
    distance = msg.data
    rospy.loginfo("Front(MB): %i cm", distance)
    
    speed = 0

    if state == states.ROBOT_PATROL_FWD:
        if distance > 200:
            speed = 100
        else:
            state = states.ROBOT_APPROACH_FWD
            
    elif state == states.ROBOT_APPROACH_FWD:
        if distance > 200:
            state = states.ROBOT_PATROL_FWD
            speed = 100
        elif distance > 50:
            speed = (distance - 25)
        else:
            state = states.ROBOT_DETERRENT_FWD
            speed = 0

    elif state == states.ROBOT_DETERRENT_FWD:
        if distance > 200:
            state = states.ROBOT_PATROL_FWD
            speed = 100

    else:
        speed = 0

    pubTargetSpeed.publish(speed)
    rospy.loginfo("Speed -> %i cm/s", speed)

def main():
    rospy.init_node('wire_bot')
    rospy.Subscriber("/rangefinder/front/MB", UInt16, RangefinderFrontMB_CallBack)

    rospy.spin()

if __name__ == '__main__':
    main()
    
