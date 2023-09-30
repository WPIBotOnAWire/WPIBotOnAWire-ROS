#!/usr/bin/env python

from enum import Enum

import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
#from sensor_msgs.msg import BatteryState

class states(Enum):
    ROBOT_IDLE = 0, 
    ROBOT_PATROL_FWD = 1
    ROBOT_APPROACH_FWD = 2
    ROBOT_DETERRENT_FWD = 3

pubTargetSpeed = rospy.Publisher('target_speed', Float32, queue_size=10)
pubFlashLight = rospy.Publisher('led_cmd', UInt16, queue_size=10)

# rosrun rosserial_python serial_node.py
# rostopic pub -1 led_cmd std_msgs/UInt16 '20' 
# rostopic pub -1 /rangefinder/front/MB std_msgs/UInt16 '210'
def Front_Distance_CallBack(msg):

    distance = msg.data
    rospy.loginfo("Front(MB): %i cm", distance)
    
    speed = 0

    global state

    if state == states.ROBOT_PATROL_FWD:

        if distance > 200: speed = 100

        else:

            state = states.ROBOT_APPROACH_FWD
            rospy.loginfo("State: " + state.name)
            
    elif state == states.ROBOT_APPROACH_FWD:

        if distance > 200:

            speed = 100

            state = states.ROBOT_PATROL_FWD
            rospy.loginfo("State: " + state.name)

        elif distance > 50: speed = (distance - 25)

        else:

            speed = 0

            state = states.ROBOT_DETERRENT_FWD
            rospy.loginfo("State: " + state.name)

            light_flash = 30
            pubFlashLight.publish(light_flash)

    elif state == states.ROBOT_DETERRENT_FWD:

        if distance > 200:

            speed = 100

            state = states.ROBOT_PATROL_FWD
            rospy.loginfo("State: " + state.name)

    else: speed = 0

    pubTargetSpeed.publish(speed)
    rospy.loginfo("Speed -> %i cm/s", speed)
    rospy.loginfo("State: " + state.name)

# use command below in the terminal to activate/deactivate
# rostopic pub -1 activation std_msgs/Bool 'true'
def Activation_CallBack(msg):

    command = msg.data
    rospy.loginfo("Activation")
    rospy.loginfo(command)

    global state

    if command: 
        state = states.ROBOT_PATROL_FWD
        rospy.loginfo("State: " + state.name)
    else: 
        state = states.ROBOT_IDLE
        rospy.loginfo("State: " + state.name)

def main():
    rospy.init_node('wire_bot')

    global state
    state = states.ROBOT_IDLE
    rospy.loginfo("State: " + state.name)

    rospy.Subscriber("/distance/front", UInt16, Front_Distance_CallBack)
    rospy.Subscriber("activation", Bool, Activation_CallBack)

    rospy.spin()

if __name__ == '__main__':
    main()