#!/usr/bin/env python

from enum import Enum

from speed_controller.srv import speed_control

import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from std_msgs.msg import String
#from sensor_msgs.msg import BatteryState

class states(Enum):
    ROBOT_IDLE = 0, 
    ROBOT_PATROL_FWD = 1,
    ROBOT_APPROACH_FWD = 2,
    ROBOT_DETERRENT_FWD = 3,
    ROBOT_MANUAL = 4

pubTargetSpeed = rospy.Publisher('target_speed_meters_per_sec', Float32, queue_size=10)
pubFlashLight = rospy.Publisher('led_cmd', UInt16, queue_size=10)

# rosrun rosserial_python serial_node.py
# rostopic pub -1 led_cmd std_msgs/UInt16 '20' 
# rostopic pub -1 /rangefinder/front/MB std_msgs/UInt16 '210'
def Front_Distance_CallBack(msg):

    distance = msg.data
    rospy.loginfo("Front(MB): %i cm", distance)
    
    tarSpeed = 0

    global state
    # global currSpeed
    # global piControl

    if state == states.ROBOT_PATROL_FWD:

        if distance > 200: tarSpeed = 30

        else:

            state = states.ROBOT_APPROACH_FWD
            rospy.loginfo("State: " + state.name)
            
    elif state == states.ROBOT_APPROACH_FWD:

        if distance > 200:

            tarSpeed = 30

            state = states.ROBOT_PATROL_FWD
            rospy.loginfo("State: " + state.name)

        elif distance > 50: tarSpeed = 20 * distance / 200 + 10

        else:

            tarSpeed = 0

            state = states.ROBOT_DETERRENT_FWD
            rospy.loginfo("State: " + state.name)

            pubFlashLight.publish(1) # turn on LED

    elif state == states.ROBOT_DETERRENT_FWD:

        if distance > 200:

            tarSpeed = 30

            state = states.ROBOT_PATROL_FWD
            rospy.loginfo("State: " + state.name)

            pubFlashLight.publish(0) # turn off LED

    elif state == states.ROBOT_IDLE: tarSpeed = 0

    # if tarSpeed != 0 and piControl: 
        
    #     tarSpeed = requestSpeedControl(tarSpeed, currSpeed)

    #     piControl = False

    if state != states.ROBOT_MANUAL:
        pubTargetSpeed.publish(tarSpeed)
        rospy.loginfo("Controlled Speed -> %i cm/s", tarSpeed)
        rospy.loginfo("State: " + state.name)

# use command below in the terminal to activate/deactivate
# rostopic pub -1 status std_msgs/String 'Arm'

def Command_CallBack(msg):

    command = msg.data
    rospy.loginfo("Command: ")
    rospy.loginfo(command)

    global state

    if command == "Arm": 
        state = states.ROBOT_PATROL_FWD
        rospy.loginfo("State: " + state.name)
    elif command == "Stop": 
        state = states.ROBOT_IDLE
        pubTargetSpeed.publish(0)
        rospy.loginfo("State: " + state.name)
    elif command == "Emergency Stop": 
        state = states.ROBOT_IDLE
        pubTargetSpeed.publish(0)
        rospy.loginfo("State: " + state.name)
    elif command == "Forward": 
        state = states.ROBOT_MANUAL
        rospy.loginfo("State: " + state.name)
        pubTargetSpeed.publish(20)
    elif command == "Backward": 
        state = states.ROBOT_MANUAL
        rospy.loginfo("State: " + state.name)
        pubTargetSpeed.publish(-20)

# def Encoder_Callback(msg):

#     global currSpeed
#     global piControl

#     currSpeed = msg.data
#     piControl = True

# def requestSpeedControl(targetSpeed, currentSpeed):

#     rospy.wait_for_service('speed_controlling')
    
#     try:

#         speed_controlling = rospy.ServiceProxy('speed_controlling', speed_control)
#         controller = speed_controlling(targetSpeed,currentSpeed)

#         speed = controller.controlled_speed
#         return speed

#     except rospy.ServiceException as e: rospy.loginfo("Service call failed")

def main():
    rospy.init_node('wire_bot')

    global state
    state = states.ROBOT_IDLE
    rospy.loginfo("State: " + state.name)

    rospy.Subscriber("/distance/fore", UInt16, Front_Distance_CallBack)
    rospy.Subscriber("status", String, Command_CallBack)
    # rospy.Subscriber("/encoder/meters_per_second", Float32, Encoder_Callback)

    rospy.spin()

if __name__ == '__main__':
    main()
