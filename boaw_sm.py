#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool, String, Int32
import threading

#Global Variables
patrolSpeed = 0.10 #10% motor power
approachSpeed = 0.07 #7% motor speed
approachDist = 20 #inches
stopDist = 10 #inches
rfBackGlobal = 999 #inch
rfFrontGlobal = 999 #inch
switchGlobal = 'ON'

# define state Static
class Static(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ON', 'OFF'])
        self.switch = switchGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state STATIC')
        if self.switch == 'ON':
            return 'ON'   #switch to Move State
        else:
            robot_speed = 0
            robotSpeedPub.publish(robot_speed)
            return 'OFF'



# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        self.rfReading = rfFrontGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE')
        if self.rfReading <= approachDist:    #0.5 meters
            robot_speed = 0         #stop the robot
            robotSpeedPub.publish(robot_speed)
            return True     #switch to Approach State
        else:
            robot_speed = patrolSpeed         #move the robot forward
            robotSpeedPub.publish(robot_speed)
            return False


# define state Approach
class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        self.rfReading = rfFrontGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state APPROACH')
        if self.rfReading <= stopDist:    #0.2 meters
            return True
        elif ((self.rfReading > stopDist) and (self.rfReading < approachDist)):
            #velocity curve
            robot_speed = patrolSpeed         #replace with actual values
            robotSpeedPub.publish(robot_speed)
            return False


# define state Deterrents_On
class Deterrents_On(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        self.rfReading = rfFrontGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state DETERRENTS_ON')
        if self.rfReading <= stopDist:    #0.2 meters
            #turn on the the deterrents
            #flash the LEDS for 5 seconds
            flashLightStatus = True     #Turn on the leds
            flashLightPub.publish(flashLightStatus)
            #create the sound for 5 seconds
            soundStatus = 4000          #Turn on the speaker hz
            soundPub.publish(soundStatus)
            return False
        elif self.rfReading > approachDist:
            return True


# define state Deterrents_Off
class Deterrents_Off(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        self.rfReading = rfFrontGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state DETERRENTS_ON')
        #turn on the the deterrents
        #flash the LEDS for 5 seconds
        #create the sound for 5 seconds
        flashLightStatus = False       #turn off the leds
        flashLightPub.publish(flashLightStatus)
        #create the sound for 5 seconds
        soundStatus = False            #turn of the speakers
        globals()['switchGlobal'] = 'OFF'
        soundPub(soundStatus)
        return True


def RfFrontCallback(data):
    # assign rangefinder reading
    globals()['rfFrontGlobal'] = data.data

def RfBackCallback(data):
    globals()['rfBackGlobal'] = data.data


# Subscribers
rospy.Subscriber("/rangefinder/front", Float32 , RfFrontCallback)
rospy.Subscriber("/rangefinder/back", Float32, RfBackCallback)

# Publishers
robotSpeedPub = rospy.Publisher('/motor_speed', Float32, queue_size=10)
flashLightPub = rospy.Publisher('/deterrents/led', Bool, queue_size=10)
soundPub = rospy.Publisher('/play_sound', Int32, queue_size=10)


def main():
    rospy.init_node('BOAW_SM')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[True, False])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STATIC', Static(), 
                               transitions={'ON':'MOVE', 'OFF':'STATIC'})
        smach.StateMachine.add('MOVE', Move(), 
                               transitions={True :'APPROACH', False:'MOVE'})
        smach.StateMachine.add('APPROACH', Approach(), 
                               transitions={True :'DETERRENTS_ON', False:'APPROACH'})
        smach.StateMachine.add('DETERRENTS_ON', Deterrents_On(), 
                               transitions={True :'DETERRENTS_OFF', False: 'DETERRENTS_ON'})
        smach.StateMachine.add('DETERRENTS_OFF', Deterrents_Off(), 
                               transitions={True :'STATIC', False:'STATIC'})

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    my_smach_con.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()


if __name__ == '__main__':
    main()

