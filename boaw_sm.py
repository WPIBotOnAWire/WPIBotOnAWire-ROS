#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool

#Global Variables
rfReadingGlobal = 0
switchGlobal = 'OFF'
robotSpeedpub

# Subscribers
rospy.Subscriber("/rangefinder/front", Float32 , RfCallback)
rospy.Subscriber("/switch", String , SwitchCallback)

# Publishers
robotSpeedPub = rospy.Publisher('/motor_speed', Float32, queue_size=10)
flashLightPub = rospy.Publisher('/deterrents/led', Bool, queue_size=10)
soundPub = rospy.Publisher('/deterrents/speaker', Bool, queue_size=10)

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
        self.rfReading = rfReadingGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE')
        if self.rfReading <= 0.5:    #0.5 meters
            robot_speed = 0         #stop the robot
            robotSpeedPub.publish(robot_speed)
            return True     #switch to Approach State
        else:
            robot_speed = 1         #move the robot forward
            robotSpeedPub.publish(robot_speed)
            return False


# define state Approach
class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        self.rfReading = rfReadingGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state APPROACH')
        if self.rfReading <= 0.2:    #0.2 meters
            return True
        elif ((self.rfReading > 0.2) and (self.rfReading < 0.5)):
            #velocity curve
            robot_speed = 0.2         #replace with actual values
            robotSpeedPub.publish(robot_speed)
            return False


# define state Deterrents_On
class Deterrents_On(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        self.rfReading = rfReadingGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state DETERRENTS_ON')
        if self.rfReading <= 0.2:    #0.2 meters
            #turn on the the deterrents
            #flash the LEDS for 5 seconds
            flashLightStatus = True     #Turn on the leds
            flashLightPub(flashLightStatus)
            #create the sound for 5 seconds
            soundStatus = True          #Turn on the speaker
            soundPub(soundStatus)
            return False
        elif self.rfReading > 0.5:
            return True


# define state Deterrents_Off
class Deterrents_Off(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        self.rfReading = rfReadingGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state DETERRENTS_ON')
        #turn on the the deterrents
        #flash the LEDS for 5 seconds
        #create the sound for 5 seconds
        flashLightStatus = False       #turn off the leds
        flashLightPub(flashLightStatus)
        #create the sound for 5 seconds
        soundStatus = False            #turn of the speakers
        globals()['switchGlobal'] = 'OFF'
        soundPub(soundStatus)
        return True


def RfCallback(data):
    # assign rangefinder reading
    globals()['rfReadingGlobal'] = data

def SwitchCallback(data):
    # assign rangefinder reading
    globals()['switchGlobal'] = data


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
    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()

