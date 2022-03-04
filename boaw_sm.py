#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool, String, Int32
import threading

#Constants
PATRAOL_SPEED = 0.15 #10% motor power
APPROACH_SPEED = 0.07 #7% motor speed
APPROACH_DIST = 20 #inches
STOP_DIST = 10 #inches
ENC_FWD_LIMIT = 1000 # Ticks
ENC_REV_LIMIT = -1000 # Ticks
ROBOT_ACCEL = 0.0005 # 0.1% per tick

#Global Variables
rfBackGlobal = 999 #inch
rfFrontGlobal = 999 #inch
encGlobal = 0 #ticks
currRobotSpeed = 0.0 # % motor
switchGlobal = 'ON'



# define state Static
class Static(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ON', 'OFF'])
        self.switch = switchGlobal

    def execute(self, userdata):
        # rospy.loginfo('Executing state STATIC')
        if self.switch == 'ON':
            return 'ON'   #switch to Move State
        else:
            robot_speed = 0
            robotSpeedPub.publish(robot_speed)
            return 'OFF'



# define state Move
class FWD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ENC_LIM','RF_LIM' False])
        self.rfReading = rfFrontGlobal
        self.encReading = encGlobal
        robotSpeedPub.publish(PATRAOL_SPEED)

    def execute(self, userdata):
        self.encReading = encGlobal
        self.rfReading = rfFrontGlobal
        rospy.loginfo('Encorder: '+str(self.encReading))
        if(self.encReading > ENC_FWD_LIMIT):
            currRobotSpeed = PATRAOL_SPEED
            return 'ENC_LIM'

        if(self.rfReading < APPROACH_DIST):
            robotSpeedPub.publish(0)
            return 'RF_LIM'
        # rospy.loginfo('Executing state Fwd')
        # if self.rfReading <= APPROACH_DIST:    #0.5 meters
        #     robot_speed = 0         #stop the robot
        #     robotSpeedPub.publish(robot_speed)
        #     return True     #switch to Approach State
        # else:
        #     robot_speed = PATRAOL_SPEED
             #move the robot forward
        #     robotSpeedPub.publish(robot_speed)
        #     return False
        robotSpeedPub.publish(PATRAOL_SPEED)
        return False

class REV(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ENC_LIM', False])
        self.encReading = encGlobal
        robotSpeedPub.publish(-PATRAOL_SPEED)

    def execute(self, userdata):
        self.encReading = encGlobal
        rospy.loginfo('Encorder: '+str(self.encReading))
        if(self.encReading < ENC_REV_LIMIT):
            currRobotSpeed = -PATRAOL_SPEED
            return 'ENC_LIM'
        robotSpeedPub.publish(-PATRAOL_SPEED)
        return False

class FWD2REV(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        robotSpeedPub.publish(currRobotSpeed)

    def execute(self, userdata):
        robotSpeedPub.publish(currRobotSpeed)
        rospy.loginfo('current speed: '+str(currRobotSpeed))
        globals()['currRobotSpeed'] = currRobotSpeed - ROBOT_ACCEL
        if currRobotSpeed <= -PATRAOL_SPEED:
            return True
        return False

class REV2FWD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        robotSpeedPub.publish(currRobotSpeed)

    def execute(self, userdata):
        robotSpeedPub.publish(currRobotSpeed)
        globals()['currRobotSpeed'] = currRobotSpeed + ROBOT_ACCEL
        rospy.loginfo('current speed: '+str(currRobotSpeed))
        if currRobotSpeed >= PATRAOL_SPEED:
            return True
        return False

class OBJ(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CLEAR', False])
        self.rfReading = rfFrontGlobal

    def execute(self, userdata):
        rospy.loginfo('Executing state DETERRENTS_ON')
        self.rfReading = rfFrontGlobal
         #turn on the the deterrents
            soundPub.publish(4000)
            flashLightPub.publish(True)
            rospy.sleep(0.25)
            flashLightPub.publish(False)
            rospy.sleep(0.25)
            flashLightPub.publish(True)
            rospy.sleep(0.25)
            flashLightPub.publish(False)
            rospy.sleep(0.25)

        if self.rfReading > APPROACH_DIST:
            return 'CLEAR' 
           
        return False

# # define state Approach
# class Approach(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=[True, False])
#         self.rfReading = rfFrontGlobal

#     def execute(self, userdata):
#         rospy.loginfo('Executing state APPROACH')
#         if self.rfReading <= STOP_DIST:    #0.2 meters
#             return True
#         elif ((self.rfReading > STOP_DIST) and (self.rfReading < APPROACH_DIST)):
#             #velocity curve
#             robot_speed = PATRAOL_SPEED
#                  #replace with actual values
#             robotSpeedPub.publish(robot_speed)
#             return False


# # define state Deterrents_On
# class Deterrents_On(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=[True, False])
#         self.rfReading = rfFrontGlobal

#     def execute(self, userdata):
#         rospy.loginfo('Executing state DETERRENTS_ON')
#         if self.rfReading <= STOP_DIST:    #0.2 meters
#             #turn on the the deterrents
#             #flash the LEDS for 5 seconds
#             flashLightStatus = True     #Turn on the leds
#             flashLightPub.publish(flashLightStatus)
#             #create the sound for 5 seconds
#             soundStatus = 4000          #Turn on the speaker hz
#             soundPub.publish(soundStatus)
#             return False
#         elif self.rfReading > APPROACH_DIST:
#             return True


# # define state Deterrents_Off
# class Deterrents_Off(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=[True, False])
#         self.rfReading = rfFrontGlobal

#     def execute(self, userdata):
#         rospy.loginfo('Executing state DETERRENTS_ON')
#         #turn on the the deterrents
#         #flash the LEDS for 5 seconds
#         #create the sound for 5 seconds
#         flashLightStatus = False       #turn off the leds
#         flashLightPub.publish(flashLightStatus)
#         #create the sound for 5 seconds
#         soundStatus = False            #turn of the speakers
#         globals()['switchGlobal'] = 'OFF'
#         soundPub(soundStatus)
#         return True


def RfFrontCallback(data):
    # assign rangefinder reading
    globals()['rfFrontGlobal'] = data.data
    # rospy.loginfo("Front RF Callback")

def RfBackCallback(data):
    globals()['rfBackGlobal'] = data.data
    # rospy.loginfo("Back RF Callback")

def EncCallback(data):
    globals()['encGlobal'] = data.data
    # rospy.loginfo("Encoder Callback")


# Subscribers
rospy.Subscriber("/rangefinder/front", Float32 , RfFrontCallback)
rospy.Subscriber("/rangefinder/back", Float32, RfBackCallback)
rospy.Subscriber("/encoder", Float32, EncCallback)

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
                               transitions={'ON':'FWD', 'OFF':'STATIC'})
        smach.StateMachine.add('FWD', FWD(), 
                               transitions={'ENC_LIM' :'FWD2REV', False:'FWD', 'RF_LIM:OBS'} )
        smach.StateMachine.add('FWD2REV', FWD2REV(), 
                               transitions={True :'REV', False:'FWD2REV'})
        smach.StateMachine.add('REV', REV(), 
                               transitions={'ENC_LIM' :'REV2FWD', False: 'REV'})
        smach.StateMachine.add('REV2FWD', REV2FWD(), 
                               transitions={True :'FWD', False:'REV2FWD'})
        smach.StateMachine.add('OBJ', OBJ(), 
                               transitions={'CLEAR':'FWD', False:'OBJ'})

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

