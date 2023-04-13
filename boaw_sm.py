#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import BatteryState
import threading

#Constants

PATROL_FWD_SPEED = 50
APPROACH_FWD_SPEED = 30
PATROL_REV_SPEED = -50
APPROACH_REV_SPEED = -30
APPROACH_DIST = 0.15 #meters
STOP_DIST = .25 #meters
ENC_FWD_LIMIT = 300# Ticks
ENC_REV_LIMIT = -300 # Ticks
ROBOT_ACCEL = 0.0000001 # 0.1% per tick
START_CHARGING_THRESH = 15050 #mV
DONE_CHARDING_THRESH = 15300 #mV
BATTERY_CHARGING_THRESH = 100 #mV - A voltage jump of xmV is needed to detect as charged

#Global Variables
rfBackGlobal = 2 #meter
rfFrontGlobal = 2 #meter
encGlobal = -1 #ticks
batGlobal = 0 #ticks
currRobotSpeed = 0.0 # %motor
voltageBeforeCharging = 99999 #mV
switchGlobal = True #this can be used by adding an button on the bot and having that start or stop the state machine
manualGlobal = False
aiGlobal = False
forward = True # use this to keep patrolling the wire after DETERRING
lasPos = 0

def RfFrontCallback(msg):
    # assign rangefinder reading
    globals()['rfFrontGlobal'] = msg.data
    # rospy.loginfo("Front RF Callback")

def RfBackCallback(msg):
    globals()['rfBackGlobal'] = msg.data
    # rospy.loginfo("Back RF Callback")

def EncCallback(msg):
    globals()['encGlobal'] = msg.data
    # rospy.loginfo("Encoder Callback")

def BatCallback(msg):
    globals()['batGlobal'] = msg.voltage
    # rospy.loginfo("Encoder Callback")

def SwitchCallback(msg):
    globals()['switchGlobal'] = msg.data
    # rospy.loginfo("Switch Callback")

def ManualCallback(msg):
    globals()['manualGlobal'] = msg.data
    # rospy.loginfo("Encoder Callback")

def aiCallback(msg):
    raven = False
    rospy.loginfo("poENC_REV_LIMITssible raven: %s", msg.data)
    if msg.data == 'Raven':
        rospy.loginfo("AM RAVEN: %s", raven)
        raven = True
    globals()['aiGlobal'] = raven
    # rospy.loginfo(rospy.get_caller_id() + "AI Detected: %s", msg.data)

def direction():
    if forward:
        direction = 'FWD'
    else: 
        direction = 'REV'
    return direction


# Subscribers
rospy.Subscriber("/rangefinder/front", Float32 , RfFrontCallback)
rospy.Subscriber("/rangefinder/back", Float32, RfBackCallback)
rospy.Subscriber("/encoder", Int32, EncCallback)
rospy.Subscriber("/battery", BatteryState, BatCallback)
rospy.Subscriber("/switch", Bool, SwitchCallback)
rospy.Subscriber("/manual_override", Bool, ManualCallback)
rospy.Subscriber("/ai_detection",String, aiCallback)

# Publishers
robotSpeedPub = rospy.Publisher('/motor_speed', Float32, queue_size=10)
flashLightPub = rospy.Publisher('/deterrents/led', Bool, queue_size=10)
soundPub = rospy.Publisher('/play_sound', Int32, queue_size=10)
statePub = rospy.Publisher('/robot_state', String, queue_size=10)


# define state Static
# this state is when the robot is disabled or in teleop mode
class Static(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['AI_TRUE', 'AI_FALSE'])
        self.switch = switchGlobal
    def execute(self, userdata):
        statePub.publish("Robot Disabled")
        self.switch = switchGlobal
        self.aiGlobal = aiGlobal
        if not aiGlobal:
            robotSpeedPub.publish(0)
        if aiGlobal:
            return 'AI_TRUE'   #switch to Move State
        else:
            return 'AI_FALSE'



# define state Move
# in this state the robot is moving along the wire
class FWD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ENC_ZERO_TRUE','ENC_ZERO_FALSE'])
        self.rfReading = rfFrontGlobal
        self.encReading = encGlobal
        robotSpeedPub.publish(PATROL_FWD_SPEED)

    def execute(self, userdata):
        statePub.publish("Patrolling Forwards")
        self.switch = switchGlobal
        forward = True
        self.encReading = encGlobal
        self.rfReading = rfFrontGlobal
        self.batReading = batGlobal
        
        #rospy.loginfo('Batt: '+str(self.batReading))
        rospy.loginfo('aiGlobal: '+str(aiGlobal))
        rospy.loginfo('FrontRF: '+str(self.rfReading))
        rospy.loginfo('Encorder: '+str(self.encReading))
        if(self.encReading > 0):
            currRobotSpeed = PATROL_FWD_SPEED
            return 'ENC_LIM_FALSE'
        if(self.encReading <= 0):
            currRobotSpeed = 0
            return 'ENC_LIM_TRUE'


class REV(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RF_LIMIT_FALSE','RF_LIMIT_TRUE'])
        self.rfReading = rfBackGlobal
        self.encReading = encGlobal
        robotSpeedPub.publish(PATROL_REV_SPEED)

    def execute(self, userdata):
        statePub.publish("Patrolling Forwards")
        self.switch = switchGlobal
        forward = False
        self.encReading = encGlobal
        self.rfReading = rfBackGlobal
        
        #rospy.loginfo('Batt: '+str(self.batReading))
        #rospy.loginfo('aiGlobal: '+str(aiGlobal))
        rospy.loginfo('FrontRF: '+str(self.rfReading))
        rospy.loginfo('Encoder: '+str(self.encReading))
        if(self.rfReading < APPROACH_DIST):
            currRobotSpeed = PATROL_REV_SPEED
            return 'ENC_LIMIT_FALSE'

        if(self.rfReading >= APPROACH_DIST):
            robotSpeedPub.publish(0)
            return 'ENC_LIMIT_TRUE'

# this state is triggered when a bird is detected while the robot is in forward
class DETERRING(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DONE'])
        self.rfReading = rfFrontGlobal

    def execute(self, userdata):
        statePub.publish("Deterring")
        self.switch = switchGlobal

        rospy.loginfo('FrontRF: '+str(self.rfReading))
        self.rfReading = rfFrontGlobal
         #turn on the the deterrents
        rospy.loginfo('I blink here')
        soundPub.publish(4000)
        flashLightPub.publish(True)
        rospy.sleep(0.26)
        flashLightPub.publish(False)
        rospy.sleep(0.26)
        flashLightPub.publish(True)
        rospy.sleep(0.26)
        flashLightPub.publish(False)
        rospy.sleep(0.26)
        flashLightPub.publish(False)
        aiGlobal = False
        rospy.sleep(4)

        return 'DONE'





def main():
    rospy.init_node('BOAW_SM')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[True, False])
    

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STATIC', Static(), 
                               transitions={'AI_TRUE':'REV', 'AI_FALSE':'STATIC'})
        smach.StateMachine.add('FWD', FWD(), 
                               transitions={'ENC_ZERO_TRUE' :'STATIC', 'ENC_ZERO_FALSE':'FWD'} )
        smach.StateMachine.add('REV', REV(), 
                               transitions={'RF_LIMIT_FALSE' :'REV', 'RF_LIMIT_TRUE': 'DETERRING'})
        smach.StateMachine.add('DETERRING', DETERRING(), 
                               transitions={'DONE': 'FWD'})

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
    


