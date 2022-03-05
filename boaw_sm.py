#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import BatteryState
import threading

#Constants
PATRAOL_FWD_SPEED = 0.10 #10% motor power
APPROACH_FWD_SPEED = 0.07 #7% motor speed
PATRAOL_REV_SPEED = -0.10 #10% motor power
APPROACH_REV_SPEED = -0.07 #7% motor speed
APPROACH_DIST = 20 #inches
STOP_DIST = 10 #inches
ENC_FWD_LIMIT = -12000 # Ticks
ENC_REV_LIMIT = 12000 # Ticks
ROBOT_ACCEL = 0.0015 # 0.1% per tick
START_CHARGING_THRESH = 16000 #mV
DONE_CHARDING_THRESH = 16200 #mV 
BATTERY_CHARGING_THRESH = 40 #mV - A voltage jump of xmV is needed to detect as charged

#Global Variables
rfBackGlobal = 999 #inch
rfFrontGlobal = 999 #inch
encGlobal = 0 #ticks
batGlobal = 0 #ticks
currRobotSpeed = 0.0 # %motor
voltageBeforeCharging = 99999 #mV
switchGlobal = 'ON' #this can be used by adding an button on the bot and having that start or stop the state machine




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
            robotSpeedPub.publish(0)
            return 'OFF'



# define state Move
class FWD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ENC_LIM','RF_LIM', False, 'BAT_LOW'])
        self.rfReading = rfFrontGlobal
        self.encReading = encGlobal
        robotSpeedPub.publish(PATRAOL_FWD_SPEED)

    def execute(self, userdata):
        self.encReading = encGlobal
        self.rfReading = rfFrontGlobal
        self.batReading = batGlobal
        rospy.loginfo('Batt: '+str(self.batReading))
        rospy.loginfo('FrontRF: '+str(self.rfReading))
        rospy.loginfo('Encorder: '+str(self.encReading))
        if(self.encReading < ENC_FWD_LIMIT):
            currRobotSpeed = PATRAOL_FWD_SPEED
            return 'ENC_LIM'

        if(self.rfReading < APPROACH_DIST):
            robotSpeedPub.publish(0)
            return 'RF_LIM'

        if(self.batReading < START_CHARGING_THRESH):
            return 'BAT_LOW'

        robotSpeedPub.publish(PATRAOL_FWD_SPEED)
        return False

class REV(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ENC_LIM', False])
        self.encReading = encGlobal
        robotSpeedPub.publish(PATRAOL_REV_SPEED)

    def execute(self, userdata):
        self.encReading = encGlobal
        rospy.loginfo('Encorder: '+str(self.encReading))
        if(self.encReading > ENC_REV_LIMIT):
            currRobotSpeed = PATRAOL_REV_SPEED
            return 'ENC_LIM'
        robotSpeedPub.publish(PATRAOL_REV_SPEED)
        return False

class FWD2REV(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[True, False])
        robotSpeedPub.publish(currRobotSpeed)

    def execute(self, userdata):
        robotSpeedPub.publish(currRobotSpeed)
        rospy.loginfo('current speed: '+str(currRobotSpeed))
        globals()['currRobotSpeed'] = currRobotSpeed - ROBOT_ACCEL
        if currRobotSpeed <= PATRAOL_REV_SPEED:
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
        if currRobotSpeed >= PATRAOL_FWD_SPEED:
            return True
        return False

class OBS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CLEAR', False])
        self.rfReading = rfFrontGlobal

    def execute(self, userdata):
        rospy.loginfo('FrontRF: '+str(self.rfReading))
        self.rfReading = rfFrontGlobal
         #turn on the the deterrents
        soundPub.publish(4000)
        flashLightPub.publish(True)
        rospy.sleep(0.26)
        flashLightPub.publish(False)
        rospy.sleep(0.26)
        flashLightPub.publish(True)
        rospy.sleep(0.26)
        flashLightPub.publish(False)
        rospy.sleep(0.26)

        if self.rfReading > APPROACH_DIST:
            return 'CLEAR' 
           
        return False

class APPROACH_DOCK(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DOCKED', False])
        globals()['voltageBeforeCharging'] = batGlobal
    def execute(self, userdata):
        self.batReading = batGlobal
        self.encReading = encGlobal
        rospy.loginfo('Bat Voltage: '+str(self.batReading))
        rospy.loginfo('Enc Reading: '+str(self.encReading))
        if(self.encReading < ENC_FWD_LIMIT):
            robotSpeedPub.publish(APPROACH_FWD_SPEED)
        else:
            robotSpeedPub.publish(PATRAOL_FWD_SPEED)

        if self.batReading > batGlobal + BATTERY_CHARGING_THRESH:
            return 'DOCKED'

        return False
        

        
class CHARGING(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CHARGED', False])

    def execute(self, userdata):
        self.batReading = batGlobal
        rospy.loginfo('Bat Voltage: '+str(self.batReading))
        robotSpeedPub.publish(0)
        if self.batReading > DONE_CHARDING_THRESH:
            return 'CHARGED'

        return False



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


# Subscribers
rospy.Subscriber("/rangefinder/front", Float32 , RfFrontCallback)
rospy.Subscriber("/rangefinder/back", Float32, RfBackCallback)
rospy.Subscriber("/encoder", Float32, EncCallback)
rospy.Subscriber("/battery", BatteryState, BatCallback)

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
                               transitions={'ENC_LIM' :'FWD2REV', False:'FWD', 'RF_LIM':'OBS','BAT_LOW':'APPROACH_DOCK'} )
        smach.StateMachine.add('FWD2REV', FWD2REV(), 
                               transitions={True :'REV', False:'FWD2REV'})
        smach.StateMachine.add('REV', REV(), 
                               transitions={'ENC_LIM' :'REV2FWD', False: 'REV'})
        smach.StateMachine.add('REV2FWD', REV2FWD(), 
                               transitions={True :'FWD', False:'REV2FWD'})
        smach.StateMachine.add('OBS', OBS(), 
                               transitions={'CLEAR':'FWD', False:'OBS'})
        smach.StateMachine.add('APPROACH_DOCK', APPROACH_DOCK(), 
                               transitions={'DOCKED':'FWD', False:'APPROACH_DOCK'})
        smach.StateMachine.add('CHARGING', CHARGING(), 
                               transitions={'CHARGED':'REV', False:'CHARGING'})

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

