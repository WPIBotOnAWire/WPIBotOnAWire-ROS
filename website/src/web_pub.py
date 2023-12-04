#!/usr/bin/env python
#from web_server import state
import rospy
from std_msgs.msg import String
import json
dist = 0

def talker():
    global dist
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        global state
        #-----TESTING----
        dist += 1
        state = "Patrolling Right"
        wire_end = 1000
        battery = 100
        #----------------
        data = {"state": state, "dist": dist, "wire_end": wire_end, "battery":battery}
        data = json.dumps(data) #Stringify the JSON for ros
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
