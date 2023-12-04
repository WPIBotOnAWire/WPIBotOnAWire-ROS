#!/usr/bin/env python
import rospy, requests
import json
from std_msgs.msg import String

def callback(data):
    print(data.data)
    json_data = json.loads(data.data)
    print(json_data)
    rospy.loginfo(rospy.get_caller_id() + "Sending Data: %s", data.data)
    url = "http://130.215.124.178:5000/update_globals"
    requests.post(url, json=json_data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
