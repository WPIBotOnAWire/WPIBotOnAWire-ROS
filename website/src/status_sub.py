#!/usr/bin/env python
import rospy, requests
import json
from std_msgs.msg import String

def status_listener():

    rospy.init_node('status_listener_node', anonymous=True)

    web_server_url = 'http://130.215.124.178:5000/receiver'

    pub = rospy.Publisher('status', String, queue_size=10)

    rate = rospy.Rate(1)

    

    while not rospy.is_shutdown():
        try:
            response = requests.get(web_server_url)

            global prevCommand
            
            if response.status_code == 200:
                data = response.json()

                if data['command'] != prevCommand:
                    rospy.loginfo(f"Received status: {data['command']}")
                    pub.publish(data['command'])
                    prevCommand = data['command'] 

            else:
                rospy.logwarn(f"Failed to retreive data. Status code: {response.status_code}")

        except requests.RequestException as e:
            rospy.logerr(f"Request failed: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        global prevCommand
        prevCommand = ''
        status_listener()
    except rospy.ROSInterruptException:
        pass
