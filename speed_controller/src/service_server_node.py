#!/usr/bin/env python

import rospy

wire_bot = __import__("Wire-Bot")
#wire_bot = __import__("distance-estimator")
#import speed_controller
from speed_controller.srv import speed_control, speed_controlResponse

def callback(request):
    return speed_controlResponse(request.target_speed - request.current_speed)

def main():
    rospy.init_node("speed_control_service")
    service = rospy.Service("speed_controlling", speed_control, callback)
    rospy.loginfo("Started speed contrller service")
    rospy.spin()

if __name__ == '__main__':

    main()