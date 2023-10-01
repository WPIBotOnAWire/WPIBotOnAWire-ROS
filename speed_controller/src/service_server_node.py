#!/usr/bin/env python

import rospy

from speed_controller.srv import speed_control, speed_controlResponse

def callback(request):

    speed = (request.target_speed + request.current_speed)/2
    return speed_controlResponse(speed)

def main():
    rospy.init_node("speed_control_service")
    service = rospy.Service("speed_controlling", speed_control, callback)
    rospy.loginfo("Started speed contrller service")
    rospy.spin()

if __name__ == '__main__':

    main()