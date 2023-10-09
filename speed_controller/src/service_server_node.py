#!/usr/bin/env python

import rospy

from speed_controller.srv import speed_control, speed_controlResponse

kp = 1
ki = 0
error = 0
errorSum = 0

def callback(request):

    global kp
    global ki
    global error
    global errorSum

    error = request.target_speed - request.current_speed
    errorSum += error

    speed = kp * error + ki * errorSum

    rospy.loginfo("Error: %i cm/s", error)
    rospy.loginfo("Error Sum: %i cm/s", errorSum)
    rospy.loginfo("Speed: %i cm/s", speed)

    return speed_controlResponse(speed)

def main():
    rospy.init_node("speed_control_service")
    service = rospy.Service("speed_controlling", speed_control, callback)
    rospy.loginfo("Started speed contrller service")
    rospy.spin()

if __name__ == '__main__':

    main()