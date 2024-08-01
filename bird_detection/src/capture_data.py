#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
import os
import cv2 as cv

## TODO: Make sure that these are the right cam ports for the front and the back
## For now, camera 1 commented out for testing
frontCamera = cv.VideoCapture(0)
#backCamera = cv.VideoCapture(1)

#Used to give each image a unique name
foreIndex = 0 
aftIndex = 0

#Image path and naming schemes
imagePath = "/home/boaw/bird_photos/"
aftNameStem = "aft_img_"
foreNameStem = "fore_img_"

#Images will be saved as a png format, can easily be changed to a jpeg if needed
imgType = ".png"

# Used to keep track of the last image so we don't save too many images
lastImageTimeFore = 0

def front_capture_callback(msg: UInt16):
    global lastImageTimeFore
    global foreIndex

    currentTime = rospy.get_time()
    if msg.data < 200:
        if currentTime - lastImageTimeFore > 5:
            result, image = frontCamera.read()
            name = imagePath + foreNameStem + str(foreIndex) + "-bird" + imgType
            foreIndex += 1
            cv.imwrite(filename=name, img=image)
            rospy.loginfo(name)
            lastImageTimeFore = currentTime
    
    else:
        if currentTime - lastImageTimeFore > 20:
            result, image = frontCamera.read()
            name = imagePath + foreNameStem + str(foreIndex) + "-nobird" + imgType
            foreIndex += 1
            cv.imwrite(filename = name, img = image)
            rospy.loginfo(name)
            lastImageTimeFore = currentTime
    return

def back_capture_callback(msg: UInt16):

    #If we are more than 2 meters away from a bird, don't bother taking a picture
    while msg.data < 200:    
        result, image = backCamera.read()
        name = imagePath + aftNameStem + aftIndex + imgType
        aftIndex += 1
        cv.imwrite(filename=name, img=image)
        #Sleep for a second so there aren't infinite photos of the cormorant
        rospy.sleep(1)
    return


def main():
    if not os.path.exists(imagePath):
        os.mkdir(imagePath)
    rospy.init_node("capture_data")
    lastImageTime = rospy.get_time()
    rospy.Subscriber("/distance/fore", UInt16, front_capture_callback)
#    rospy.Subscriber("/distance/aft", UInt16, back_capture_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
