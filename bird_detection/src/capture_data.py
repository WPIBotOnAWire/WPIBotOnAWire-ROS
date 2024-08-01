#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
import os
import cv2 as cv

# Image path
imagePath = "/home/boaw/bird_photos/"

# Images will be saved as a png format, can easily be changed to a jpeg if needed
imgType = ".png"


class image_capture_manager: 
    def __init__(self, topic, iCam): 
        # initialize the subscriber node now. 
        self.imgType = ".png"
        self.lastImageTime = 0
        self.file_index = 0
        self.camera_index = iCam
        self.camera = cv.VideoCapture(iCam)
        self.distance_sub = rospy.Subscriber(topic, UInt16, self.callback)
        
    def callback(self, distance_msg):
        distance = distance_msg.data
        currentTime = rospy.get_time()

        time_since_last_image = currentTime - self.lastImageTime
        if distance < 200:
            if time_since_last_image > 5:
                self.save_image(True)
                self.lastImageTime = currentTime
        else:
            if time_since_last_image > 20:
                self.save_image(False)
                self.lastImageTime = currentTime

    def create_filename(self, hasBird):
        name = imagePath + str(self.camera_index) + "-" + str(self.file_index) + "-" + str(hasBird) + imgType
        rospy.loginfo(name)
        
        self.file_index += 1
        return name
    
    def save_image(self, hasBird):
        result, image = self.camera.read()
        name = self.create_filename(hasBird)
        cv.imwrite(filename = name, img = image)



def main():
    if not os.path.exists(imagePath):
        os.mkdir(imagePath)
    rospy.init_node("capture_data")

    ## TODO: Make sure that these are the right cam ports for the front and the back
    front_camera = image_capture_manager("/distance/fore", 0)
#    aft_camera = image_capture_manager("/distance/aft", 1)

    rospy.spin()

if __name__ == '__main__':
    main()
