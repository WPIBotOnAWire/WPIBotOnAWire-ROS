import rospy
from std_msgs.msg import UInt16
import os
import cv2 as cv

##TODO: Make sure that these are the right cam ports for the front and the back
frontCamera = cv.VideoCapture(0)
backCamera = cv.VideoCapture(1)
#Used to give each image a unique name
foreIndex, aftIndex = 0
#Image path and naming schemes
imagePath = "~/home/boaw/bird_photos/"
aftNameStem = "aft_img_"
foreNameStem = "fore_img_"
#Images will be saved as a png format, can easily be changed to a jpeg if needed
imgType = ".png"

def front_capture_callback(msg: UInt16):
    #If we are more than 2 meters away from a bird, don't bother taking a picture
    
    while msg.data < 200:    
        result, image = frontCamera.read()
        name = imagePath + foreNameStem + foreIndex + imgType
        foreIndex += 1
        cv.imwrite(filename=name, img=image)
        #Sleep for a second so there aren't infinite photos of the cormorant
        rospy.sleep(1)
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
        os.mkdirs(imagePath)
    rospy.init_node("capture_data")
    rospy.Subscriber("/distance/fore", UInt16, front_capture_callback)
    rospy.Subscriber("/distance/aft", UInt16, back_capture_callback)
    rospy.spin()

if __name__ == '__main__':
    main()