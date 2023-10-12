#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import os
import torch
import ssl
ssl._create_default_https_context = ssl._create_unverified_context

# Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

# Publishers
pubBirdExistence = rospy.Publisher('/bird_detect', Bool, queue_size=10)


# Image
im = 'images/bird.jpg'
fileDir = os.path.dirname(os.path.abspath(__file__))
im = os.path.join(fileDir, im)



def birdDetect(source):

    rospy.loginfo("Detecting bird")

    # Inference
    results = model(im)

    elemPerRow = 7
    rows = int(results.pandas().xyxy[0].size / elemPerRow)
    names = results.pandas().xyxy[0].name
    confidences = results.pandas().xyxy[0].confidence

    for index in range(rows):

        name = names[index]
        confidence = confidences[index]

        foundBird = name == "bird" and confidence > 0.5

        if foundBird: 

            rospy.loginfo("Detected bird")
            pubBirdExistence.publish(True)            
            return True

    rospy.loginfo("Detected bird")
    pubBirdExistence.publish(False)         
    return False

    

def main():
    rospy.init_node('bird_detection')

    birdDetect(im)

    rospy.spin()

if __name__ == '__main__':
    main()