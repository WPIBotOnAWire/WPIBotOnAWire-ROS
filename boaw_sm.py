import tensorflow as tf
import os
from PIL import Image
import numpy as np
import cv2
import time
import warnings
import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import BatteryState
import os
from PIL import Image
import numpy as np
import time

warnings.filterwarnings('ignore') 

# Publishers
flashLightPub = rospy.Publisher('/deterrents/led', Bool, queue_size=10)
soundPub = rospy.Publisher('/play_sound', Int32, queue_size=10)

time_last_fired = 0;

def fire_deterrents():
    print("Deterring")
    #rospy.loginfo("Raven Detected")
    soundPub.publish(4000)
    # flashLightPub.publish(True)
    # rospy.sleep(1)
    # flashLightPub.publish(False)
    # rospy.sleep(1)
    # flashLightPub.publish(True)
    # rospy.sleep(1)
    # flashLightPub.publish(False)
    # rospy.sleep(1)
    # flashLightPub.publish(True)
    # rospy.sleep(1)
    # flashLightPub.publish(False)
    # rospy.sleep(1)
    # flashLightPub.publish(True)
    # rospy.sleep(1)
    # flashLightPub.publish(False)
    # rospy.sleep(1)

def show_webcam(mirror=False):
        if mirror: 
            img = cv2.flip(img, 1)

def convert_to_opencv(image):
    # RGB -> BGR conversion is performed as well.
    image = image.convert('RGB')
    r,g,b = np.array(image).T
    opencv_image = np.array([b,g,r]).transpose()
    return opencv_image

def crop_center(img,cropx,cropy):
    h, w = img.shape[:2]
    startx = w//2-(cropx//2)
    starty = h//2-(cropy//2)
    return img[starty:starty+cropy, startx:startx+cropx]

def resize_down_to_1600_max_dim(image):
    h, w = image.shape[:2]
    if (h < 1600 and w < 1600):
        return image

    new_size = (1600 * w // h, 1600) if (h > w) else (1600, 1600 * h // w)
    return cv2.resize(image, new_size, interpolation = cv2.INTER_LINEAR)

def resize_to_256_square(image):
    h, w = image.shape[:2]
    return cv2.resize(image, (256, 256), interpolation = cv2.INTER_LINEAR)

def run(filename, labels_filename):
    graph_def = tf.compat.v1.GraphDef()
    labels = []

    # These are set to the default names from exported models, update as needed.
    # filename = "model.pb"
    # labels_filename = "labels.txt"

    # Import the TF graph
    with tf.io.gfile.GFile(filename, 'rb') as f:
        graph_def.ParseFromString(f.read())
        tf.import_graph_def(graph_def, name='')

    # Create a list of labels.
    with open(labels_filename, 'rt') as lf:
        for l in lf:
            labels.append(l.strip())

    output_layer = 'loss:0'
    input_node = 'Placeholder:0'

    with tf.compat.v1.Session() as sess:
        # Get the input size of the model
        input_tensor_shape = sess.graph.get_tensor_by_name(input_node).shape.as_list()
        network_input_size = input_tensor_shape[1]

    cam = cv2.VideoCapture(0)
    with tf.compat.v1.Session() as sess:
        while True:
            ret_val, image = cam.read()
            cv2.imshow('webcam feed', image)


            # Convert to OpenCV format
            #image = convert_to_opencv(image)

            # If the image has either w or h greater than 1600 we resize it down respecting
            # aspect ratio such that the largest dimension is 1600
            image = resize_down_to_1600_max_dim(image)

           # We next get the largest center square
            h, w = image.shape[:2]
            min_dim = min(w,h)
            max_square_image = crop_center(image, min_dim, min_dim)

            # Resize that square down to 256x256
            augmented_image = resize_to_256_square(max_square_image)


            # Crop the center for the specified network_input_Size
            augmented_image = crop_center(augmented_image, network_input_size, network_input_size)

            try:
                tic = time.perf_counter()
                prob_tensor = sess.graph.get_tensor_by_name(output_layer)
                predictions = sess.run(prob_tensor, {input_node: [augmented_image] })
                toc = time.perf_counter()
            except KeyError:
                print ("Couldn't find classification output layer: " + output_layer + ".")
                print ("Verify this a model exported from an Object Detection project.")
                exit(-1)

            # Print the highest probability label
            highest_probability_index = np.argmax(predictions)
            print('Classified as: ' + labels[highest_probability_index])
            #print("Raven Probability: " + str(predictions))
            print(f"Processed in {toc - tic:0.4f} seconds")
            print("------------------------------")
            tic2 = time.perf_counter()
            global time_last_fired
            elapsed_time = tic2 - time_last_fired
            print(elapsed_time)
            if(elapsed_time > 5):
                if(str(labels[highest_probability_index]) == 'Raven'):
                    fire_deterrents()
                    time_last_fired = time.perf_counter()

            	
            if cv2.waitKey(50) == 27: 
                break  # esc to quit
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node('BOAW_SM')
    filename = "model.pb"
    labels_filename = "labels.txt"
    run(filename, labels_filename)

    

