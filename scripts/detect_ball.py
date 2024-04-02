#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

img_received = False
bridge = CvBridge()

#Thought Proccess time: well I went insnae on this. I took the time and looked at all the resourses 
#you gave us through the slides and figured out a way to create a retangle filter for the table. But
#This just came donw to me toying with the values until I was okay with it.


# get the image message
def get_image(ros_img):
    global img_received
    global bridge
    # convert to opencv image
    rgb_img = bridge.imgmsg_to_cv2(ros_img, "rgb8")
    # Process the image to detect ball
    processed_img = process_image(rgb_img)
    # Publish the processed image
    img_pub.publish(bridge.cv2_to_imgmsg(processed_img, encoding="mono8"))
    # raise flag
    img_received = True

def process_image(rgb_img):
    # Convert RGB image to HSV
    hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
    # Define range of ball color in HSV
    lower_color = np.array([20, 1, 100])
    upper_color = np.array([60, 255, 255])
    # Threshold the HSV image to get only ball color
    mask = cv2.inRange(hsv_img, lower_color, upper_color)
    # Remove background noise
    mask = remove_background_noise(mask)
    return mask

def remove_background_noise(mask):
    # Create a mask for region of interest
    roi_mask = np.zeros_like(mask)
    height, width = mask.shape
    cv2.rectangle(roi_mask, (width//4, height//4), (width*3//4, height*3//4), 255, -1)
    # Apply bitwise_and operation to select only desired regions
    masked_img = cv2.bitwise_and(mask, roi_mask)
    return masked_img

if __name__ == '__main__':
    # define the node and subcribers and publishers
    rospy.init_node('detect_ball', anonymous=True)
    # define a subscriber to receive images
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
    # define a publisher to publish processed images
    img_pub = rospy.Publisher('/ball_2D', Image, queue_size=1)
    
    rospy.spin()
