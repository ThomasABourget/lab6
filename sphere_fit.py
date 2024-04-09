#!/usr/bin/env python3
import rospy
from robot_vision_lectures.msg import XYZarray, SphereParams
from turtlesim.msg import Pose
import numpy as np


pos_msg = Pose()
pos_received = False

def pose_callback(data):
    global pos_msg, pos_received
    pos_msg = data
    pos_received = True

alpha = 0.2  # Filter gain, adjust as needed

def low_pass_filter(current_value, previous_value):
    return alpha * current_value + (1 - alpha) * previous_value

def fit_sphere(x, y, z):
    # Construct matrices A and B
    A = np.vstack((x, y, z, np.ones_like(x))).T
    B = x**2 + y**2 + z**2

    # Solve for P
    P, _, _, _ = np.linalg.lstsq(A, B, rcond=None)

    # Extract center coordinates
    x_c, y_c, z_c, _ = 0.5 * P

    # Calculate radius
    radius = np.sqrt(P[3] + x_c**2 + y_c**2 + z_c**2)
    return x_c, y_c, z_c, radius


def xyz_callback(data):
    global x_c_previous, y_c_previous, z_c_previous, radius_previous
    x = []
    y = []
    z = []
    for point in data.points:
        x.append(point.x)
        y.append(point.y)
        z.append(point.z)
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    x_c, y_c, z_c, radius = fit_sphere(x, y, z)
    x_c_filtered = low_pass_filter(x_c, x_c_previous)
    y_c_filtered = low_pass_filter(y_c, y_c_previous)
    z_c_filtered = low_pass_filter(z_c, z_c_previous)
    radius_filtered = low_pass_filter(radius, radius_previous)
    x_c_previous = x_c_filtered
    y_c_previous = y_c_filtered
    z_c_previous = z_c_filtered
    radius_previous = radius_filtered
    sphere_params_msg = SphereParams()
    sphere_params_msg.xc = x_c_filtered
    sphere_params_msg.yc = y_c_filtered
    sphere_params_msg.zc = z_c_filtered
    sphere_params_msg.radius = radius_filtered
    sphere_pub.publish(sphere_params_msg)

if __name__ == '__main__':
    rospy.init_node('sphere_fit', anonymous=True)
    xyz_sub = rospy.Subscriber('/xyz_cropped_ball', XYZarray, xyz_callback)
    sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=10)

    pos_sub = rospy.Subscriber('/turtle1/noisy_pose', Pose, pose_callback)
    pos_pub = rospy.Publisher('/turtle1/filtered_pose', Pose, queue_size=10)

    loop_rate = rospy.Rate(10)

    fil_in = 0.0
    fil_out = 5.5
    fil_gain = 0.05

    x_c_previous = 0.0
    y_c_previous = 0.0
    z_c_previous = 0.0
    radius_previous = 0.0

    while not rospy.is_shutdown():
        if pos_received:
            fil_in = pos_msg.x
            fil_out = fil_gain * fil_in + (1 - fil_gain) * fil_out
            pos_msg.x = fil_out
            pos_pub.publish(pos_msg)

        loop_rate.sleep()
