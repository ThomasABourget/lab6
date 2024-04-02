#!/usr/bin/env python3

import rospy
from robot_vision_lectures.msg import XYZarray, SphereParams
import numpy as np

# Define global variables for filtered values
filtered_x_c = 0.0
filtered_y_c = 0.0
filtered_z_c = 0.0
filtered_radius = 0.0

# Low-pass filter gains
x_c_gain = 0.1
y_c_gain = 0.1
z_c_gain = 0.1
radius_gain = 0.1

def fit_sphere(x, y, z):
    # Construct matrices A and B
    A = np.vstack((x, y, z, np.ones_like(x))).T
    B = x**2 + y**2 + z**2

    # Solve for P
    P, _, _, _ = np.linalg.lstsq(A, B, rcond=None)

    # Extract center coordinates
    x_c, y_c, z_c, _ = 0.5 * P

    # Calculate radius using the formula
    radius = np.sqrt(P[3] + x_c**2 + y_c**2 + z_c**2)
    
    return x_c, y_c, z_c, radius

def apply_low_pass_filter(new_val, filtered_val, gain):
    return gain * new_val + (1 - gain) * filtered_val

def xyz_callback(data):
    global filtered_x_c, filtered_y_c, filtered_z_c, filtered_radius

    x = []
    y = []
    z = []
    
    for point in data.points:
        x.append(point.x)
        y.append(point.y)
        z.append(point.z)

    # Convert to numpy arrays
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)

    # Fit sphere to the data
    x_c, y_c, z_c, radius = fit_sphere(x, y, z)

    # Apply low-pass filter to each parameter
    filtered_x_c = apply_low_pass_filter(x_c, filtered_x_c, x_c_gain)
    filtered_y_c = apply_low_pass_filter(y_c, filtered_y_c, y_c_gain)
    filtered_z_c = apply_low_pass_filter(z_c, filtered_z_c, z_c_gain)
    filtered_radius = apply_low_pass_filter(radius, filtered_radius, radius_gain)

    # Publish the parameters of the filtered sphere
    sphere_params_msg = SphereParams()
    sphere_params_msg.xc = filtered_x_c
    sphere_params_msg.yc = filtered_y_c
    sphere_params_msg.zc = filtered_z_c
    sphere_params_msg.radius = filtered_radius
    sphere_pub.publish(sphere_params_msg)  # Publishing should use sphere_pub

if __name__ == '__main__':
    rospy.init_node('sphere_fit', anonymous=True)
    xyz_sub = rospy.Subscriber('/xyz_cropped_ball', XYZarray, xyz_callback)
    sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Spin to receive messages
        rospy.spin()
