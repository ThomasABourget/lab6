#!/usr/bin/env python3
#Current Thought Process: So i had this working I think. But my wonderfull computer blue screened and I didnt save the working verstion and was not able to get back to the orignal state that worked. So for
#now I will turn this version in that I had that I belive also worked. I will work to get this working.



import rospy
from robot_vision_lectures.msg import XYZarray, SphereParams
from turtlesim.msg import Pose
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

# Parameters for inertia dynamics
mass = 1.0  # Mass of the object
damping = 1.0  # Damping coefficient

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

def pose_callback(data):
    global filtered_x_c, filtered_y_c, filtered_z_c, filtered_radius

    # Get the current position
    x = data.x
    y = data.y
    z = data.z

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

    # Add a subscriber to read the noisy position information
    pos_sub = rospy.Subscriber('/turtle1/noisy_pose', Pose, pose_callback)

    # Add a publisher for publishing the filtered position
    sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=10)

    # Set a 10 Hz frequency for this loop
    loop_rate = rospy.Rate(10)

    # Run the control loop
    while not rospy.is_shutdown():
        rospy.spin()
