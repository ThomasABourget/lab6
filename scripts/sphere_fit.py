#!/usr/bin/env python3

#I dont know why this didnt work I kept getthing the same error over and over. I know I need to change the values of the x,y,z to a similiar format of the hw7 from 340 however 
#i dont know how to change that to the proper form with the context of this code.


import rospy
from robot_vision_lectures.msg import XYZarray, SphereParams
import numpy as np



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

    # Publish the parameters of the fitted sphere
    sphere_params_msg = SphereParams()
    sphere_params_msg.xc = x_c
    sphere_params_msg.yc = y_c
    sphere_params_msg.zc = z_c
    sphere_params_msg.radius = radius
    sphere_pub.publish(sphere_params_msg)  # Publishing should use sphere_pub
    print(sphere_params_msg)
if __name__ == '__main__':
    rospy.init_node('sphere_fit', anonymous=True)
    xyz_sub = rospy.Subscriber('/xyz_cropped_ball', XYZarray, xyz_callback)
    sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Spin to receive messages
        rospy.spin()

	
if __name__ == '__main__':
	rospy.init_node('sphere_fit', anonymous = True)
	xyz_sub = rospy.Subscriber('/xyz_cropped_ball', XYZarray, xyz_callback)
	sphere_pub = rospy.Publisher('/sphere_params',SphereParams, queue_size = 10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		# pause until the next iteration			
		rate.sleep()
	


