#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import time
from cv_bridge import CvBridge, CvBridgeError
import pickle
import os

bridge = CvBridge()
image_left = []
image_right = []

def image_left_callback(data):
    global image_left
    global bridge
    image_left = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')


def image_right_callback(data):
    global image_right
    global bridge
    image_right = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    # image_right = data

def collect_images(site, vel_lin, vel_ang):
    global image_left
    global image_right
    global bridge

    rate = rospy.Rate(30)
    data_img = []
    for i in range(30):
        combined_image = np.hstack((image_left, image_right))
        if len(data_img) == 0:
            data_img = [combined_image]
        else:
            data_img.append(combined_image)
        rate.sleep()
    filename = r'../simdata/office_' + str(site) + '_' + str(vel_lin) + '_' + str(vel_ang) + '.p'
    pickle.dump(data_img, open(filename, 'wb'))

def vel_trial(vel_msg, vel_lin, vel_ang, vel_pub, site):
    

    rospy.sleep(1)
    vel_msg.linear.x = vel_lin
    vel_msg.angular.z = vel_ang
    vel_pub.publish(vel_msg)
    rospy.sleep(0.5)
    
    collect_images(site, vel_lin, vel_ang)

    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)
    rospy.sleep(0.5)

def angle_trial(vel_msg, speeds, vel_pub, state_reset, set_state, label, site):
    for i in range(len(speeds)):
        rospy.loginfo(label + ' - Vel ' + str(i+1) + '/' + str(2*len(speeds)))
        vel_trial(vel_msg, speeds[i], 0.0, vel_pub, site)
        _ = set_state(state_reset)
    for i in range(len(speeds)):
        rospy.loginfo(label + ' - Vel ' + str(i+1+len(speeds)) + '/' + str(2*len(speeds)))
        vel_trial(vel_msg, 0.0, speeds[i], vel_pub, site)
        _ = set_state(state_reset)

def run_node():
    global image_left
    global image_right
    # rospy.loginfo('Creating Velocity Publisher')
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    # rospy.sleep(30)
    #input('Gazebo Ready? Press Enter to Continue.')
    # rospy.init_node('sns_controller', anonymous=True)
    model_coords = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    num_sites = 199

    speeds = np.array([-0.5, -0.45, -0.4, -0.35, -0.3, -0.25, -0.2, -0.15, -0.1,
        0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5])
    w = [1, 1, 0 ,-1]
    z = [0, 1, 1, 1]
    angs = ['0', '90', '180', '270']

    flywheel_state = ModelState()
    flywheel_state.model_name = 'robot'
    flywheel_state.pose.position.z = 0.05
    flywheel_state.pose.orientation.x = 0
    flywheel_state.pose.orientation.y = 0
    t_global = time.time()
    for i in range(num_sites):
        t_start = time.time()
        rospy.loginfo(type(image_left))
        site_label = 'Site ' + str(i+1) + '/' + str(num_sites)
        # rospy.loginfo(site_label)
        site_name = 'unit_sphere_' + str(i)
        site_coords = model_coords(site_name, 'link')
        flywheel_state.pose.position.x =  site_coords.pose.position.x-0.5
        flywheel_state.pose.position.y = site_coords.pose.position.y+0.5

        for j in range(len(w)):
            label = site_label + ' - ' + angs[j] + ' Degrees'
            flywheel_state.pose.orientation.w = w[j]
            flywheel_state.pose.orientation.z = z[j]
            _ = set_state(flywheel_state)
            angle_trial(vel_msg, speeds, vel_pub, flywheel_state, set_state, label, i)
        rospy.loginfo(site_label + 'Finished in ' + str(time.time()-t_start) + 'sec')
        rospy.loginfo('Total Time: ' + str(time.time()-t_global) + 'sec')

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     pass
        # vel_pub.publish(vel_msg)
        # rospy.loginfo('Linear Velocity: %.2f, Angular Velocity: %.2f'%(speed,heading))
        # rospy.loginfo(scan_angles)


def wait_for_gazebo():
    # Wait for the Gazebo service to be available
    rospy.loginfo("Waiting for Gazebo services...")
    rospy.wait_for_message('/camera_left/image_raw', rospy.AnyMsg)
    rospy.loginfo("Gazebo is ready.")


if __name__ == '__main__':
    try:
        rospy.loginfo('Initializing Node')
        rospy.init_node('flywheel')
        # rospy.loginfo('Creating Scan Subscriber')
        # rospy.Subscriber('/front/scan', numpy_msg(LaserScan),scan_callback)
        # run_node()
        wait_for_gazebo()
        rospy.loginfo('Initializing Subscribers')
        rospy.Subscriber('/camera_left/image_raw', Image, image_left_callback)
        rospy.Subscriber('/camera_right/image_raw', Image, image_right_callback)
        rospy.loginfo(os.getcwd())
        run_node()
    except rospy.ROSInterruptException: pass
