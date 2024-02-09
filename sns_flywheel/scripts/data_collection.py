import rospy
import numpy as np
import pickle
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JoyFeedbackArray, JoyFeedback, Image
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError

ang_max = 0.5
mode_manual = True
joy_axes = [0.0, 0.0]
start_button = [0]
imu_data = [0.0, 0.0, 0.0]
bridge = CvBridge()
image = []

def new_folder(path):
    folderpath = os.path.join(os.getcwd(),path)
    if not os.path.exists(folderpath):
        os.makedirs(folderpath)
        if os.path.exists(folderpath):
            return (True, folderpath)
    return (False, folderpath)

def send_vibration(intensity):
    feedback_msg = JoyFeedbackArray()
    #feedback_msg.controller_id = 0

    feedback = JoyFeedback()
    feedback.type = JoyFeedback.TYPE_RUMBLE
    feedback.id = 0
    feedback.intensity = intensity
    feedback_msg.array.append(feedback)

    pub_rumble.publish(feedback_msg)

def image_callback(data):
    global image
    global bridge
    image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def imu_callback(data):
    global imu_data
    imu_data[0] = data.angular.x
    imu_data[1] = data.angular.y
    imu_data[2] = data.angular.z

def joy_callback(data):
    global mode_manual, joy_axes, start_button
    start_button  = data.buttons[7]
    joy_axes[0] = data.axes[1]
    joy_axes[1] = data.axes[0]
    if start_button == 1:
        if mode_manual:
            mode_manual = False
            rospy.loginfo("Switching to data collection")
        else:
            mode_manual = True
            #send_vibration(0.5)
            rospy.loginfo("Aborting data collection, switching to manual control")
            #rospy.sleep(2)
            #send_vibration(0.0)

def teleoperate():
    rospy.loginfo('Teleoperate')
    global joy_axes

    while not rospy.is_shutdown() and mode_manual:
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5*joy_axes[0]
        cmd_vel.angular.z = 1*joy_axes[1]
        cmd_vel_pub.publish(cmd_vel)

def collect_data():
    global imu_data, image, mode_manual
    rospy.loginfo("Collecting Data")
    rate = rospy.Rate(30)
    data_ang = []
    data_imu = []
    data_img = []
    cmd_vel = Twist()
    angs = np.array([-1, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1])*ang_max
    #angs = np.array([-0.1])
    directory = str(rospy.get_rostime().secs)
    #f = new_folder(directory)
    #if f[0]:
    #    rospy.loginfo('Folder Successfully created')
    os.makedirs(r'../trials/trial'+directory)
    for i in range(len(angs)):
        if rospy.is_shutdown() or mode_manual:
            break
        else:
            rospy.loginfo('Speed %i/%i'%(i+1,len(angs)))
            cmd_vel.angular.z = angs[i]
            cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(5)
            data_ang = []
            data_imu = []
            data_img = []
            for j in range(90):
                if rospy.is_shutdown() or mode_manual:
                    break
                else:
                    if len(data_ang) == 0:
                        data_ang = [angs[i]]
                        data_imu = [imu_data]
                        data_img = [image]
                    else:
                        data_ang.append(angs[i])
                        data_imu.append(imu_data)
                        data_img.append(image)
                    rate.sleep()
            data = {'cmd_vel': data_ang, 'imu': data_imu, 'image': data_img}
            filename = r'../trials/trial' + directory + '/' + str(rospy.get_rostime().secs) + '.p'
            pickle.dump(data, open(filename, 'wb'))
    cmd_vel.angular.z = 0.0
    cmd_vel.linear.x = -0.05
    cmd_vel_pub.publish(cmd_vel)
    rospy.sleep(1)
    cmd_vel.linear.x = 0.05
    cmd_vel_pub.publish(cmd_vel)
    rospy.sleep(1)
    cmd_vel.linear.x = 0.0
    cmd_vel_pub.publish(cmd_vel)

    mode_manual = True

    #data = {'cmd_vel': data_ang, 'imu': data_imu, 'image': data_img}
    #filename =r'/home/sns/catkin_ws/src/sns_ctrl/trials/trial' + str(rospy.get_rostime().secs) + '.p'
    #pickle.dump(data, open(filename, 'wb'))
    rospy.loginfo('Trial done, returning to manual')


#if __name__ == '__ main__':
print('Begin')
rospy.init_node('robot_ctrl')
rospy.loginfo('Initializing publisher')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub_rumble = rospy.Publisher('/joy/set_feedback', JoyFeedbackArray, queue_size=10)
rospy.loginfo('Initializing subscriber')
rospy.Subscriber('joy', Joy, joy_callback)
rospy.Subscriber('imu', Twist, imu_callback)
rospy.Subscriber('camera/combined', Image, image_callback)
rospy.loginfo('Press \'start\' to switch between teleoperation and data collection')
rospy.loginfo(os.getcwd())

while not rospy.is_shutdown():
    rospy.loginfo('In main loop')
    if mode_manual:
        teleoperate()
    else:
        collect_data()

