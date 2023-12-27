#!/usr/bin/env python3

import rospy
from cr_control.msg import trial_sample, wheel_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# Initialize the node and create publisher
rospy.init_node('collect_data_node', anonymous=True)
pub = rospy.Publisher('/trial_sample', trial_sample, queue_size=10)
last_wheel_data = None
last_zed_camera_data = None

# Define callback functions for the topics
def wheel_data_callback(msg):
    global last_wheel_data
    last_wheel_data = msg

def zed_camera_callback(msg):
    global last_zed_camera_data
    last_zed_camera_data = msg

def vicon_bridge_callback(msg):
    global pub
    
    trial_sample_msg = trial_sample()

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"

    trial_sample_msg.header = header
    trial_sample_msg.wheel_data = last_wheel_data
    trial_sample_msg.depth = last_zed_camera_data
    trial_sample_msg.vicon = msg
    
    #rospy.loginfo(trial_sample_msg)

    pub.publish(trial_sample_msg)


def listener():
    # Create subscribers for each topic
    rospy.Subscriber("/wheel/data", wheel_data, wheel_data_callback)
    rospy.Subscriber("/zedm/zed_node/depth/depth_registered", Image, zed_camera_callback)
    rospy.Subscriber("/vicon/cuberover/cuberover", TransformStamped, vicon_bridge_callback)

    rospy.Publisher("/trial_sample", trial_sample, queue_size=10)

    # Keep the node running until shut down
    rospy.spin()

if __name__ == '__main__':
    listener()





