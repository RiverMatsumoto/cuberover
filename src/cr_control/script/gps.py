#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import serial

def gps_publisher():
    pub = rospy.Publisher('gps_data', NavSatFix, queue_size=10)
    rospy.init_node('gps_node', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Adjust port and baud rate as necessary

    while not rospy.is_shutdown():
        data = ser.readline().decode('utf-8').strip()
        # Parse GPS data here and publish NavSatFix message
        # Example: latitude, longitude, and altitude are extracted from data
        # and assigned to gps_msg.latitude, gps_msg.longitude, and gps_msg.altitude
        gps_msg = NavSatFix()
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = altitude
        pub.publish(gps_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass

