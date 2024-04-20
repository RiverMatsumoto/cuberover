#!/usr/bin/env python3
import rospy
import threading
import serial
import serial.tools.list_ports
import subprocess
from std_msgs.msg import UInt32, String

class SerialHandler:
    def __init__(self, port, baudrate):
        self.serial_port = serial.Serial(port, baudrate)
        self.read_thread = threading.Thread(target=self.read)
        self.read_thread.daemon = True
        self.read_thread.start()
        self.response_pub = rospy.Publisher('/arm/log', String, queue_size=5)
    
    def write(self, msg: str):
        self.serial_port.write(msg.encode())

    def read(self):
        while not rospy.is_shutdown():
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().strip().decode()
                self.response_pub.publish(String(data))
    
    def close(self):
        self.serial_port.close()
                
def joint1_cb(msg: UInt32, arduino_serial: SerialHandler):
    arduino_serial.write(f'positionm1 128 1500 1500 1500 {msg.data}')


def find_arduino_usb():
    try:
        # This command lists USB devices and grep for Arduino
        result = subprocess.run(['sudo dmesg | grep Arduino -i -A5'], shell=True, text=True, capture_output=True).stdout
    except Exception as e:
        print("Failed to run system command:", str(e))
    lines = result.splitlines()
    lines = [''.join(map(str, line.split(']')[1:])) for line in lines]
    usb_identifier = lines[0].split(' ')[2]
    for line in lines:
        if 'ttyACM' in line and usb_identifier in line:
            return '/dev/' + line.split(' ')[3][:-1] # should return in form ttyACM#, -1 for colon ":" at end

if __name__ == "__main__":

    rospy.init_node('arm_arduino_hwin_node')
    arduino_usb_path = find_arduino_usb()
    if arduino_usb_path:
        rospy.loginfo("Arduino devices found on the following ports: " + arduino_usb_path)
    else:
        rospy.logerr("No Arduino devices were found. Exiting")
        rospy.signal_shutdown("No Arduino devices were found. Exiting")
        exit(1)


    port = rospy.get_param('/arm/config/serial_port', arduino_usb_path)
    baudrate = rospy.get_param('/arm/config/baudrate', 57600) 

    rospy.loginfo(f'Connecting to arduino over serial port: {arduino_usb_path}')
    arduino_serial = SerialHandler(port, baudrate)

    rospy.Subscriber('/arm/joint1', UInt32, joint1_cb, arduino_serial, queue_size=5)
    rospy.spin()
    arduino_serial.close()