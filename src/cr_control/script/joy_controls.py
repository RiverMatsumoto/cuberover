#!/usr/bin/env python3

from copy import deepcopy
import rospy
from std_msgs.msg import Empty, Float64, Int8
from sensor_msgs.msg import Joy

xbox_btn_names = ('a', 'b', 'y', 'x', 'lb', 'rb', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')
bitdo_btn_names = ('a', 'b', 'empty' 'x', 'y', 'empty', 'lb', 'rb', 'lt', 'rt', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')

class JoyControls:
    def __init__(self):
        rospy.init_node("joy_controls_node")
        rospy.loginfo("started joy controls")
        self.toggle_light_pub = rospy.Publisher("cuberover/toggle_light", Empty, queue_size=5)
        self.servo_pub = rospy.Publisher("cuberover/servo_angle", Float64, queue_size=5)
        self.la_pub = rospy.Publisher("cuberover/linear_actuator", Int8, queue_size=5)
        self.prev_buttons = tuple([0] * 20)
        self.prev_dpad = tuple([0] * 10)
        self.servo_angle = 0

        controller = rospy.get_param('cuberover/config/controller_type')
        if controller == '8bitdo':
            self.btn_names = ('a', 'b', 'empty', 'x', 'y', 'empty', 'lb', 'rb', 'lt', 'rt', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')
            self.axes_names = ('left_x', 'left_y', 'right_x', 'right_y', 'rt', 'lt', 'dpad_x', 'dpad_y')
        elif controller == 'xbox':
            self.btn_names = ('a', 'b', 'x', 'y', 'lb', 'rb', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')
            self.axes_names = ('left_x', 'left_y', 'right_x', 'right_y', 'rt', 'lt', 'dpad_x', 'dpad_y')
            
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=10)
        rospy.spin()

    def joy_callback(self, msg):
        # check that button was initially pressed versus released
        dpad = dict(zip(self.axes_names[6:], msg.axes[6:]))
        pressed = dict(zip(self.btn_names, msg.buttons))
        started = dict(zip(self.btn_names, (btn[0] == 1 and btn[1] == 0 for btn in zip(msg.buttons, self.prev_buttons))))
        released = dict(zip(self.btn_names, (btn[0] == 0 and btn[1] == 1 for btn in zip(msg.buttons, self.prev_buttons))))
        axes = dict(zip(self.axes_names, msg.axes))
        dpad_started = dict(zip(self.axes_names[6:], (dpad_temp[0] != 0 and dpad_temp[1] == 0 for dpad_temp in zip(msg.axes[6:], self.prev_dpad))))
        if started['x']:
            self.toggle_light_pub.publish(Empty())
        if pressed['b']:
            if axes['left_y'] > 0:
                self.la_pub.publish(Int8(1))
            elif axes['left_y'] < 0:
                self.la_pub.publish(Int8(-1))
            else:
                self.la_pub.publish(Int8(0))
        else:
            self.la_pub.publish(Int8(0))
        if dpad_started['dpad_y']:
            if dpad['dpad_y'] > 0:
                self.servo_pub.publish(Float64(90.0))
            if dpad['dpad_y'] < 0:
                self.servo_pub.publish(Float64(0.0))
        
        
        self.prev_dpad = msg.axes[6:]
        self.prev_buttons = msg.buttons
        
def main():
    JoyControls()

if __name__ == "__main__":
    main()

# 0 A
# 1 B
# 2 X
# 3 Y
# 4 LB
# 5 RB
# 6 back
# 7 start
# 8 power
# 9 Button stick left
# 10 Button stick right

# Table of index number of /joy.axes:
# Axis name on the actual controller
# 0 Left/Right Axis stick left
# 1 Up/Down Axis stick left
# 2 Left/Right Axis stick right
# 3 Up/Down Axis stick right
# 4 RT
# 5 LT
# 6 cross key left/right
# 7 cross key up/down