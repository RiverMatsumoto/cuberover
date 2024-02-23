#!/usr/bin/env python3

from copy import deepcopy
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class JoyControls:
    def __init__(self):
        rospy.init_node("joy_controls_node")
        rospy.loginfo("started joy controls")
        self.toggle_light_pub = rospy.Publisher("cuberover/toggle_light", Empty, queue_size=20)
        self.prev_buttons = tuple([0] * 20)
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=20)

        rospy.spin()

    def button_started(self, curr_state, prev_state):
        return curr_state == 1 and prev_state == 0

    def joy_callback(self, msg):
        if self.button_started(msg.buttons[0], self.prev_buttons[0]):
            self.toggle_light_pub.publish(Empty())
        self.prev_buttons = msg.buttons
        
def main():
    JoyControls()

if __name__ == "__main__":
    main()

