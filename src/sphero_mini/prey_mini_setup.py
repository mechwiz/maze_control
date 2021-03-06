#!/usr/bin/env python3
import rospy
import sphero_mini

from std_msgs.msg import ColorRGBA, Int16

class preymini_setup:

    def __init__(self):
        self.prey_head = 0
        self.prey = 0
        self.prey_color_sub = rospy.Subscriber("set_color",ColorRGBA,self.prey_color)
        self.prey_vel_sub = rospy.Subscriber("cmd_vel",Int16,self.prey_speed)
        self.prey_heading_sub = rospy.Subscriber("set_heading",Int16,self.prey_heading)

    def prey_color(self, data):
        self.prey.setLEDColor(red=int(data.r),green=int(data.g),blue=int(data.b))

    def prey_speed(self, data):
        self.prey.roll(int(data.data),self.prey_head)

    def prey_heading(self, data):
        angle = data.data

        if angle < 0:
            angle += 360

        if angle > 360:
            angle -= 360

        self.prey_head = int(angle)

def main():
    rospy.init_node('preymini_setup', anonymous=False)
    ic = preymini_setup()
    rospy.sleep(1)
    bt_addr = rospy.get_param('prey_mini_setup/bt_address')
    ic.prey = sphero_mini.sphero_mini(bt_addr, verbosity = 1)
    # ic.prey = sphero_mini.sphero_mini('FA:43:D0:18:52:AA', verbosity = 1)
    print("Connected to Prey")
    rospy.sleep(1)
    r,g,b = rospy.get_param('prey_mini_setup/color')
    ic.prey.setLEDColor(red=int(r),green=int(g),blue=int(b))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
