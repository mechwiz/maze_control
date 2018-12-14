#!/usr/bin/env python3
import rospy
import sphero_mini

from std_msgs.msg import ColorRGBA, Int16

class predatormini_setup:

    def __init__(self):
        self.predator_head = 0
        self.predator = 0
        self.predator_color_sub = rospy.Subscriber("set_color",ColorRGBA,self.predator_color)
        self.predator_vel_sub = rospy.Subscriber("cmd_vel",Int16,self.predator_speed)
        self.predator_heading_sub = rospy.Subscriber("set_heading",Int16,self.predator_heading)

    def predator_color(self, data):
        self.predator.setLEDColor(red=int(data.r),green=int(data.g),blue=int(data.b))

    def predator_speed(self, data):
        self.predator.roll(int(data.data),self.predator_head)

    def predator_heading(self, data):
        angle = data.data

        if angle < 0:
            angle += 360

        if angle > 360:
            angle -= 360

        self.predator_head = int(angle)

def main():
    rospy.init_node('predatormini_setup', anonymous=False)
    ic = predatormini_setup()
    rospy.sleep(1)
    bt_addr = rospy.get_param('predator_mini_setup/bt_address')
    ic.predator = sphero_mini.sphero_mini(bt_addr, verbosity = 1)
    # ic.predator = sphero_mini.sphero_mini('F0:93:98:6B:98:79', verbosity = 1)
    print("Connected to Predator")

    rospy.sleep(1)
    r,g,b = rospy.get_param('predator_mini_setup/color')
    ic.predator.setLEDColor(red=int(r),green=int(g),blue=int(b))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
