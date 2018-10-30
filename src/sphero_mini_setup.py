#!/usr/bin/env python3
import rospy
import sphero_mini

from std_msgs.msg import ColorRGBA, Int16

class sphero_setup:

    def __init__(self):
        self.prey_head = 0
        self.predator_head = 0
        self.prey = 0
        self.predator = 0
        self.prey_color_sub = rospy.Subscriber("/prey/set_color",ColorRGBA,self.prey_color)
        self.predator_color_sub = rospy.Subscriber("/predator/set_color",ColorRGBA,self.predator_color)
        # self.prey_odm_sub = rospy.Subscriber("/prey/odom",Odometry,self.prey_odom)
        self.prey_vel_sub = rospy.Subscriber("/prey/cmd_vel",Int16,self.prey_speed)
        # self.predator_odm_sub = rospy.Subscriber("/predator/odom",Odometry,self.predator_odom)
        self.predator_vel_sub = rospy.Subscriber("/predator/cmd_vel",Int16,self.predator_speed)
        self.prey_heading_sub = rospy.Subscriber("/prey/set_heading",Int16,self.prey_heading)
        self.predator_heading_sub = rospy.Subscriber("/predator/set_heading",Int16,self.predator_heading)

    def prey_color(self, data):
        self.prey.setLEDColor(red=int(data.r),green=int(data.g),blue=int(data.b))

    def predator_color(self, data):
        self.predator.setLEDColor(red=int(data.r),green=int(data.g),blue=int(data.b))

    def prey_speed(self, data):
        self.prey.roll(int(data.data),self.prey_head)

    def predator_speed(self, data):
        self.predator.roll(data.data,self.predator_head)

    def prey_heading(self, data):
        angle = data.data

        if angle < 0:
            angle += 360

        if angle > 360:
            angle -= 360

        self.prey_head = int(angle)

    def predator_heading(self, data):
        self.predator_head = data.data

def main():
    rospy.init_node('sphero_finder', anonymous=True)
    ic = sphero_setup()
    rospy.sleep(1)
    ic.prey = sphero_mini.sphero_mini('FD:62:56:A7:AB:2B', verbosity = 2)
    # ic.predator = sphero_mini.sphero_mini('FD:62:56:A7:AB:2B', verbosity = 1)
    print("Connected")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
