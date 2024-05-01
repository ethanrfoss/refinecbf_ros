#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))) # FIXME: Make it so that the directory is autoatically in search path
from template.teleop import Teleop


class JackalTeleop(Teleop):

    external_control_msg_type = Twist

    def __init__(self):
        
        self.control_space = rospy.get_param("~/env/control_space")
        super().__init__()
        

    def keys_to_control(self,keys):

        target_linear_vel = 0
        target_angular_vel = 0
        to_publish = False

        if 'w' in keys:
            target_linear_vel = (self.control_space["lo"][1]+self.control_space["hi"][1])/2
            to_publish = True
        elif 'a' in keys:
            target_angular_vel = self.control_space["lo"][0]/2
            to_publish = True
        elif 'd' in keys:
            target_angular_vel = self.control_space["hi"][0]/2
            to_publish = True
        elif 's' in keys:
            target_linear_vel = 0
            target_angular_vel = 0
            to_publish = True
        else:
            target_linear_vel = 0
            target_angular_vel = 0
            to_publish = False
        
        external_control = Twist()
        external_control.angular.z = target_angular_vel
        external_control.linear.x = target_linear_vel

        return external_control,to_publish
        

if __name__ == '__main__':
    rospy.init_node('jackal_teleoperation', anonymous=True)
    JackalTeleop()
    rospy.spin()
