#!/usr/bin/env python3

import rospy
import numpy as np
import jax.numpy as jnp
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

class Teleop:
    
    external_control_msg_type = None

    def __init__(self):

        
        # Publisher for Control array
        external_control_topic = rospy.get_param("~topics/robot_external_control")
        self.external_control_publisher = rospy.Publisher(external_control_topic, self.external_control_msg_type, queue_size=10)

        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

        # Run Teleop:
        while not rospy.is_shutdown():
            key = self.get_key()
            if (key == '\x03'):
                break
            external_control,to_publish = self.keys_to_control(key)
            if to_publish:
                self.external_control_publisher.publish(external_control)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def get_key(self):
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keys_to_control(self, keys):
        raise NotImplementedError("Must Be Subclassed")
