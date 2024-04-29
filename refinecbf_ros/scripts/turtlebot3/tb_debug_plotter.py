#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from refinecbf_ros.msg import ValueFunctionMsg, Array, Obstacles
from refinecbf_ros.config import Config
from geometry_msgs.msg import Twist
import numpy as np
import jax.numpy as jnp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))) # FIXME: Make it so that the directory is autoatically in search path

class TurtlebotDebugPlotter():

    def __init__(self):
        
        # Config:
        config = Config(hj_setup=True)

        self.state_safety_idis = config.safety_states
        self.grid = config.grid
        self.state_domain = config.state_domain
        self.control_space = config.control_space

        # Setup figure:
        self.fig,(self.ax1,self.ax2) = plt.subplots(1,2)
        # div = make_axes_locatable(self.ax)
        # self.cax = div.append_axes('right', '5%', '5%')
        # self.cont = self.ax1.contour([],[],[])
        # self.cont_sdf = self.ax1.contour([],[],[])
        self.ln1, = self.ax1.plot([],[],'r-')
        self.x_data,self.y_data = [],[]

        # Control lInes:
        self.ln3, = self.ax2.plot([],[],'r-')
        self.external_control_wt_data, self.external_control_w_data = [],[]
        self.ln4, = self.ax2.plot([],[],'r--')
        self.external_control_vt_data,self.external_control_v_data = [],[]
        self.ln5, = self.ax2.plot([],[],'g-')
        self.safe_control_wt_data,self.safe_control_w_data = [],[]
        self.ln6, = self.ax2.plot([],[],'g--')
        self.safe_control_vt_data,self.safe_control_v_data = [],[]

        # Init Time
        self.init_time = rospy.Time.now().to_sec()

        self.vf = None
        self.sdf = None
        self.robot_state = None
        self.safe_control = None
        self.external_control = None
        self.cont_sdf = None
        self.cont = None

        # Subscriber for SDF and VF:
        self.vf_update_method = rospy.get_param("~vf_update_method")
        sdf_update_topic = rospy.get_param("~topics/sdf_update")
        vf_topic = rospy.get_param("~topics/vf_update")

        if self.vf_update_method == "pubsub":
            self.sdf_update_sub = rospy.Subscriber(
                sdf_update_topic, ValueFunctionMsg, self.callback_sdf_pubsub
            )
            self.vf_update_sub = rospy.Subscriber(vf_topic, ValueFunctionMsg, self.callback_vf_pubsub)
        elif self.vf_update_method == "file":
            self.sdf_update_sub = rospy.Subscriber(sdf_update_topic, Bool, self.callback_sdf_file)
            self.vf_update_sub = rospy.Subscriber(vf_topic, Bool, self.callback_vf_file)
        else:
            raise NotImplementedError("{} is not a valid vf update method".format(self.vf_update_method))

        # Subscriber for Robot State:
        cbf_state_topic = rospy.get_param("~topics/cbf_state")
        state_sub = rospy.Subscriber(cbf_state_topic, Array, self.callback_state)

        # Set up safe control subscriber
        self.cbf_safe_control_topic = rospy.get_param("~topics/cbf_safe_control")
        rospy.Subscriber(self.cbf_safe_control_topic, Array, self.callback_safe_control)

        # Set up external control subscriber
        self.robot_external_control_topic = rospy.get_param("~topics/robot_external_control")
        rospy.Subscriber(self.robot_external_control_topic, Twist, self.callback_external_control)


    def callback_sdf_pubsub(self, sdf_msg):
        self.sdf = np.array(sdf_msg.vf).reshape(self.grid.shape)

    def callback_sdf_file(self, sdf_msg):
        if not sdf_msg.data:
            return
        self.sdf = np.array(np.load("./sdf.npy")).reshape(self.grid.shape)

    def callback_vf_pubsub(self, vf_msg):
        self.vf = np.array(vf_msg.vf).reshape(self.grid.shape)
    
    def callback_vf_file(self, vf_msg):
        if not vf_msg.data:
            return
        self.vf = np.array(np.load("./vf.npy")).reshape(self.grid.shape)

    def callback_state(self,state_msg):
        self.robot_state = jnp.reshape(np.array(state_msg.value)[self.state_safety_idis], (-1, 1)).T
        self.x_data.append(self.robot_state[0][0])
        self.y_data.append(self.robot_state[0][1])

    def callback_safe_control(self,control_msg):
        self.safe_control = np.array(control_msg.value)
        self.safe_control_vt_data.append(rospy.Time.now().to_sec()-self.init_time)
        self.safe_control_wt_data.append(rospy.Time.now().to_sec()-self.init_time)
        self.safe_control_v_data.append(self.safe_control[1])
        self.safe_control_w_data.append(self.safe_control[0])

    def callback_external_control(self,twist_msg):
        self.external_control = np.array([twist_msg.angular.z,twist_msg.linear.x])
        self.external_control_vt_data.append(rospy.Time.now().to_sec()-self.init_time)
        self.external_control_wt_data.append(rospy.Time.now().to_sec()-self.init_time)
        self.external_control_v_data.append(self.external_control[1])
        self.external_control_w_data.append(self.external_control[0])

    def initialize_plot(self):
        self.ax1.set_xlim(self.state_domain['lo'][0],self.state_domain['hi'][0])
        self.ax1.set_ylim(self.state_domain['lo'][1],self.state_domain['hi'][1])
        self.ax1.set_aspect('equal')
        self.ax2.set_xlim(0,50)
        self.ax2.set_ylim(min(self.control_space['lo'][0],self.control_space['lo'][1]),min(self.control_space['hi'][0],self.control_space['hi'][1]))

    def update_plot(self,frame):
        
        if self.sdf is not None:
            sdf = self.sdf[:, :, self.grid.nearest_index(self.robot_state)[0][2]].T
            if self.cont_sdf is not None:
                self.cont_sdf.collections[0].remove()
            self.cont_sdf = self.ax1.contour(self.grid.coordinate_vectors[0], self.grid.coordinate_vectors[1],sdf, levels=[0], color='k',linewidths=4)
            # self.cax.cla()
            # self.fig.colorbar(cf,cax=self.cax)

        if self.vf is not None:
            # vf = self.vf[:, :, self.grid.nearest_index(self.robot_state)[0][2]].T
            vf = self.vf[:, :, self.grid.shape[2]//2].T
            vmax = np.abs(vf).max()
            if self.cont is not None:
                self.cont.collections[0].remove()
            # cf = self.ax.contourf(self.grid.coordinate_vectors[0], self.grid.coordinate_vectors[1], vf, vmax=vmax,vmin=-vmax)
            self.cont = self.ax1.contour(self.grid.coordinate_vectors[0], self.grid.coordinate_vectors[1],vf, levels=[0], colors='green')
            

        if self.robot_state is not None:
            self.ln1.set_data(self.x_data,self.y_data)

        if self.safe_control is not None:
            self.ln5.set_data(self.safe_control_wt_data,self.safe_control_w_data)
            self.ln6.set_data(self.safe_control_vt_data,self.safe_control_v_data)

        if self.external_control is not None:
            self.ln3.set_data(self.external_control_wt_data,self.external_control_w_data)
            self.ln4.set_data(self.external_control_vt_data,self.external_control_v_data)



if __name__ == '__main__':
    rospy.init_node('tb_visualization', anonymous=True)
    TB = TurtlebotDebugPlotter()

    ani = FuncAnimation(TB.fig,TB.update_plot,init_func=TB.initialize_plot,interval=100)
    plt.show(block=True)
