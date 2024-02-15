#!/usr/bin/env python

from tkinter import W
import sympy as sp
import numpy as np
import math
import time
import rospy
import sys, select, os
import scipy.sparse as sparse
from scipy.integrate import ode
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import *
from sensor_msgs.msg import Joy, JointState, Imu
from sympy.physics.mechanics import *
import matplotlib.pyplot as plt
from pyMPC.mpc import MPCController
import dynamics

def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")

def quaternion_to_euler_angle(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    
    ysqr = y * y
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)
    
    return X, Y, Z

def print_graph():
    
    plt.subplot(221);
    plt.plot(sec_store, u_store)
    plt.xlabel('sec[s]')
    plt.ylabel('torque[Nm]')
    plt.title('Input torque [Nm]')
    plt.legend([r'$\tau$[Nm]'],loc='best')
    plt.grid(True, axis='y')
    
    # plt.subplot(332);
    # plt.plot(sec_store, vel_store)
    # plt.xlabel('sec[s]')
    # plt.ylabel('Wheel Velocity[rad/s]')
    # plt.title('Wheel Velocity [rad/s]')
    # plt.legend([r'$\theta_w$[rad/s]', r'$\theta_{wdes}$[rad/s]'],loc='best')
    # plt.grid(True, axis='y')
    
    plt.subplot(222);
    plt.plot(sec_store, deg_store)
    plt.xlabel('sec[s]')
    plt.ylabel('tilt[deg]')
    plt.title('Tilt angle[deg]')
    plt.ylim(-15,15)
    plt.xlim(0,15)
    plt.legend([r'$\theta$[deg]'],loc='best')
    plt.grid(True, axis='y')
    plt.show()
    
def steering_control(v, w):
    d = 0.175
    r = 0.115
    
    fast = (d*w + v)/r
    slow = (v-d*w)/r
    
    return slow, fast

def steering_control_left(v, w):
    d = 0.175
    r = 0.115
    
    fast = (d*w + v)/r
    slow = (v-d*w)/r
    # print('fast: ', fast)
    # print('slow: ', slow)
    
    return slow, fast

def steering_control_right(v, w):
    d = 0.175
    r = 0.115
    
    slow = (d*w + v)/r
    fast = (v-d*w)/r
    # print('fast: ', fast)
    # print('slow: ', slow)
    
    return slow, fast
    
def joy_callback(msg):
    global left_cmd_vel
    global right_cmd_vel
    
    # V = 1      # linear velocity
    # w = 1.2  #yaw angular velocity
    
    V = msg.axes[1] * 2
    w = msg.axes[2] * 2
    
    if msg.axes[2] == 0:
        left_cmd_vel, right_cmd_vel = steering_control(V, w)
    
    elif msg.axes[2] > 0: # Rotate to the left
        slow, fast = steering_control_left(V, w)
        left_cmd_vel = slow
        right_cmd_vel = fast
        
    elif msg.axes[2] < 0: # Rotate to the right
        slow, fast = steering_control_right(V, w)
        left_cmd_vel = fast
        right_cmd_vel = slow
    
    print(w)

def wheel_callback(msg):
    global wheel_left_pos
    global wheel_left_vel
    global wheel_right_pos
    global wheel_right_vel
    
    wheel_left_pos= msg.position[0]
    wheel_left_vel = msg.velocity[0]
    wheel_right_pos= msg.position[1]
    wheel_right_vel = msg.velocity[1]
    
def imu_callback(msg):
    global pitch_ang
    global pitch_vel
    
    X, Y, Z = quaternion_to_euler_angle(msg.orientation)
    
    pitch_ang= -Y
    # print(msg.angular_velocity.y)
    pitch_vel = -msg.angular_velocity.y
    
#######################################################################################################

# x_next = dynamics.get_x_next()

T_w1, T_w2 = sp.symbols('T_w1, T_w2')
theta_P, theta_w1, theta_w2 = dynamicsymbols('theta_P, theta_w1, theta_w2')

Ac = np.array([[0,0,0,1],
              [-37.23668979,0,0,0],
              [-37.23668979,0,0,0],
              [52.35843714,0,0,0]])

Bc = np.array([[0,0],
              [9.54194701,-0.64401618],
              [-0.64401618,9.54194701],
              [-3.2045756, -3.2045756]])

# Ac = np.array([[0,0,0,1,0,0],
#               [0,0,0,0,1,0],
#               [0,0,0,0,0,1],
#               [0,0,-25.03210719,0,0,0],
#               [0,0,-25.03210719,0,0,0],
#               [0,0,29.1477018,0,0,0]])

# Bc = np.array([[0,0],
#               [0,0],
#               [0,0],
#               [1.6579922,0.00699069],
#               [0.00699069,1.6579922],
#               [-0.46024303, -0.46024303]])

[nx, nu] = Bc.shape # number of states and number or inputs


# Simple forward euler discretization
Ts = 0.01
Ad = np.eye(nx) + Ac*Ts
Bd = Bc*Ts

RAD2DEG = 180/np.pi
DEG2RAD = np.pi/180

loop_cnt = 0

deg_store = []
sec_store = []
u_store = []
vel_store = []
tilt_store = []

############################################################################################
 
if __name__ == '__main__':
    try:                
        
        rospy.init_node('Balancing_Controller', anonymous=False) 

        left_cmd_vel = 0
        right_cmd_vel = 0
        wheel_left_vel = 0
        wheel_right_vel = 0
        pitch_ang = 0
        pitch_vel = 0
        wheel_vel = 0
        sec = 0

        pub_rw = rospy.Publisher('/balancing_robot/right_wheel/command', Float64, queue_size=100)
        pub_lw = rospy.Publisher('/balancing_robot/left_wheel/command', Float64, queue_size=100)
        
        rospy.Subscriber('joy', Joy, joy_callback)
        rospy.Subscriber("/balancing_robot/joint_states", JointState, wheel_callback)
        rospy.Subscriber("/imu", Imu, imu_callback)
        
        
        # MPC reference input and states (set-points)
        xref = np.array([0, 0, 0, 0]) # reference state
        uref = np.array([0, 0])    # reference input
        uminus1 = np.array([0, 0])     # input at time step negative one - used to penalize the first delta u at time instant 0. Could be the same as uref.

        # Constraints
        xmin = np.array([-0.174444444, -80, -80, -25])
        xmax = np.array([ 0.174444444, 80, 80, 25])

        umin = np.array([-30, -30])
        umax = np.array([30, 30])

        # Constraints input variation with respect to previous sample
        Dumin = np.array([-40, -40])
        Dumax = np.array([40, 40])
        
        Qx = sparse.diags([300, 10, 10, 300])   # Quadratic cost for states x0, x1, ..., x_N-1
        QxN = sparse.diags([300, 10, 10, 300])  # Quadratic cost for xN
        Qu = 1 * sparse.eye(2)        # Quadratic cost for u0, u1, ...., u_N-1
        QDu = 1 * sparse.eye(2)       # Quadratic cost for Du0, Du1, ...., Du_N-1

        x0 = np.array([pitch_ang,wheel_left_vel,wheel_right_vel,pitch_vel])# initial state
        t0 = 0
        
        Np = 150

        K = MPCController(Ad,Bd,Np=Np, x0=x0,xref=xref,uminus1=uminus1,
                Qx=Qx, QxN=QxN, Qu=Qu,QDu=QDu,
                xmin=xmin,xmax=xmax,umin=umin,umax=umax,Dumin=Dumin,Dumax=Dumax)
        K.setup() # this initializes the QP problem for the first step
        uMPC = uminus1
        
        rate = rospy.Rate(1000)
        gazebo_setting()
        
        cur_time = time.time()    
        # sec_time = time.time()           
         
        while True:
            last_time = cur_time
            cur_time = time.time()
            # sec_cur_time = time.time()
            dt = cur_time - last_time 
            sec =  dt + sec
            
            K.update(x0, uMPC, xref) # update with measurement
            uMPC = K.output() # MPC step (u_k value)
            print('torque:  ', uMPC)
            
            pub_lw.publish(uMPC[0])
            pub_rw.publish(uMPC[1])
            
            # print(x0)
            # print(xref)
            # print('Pitch             (deg): ', pitch_ang*RAD2DEG)
            print('dt: ', dt, 'freq: ', 1/dt)
            print('==============================')
            
            deg_store.append(pitch_ang*RAD2DEG)
            u_store.append(uMPC[0])
            vel_store.append(wheel_vel)
            sec_store.append(sec)
            
            if loop_cnt == 10000:
                print_graph()
            
            rate.sleep()

            x0 = np.array([pitch_ang,wheel_left_vel,wheel_right_vel,pitch_vel])# initial state
            
            wheel_vel = (wheel_left_vel + wheel_right_vel) / 2
            xref = np.array([0,left_cmd_vel,right_cmd_vel,0])
            
            
            

            loop_cnt= loop_cnt + 1
                
    except rospy.ROSInterruptException:
        pass
