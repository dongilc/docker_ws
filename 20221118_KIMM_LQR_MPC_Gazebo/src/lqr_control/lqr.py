#!/usr/bin/env python

from tkinter import W
import sympy as sp
import numpy as np
import math
import time
import rospy
import sys, select, os
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import *
from sensor_msgs.msg import Joy, JointState, Imu
import control
from sympy.physics.mechanics import *
import matplotlib.pyplot as plt

def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")

def pitch_K_gain():
    
    K, S, E = control.lqr(A, B, Q, R)
    
    return K, S, E

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
    plt.ylim(-20,20)
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
    
    
    V = msg.axes[1] * 1.5
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
    
    print(msg.axes)

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
    
    # print(msg)
#######################################################################################################
    
imu_acc_data = [0,0,0]
imu_vel_data = [0,0,0]
imu_ori_data = [0,0,0]

    
# A = np.array([[0,0,1,0],
#               [0,0,0,1],
#               [0,-40.04556334,0,0],
#               [0,34.32730981,0,0]])

# B = np.array([[0,0],
#               [0,0],
#               [1.32620141,1.32620141],
#               [-0.63177772, -0.63177772]])

# # q = [phi, theta, phi_dot, theta_dot]
# Q = sp.Matrix([ [0.3,    0,    0,    0],
#                 [0,    0.01,    0,    0],
#                 [0,    0,    0.1,    0],
#                 [0,    0,    0,    0.01]])

# R = sp.Matrix([ [1, 0],
#                 [0, 1]])

# A = np.array([[0,0,0,1,0,0],
#               [0,0,0,0,1,0],
#               [0,0,0,0,0,1],
#               [0,0,-25.03210719,0,0,0],
#               [0,0,-25.03210719,0,0,0],
#               [0,0,29.1477018,0,0,0]])

# B = np.array([[0,0],
#               [0,0],
#               [0,0],
#               [1.6579922,0.00699069],
#               [0.00699069,1.6579922],
#               [-0.46024303, -0.46024303]])

# # q = [theta_w1, theta_w2, theta_P, dtheta_w1, dtheta_w2, dtheta_P]
# Q = sp.Matrix([ [0.3,  0,    0,    0,    0,    0],
#                 [0,    0.3,  0,    0,    0,    0],
#                 [0,    0,    0.01, 0,    0,    0],
#                 [0,    0,    0,    0.1,  0,    0],
#                 [0,    0,    0,    0,    0.1,  0],
#                 [0,    0,    0,    0,    0,    0.01]])

# R = sp.Matrix([ [1, 0],
#                 [0, 1]])

A = np.array([[0,0,0,1,0,0],
              [0,0,0,0,1,0],
              [0,0,0,0,0,1],
              [0,0,-37.23668979,0,0,0],
              [0,0,-37.23668979,0,0,0],
              [0,0,52.35843714,0,0,0]])

B = np.array([[0,0],
              [0,0],
              [0,0],
              [9.54194701,-0.64401618],
              [-0.64401618,9.54194701],
              [-3.2045756, -3.2045756]])

# q = [theta_w1, theta_w2, theta_P, dtheta_w1, dtheta_w2, dtheta_P]
Q = sp.Matrix([ [0.3,  0,    0,    0,    0,    0],
                [0,    0.3,  0,    0,    0,    0],
                [0,    0,    200, 0,    0,    0],
                [0,    0,    0,    10,  0,    0],
                [0,    0,    0,    0,    10,  0],
                [0,    0,    0,    0,    0,    200]])

R = sp.Matrix([ [1, 0],
                [0, 1]])


K, S, E = pitch_K_gain()
Kl = K[0][2:6]
Kr = K[1][2:6]

# print(K)
# K1 = K[0][1:4]
# ss0 = [A, B, C, D]
# sys0 = control.ss(*[pl.array(mat_i).astype(float) for mat_i in ss0])
# sysc = sys0.feedback(K)

RAD2DEG = 180/np.pi

print('K: ', K)
# print('K1: ', K1)

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
        
        pub_rw = rospy.Publisher('/balancing_robot/right_wheel/command', Float64, queue_size=100)
        pub_lw = rospy.Publisher('/balancing_robot/left_wheel/command', Float64, queue_size=100)
        
        rospy.Subscriber('joy', Joy, joy_callback)
        rospy.Subscriber("/balancing_robot/joint_states", JointState, wheel_callback)
        rospy.Subscriber("/imu", Imu, imu_callback)

        rate = rospy.Rate(100)
        gazebo_setting()
        
        cur_time = time.time()    
        sec_time = time.time()           
         
        while True:

            last_time = cur_time
            cur_time = time.time()
            sec_cur_time = time.time()
            dt = cur_time - last_time 
            sec =  sec_cur_time - sec_time
            
            x0 = np.array([pitch_ang,wheel_left_vel,wheel_right_vel,pitch_vel])
            # x0 = np.array([body_ori_y,right_wheel_vel_x,body_vel_y])

            # wheel_vel = linvel2wheelvel(0)
            
            xd = np.array([0,left_cmd_vel,right_cmd_vel,0])
            # u = -K1 @ ( x0 )
            # print(xd)
            ul = -Kl @ ( x0 - xd )
            ur = -Kr @ ( x0 - xd )
            
            pub_lw.publish(ul)
            pub_rw.publish(ur)
            
            u_store.append(ul)
            deg_store.append(pitch_ang*RAD2DEG)
            sec_store.append(sec)

            loop_cnt= loop_cnt + 1

            print('dt: ', dt, 'freq: ', 1/dt)
            print('====================================') 
            
            if loop_cnt == 1500:
                print_graph()
            rate.sleep()
            
            
                
                

    except rospy.ROSInterruptException:
        pass
