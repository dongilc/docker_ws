import sympy as sp
import numpy as np
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector import time_derivative
from sympy.physics.vector import ReferenceFrame
N = ReferenceFrame('N')
import pylab as pl
import control
from EoM import *
from sympy.physics.mechanics import *
from numpy.linalg import matrix_rank, eig
import math
import intelligent_robotics as ir

def rot_x(theta):
    rot_x = sp.Matrix([[1,0,0],
                       [0,sp.cos(theta),-sp.sin(theta)],
                       [0,sp.sin(theta),sp.cos(theta)]])
    return rot_x

def rot_y(theta):
    rot_y = sp.Matrix([[sp.cos(theta),0,sp.sin(theta)],
                       [0,1,0],
                       [-sp.sin(theta),0,sp.cos(theta)]])
    return rot_y

def rot_z(theta):
    rot_z = sp.Matrix([[sp.cos(theta),-sp.sin(theta),0],
                       [sp.sin(theta),sp.cos(theta),0],
                       [0,0,1]])
    return rot_z

def trans(a,b,c):
    x = sp.Matrix([[a],[b],[c]])
    return x

def get_torque_from_L(L,q,qd):
    round_L_round_q = sp.zeros(len(q),1);
    i = 0;
    for q_i in q:
        round_L_round_q_i = [];
        round_L_round_q_i = sp.diff(L,q_i);
        round_L_round_q[i] = round_L_round_q_i;
        i+=1;
     
    d_dt_round_L_round_qd = sp.zeros(len(qd),1);
    i = 0;
    for qd_i in qd:
        round_L_round_qd_i = [];
        d_dt_round_L_round_qd_i = [];
        round_L_round_qd_i = sp.diff(L,qd_i);
        d_dt_round_L_round_qd_i = time_derivative(round_L_round_qd_i,N);
        d_dt_round_L_round_qd[i] = d_dt_round_L_round_qd_i;
        i+=1;
        
    tau = d_dt_round_L_round_qd - round_L_round_q 
    return tau

def get_x_next():
    r, l, D = sp.symbols('r, l, D')
    m_w, m_b, g = sp.symbols('m_w, m_b, g')
    I_w, I_b = sp.symbols('I_w, I_b')
    theta_P, theta_w1, theta_w2 = dynamicsymbols('theta_P, theta_w1, theta_w2')

    # Displacement Vector
    x_c1 = trans(r*theta_w1,0,0)
    x_w1 = x_c1 +  trans(0,D,r)
    x_c2 = trans(r*theta_w2,0,0)
    x_w2 = x_c2 +  trans(0,-D,r)
    x_b = 0.5*(x_w1 + x_w2) +  rot_y(theta_P) @ trans(0,0,l)


    # Velocity Vector
    v_w1 = sp.simplify(time_derivative(x_w1,N))
    v_w2 = sp.simplify(time_derivative(x_w2,N))
    v_b = sp.simplify(time_derivative(x_b,N))

    # Angular Velocity Vector
    w_w1 = trans(0, theta_w1.diff(), 0)
    w_w2 = trans(0, theta_w2.diff(), 0)
    w_b = trans(0, theta_P.diff(), 0)


    # Generalized Coordinates
    q = sp.Matrix([[theta_w1], [theta_w2], [theta_P]])
    qd = q.diff()
    qdd = qd.diff()

    # Kinetic Energy
    T_trans = 0.5*(m_w*v_w1.dot(v_w1) + m_w*v_w2.dot(v_w2) + m_b*v_b.dot(v_b))


    T_body = I_b* w_b[1]**2 
    T_wheel1 =I_w* w_w1[1]**2 
    T_wheel2 =I_w* w_w2[1]**2 

    T_rot = 0.5*(T_body + T_wheel1  +  T_wheel2)

    # Potential Energy
    V =  m_b*g*(l*sp.cos(theta_P)) + 2*m_w*g*r 

    # Lagrangian
    L = T_trans + T_rot - V

    tau = get_torque_from_L(L,q,qd)
    tau = sp.simplify(tau)

    T_w1, T_w2 = sp.symbols('T_w1, T_w2')
    u = sp.Matrix([[T_w1], [T_w2]])

    u_matrix = sp.Matrix([[-T_w1], [-T_w2], [ T_w1+ T_w2]])
    tau_eq = tau+u_matrix
    
    M, C, G, W = get_EoM_from_T(tau_eq,qdd,g, u)
    
    param = {I_w:0.199893505, I_b:2.139168508, m_b:40, m_w:15, r:0.165,l:0.3,  g:9.81}

    Ml = msubs(M, param)
    Cl = msubs(C, param)
    Gl = msubs(G, param)
    Wl = msubs(W, param)

    Ml_inv = Ml.inv()
    qdd_rhs_A = Ml_inv*(-Cl -Gl)
    qdd_rhs_B = Ml_inv*Wl*u

    X = q.col_join(qd)
    Xd_A = qd.col_join(qdd_rhs_A)
    Xd_B = qd.col_join(qdd_rhs_B)
    U = u

    A = Xd_A.jacobian(X)
    B = Xd_B.jacobian(U)
    C = X.jacobian(X)
    D = X.jacobian(U)
    
    x_next = A@X + B@U
    
    return x_next

def get_ss():
    r, l, D = sp.symbols('r, l, D')
    m_w, m_b, g = sp.symbols('m_w, m_b, g')
    I_w, I_b = sp.symbols('I_w, I_b')
    theta_P, theta_w1, theta_w2 = dynamicsymbols('theta_P, theta_w1, theta_w2')

    # Displacement Vector
    x_c1 = trans(r*theta_w1,0,0)
    x_w1 = x_c1 +  trans(0,D,r)
    x_c2 = trans(r*theta_w2,0,0)
    x_w2 = x_c2 +  trans(0,-D,r)
    x_b = 0.5*(x_w1 + x_w2) +  rot_y(theta_P) @ trans(0,0,l)


    # Velocity Vector
    v_w1 = sp.simplify(time_derivative(x_w1,N))
    v_w2 = sp.simplify(time_derivative(x_w2,N))
    v_b = sp.simplify(time_derivative(x_b,N))

    # Angular Velocity Vector
    w_w1 = trans(0, theta_w1.diff(), 0)
    w_w2 = trans(0, theta_w2.diff(), 0)
    w_b = trans(0, theta_P.diff(), 0)


    # Generalized Coordinates
    q = sp.Matrix([[theta_w1], [theta_w2], [theta_P]])
    qd = q.diff()
    qdd = qd.diff()

    # Kinetic Energy
    T_trans = 0.5*(m_w*v_w1.dot(v_w1) + m_w*v_w2.dot(v_w2) + m_b*v_b.dot(v_b))


    T_body = I_b* w_b[1]**2 
    T_wheel1 =I_w* w_w1[1]**2 
    T_wheel2 =I_w* w_w2[1]**2 

    T_rot = 0.5*(T_body + T_wheel1  +  T_wheel2)

    # Potential Energy
    V =  m_b*g*(l*sp.cos(theta_P)) + 2*m_w*g*r 

    # Lagrangian
    L = T_trans + T_rot - V

    tau = get_torque_from_L(L,q,qd)
    tau = sp.simplify(tau)

    T_w1, T_w2 = sp.symbols('T_w1, T_w2')
    u = sp.Matrix([[T_w1], [T_w2]])

    u_matrix = sp.Matrix([[-T_w1], [-T_w2], [ T_w1+ T_w2]])
    tau_eq = tau+u_matrix

    linearlize_eq = {sp.sin(theta_P):theta_P, sp.cos(theta_P):1, theta_P.diff()**2:0}
    tau_linear = sp.simplify(tau_eq.subs(linearlize_eq))

    Ml, Cl, Gl, Wl = get_EoM_from_T(tau_linear,qdd,g, u)

    param = {I_w:0.199893505, I_b:2.139168508, m_b:40, m_w:15, r:0.165,l:0.3,  g:9.81}

    Mlp = msubs(Ml, param)
    Clp = msubs(Cl, param)
    Glp = msubs(Gl, param)
    Wlp = msubs(Wl, param)

    Mlp_inv = Mlp.inv()
    qdd_rhs_A = Mlp_inv*(-Clp -Glp)
    qdd_rhs_B = Mlp_inv*Wlp*u

    X = q.col_join(qd)
    Xd_A = qd.col_join(qdd_rhs_A)
    Xd_B = qd.col_join(qdd_rhs_B)
    U = u

    A = Xd_A.jacobian(X)
    B = Xd_B.jacobian(U)
    C = X.jacobian(X)
    D = X.jacobian(U)

    
    
    return A, B