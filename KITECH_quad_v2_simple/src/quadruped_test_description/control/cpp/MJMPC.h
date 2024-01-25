#ifndef MJMPC_H
#define MJMPC_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <ctime> 
#include <limits>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetModelState.h>

class MJMPC
{
private:

    double r = 0.22;
    int mpcWindow = 70;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 6, 6> a;
    Eigen::Matrix<double, 6, 2> b;

    // allocate the constraints vector
    Eigen::Matrix<double, 6, 1> xMax;
    Eigen::Matrix<double, 6, 1> xMin;
    Eigen::Matrix<double, 2, 1> uMax;
    Eigen::Matrix<double, 2, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 6> Q;
    Eigen::DiagonalMatrix<double, 2> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 6, 1> x0;
    Eigen::Matrix<double, 6, 1> xRef;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // controller input and QPSolution vector
    Eigen::Vector2d ctr;
    Eigen::VectorXd QPSolution;
    
public:

    double roll = 0.;
    double pitch = 0.;
    double pitch_vel = 0.;
    double yaw = 0.;
    double x_pos = 0.;
    double x_acc = 0.;
    double wl_pos = 0.;
    double wr_pos = 0.;
    double wl_dot = 0.;
    double wr_dot = 0.;
    double x_dot = 0.;
    double x_dot_ref = 0.;
    double xl = 0.;
    double xl_dot = 0.;
    double xl_ref = 0.;
    double yaw_vel = 0;
    double accel = 0.;
    double pitch_ref = 0.;
    double front_laser = 0;
    double rear_laser = 0;
    double x_pos_init = 0.;
    double x_pos_current = 0.;
    double over_dt = 0;
    double x_vel_ref = 0.;
    double sol_xl = 0.;
    double delta_xdot = 0.;
    double theta_ref = 0.;
    double x_dot_past = 0.;
    double theta_temp = 0.;
    double xl_temp = 0.;
    double acc = 0.;
    double slope_tau = 0.;

    std_msgs::Float64 tau1;
    std_msgs::Float64 tau2;
    std_msgs::Float64 F_l;

    void gazebo_setting();

    void joint_callback(const sensor_msgs::JointState::ConstPtr& joint_msg);

    void joyCallback(const sensor_msgs::JoyConstPtr& joy_msg);

    void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);

    double MeanWithinSTD(const sensor_msgs::LaserScan::ConstPtr& scan);
    // void laser1_callback(const sensor_msgs::LaserScan::ConstPtr& laser1_msg);
    // void laser2_callback(const sensor_msgs::LaserScan::ConstPtr& laser2_msg);

    // trapezoidal profile generator
    void GenProfile(double v_ref, double *vout);

    void GenProfileBrake(double v_ref, double *vout);
    
    // trapezoidal profile generator
    void pitchGenProfile(double v_ref, double *vout);

    // trapezoidal profile generator
    void breakGenProfile(double v_ref, double *vout);

    // trapezoidal profile generator
    void xlGenProfile(double v_ref, double *vout);
    

    // OSQP-MPC
    void setDynamicsMatrices(Eigen::Matrix<double, 6, 6> &a, Eigen::Matrix<double, 6, 2> &b, double dt);
    

    void setInequalityConstraints(Eigen::Matrix<double, 6, 1> &xMax, Eigen::Matrix<double, 6, 1> &xMin,
                                Eigen::Matrix<double, 2, 1> &uMax, Eigen::Matrix<double, 2, 1> &uMin);
    
    void setWeightMatrices(Eigen::DiagonalMatrix<double, 6> &Q, Eigen::DiagonalMatrix<double, 2> &R);

    void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::DiagonalMatrix<double, 2> &R, int mpcWindow,
                            Eigen::SparseMatrix<double> &hessianMatrix);
    
    void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::Matrix<double, 6, 1> &xRef, int mpcWindow,
                            Eigen::VectorXd &gradient);

    void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 6, 6> &dynamicMatrix, const Eigen::Matrix<double, 6, 2> &controlMatrix,
                                    int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix);

    void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 6, 1> &xMax, const Eigen::Matrix<double, 6, 1> &xMin,
                                    const Eigen::Matrix<double, 2, 1> &uMax, const Eigen::Matrix<double, 2, 1> &uMin,
                                    const Eigen::Matrix<double, 6, 1> &x0,
                                    int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
    
    void updateConstraintVectors(const Eigen::Matrix<double, 6, 1> &x0,
                                Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
    
    double getErrorNorm(const Eigen::Matrix<double, 6, 1> &x,
                        const Eigen::Matrix<double, 6, 1> &xRef);
    
    double cal_xl_ref(double ref_vel, double acc, double theta, double *sol_xl);
   
    double slope_torque(double alpha);
   
};
