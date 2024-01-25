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


double roll = 0.;
double pitch = 0.;
double pitch_vel = 0.;
double yaw = 0.;
double x_pos = 0.;
double x_dot = 0.;
double wl_pos = 0.;
double wr_pos = 0.;
double wl_dot = 0.;
double wr_dot = 0.;
double x_dot_ref = 0.;
double xl = 0.;
double xl_dot = 0.;
double xl_ref = 0.;
double yaw_vel = 0;
double accel = 0.;
double pitch_ref = 0.;
double front_laser = 0;
double rear_laser = 0;

std::vector<float> x_append;
std::vector<float> xl_append;
std::vector<float> theta_append;
std::vector<float> xdot_append;
std::vector<float> xl_dot_append;
std::vector<float> theta_dot_append;
std::vector<float> x_ref_append;
std::vector<float> xdot_ref_append;
std::vector<float> xl_ref_append;
std::vector<float> theta_ref_append;
std::vector<float> tauL_append;
std::vector<float> tauR_append;
std::vector<float> Fl_append;
std::vector<float> duration_append;


// GAZEBO ROS
void gazebo_setting() {
    // gazebo initial settings
    std::system("rosservice call /gazebo/set_physics_properties \"{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}\"");
}

void joint_callback(const sensor_msgs::JointState::ConstPtr& joint_msg) 
{
    // Accessing message data goes here.
    // For example:
    // double position = msg->position[0];
    // ROS_INFO("Joint position: %f", position);
    xl = joint_msg->position[0];
    wl_pos = joint_msg->position[1];
    wr_pos = joint_msg->position[2];

    xl_dot = joint_msg->velocity[0];
    wl_dot = joint_msg->velocity[1];
    wr_dot = joint_msg->velocity[2];
}

void joyCallback(const sensor_msgs::JoyConstPtr& joy_msg) {
    // Joystick Callback
    //   msg->axes[0]; //lateral
    //   msg->axes[1]; //forward
    //   msg->axes[2]; //cylinder
    // double d = 0.09975;
    // double r = 0.065;

    x_dot_ref = joy_msg->axes[1] * 4.33;
    yaw_vel = joy_msg->axes[0];
    // xl_ref = joy_msg->axes[3] * -0.08;
    pitch_ref = joy_msg->axes[3] * (8*M_PI/180);


}

// void wheel_callback(const sensor_msgs::JointState& msg) {
//   // TODO: Convert this function to C++
// }

void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    // gazebo IMU callback
    //quaternion to euler
    double qx = imu_msg->orientation.x;
    double qy = imu_msg->orientation.y;
    double qz = imu_msg->orientation.z;
    double qw = imu_msg->orientation.w;

    double ysqr = qy*qy;

    double t0 = +2.0 * (qw*qx+qy+qz);
    double t1 = +1.0 * (qx*qx+ysqr);
    roll = std::atan2(t0, t1);

    double t2 = +2.0 * (qw * qy - qz * qx);
    if (t2 > +1.0){
    t2 = +1.0;
    }
    else if (t2 < -1.0){
    t2 = -1.0;
    }
    pitch = std::asin(t2);

    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    yaw = std::atan2(t3, t4);

    pitch_vel = imu_msg->angular_velocity.y;
    accel = imu_msg->linear_acceleration.x;

}

double MeanWithinSTD(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(scan->ranges.empty())
    {
        ROS_WARN("LaserScan ranges are empty!");
        return std::numeric_limits<double>::quiet_NaN();  // Return NaN
    }

    double sum = 0;
    double sum_of_squares = 0;
    double avg_within_std_dev = 0;
    int count = 0;

    // Calculate the mean
    for(double range : scan->ranges)
    {
        if(!std::isnan(range) && !std::isinf(range)) // check for valid data
        {
            sum += range;
            count++;
        }
    }

    if(count == 0)
    {
        ROS_WARN("No valid data in LaserScan ranges!");
        return std::numeric_limits<double>::quiet_NaN();  // Return NaN
    }

    double mean = sum / count;

    // Calculate the sum of squares of differences from the mean
    for(double range : scan->ranges)
    {
        if(!std::isnan(range) && !std::isinf(range))
        {
            sum_of_squares += std::pow(range - mean, 2);
        }
    }

    double std_deviation = std::sqrt(sum_of_squares / count);
    // ROS_INFO("Standard deviation: %.2f meters", std_deviation);

    // Calculate average of values within one standard deviation
    sum = 0;
    count = 0;
    for(double range : scan->ranges)
    {
        if(!std::isnan(range) && !std::isinf(range))
        {
            if(fabs(range - mean) <= std_deviation){
                sum += range;
                count++;
            }
        }
    }
    avg_within_std_dev = sum / count;
    // ROS_INFO("Average deviation: %.2f meters", avg_within_std_dev);

    return avg_within_std_dev;
}

// void laser1_callback(const sensor_msgs::LaserScan::ConstPtr& laser1_msg){
    
//     front_laser = MeanWithinSTD(laser1_msg);
// }

// void laser2_callback(const sensor_msgs::LaserScan::ConstPtr& laser2_msg){
//     rear_laser = MeanWithinSTD(laser2_msg);
// }

// trapezoidal profile generator
void GenProfile(double v_ref, double *vout)
{
double da = 0;
double dv = 0;

double Amax = 0.3;
double dt = 0.002;

// Profile
if(v_ref == *vout) {
    dv = 0;
}
else {
    da = (v_ref - *vout)/dt;
    if(fabs(da) >= Amax) {
    if(da>0) da = Amax;
    else 	 da = -Amax;
    }
}
dv = da*dt;
*vout += dv;
}

void GenProfileBrake(double v_ref, double *vout)
{
double da = 0;
double dv = 0;

double Amax = 1;
double dt = 0.002;

// Profile
if(v_ref == *vout) {
    dv = 0;
}
else {
    da = (v_ref - *vout)/dt;
    if(fabs(da) >= Amax) {
    if(da>0) da = Amax;
    else 	 da = -Amax;
    }
}
dv = da*dt;
*vout += dv;
}

// trapezoidal profile generator
void pitchGenProfile(double v_ref, double *vout)
{
double da = 0;
double dv = 0;

double Amax = 0.1;
double dt = 0.002;

// Profile
if(v_ref == *vout) {
    dv = 0;
}
else {
    da = (v_ref - *vout)/dt;
    if(fabs(da) >= Amax) {
    if(da>0) da = Amax;
    else 	 da = -Amax;
    }
}
dv = da*dt;
*vout += dv;
}

// trapezoidal profile generator
void breakGenProfile(double v_ref, double *vout)
{
double da = 0;
double dv = 0;

double Amax = 1.0;
double dt = 0.002;

// Profile
if(v_ref == *vout) {
    dv = 0;
}
else {
    da = (v_ref - *vout)/dt;
    if(fabs(da) >= Amax) {
    if(da>0) da = Amax;
    else 	 da = -Amax;
    }
}
dv = da*dt;
*vout += dv;
}

// trapezoidal profile generator
void xlGenProfile(double v_ref, double *vout)
{
double da = 0;
double dv = 0;

double Amax = 0.7;
double dt = 0.002;

// Profile
if(v_ref == *vout) {
    dv = 0;
}
else {
    da = (v_ref - *vout)/dt;
    if(fabs(da) >= Amax) {
    if(da>0) da = Amax;
    else 	 da = -Amax;
    }
}
dv = da*dt;
*vout += dv;
}


// OSQP-MPC
void setDynamicsMatrices(Eigen::Matrix<double, 6, 6> &a, Eigen::Matrix<double, 6, 2> &b, double dt)
{
    a << 0., 0., 0., 1, 0., 0.,
         0., 0., 0., 0., 1, 0.,
         0., 0., 0., 0., 0., 1,
         0., 0., -5.62489935e+01, 0., 0., 0.,
         0., 0., 6.60501937e+01, 0., 0., 0.,
         0., 0., 2.88480108e-02, 0., 0., 0.;

    b << 0., 0.,
         0., 0.,
         0., 0.,
         1.86568327e-02, -1.86543474e-02,
         -1.86543474e-02, 2.24226412e-02,
         -8.14745251e-06, -1.68661013e-03;

    Eigen::MatrixXd c = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd d = Eigen::MatrixXd::Zero(6,2);

    // Continous Model to Discrete Model
    // double Ts = 0.01;
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(a.rows() + b.cols(), a.rows() + b.cols());
    M.topLeftCorner(a.rows(), a.cols()) = a * dt;
    M.topRightCorner(a.rows(), b.cols()) = b * dt;

    // Compute the matrix exponential
    Eigen::MatrixXd M_exp = M.exp();

    // Extract the discrete-time system matrices
    Eigen::MatrixXd Ad = M_exp.topLeftCorner(a.rows(), a.cols());
    Eigen::MatrixXd Bd = M_exp.topRightCorner(a.rows(), b.cols());

    // Output and feedthrough matrices remain the same
    Eigen::MatrixXd Cd = c;
    Eigen::MatrixXd Dd = d;

    a = Ad;
    b = Bd;

    std::cout << a <<std::endl;
    std::cout << b <<std::endl;
}

void setInequalityConstraints(Eigen::Matrix<double, 6, 1> &xMax, Eigen::Matrix<double, 6, 1> &xMin,
                              Eigen::Matrix<double, 2, 1> &uMax, Eigen::Matrix<double, 2, 1> &uMin)
{
    // double u0 = 10.5916;

    // input inequality constraints
    // payload:200kg, +0.2m -> F:1500
    // uMin << -500.,
    //         -2500.;

    // uMax << 500.,
    //         2500.;

    uMin << -800.,
            -2500.;

    uMax << 800.,
            2500.;

    // state inequality constraints
    xMin << -OsqpEigen::INFTY, -0.25, -OsqpEigen::INFTY, -8.33, -0.3, -0.3;
    xMax << OsqpEigen::INFTY, 0.25, OsqpEigen::INFTY, 8.33, 0.3, 0.3; //0.392, 0.235
    // xMin << -OsqpEigen::INFTY, -0.08, -10*M_PI/180., -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    // xMax << OsqpEigen::INFTY, 0.08, 10*M_PI/180, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;
}

// void updateInequalityConstraints(Eigen::Matrix<double, 6, 1> &xMax, Eigen::Matrix<double, 6, 1> &xMin,
//                               Eigen::Matrix<double, 2, 1> &uMax, Eigen::Matrix<double, 2, 1> &uMin)
// {
//     // double u0 = 10.5916;

//     // input inequality constraints
//     uMin << -6.,
//             -100.;

//     uMax << 6.,
//             100.;

//     // state inequality constraints
//     xMin << -OsqpEigen::INFTY, -0.1, -10*M_PI/180., -3.0, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
//     xMax << OsqpEigen::INFTY, 0.1, 10*M_PI/180, 3.0, OsqpEigen::INFTY, OsqpEigen::INFTY;;
// }

void setWeightMatrices(Eigen::DiagonalMatrix<double, 6> &Q, Eigen::DiagonalMatrix<double, 2> &R)
{
    // Q.diagonal() << 15000, 55500, 15500000, 15000, 2250, 25000; // theta:infy
    // R.diagonal() << 1e-0, 1e-9;

    Q.diagonal() << 100, 0, 1000000000, 10000, 0, 100;
    R.diagonal() << 1e-10, 1e-0;
}


void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::DiagonalMatrix<double, 2> &R, int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix)
{

    hessianMatrix.resize(6*(mpcWindow+1) + 2 * mpcWindow, 6*(mpcWindow+1) + 2 * mpcWindow);

    //populate hessian matrix
    for(int i = 0; i<6*(mpcWindow+1) + 2 * mpcWindow; i++){
        if(i < 6*(mpcWindow+1)){
            int posQ=i%6;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR=i%2;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::Matrix<double, 6, 1> &xRef, int mpcWindow,
                         Eigen::VectorXd &gradient)
{

    Eigen::Matrix<double,6,1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(6*(mpcWindow+1) +  2*mpcWindow, 1);
    for(int i = 0; i<6*(mpcWindow+1); i++){
        int posQ=i%6;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 6, 6> &dynamicMatrix, const Eigen::Matrix<double, 6, 2> &controlMatrix,
                                 int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(6*(mpcWindow+1)  + 6*(mpcWindow+1) + 2 * mpcWindow, 6*(mpcWindow+1) + 2 * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<6*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<6; j++)
            for(int k = 0; k<6; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(6 * (i+1) + j, 6 * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 6; j++)
            for(int k = 0; k < 2; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(6*(i+1)+j, 2*i+k+6*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<6*(mpcWindow+1) + 2*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*6,i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 6, 1> &xMax, const Eigen::Matrix<double, 6, 1> &xMin,
                                   const Eigen::Matrix<double, 2, 1> &uMax, const Eigen::Matrix<double, 2, 1> &uMin,
                                   const Eigen::Matrix<double, 6, 1> &x0,
                                   int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(6*(mpcWindow+1) +  2 * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(6*(mpcWindow+1) +  2 * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(6*i,0,6,1) = xMin;
        upperInequality.block(6*i,0,6,1) = xMax;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(2 * i + 6 * (mpcWindow + 1), 0, 2, 1) = uMin;
        upperInequality.block(2 * i + 6 * (mpcWindow + 1), 0, 2, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(6*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,6,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*6*(mpcWindow+1) +  2*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*6*(mpcWindow+1) +  2*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}


void updateConstraintVectors(const Eigen::Matrix<double, 6, 1> &x0,
                             Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    lowerBound.block(0,0,6,1) = -x0;
    upperBound.block(0,0,6,1) = -x0;
}


double getErrorNorm(const Eigen::Matrix<double, 6, 1> &x,
                    const Eigen::Matrix<double, 6, 1> &xRef)
{
    // evaluate the error
    Eigen::Matrix<double, 6, 1> error = x - xRef;

    // return the norm
    return error.norm();
}

double cal_xl_ref(double ref_vel, double acc, double theta, double *sol_xl)
{
    const double g = 9.8;
    const double h_c = 0.05774;
    const double h_t = 0.28433;
    const double m_c = 31.914;
    const double m_t = 424.561;

    *sol_xl = -(-g*h_c*m_c*sin(theta) - g*h_t*m_t*sin(theta) + h_c*m_c*cos(theta)*acc + h_t*m_t*cos(theta)*acc)/(m_t*(g*cos(theta) + sin(theta)*acc))+0.05;

}

double slope_torque(double alpha)
{
    const double g = 9.8;
    const double h_c = 0.05774;
    const double h_t = 0.28433;
    const double m_c = 31.914;
    const double m_t = 424.561;
    const double m_w = 9.595;
 
    return g*(m_t + m_c + m_w)*sin(alpha);
}


int main(int argc, char **argv)
{
    // Gazebo ROS settings
    ros::init(argc, argv, "dm_test_v1");
    ros::NodeHandle n;

    ros::Publisher pub_wl = n.advertise<std_msgs::Float64>("/quadruped_robot/LW_pos/command", 100);
    ros::Publisher pub_wr = n.advertise<std_msgs::Float64>("/quadruped_robot/RW_pos/command", 100);
    ros::Publisher pub_xl = n.advertise<std_msgs::Float64>("/quadruped_robot/LM_pos/command", 100);

    ros::Subscriber sub_joy = n.subscribe<sensor_msgs::Joy>("/joy", 100, joyCallback);
    ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("/imu", 100, imu_callback);
    // ros::Subscriber sub_laser1 = n.subscribe<sensor_msgs::LaserScan>("/laser1/scan", 100, laser1_callback);
    // ros::Subscriber sub_laser2 = n.subscribe<sensor_msgs::LaserScan>("/laser2/scan", 100, laser2_callback);
    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/quadruped_robot/joint_states", 1000, joint_callback);
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

    gazebo_msgs::GetLinkState link_srv;
    link_srv.request.link_name = "base_footprint"; // 원하는 링크 이름으로 변경
    link_srv.request.reference_frame = "world"; // 원하는 참조 프레임으로 변경

    double hz = 500.; //500.
    double init_wl_pos = wl_pos;
    double init_wr_pos = wr_pos;
    double x_pos_ref = 0;
    double r = 0.22;
    double temp = 0;
    int cnt = 0;
    bool update = false;
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
    // set the preview window
    int mpcWindow = 100; //75 infy

    gazebo_setting();
    ros::Rate loop_rate(hz);  // 500 Hz
    

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

    Eigen::Matrix<double, 6, 1> x_next;

    // x_pos = ((wl_pos - init_wl_pos)+(wr_pos - init_wr_pos))/2 * r;
    if(client.call(link_srv)) x_pos = link_srv.response.link_state.pose.position.x;
    if(client.call(link_srv)) x_dot = link_srv.response.link_state.twist.linear.x;

    x_pos_init = x_pos;
    x_pos_current = x_pos - x_pos_init;
    // x_dot = ((wl_dot + wr_dot)/2) * r;
    x_pos_ref += x_dot_ref * 1/hz;

    // set the initial and the desired states
    x0 << x_pos_current, xl, pitch, x_dot, xl_dot, pitch_vel;
    xRef <<  0, 0, 0, 0, 0, 0;

    // set MPC problem quantities
    setDynamicsMatrices(a, b, 1/hz);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(6 * (mpcWindow + 1) + 2 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 6 * (mpcWindow + 1) + 2 * mpcWindow);
    if(!solver.data()->setHessianMatrix(hessian)) return 1;
    if(!solver.data()->setGradient(gradient)) return 1;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    if(!solver.data()->setLowerBound(lowerBound)) return 1;
    if(!solver.data()->setUpperBound(upperBound)) return 1;

    // instantiate the solver
    if(!solver.initSolver()) return 1;

    // controller input and QPSolution vector
    Eigen::Vector2d ctr;
    Eigen::VectorXd QPSolution;

    std_msgs::Float64 tau1;
    std_msgs::Float64 tau2;
    std_msgs::Float64 F_l;

    bool driving = false;

    while (ros::ok()){        

        clock_t start = clock();

        // solve the QP problem
        if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(6 * (mpcWindow + 1), 0, 2, 1);

        if (x_dot_ref != 0.){
            GenProfile(x_dot_ref, &temp);
            x_vel_ref = temp;
            acc = temp;
            driving = true;
            x_pos_ref = x_pos_ref + (x_vel_ref*1/hz);
        }
        else if(x_dot_ref == 0.){
            GenProfileBrake(0., &temp);
            x_vel_ref = temp;
            // x_vel_ref = 0.;

            if(driving==true){
                // double bacc = -acc;
                acc = 0;
                if (x_dot < 0){
                    xl_ref = -0.05;
                }
                else if (x_dot > 0){
                    xl_ref = 0.05;
                }
                else xl_ref = 0.;
                     
                x_pos_ref = x_pos_current;
                if(fabs(x_dot)<0.1){
                    driving = false;
                }
            }

            else acc = 0.;
            xl_ref = 0.;            
        }

        // //ground inclination
        // double L = 2.15;
        // double eta = atan2(L, rear_laser-front_laser);  // angle calculated by distance sensor. atan2(y,x)
        // double alpha = M_PI/2. - pitch - eta;
        // slope_tau = slope_torque(alpha);
        // cal_xl_ref(x_vel_ref, acc, -alpha, &sol_xl);

        // if (cnt >=2000){
        //     tau1.data = (ctr[0]*r)/2 - (yaw_vel*150) + (slope_tau/2);
        //     tau2.data = (ctr[0]*r)/2 + (yaw_vel*150) + (slope_tau/2);
        // }
        
        x_next = a*x0 + b*ctr;

        tau1.data = (ctr[0]*r)/2 - (yaw_vel*150);
        tau2.data = (ctr[0]*r)/2 + (yaw_vel*150);
        // tau.data = ctr[0]/2;
        // F_l.data = ctr[1];
        F_l.data = 0;

        

        pub_wl.publish(tau1);
        pub_wr.publish(tau2);
        pub_xl.publish(F_l);

        

        // x_pos = ((wl_pos - init_wl_pos)+(wr_pos - init_wr_pos))/2 * r;
        
        if(client.call(link_srv)) x_pos = link_srv.response.link_state.pose.position.x;
        if(client.call(link_srv)) x_dot = link_srv.response.link_state.twist.linear.x;
        // x_dot = x_dot/1/hz;

        x_pos_current = x_pos - x_pos_init;
        // x_dot = ((wl_dot + wr_dot)/2) * r;
        delta_xdot = x_dot- x_dot_past;
        double x_ddot = delta_xdot/1/hz;
        // GenProfile(x_dot_ref, &temp);
        // x_vel_ref = temp;

        // x_pos_ref = x_pos_ref + (x_dot_ref*1/hz);

        pitchGenProfile(pitch_ref, &theta_temp);
        // x_ddot = x_vel_ref - x_dot;
        // cal_xl_ref(x_vel_ref, tau1.data, tau2.data, x_ddot, theta_temp, sol_xl); //x_ddot
        // double xl_vel_ref = sol_xl/1/hz;
        x0 << x_pos_current, xl, pitch, x_dot, xl_dot, pitch_vel;
        xRef <<  x_pos_ref, 0, 0, 0, 0, 0;

        
        // if (cnt >=2000){
        //     xRef <<  x_pos_ref, sol_xl, 0, x_vel_ref, 0, 0;
        // }
        
        // else {
        //     xRef <<  x_pos_ref, 0, 0, x_vel_ref, 0, 0;
        // }

        if (cnt == 1500){
            std::cout << "update parameters" << std::endl;
            // set MPC problem quantities
            // updateInequalityConstraints(xMax, xMin, uMax, uMin);
            // Q.diagonal() << 400000000, 5000000000, 5000000000, 800000000, 5500000000, 450000000; // theta:infy
            // Q.diagonal() << 55000, 555000, 21000000, 55000, 105000, 850000;
            // R.diagonal() << 1e-5, 1e+0;
            // xMin << -OsqpEigen::INFTY, -0.25, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -1.0, -0.78;
            // xMax << OsqpEigen::INFTY, 0.25, OsqpEigen::INFTY, OsqpEigen::INFTY, 1.0, 0.78;
            
            // castMPCToQPHessian(Q, R, mpcWindow, hessian);
            // castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
            // castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

            // solver.updateLowerBound(lowerBound);
            // solver.updateUpperBound(upperBound);
            // update the constraint bound
            // solver.updateHessianMatrix(hessian);
            // solver.updateGradient(gradient);
            updateConstraintVectors(x0, lowerBound, upperBound);
            if(!solver.updateBounds(lowerBound, upperBound)) return 1;

            // sol_xl = 0.235565711055099;
            

            update = true;

        }
        else{

            castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
            solver.updateGradient(gradient);

            // update the constraint bound
            updateConstraintVectors(x0, lowerBound, upperBound);
            if(!solver.updateBounds(lowerBound, upperBound)) return 1;

        }

        ros::spinOnce();
        loop_rate.sleep();

        clock_t end = clock();
        double duration = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        if (duration > 0.002) over_dt += 1;

        cnt += 1;
        x_dot_past = x_dot;

        x_append.push_back(x0(0));
        xl_append.push_back(x0(1));
        theta_append.push_back(x0(2));
        xdot_append.push_back(x0(3));
        xl_dot_append.push_back(x0(4));
        theta_dot_append.push_back(x0(5));
        x_ref_append.push_back(xRef(0));
        xdot_ref_append.push_back(xRef(3));
        xl_ref_append.push_back(xRef(1));
        theta_ref_append.push_back(xRef(2));
        tauL_append.push_back(tau1.data);
        tauR_append.push_back(tau2.data);
        Fl_append.push_back(F_l.data);
        duration_append.push_back(duration);

        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << "dt : " << duration << std::endl;
        // std::cout << "mpc window :" << mpcWindow << std::endl;
        // std::cout << "acceleration : " << x_ddot << std::endl;
        // std::cout << "xNext :" << x_next.transpose() << std::endl;
        std::cout << "x :" << x0.transpose() << std::endl;
        std::cout << "xRef :" << xRef.transpose() << std::endl;
        // std::cout << "vRef :" << xRef(3) << std::endl;
        // std::cout << "vRobot :" << x0(3) << std::endl;
        std::cout << "xlRef :" << xRef(1) << std::endl;
        std::cout << "Wheel Torque L : " << tau1.data << std::endl;
        std::cout << "Wheel Torque R : " << tau2.data << std::endl;
        std::cout << "Cylinder Force : " << F_l.data << std::endl;
        // std::cout << "Update : " << update << std::endl;
        // std::cout << "acc : " << acc << std::endl;
        // std::cout << "driving : " << driving << std::endl;
        // std::cout << "Yaw taus : " << yaw_vel*150  << std::endl;
        // std::cout << "Front Laser : " << front_laser << std::endl;
        // std::cout << "Rear Laser : " << rear_laser << std::endl;
        // std::cout << "Slope angle : " << alpha << std::endl;
        // std::cout << "Slope torque : " << slope_tau  << std::endl;


      }

    // store data
    bool save_data = false;
    int payload = 11;
    int date = 231201;

    if (save_data == true){

        std::ostringstream x_dir; 
        x_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/x_store_" << payload << "_" << date << ".csv";
        std::ofstream x_store(x_dir.str());
        for (float value : x_append) {
            x_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        x_store.close();
        
        std::ostringstream xl_dir; 
        xl_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/xl_store_" << payload << "_" << date << ".csv";
        std::ofstream xl_store(xl_dir.str());
        for (float value : xl_append) {
            xl_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        xl_store.close();

        std::ostringstream theta_dir; 
        theta_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/theta_store_" << payload << "_" << date << ".csv";
        std::ofstream theta_store(theta_dir.str());
        for (float value : theta_append) {
            theta_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        theta_store.close();

        std::ostringstream xdot_dir; 
        xdot_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/xdot_store_" << payload << "_" << date << ".csv";
        std::ofstream xdot_store(xdot_dir.str());
        for (float value : xdot_append) {
            xdot_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        xdot_store.close();

        std::ostringstream xl_dot_dir; 
        xl_dot_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/xl_dot_store_" << payload << "_" << date << ".csv";
        std::ofstream xl_dot_store(xl_dot_dir.str());
        for (float value : xl_dot_append) {
            xl_dot_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        xl_dot_store.close();

        std::ostringstream theta_dot_dir; 
        theta_dot_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/theta_dot_store_" << payload << "_" << date << ".csv";
        std::ofstream theta_dot_store(theta_dot_dir.str());
        for (float value : theta_dot_append) {
            theta_dot_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        theta_dot_store.close();

        std::ostringstream x_ref_dir; 
        x_ref_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/x_ref_store_" << payload << "_" << date << ".csv";
        std::ofstream x_ref_store(x_ref_dir.str());
        for (float value : x_ref_append) {
            x_ref_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        x_ref_store.close();

        std::ostringstream xdot_ref_dir; 
        xdot_ref_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/xdot_ref_store_" << payload << "_" << date << ".csv";
        std::ofstream xdot_ref_store(xdot_ref_dir.str());
        for (float value : xdot_ref_append) {
            xdot_ref_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        xdot_ref_store.close();

        std::ostringstream xl_ref_dir; 
        xl_ref_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/xl_ref_store_" << payload << "_" << date << ".csv";
        std::ofstream xl_ref_store(xl_ref_dir.str());
        for (float value : xl_ref_append) {
            xl_ref_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        xl_ref_store.close();

        std::ostringstream theta_ref_dir; 
        theta_ref_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/theta_ref_store_" << payload << "_" << date << ".csv";
        std::ofstream theta_ref_store(theta_ref_dir.str());
        for (float value : theta_ref_append) {
            theta_ref_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        theta_ref_store.close();

        std::ostringstream tauL_dir; 
        tauL_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/tauL_store_" << payload << "_" << date << ".csv";
        std::ofstream tauL_store(tauL_dir.str());
        for (float value : tauL_append) {
            tauL_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        tauL_store.close();

        std::ostringstream tauR_dir; 
        tauR_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/tauR_store_" << payload << "_" << date << ".csv";
        std::ofstream tauR_store(tauR_dir.str());
        for (float value : tauR_append) {
            tauR_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        tauR_store.close();

        std::ostringstream Fl_dir; 
        Fl_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/Fl_store_" << payload << "_" << date << ".csv";
        std::ofstream Fl_store(Fl_dir.str());
        for (float value : Fl_append) {
            Fl_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        Fl_store.close();

        std::ostringstream duration_dir; 
        duration_dir << "/home/david/OneDrive/my_ws/KITECH_quad_v2_simple/src/quadruped_test_description/control/data/duration_store_" << payload << "_" << date << ".csv";
        std::ofstream duration_store(duration_dir.str());
        for (float value : duration_append) {
            duration_store << value << '\n'; // or file << value << ','; depending on your desired format
        }
        duration_store.close();
    }

    return 0;
}