#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib> 
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetModelState.h>


// Eigen::DiagonalMatrix<double, 8> Q;
// Eigen::DiagonalMatrix<double, 3> R;
double roll = 0.;
double pitch = 0.;
double pitch_vel = 0.;
double pitch_ref = 0.;
double yaw = 0.;
double xl = 0.;
double xl_dot = 0.;
double xl_ref = 0.;
double x_pos = 0.;
double x_dot = 0.;
double x_pos_ref = 0.;
double x_dot_ref = 0.;
double yaw_vel_ref = 0.;
double wl_pos = 0.;
double wr_pos = 0.;
double wl_vel = 0.;
double wr_vel = 0.;
// double theta; //pitch
// double theta_dot; //pitch_vel

void gazebo_setting() {
    // gazebo initial settings
    std::system("rosservice call /gazebo/set_physics_properties \"{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}\"");
}
// trapezoidal profile generator
void GenProfile(double v_ref, double *vout)
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
// trapezoidal profile generator
void GenProfilePitch(double v_ref, double *vout)
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
    wl_vel = joint_msg->velocity[1];
    wr_vel = joint_msg->velocity[2];
}

void joyCallback(const sensor_msgs::JoyConstPtr& joy_msg) {
    // Joystick Callback
    //   msg->axes[0]; //lateral
    //   msg->axes[1]; //forward
    //   msg->axes[2]; //cylinder
    static double temp = 0.;
    
    // double vx = joy_msg->axes[1]*3.5;
    // GenProfile(vx, &temp);
    // x_dot_ref = temp;
    x_dot_ref = joy_msg->axes[1]*3.0;
    yaw_vel_ref = joy_msg->axes[0]*1.5;
    pitch_ref = joy_msg->axes[4]*10*M_PI/180;

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

}

double cal_xl_ref(double ref_vel, double tauL, double tauR, double acc, double theta, double &sol_xl)
{
    const double g = 9.8;
    const double h_c = 0.034617;
    const double h_t = 0.11198;
    const double m_c = 4.124;
    const double m_t = 8.885;

    if (ref_vel==0){
        acc = 0.;
        sol_xl = (tauL+tauR - g*h_c*m_c*sin(theta) - g*h_t*m_t*sin(theta) + h_c*m_c*cos(theta)*acc + h_t*m_t*cos(theta)*acc)/(m_t*(g*cos(theta) + sin(theta)*acc));
    }
    else sol_xl = 0.;
    // sol_xl = -sol_xl;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "dm_test_v1");
    ros::NodeHandle n;

    ros::Publisher pub_wl = n.advertise<std_msgs::Float64>("/dm_mini_v1/LW_cont/command", 100);
    ros::Publisher pub_wr = n.advertise<std_msgs::Float64>("/dm_mini_v1/RW_cont/command", 100);
    ros::Publisher pub_xl = n.advertise<std_msgs::Float64>("/dm_mini_v1/LM_cont/command", 100);

    ros::Subscriber sub_joy = n.subscribe<sensor_msgs::Joy>("/joy", 100, joyCallback);
    ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("/imu", 100, imu_callback);
    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/dm_mini_v1/joint_states", 1000, joint_callback);
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

    // TODO: Implement service client

    gazebo_msgs::GetLinkState link_srv;
    link_srv.request.link_name = "fixed_base"; // 원하는 링크 이름으로 변경
    link_srv.request.reference_frame = "world"; // 원하는 참조 프레임으로 변경

    gazebo_setting();
    ros::Rate loop_rate(500);  // 1000 Hz(1kHz)
    
    int cnt = 0;
    double dt = 1./500.;
    double r = 0.065;
    static double temp = 0.;
    static double pitch_temp = 0.;
    bool update = false;
    double x_vel = 0.;
    double x_vel_past = 0.;
    double x_ddot = 0.;
    double sol_xl = 0.;

    Eigen::VectorXd x0(6,1);
    Eigen::VectorXd xRef(6,1);
    Eigen::VectorXd Kw(6,1);
    Eigen::VectorXd Kl(6,1);
    std_msgs::Float64 tau_wl;
    std_msgs::Float64 tau_wr;
    std_msgs::Float64 lin_force;

    //
    GenProfile(x_dot_ref, &temp);
    double x_vel_ref = temp;
    x_dot = ((wl_vel + wr_vel)/2)*r;
    x_pos_ref = x_pos_ref + x_dot_ref*dt;
    if(client.call(link_srv)) x_pos = link_srv.response.link_state.pose.position.x;
    x0 << x_pos, xl, pitch, x_dot, xl_dot, pitch_vel;
    xRef << x_pos_ref, 0, 0, x_dot_ref, 0, 0;
    std::cout << "Setting Done1"<< std::endl;

    // Compute LQR Gain
    // std::cout << "Compute LQR Gain"<< std::endl;
    // Eigen::MatrixXd K = computeLQR(a, b, Q, R);
    // std::cout << "K" <<  K << std::endl;
    // std::cout << "Compute LQR Gain Done"<< std::endl;
    std::cout << "Setting Done2"<< std::endl;
    Kw << -13.74507915, -49.95617883, -462.92774076, -32.97574884, -51.45596407, -112.31529698;
    Kl << -0.19447606, 1.79810472, -5.50340304, -0.48422866, 2.01479435, -3.88482886;
    std::cout << "Setting Done3"<< std::endl;

    while (ros::ok()) {
        // TODO: Convert the rest of the main loop to C++

        if(client.call(link_srv)) x_pos = link_srv.response.link_state.pose.position.x;
        if(client.call(link_srv)) x_vel = link_srv.response.link_state.twist.linear.x;

        GenProfile(x_dot_ref, &temp);
        GenProfilePitch(pitch_ref, &pitch_temp);
        
        x_vel_ref = temp;
        x_dot = ((wl_vel + wr_vel)/2)*r;
        x_ddot = (x_vel - x_vel_past)/dt;
        x_pos_ref = x_pos_ref + (x_vel_ref*dt);
        
        x0 << x_pos, xl, pitch, x_dot, xl_dot, pitch_vel;

        std::cout << "Cal"<< std::endl;
        Eigen::VectorXd x0_diff = x0 - xRef;
        
        double tau = -(Kw.dot(x0_diff)*r)/2;
        double Fl = -Kl.dot(x0_diff);
        tau_wl.data = tau - yaw_vel_ref;
        tau_wr.data = tau + yaw_vel_ref;
        lin_force.data = Fl;

        pub_wl.publish(tau_wl);
        pub_wr.publish(tau_wr);
        pub_xl.publish(lin_force);

        cal_xl_ref(x_vel_ref, tau_wl.data, tau_wr.data, x_ddot, pitch_temp, sol_xl); //x_ddot
        xRef << x_pos_ref, sol_xl, pitch_temp, x_vel_ref, 0, 0;

        if (cnt==1500){
            Kw << -14.0278592,   -41.79580801, -461.65829135,  -32.65567872,  -46.56701515, -112.42321334;
            Kl << -0.18423437 , 1.61861191, -5.0538298,  -0.44070443,  1.90684395, -3.66789175;
            update = true;
        
        }

        // std::cout << "K : " << K << std::endl;
        std::cout << "tau : "<< tau_wl << std::endl;
        std::cout << "tau : "<< tau_wr << std::endl;
        std::cout << "Fl : "<< lin_force << std::endl;
        std::cout << "x0 : " << x0.transpose() << std::endl;
        std::cout << "xRef : " << xRef.transpose()<< std::endl;
        std::cout << "xl_ref : "<< sol_xl << std::endl;
        std::cout << "theta_ref : "<< pitch_temp << std::endl;
        std::cout << "acceleration : "<< x_ddot << std::endl;
        std::cout << "Update : "<< update << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
        cnt += 1;
        x_vel_past = x_dot;
    }
  
  return 0;
}