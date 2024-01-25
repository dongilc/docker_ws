#include "MJMPC.h"

// GAZEBO ROS
void MJMPC::gazebo_setting() {
    // gazebo initial settings
    std::system("rosservice call /gazebo/set_physics_properties \"{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}\"");
}

void MJMPC::joint_callback(const sensor_msgs::JointState::ConstPtr& joint_msg) 
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

void MJMPC::joyCallback(const sensor_msgs::JoyConstPtr& joy_msg) {
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

void MJMPC::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
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

double MJMPC::MeanWithinSTD(const sensor_msgs::LaserScan::ConstPtr& scan){
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

// void MJMPC::laser1_callback(const sensor_msgs::LaserScan::ConstPtr& laser1_msg){
    
//     front_laser = MeanWithinSTD(laser1_msg);
// }

// void MJMPC::laser2_callback(const sensor_msgs::LaserScan::ConstPtr& laser2_msg){
//     rear_laser = MeanWithinSTD(laser2_msg);
// }

// trapezoidal profile generator
void MJMPC::GenProfile(double v_ref, double *vout)
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

void MJMPC::GenProfileBrake(double v_ref, double *vout)
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
void MJMPC::pitchGenProfile(double v_ref, double *vout)
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
void MJMPC::breakGenProfile(double v_ref, double *vout)
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
void MJMPC::xlGenProfile(double v_ref, double *vout)
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
void MJMPC::setDynamicsMatrices(Eigen::Matrix<double, 6, 6> &a, Eigen::Matrix<double, 6, 2> &b, double dt)
{
    // a << 0., 0., 0., 1, 0., 0.,
    //      0., 0., 0., 0., 1, 0.,
    //      0., 0., 0., 0., 0., 1,
    //      0., 0., -94.02606754, 0., 0., 0.,
    //      0., 0., 103.5339867, 0., 0., 0.,
    //      0., 0., 1.06243043, 0., 0., 0.;

    // b << 0., 0.,
    //      0., 0.,
    //      0., 0.,
    //       0.02257459, -0.02250892,
    //      -0.02250892,  0.0252477,
    //      -0.00023098, -0.00134846;
    a << 0., 0., 0., 1, 0., 0.,
         0., 0., 0., 0., 1, 0.,
         0., 0., 0., 0., 0., 1,
         0., 0., -1.12497987e+02, 0., 0., 0.,
         0., 0., 1.32100387e+02, 0., 0., 0.,
         0., 0., 2.88480108e-02, 0., 0., 0.;

    b << 0., 0.,
         0., 0.,
         0., 0.,
          0.01866091, -0.01865435,
         -0.01781104,  0.02242264,
         -0.00139304, -0.00084331;

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

void MJMPC::setInequalityConstraints(Eigen::Matrix<double, 6, 1> &xMax, Eigen::Matrix<double, 6, 1> &xMin,
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

void MJMPC::setWeightMatrices(Eigen::DiagonalMatrix<double, 6> &Q, Eigen::DiagonalMatrix<double, 2> &R)
{
    // Q.diagonal() << 15000, 355000, 6000000, 15000, 65000, 300000; // theta:infy
    Q.diagonal() << 15000, 355000, 7000000, 15000, 65000, 500000; // theta:infy
    // Q.diagonal() << 15000, 355000, 8000000, 15000, 65000, 500000; // mpcWindow : 70
    // Q.diagonal() << 15000, 355000, 7500000, 15000, 65000, 500000; // mpcWindow : 70

    // Q.diagonal() << 25000, 455000, 9000000, 25000, 75000, 650000; // mpcWindow : 70##2222

    // Q.diagonal() << 25000, 455000, 9000000, 25000, 75000, 650000; // mpcWindow : 70
    // Q.diagonal() << 25000, 465000, 9090000, 25000, 80000, 660000; // mpcWindow : 70 payload:200kg, +0.0m
    // Q.diagonal() << 115000, 755000, 14300000, 25000, 95000, 950000; // mpcWindow : 70

    // Q.diagonal() << 117000, 760000, 14300000, 27000, 99000, 950000; // mpcWindow : 70
    // Q.diagonal() << 35000, 485000, 14800000, 25000, 85000, 880000; // mpcWindow : 70
    // Q.diagonal() << 35000, 555000, 15000000, 35000, 85000, 750000; // mpcWindow : 70, payload:200kg, +0.2m
    // Q.diagonal() << 35000, 555000, 15000000, 35000, 85000, 750000; // mpcWindow : 70, payload:200kg, +0.5m
    // Q.diagonal() << 55000, 555000, 21000000, 55000, 105000, 850000; // mpcWindow : 70, payload:200kg, +0.5m 뒤로 넘어져있을때
    // Q.diagonal() << 55000, 555000, 22000000, 55000, 365000, 950000; // mpcWindow : 70, payload:200kg, +0.5m 앞으로 넘어져있을때
    // Q.diagonal() << 25000, 555000, 28000000, 25000, 365000, 950000; // mpcWindow : 70, payload:200kg, +0.5m

    // Q.diagonal() << 25000, 455000, 15000000, 25000, 75000, 550000; // mpcWindow : 70

    R.diagonal() << 1e-1, 1e-5;
    // R.diagonal() << 1e-0, 1e-8;
}


void MJMPC::castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::DiagonalMatrix<double, 2> &R, int mpcWindow,
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

void MJMPC::castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::Matrix<double, 6, 1> &xRef, int mpcWindow,
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

void MJMPC::castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 6, 6> &dynamicMatrix, const Eigen::Matrix<double, 6, 2> &controlMatrix,
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

void MJMPC::castMPCToQPConstraintVectors(const Eigen::Matrix<double, 6, 1> &xMax, const Eigen::Matrix<double, 6, 1> &xMin,
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


void MJMPC::updateConstraintVectors(const Eigen::Matrix<double, 6, 1> &x0,
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

double MJMPC::cal_xl_ref(double ref_vel, double acc, double theta, double *sol_xl)
{
    const double g = 9.8;
    const double h_c = 0.05774;
    const double h_t = 0.28433;
    const double m_c = 31.914;
    const double m_t = 424.561;

    *sol_xl = -(-g*h_c*m_c*sin(theta) - g*h_t*m_t*sin(theta) + h_c*m_c*cos(theta)*acc + h_t*m_t*cos(theta)*acc)/(m_t*(g*cos(theta) + sin(theta)*acc))+0.05;

}

double MJMPC::slope_torque(double alpha)
{
    const double g = 9.8;
    const double h_c = 0.05774;
    const double h_t = 0.28433;
    const double m_c = 31.914;
    const double m_t = 424.561;
    const double m_w = 9.595;
 
    return g*(m_t + m_c + m_w)*sin(alpha);
}