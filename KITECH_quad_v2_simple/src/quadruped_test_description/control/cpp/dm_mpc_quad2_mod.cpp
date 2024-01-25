#include "MJMPC.h"

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

int main(int argc, char **argv)
{
    MJMPC MPC;
    // Gazebo ROS settings
    ros::init(argc, argv, "quad_v2");
    ros::NodeHandle n;

    ros::Publisher pub_wl = n.advertise<std_msgs::Float64>("/quadruped_robot/LW_pos/command", 100);
    ros::Publisher pub_wr = n.advertise<std_msgs::Float64>("/quadruped_robot/RW_pos/command", 100);
    ros::Publisher pub_xl = n.advertise<std_msgs::Float64>("/quadruped_robot/LM_pos/command", 100);

    ros::Subscriber sub_joy = n.subscribe<sensor_msgs::Joy>("/joy", 100, MPC.joyCallback);
    ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("/imu", 100, MPC.imu_callback);
    // ros::Subscriber sub_laser1 = n.subscribe<sensor_msgs::LaserScan>("/laser1/scan", 100, laser1_callback);
    // ros::Subscriber sub_laser2 = n.subscribe<sensor_msgs::LaserScan>("/laser2/scan", 100, laser2_callback);
    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/quadruped_robot/joint_states", 1000, joint_callback);
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

    gazebo_msgs::GetLinkState link_srv;
    link_srv.request.link_name = "base_footprint"; // 원하는 링크 이름으로 변경
    link_srv.request.reference_frame = "world"; // 원하는 참조 프레임으로 변경

    double hz = 500.; //500.
    double init_wl_pos = MPC.wl_pos;
    double init_wr_pos = MPC.wr_pos;
    double x_pos_ref = 0;
    double temp = 0;
    int cnt = 0;
    bool update = false;
    
    // set the preview window
     //75 infy

    gazebo_setting();
    ros::Rate loop_rate(hz);  // 500 Hz
    

   

    // x_pos = ((wl_pos - init_wl_pos)+(wr_pos - init_wr_pos))/2 * r;
    if(client.call(link_srv)) MPC.x_pos = link_srv.response.link_state.pose.position.x;
    if(client.call(link_srv)) MPC.x_dot = link_srv.response.link_state.twist.linear.x;

    MPC.x_pos_init = MPC.x_pos;
    MPC.x_pos_current = MPC.x_pos - MPC.x_pos_init;
    // MPC.x_dot = ((MPC.wl_dot + MPC.wr_dot)/2) * MPC.r;
    x_pos_ref += MPC.x_dot_ref * 1/hz;

    // set the initial and the desired states
    MPC.x0 << MPC.x_pos_current, MPC.xl, MPC.pitch, MPC.x_dot, MPC.xl_dot, MPC.pitch_vel;
    MPC.xRef <<  0, 0, 0, 0, 0, 0;

    // set MPC problem quantities
    MPC.setDynamicsMatrices(MPC.a, MPC.b, 1/hz);
    MPC.setInequalityConstraints(MPC.xMax, MPC.xMin, MPC.uMax, MPC.uMin);
    MPC.setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    MPC.castMPCToQPHessian(MPC.Q, MPC.R, MPC.mpcWindow, MPC.hessian);
    MPC.castMPCToQPGradient(MPC.Q, MPC.xRef, MPC.mpcWindow, MPC.gradient);
    MPC.castMPCToQPConstraintMatrix(MPC.a, MPC.b, MPC.mpcWindow, MPC.linearMatrix);
    MPC.castMPCToQPConstraintVectors(MPC.xMax, MPC.xMin, MPC.uMax, MPC.uMin, MPC.x0, MPC.mpcWindow, MPC.lowerBound, MPC.upperBound);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(6 * (MPC.mpcWindow + 1) + 2 * MPC.mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 6 * (MPC.mpcWindow + 1) + 2 * MPC.mpcWindow);
    if(!solver.data()->setHessianMatrix(MPC.hessian)) return 1;
    if(!solver.data()->setGradient(MPC.gradient)) return 1;
    if(!solver.data()->setLinearConstraintsMatrix(MPC.linearMatrix)) return 1;
    if(!solver.data()->setLowerBound(MPC.lowerBound)) return 1;
    if(!solver.data()->setUpperBound(MPC.upperBound)) return 1;

    // instantiate the solver
    if(!solver.initSolver()) return 1;

    // controller input and QPSolution vector

    bool driving = false;

    while (ros::ok()){        

        clock_t start = clock();

        // solve the QP problem
        if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

        // get the controller input
        MPC.QPSolution = solver.getSolution();
        MPC.ctr = MPC.QPSolution.block(6 * (MPC.mpcWindow + 1), 0, 2, 1);

        if (MPC.x_dot_ref != 0.){
            MPC.GenProfile(MPC.x_dot_ref, &temp);
            MPC.x_vel_ref = temp;
            MPC.acc = temp;
            driving = true;
            x_pos_ref = x_pos_ref + (MPC.x_vel_ref*1/hz);
        }
        else if(MPC.x_dot_ref == 0.){
            MPC.GenProfileBrake(0., &temp);
            MPC.x_vel_ref = temp;
            // x_vel_ref = 0.;

            if(driving==true){
                // double bacc = -acc;
                MPC.acc = 0;
                if (MPC.x_dot < 0){
                    MPC.xl_ref = -0.05;
                }
                else if (MPC.x_dot > 0){
                    MPC.xl_ref = 0.05;
                }
                else MPC.xl_ref = 0.;
                     
                x_pos_ref = MPC.x_pos_current;
                if(fabs(x_dot)<0.1){
                    driving = false;
                }
            }

            else MPC.acc = 0.;
            MPC.xl_ref = 0.;            
        }

        // //ground inclination
        // double L = 2.15;
        // double eta = atan2(L, rear_laser-front_laser);  // angle calculated by distance sensor. atan2(y,x)
        // double alpha = M_PI/2. - pitch - eta;
        // slope_tau = slope_torque(alpha);
        // cal_xl_ref(x_vel_ref, acc, -alpha, &sol_xl);

        if (cnt >=2000){
            MPC.tau1.data = (ctr[0]*MPC.r)/2 - (MPC.yaw_vel*150) + (MPC.slope_tau/2);
            MPC.tau2.data = (ctr[0]*MPC.r)/2 + (MPC.yaw_vel*150) + (MPC.slope_tau/2);
        }
        MPC.tau1.data = (ctr[0]*MPC.r)/2 - (MPC.yaw_vel*150);
        MPC.tau2.data = (ctr[0]*MPC.r)/2 + (MPC.yaw_vel*150);
        // tau.data = ctr[0]/2;
        MPC.F_l.data = ctr[1];
        // F_l.data = sol_xl;

        pub_wl.publish(MPC.tau1);
        pub_wr.publish(MPC.tau2);
        pub_xl.publish(MPC.F_l);

        // x_pos = ((wl_pos - init_wl_pos)+(wr_pos - init_wr_pos))/2 * r;
        
        if(client.call(link_srv)) MPC.x_pos = link_srv.response.link_state.pose.position.x;
        if(client.call(link_srv)) MPC.x_dot = link_srv.response.link_state.twist.linear.x;
        // x_acc = x_acc/1/hz;

        MPC.x_pos_current = MPC.x_pos - MPC.x_pos_init;
        // MPC.x_dot = ((MPC.wl_dot + MPC.wr_dot)/2) * MPC.r;
        x_pos_ref = x_pos_ref + (MPC.x_dot_ref*1/hz);

        pitchGenProfile(MPC.pitch_ref, &MPC.theta_temp);
        // x_ddot = x_vel_ref - x_dot;
        // cal_xl_ref(x_vel_ref, tau1.data, tau2.data, x_ddot, theta_temp, sol_xl); //x_ddot
        // double xl_vel_ref = sol_xl/1/hz;
        MPC.x0 << MPC.x_pos_current, MPC.xl, MPC.pitch, MPC.x_dot, MPC.xl_dot, MPC.pitch_vel;

        
        if (cnt >=2000){
            MPC.xRef <<  MPC.x_pos_ref, 0, 0, MPC.x_vel_ref, 0, 0;
        }
        
        else {
            MPC.xRef <<  MPC.x_pos_ref, 0, 0, MPC.x_vel_ref, 0, 0;
        }
        // xRef <<  x_pos_ref, xl_ref, 0, x_dot_ref, 0, 0;

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
            MPC.updateConstraintVectors(MPC.x0, MPC.lowerBound, MPC.upperBound);
            if(!solver.updateBounds(MPC.lowerBound, MPC.upperBound)) return 1;

            // sol_xl = 0.235565711055099;
            

            update = true;

        }
        else{

            MPC.castMPCToQPGradient(MPC.Q, MPC.xRef, MPC.mpcWindow, MPC.gradient);
            solver.updateGradient(MPC.gradient);

            // update the constraint bound
            MPC.updateConstraintVectors(MPC.x0, MPC.lowerBound, MPC.upperBound);
            if(!solver.updateBounds(MPC.lowerBound, MPC.upperBound)) return 1;

        }

        ros::spinOnce();
        loop_rate.sleep();

        clock_t end = clock();
        double duration = static_cast<double>(end - start) / CLOCKS_PER_SEC;
        if (duration > 0.002) MPC.over_dt += 1;

        cnt += 1;

        x_append.push_back(MPC.x0(0));
        xl_append.push_back(MPC.x0(1));
        theta_append.push_back(MPC.x0(2));
        xdot_append.push_back(MPC.x0(3));
        xl_dot_append.push_back(MPC.x0(4));
        theta_dot_append.push_back(MPC.x0(5));
        x_ref_append.push_back(MPC.xRef(0));
        xdot_ref_append.push_back(MPC.xRef(3));
        xl_ref_append.push_back(MPC.xRef(1));
        theta_ref_append.push_back(MPC.xRef(2));
        tauL_append.push_back(MPC.tau1.data);
        tauR_append.push_back(MPC.tau2.data);
        Fl_append.push_back(MPC.F_l.data);
        duration_append.push_back(duration);

        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << "dt : " << duration << std::endl;
        // std::cout << "mpc window :" << mpcWindow << std::endl;
        // std::cout << "acceleration : " << x_ddot << std::endl;
        std::cout << "x :" << MPC.x0.transpose() << std::endl;
        std::cout << "xRef :" << MPC.xRef.transpose() << std::endl;
        // std::cout << "vRef :" << xRef(3) << std::endl;
        // std::cout << "vRobot :" << x0(3) << std::endl;
        std::cout << "xlRef :" << MPC.xRef(1) << std::endl;
        std::cout << "Wheel Torque L : " << MPC.tau1.data << std::endl;
        std::cout << "Wheel Torque R : " << MPC.tau2.data << std::endl;
        std::cout << "Cylinder Force : " << MPC.F_l.data << std::endl;
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