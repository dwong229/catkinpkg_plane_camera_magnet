// listen to:
//          - kf_pose -> filteredPose
//          - coil params [R,m...]
//          - read traj.csv file 
// output: 
//          - roboclaw command.

// sandbox for closedlooproboclaw


#include <ros/ros.h>
#include <trajectory.h>
#include <iostream>
#include <plane_camera_magnet/PositionCommand.h>
#include <plane_camera_magnet/roboclawCmd.h>
#include "currentcompute.h"

using namespace std;

Trajectory traj, traj2;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_traj");
    ros::NodeHandle nh("~");

    // The trajectory filename
    std::string traj_filename, traj_filename2;
    nh.param("traj_filename", traj_filename, std::string("traj2mag1withI.csv"));
    traj.set_filename(traj_filename);
    if (traj.LoadTrajectory())
      ROS_INFO("Trajectory loaded");
    else
      ROS_WARN("Trajectory could not be loaded.");


    nh.param("traj_filename2", traj_filename2, std::string("traj2mag2withI.csv"));
    traj2.set_filename(traj_filename2);
    if (traj2.LoadTrajectory())
      ROS_INFO("Trajectory loaded");
    else
      ROS_WARN("Trajectory could not be loaded.");   

    ros::Rate loop_rate(10);    
    // initialize time
    traj.set_start_time();
    plane_camera_magnet::PositionCommand goal, goal2;

    plane_camera_magnet::PositionCommand mag1_actual, mag2_actual;

    plane_camera_magnet::roboclawCmd roboclawCmdDesired;

    //Setup publisher:
    ros::Publisher roboCmdDes_pub = nh.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1); // always publish newest command

    // define structs Coil and Magnet to be used in solver:
    Coil coil;
    coil.R = 8900;
    coil.d = 57.5;
    Magnet magnet;
    magnet.gamma = 6500;
    
    const int n =4; // 4I , 2 lambda
    int info;
    VectorXd b(n);
    //b << 0.004 * pow(10,-3),-0.5 * pow(10,-3),-0.1 * pow(10,-3) ,-0.03 * pow(10,-3),0.5,0.5;
    //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]

    traj.UpdateGoal(goal);
    cout << "Goal: " << goal.position.x << endl;
    mag1_actual = goal;
    mag2_actual = goal2;
    double integrateerror[4];
    integrateerror[0] = integrateerror[1] = integrateerror[2] = integrateerror[3] = 0.;

    //cout << size.traj() << endl;
    while(!traj.isCompleted())
    {
        // loop until end of traj
        traj.UpdateGoaldx(mag1_actual, goal);
        traj2.UpdateGoaldx(mag2_actual, goal2);

        cout << "Goal: " << goal.position.x << ", " << goal.position.y << ", " << goal2.position.x << ", " << goal2.position.y << endl;
        ros::spinOnce();
        //cout << "Iprecompute: " << goal.Iprecompute.at(0) << endl; // << ", " << goal.Iprecompute[1] << ", " << goal.Iprecompute[2] << ", " goal.Iprecompute[3] << endl;
        
        // take in position of robot
        //mag1_actual.position.x = -2.0;
        //mag1_actual.position.y = 0;
        mag1_actual.position.x = goal.position.x;
        mag1_actual.position.y = goal.position.y;
        mag1_actual.velocity.x = 0;
        mag1_actual.velocity.y = 0;
        mag1_actual.acceleration.x = 0;
        mag1_actual.acceleration.y = 0;
        double kx , kv , ki , kx2 , kv2, ki2;
        kx = kv = ki = kx2 = kv2 = ki2 = 0.;
        //double kv = 0.;

        //double mass = 4*0.0000707;
        double mass = 0.01;

        // compute F desired
        double Fdes[n];

        Fdes[0] = goal.acceleration.x * mass + kx * (goal.position.x - mag1_actual.position.x) + kv * (goal.position.x - mag1_actual.velocity.x) + ki * integrateerror[0];
        Fdes[1] = goal.acceleration.y * mass + kx * (goal.position.y - mag1_actual.position.y) + kv * (goal.position.y - mag1_actual.velocity.y) + ki * integrateerror[1];
        Fdes[2] = goal2.acceleration.x * mass + kx2 * (goal2.position.x - mag2_actual.position.x) + kv2 * (goal2.position.x - mag2_actual.velocity.x) + ki2 * integrateerror[2];
        Fdes[3] = goal2.acceleration.y * mass + kx2 * (goal2.position.y - mag2_actual.position.y) + kv2 * (goal2.position.y - mag2_actual.velocity.y) + ki2 * integrateerror[3];

        cout << "Fdes: " << Fdes[0] << ", " << Fdes[1] << ", " << Fdes[2] << ", " << Fdes[3] << endl;

        // compute currents to send to roboclaw using nonlinear solver.
        // update magnet variables:
        magnet.x = mag1_actual.position.x;
        magnet.y = mag1_actual.position.y;
        magnet.Fx = Fdes[0];
        magnet.Fy = Fdes[1];
        magnet.Mxmat = Mx(magnet.x,magnet.y,coil.R,coil.d);
        magnet.Mymat = My(magnet.x,magnet.y,coil.R,coil.d);        
        magnet.Bmat = computeBmat(magnet.x,magnet.y,coil.R,coil.d);
        magnet.Dxmat = Dx(magnet.x,magnet.y,coil.R,coil.d);
        magnet.Dymat = Dy(magnet.x,magnet.y,coil.R,coil.d);

        magnet.x2 = mag2_actual.position.x;
        magnet.y2 = mag2_actual.position.y;
        magnet.Fx2 = Fdes[2];
        magnet.Fy2 = Fdes[3];
        magnet.Mxmat2 = Mx(magnet.x2,magnet.y2,coil.R,coil.d);
        magnet.Mymat2 = My(magnet.x2,magnet.y2,coil.R,coil.d);
        magnet.Bmat2 = computeBmat(magnet.x2,magnet.y2,coil.R,coil.d);
        magnet.Dxmat2 = Dx(magnet.x2,magnet.y2,coil.R,coil.d);
        magnet.Dymat2 = Dy(magnet.x2,magnet.y2,coil.R,coil.d);

        //solve:
        // VectorXd b; vector<double> goal.Iprecompute;
        //b.resize(goal.Iprecompute.size());

       //VectorXd::Map(&b[0],goal.Iprecompute.size()) = goal.Iprecompute;
        b = VectorXd::Map(goal.Iprecompute.data(),goal.Iprecompute.size());
        cout << "Iprecompute: \n " << b.transpose() << endl; // << ", " << goal.Iprecompute[1] << ", " << goal.Iprecompute[2] << ", " goal.Iprecompute[3] << endl;
        VectorXd db;
        db = VectorXd::Random(4)*20 - VectorXd::Ones(4)*10;
        
        b += db;
        cout << "db: " << db.transpose() << endl;
        cout << "Iprecompute: \n " << b.transpose() << endl;

        //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
        //b = goal.Iprecompute; 
        CoilFunctor2 functor(coil, magnet); // functor( ) add arguments here.
        LevenbergMarquardt<CoilFunctor2> lm(functor);
        info = lm.minimize(b);   
        cout << "info: " << info << endl; //LM error: 1: RelativeReductionTooSmall, 2:RelativeErrorTooSmall, 3:RelativeErrorAndReductionTooSmall

        //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
        cout << "current: \n " << b.transpose() << endl;
        cout << "=======" << endl;
        roboclawCmdDesired.header.stamp = ros::Time::now();
        roboclawCmdDesired.m1 = int(b[0]);
        roboclawCmdDesired.m2 = int(b[1]);
        roboclawCmdDesired.m3 = int(b[2]);
        roboclawCmdDesired.m4 = int(b[3]);

        // publish pwm commands to roboclawCmdDesired
        roboCmdDes_pub.publish(roboclawCmdDesired);
        ros::spinOnce();

        mag1_actual = goal;
        mag2_actual = goal2;

        loop_rate.sleep();


    }
 
    return 0;

}
