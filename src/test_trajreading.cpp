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

Trajectory traj;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_traj");
    ros::NodeHandle nh("~");

    // The trajectory filename
    std::string traj_filename;
    nh.param("traj_filename", traj_filename, std::string("traj.csv"));
    traj.set_filename(traj_filename);

    if (traj.LoadTrajectory())
      ROS_INFO("Trajectory loaded");
    else
      ROS_WARN("Trajectory could not be loaded.");

    ros::Rate loop_rate(10);    
    // initialize time
    traj.set_start_time();
    plane_camera_magnet::PositionCommand goal;

    plane_camera_magnet::PositionCommand actual;

    plane_camera_magnet::roboclawCmd roboclawCmdDesired;

    //Setup publisher:
    ros::Publisher roboCmdDes_pub = nh.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1); // always publish newest command

    // define structs Coil and Magnet to be used in solver:
    Coil coil;
    coil.R = 8900;
    coil.d = 57.5;
    Magnet magnet1;
    magnet1.gamma = 6500;
    
    const int n =6; // 4I , 2 lambda
    int info;
    VectorXd b(n);
    //b << 0.004 * pow(10,-3),-0.5 * pow(10,-3),-0.1 * pow(10,-3) ,-0.03 * pow(10,-3),0.5,0.5;
    b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]

    
    //cout << size.traj() << endl;
    while(!traj.isCompleted())
    {
        // loop until end of traj
        traj.UpdateGoal(goal);

        cout << "X: " << goal.position.x << ", " << goal.position.y << ", " << goal.velocity.x << ", " << goal.velocity.y << endl;
        ros::spinOnce();
        
        // take in position of robot
        //actual.position.x = -2.0;
        //actual.position.y = 0;
        actual.position.x = goal.position.x;
        actual.position.y = goal.position.y;
        actual.velocity.x = 0;
        actual.velocity.y = 0;
        actual.acceleration.x = 0;
        actual.acceleration.y = 0;
        double kx = 0.;
        double kv = 0.;
        //double mass = 4*0.0000707;
        double mass = 1.;

        // compute F desired
        double Fdes[2];
        Fdes[0] = goal.acceleration.x * mass + kx * (goal.position.x - actual.position.x) + kv * (goal.position.x - actual.velocity.x);
        Fdes[1] = goal.acceleration.y * mass + kx * (goal.position.y - actual.position.y) + kv * (goal.position.y - actual.velocity.y);

        cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;

        // compute currents to send to roboclaw using nonlinear solver.
        // update magnet variables:
        magnet1.x = actual.position.x;
        magnet1.y = actual.position.y;
        magnet1.Fx = Fdes[0];
        magnet1.Fy = Fdes[1];
        magnet1.Mxmat = Mx(magnet1.x,magnet1.y,coil.R,coil.d);
        magnet1.Mymat = My(magnet1.x,magnet1.y,coil.R,coil.d);

        //solve:
        b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
        CoilFunctor functor(coil, magnet1); // functor( ) add arguments here.
        LevenbergMarquardt<CoilFunctor> lm(functor);
        info = lm.minimize(b);   
        cout << "info: " << info << endl; //LM error: 1: RelativeReductionTooSmall, 2:RelativeErrorTooSmall, 3:RelativeErrorAndReductionTooSmall

        //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
        VectorXd current(4);
        VectorXd c = b.head(4);
        MatrixXd Bmat = computeBmat(magnet1.x,magnet1.y,coil.R,coil.d);
        current = c * pow(c.transpose()*Bmat.transpose()*Bmat*c,0.5);
        cout << "b: " << b.transpose() << endl;
        cout << "current: \n " << current << endl;

        roboclawCmdDesired.header.stamp = ros::Time::now();
        roboclawCmdDesired.m1 = int(current[0]);
        roboclawCmdDesired.m2 = int(current[1]);
        roboclawCmdDesired.m3 = int(current[2]);
        roboclawCmdDesired.m4 = int(current[3]);

        // publish pwm commands to roboclawCmdDesired
        roboCmdDes_pub.publish(roboclawCmdDesired);
        ros::spinOnce();

        loop_rate.sleep();

    }
 
    return 0;

}
