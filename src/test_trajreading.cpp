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

    //cout << size.traj() << endl;
    while(!traj.isCompleted())
    {

        // loop until end of traj
        traj.UpdateGoal(goal);

        cout << "X: " << goal.position.x << ", " << goal.position.y << ", " << goal.velocity.x << ", " << goal.velocity.y << endl;
        ros::spinOnce();
        
        // take in position of robot
        actual.position.x = -2.0;
        actual.position.y = 0;
        actual.velocity.x = 0;
        actual.velocity.y = 0;
        actual.acceleration.x = 0;
        actual.acceleration.y = 0;
        double kx = 1;
        double kv = 1;
        double mass = 1;

        // compute F desired
        double Fdes[2];
        Fdes[0] = goal.acceleration.x * mass + kx * (goal.position.x - actual.position.x) + kv * (goal.position.x - actual.velocity.x);
        Fdes[1] = goal.acceleration.y * mass + kx * (goal.position.y - actual.position.y) + kv * (goal.position.y - actual.velocity.y);

        cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;

        // compute currents to send to roboclaw using Newton Raphson
        // *** NEED TO FINISH ***/

        roboclawCmdDesired.header.stamp = ros::Time::now();
        roboclawCmdDesired.m1 = 1;
        roboclawCmdDesired.m2 = 2;
        roboclawCmdDesired.m3 = 3;
        roboclawCmdDesired.m4 = 4;

        // publish pwm commands to roboclawCmdDesired

        loop_rate.sleep();

    }
 
    return 0;

}
