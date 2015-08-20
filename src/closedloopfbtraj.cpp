#include <ros/ros.h>
#include <trajectory.h>
#include <iostream>
#include <plane_camera_magnet/PositionCommand.h>
#include <plane_camera_magnet/roboclawCmd.h>
#include <plane_camera_magnet/xyFiltered.h>
#include <plane_camera_magnet/nonlinearsolversoln.h>
#include "currentcompute.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h> //for rvis visualization


using namespace std;

Trajectory traj;

plane_camera_magnet::PositionCommand actual;
int numrobot;
static visualization_msgs::Marker goalvis, actualvis, fdesvis;
ros::Publisher actualvis_pub;
//static visualization_msgs::Marker kfpoints, kfline_strip, kfline_list, rawpoints;

void xyFilteredcallback(const plane_camera_magnet::xyFiltered& data)
{
  
  // unpack subscribed values to vector of keypoints
    numrobot = data.actrobot;
    
    if(!data.xyWorldX.empty())
    {
        actual.position.x = double(data.xyWorldX.at(0));
        actual.position.y = double(data.xyWorldY.at(0));
        actual.velocity.x = double(data.xyWorldXdot.at(0));
        actual.velocity.y = double(data.xyWorldYdot.at(0));
    }
    //visualization:
    actualvis.header.frame_id = "/camera_frame";
    actualvis.header.stamp =  ros::Time::now();
    actualvis.ns = "closedloopfbtraj";
    actualvis.action = visualization_msgs::Marker::ADD;
    // rotation of frame in quaternions
    actualvis.pose.orientation.x = actualvis.pose.orientation.y = actualvis.pose.orientation.z =0.0;
    actualvis.pose.orientation.w =fdesvis.pose.orientation.w = 1.0;
    actualvis.type = visualization_msgs::Marker::POINTS;

    actualvis.scale.x = actualvis.scale.y = 0.2;

    actualvis.color.g = 1.0f;
    actualvis.color.a = 1.0;
    actualvis.points.resize(1);
    actualvis.points[0] = actual.position;   
    actualvis_pub.publish(actualvis);


}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "closedloopfbtraj");
    ros::NodeHandle nh("~");

    //gains:
    std::string param_filename;
    nh.param("param_filename", param_filename, std::string("closedloopfbpoint.yml"));
    ROS_INFO_STREAM("param filename " << param_filename);
    cv::FileStorage fscal(param_filename.c_str(), cv::FileStorage::READ);
    //double xgoal = (double)fscal["xgoal"];
    //double ygoal = (double)fscal["ygoal"];
    double kx = (double)fscal["kx"];
    double kv = (double)fscal["kv"];
    double ki = (double)fscal["ki"];


    //trajectory setup:
    // The trajectory filename
    std::string traj_filename;
    nh.param("traj_filename", traj_filename, std::string("traj.csv"));
    traj.set_filename(traj_filename);

    if (traj.LoadTrajectory())
      ROS_INFO("Trajectory loaded");
    else
      ROS_WARN("Trajectory could not be loaded.");

    

    //initialize commands and messages:
    plane_camera_magnet::PositionCommand goal;
    plane_camera_magnet::roboclawCmd roboclawCmdDesired;
    plane_camera_magnet::nonlinearsolversoln solversoln_msg;

    //visualization:
    goalvis.header.frame_id = fdesvis.header.frame_id ="/camera_frame";
    goalvis.header.stamp = fdesvis.header.stamp = ros::Time::now();
    goalvis.ns = fdesvis.ns = "closedloopfbpoint2mag";
    goalvis.action = fdesvis.action = visualization_msgs::Marker::ADD;
    // rotation of frame in quaternions
    goalvis.pose.orientation.x = goalvis.pose.orientation.y = goalvis.pose.orientation.z = 0.0;
    goalvis.pose.orientation.w  = fdesvis.pose.orientation.w = 1.0;
    goalvis.type = visualization_msgs::Marker::SPHERE_LIST;
    goalvis.scale.x = goalvis.scale.y = 0.4;
    goalvis.color.r = 1.0f;
    goalvis.color.a = 1.0;
    goalvis.points.resize(2);
    goalvis.points[0] = goal.position;
    //goalvis.points[1] = goal2.position;

    fdesvis.id = 2;
    fdesvis.type = visualization_msgs::Marker::LINE_LIST;
    fdesvis.scale.x = 0.1;

    fdesvis.color.b = 1.0f;
    fdesvis.color.a = 1.0;

    //Setup publisher:
    ros::Publisher roboCmdDes_pub = nh.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1); // always publish newest command
    ros::Publisher solversoln_pub = nh.advertise<plane_camera_magnet::nonlinearsolversoln>("nonlinearsolversoln",1); // always publish newest command

    ros::Publisher goalvis_pub = nh.advertise<visualization_msgs::Marker>("vis_goal", 10);
    actualvis_pub = nh.advertise<visualization_msgs::Marker>("vis_actual", 10);

    //Setup subscriber
    //ros::Subscriber xyFiltered_sub_ = nh.subscribe("/kf_pose/magnetactual",1, xyFilteredcallback);
    ros::Subscriber xyFiltered_sub_ = nh.subscribe("/kfpose_mag1/magnetactual",1, xyFilteredcallback);

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
    //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
    b << 1., 2., 600., 4., 10700, 0.; // F = [0.0283, 0]
    double freq = 100;
    ros::Rate loop_rate(freq);

    cout << "Press enter when ready to begin traj:" << endl;
    cin.ignore(1);
    // initialize time
    traj.set_start_time();
    //cout << size.traj() << endl;

    double integrateerror[2];
    integrateerror[0] = 0;
    integrateerror[1] = 0;

    while(!traj.isCompleted())
    {
        // loop until end of traj
        //traj.UpdateGoal(goal);

        // if goal updated, set integrator error to 0:
        int updateflag;
        updateflag = traj.UpdateGoaldx(actual,goal);
        if (updateflag == 1)
        {
            cout << "Next waypoint!" << endl;
            integrateerror[0] = 0.;
            integrateerror[1] = 0.;

        }

        //cout << "GOAL: X: " << goal.position.x << ", " << goal.position.y << ", " << goal.velocity.x << ", " << goal.velocity.y << endl;
        goalvis.points.resize(1);
        goalvis.points[0] = goal.position;    
        ros::spinOnce();

        // take in position + velocity of robot from xyFilteredcallback
        actual.acceleration.x = 0;
        actual.acceleration.y = 0;
        double mass = 4*0.0000707;
        //double mass = 1.;

        // compute F desired
        integrateerror[0] += (goal.position.x - actual.position.x)/freq;
        integrateerror[1] += (goal.position.y - actual.position.y)/freq;
        //cout << "integrateerror: " << integrateerror[0] <<", " << integrateerror[1] << endl;
        double maxinterror = 100;
        for(int erridx = 0; erridx < 2; erridx++)
            {
                if(abs(integrateerror[erridx]) > maxinterror)
                {
                    integrateerror[erridx] = maxinterror * integrateerror[erridx]/abs(integrateerror[erridx]);
                }
            }
        //cout << "integrateerror: " << integrateerror[0] <<", " << integrateerror[1] << endl;
        ROS_INFO_STREAM_THROTTLE(0.5,"integrateerror: " << integrateerror[0] <<", " << integrateerror[1]);

        // compute F desired
        double Fdes[2];
        //Fdes[0] = goal.acceleration.x * mass + kx * (goal.position.x - actual.position.x) + kv * (goal.position.x - actual.velocity.x);
        //Fdes[1] = goal.acceleration.y * mass + kx * (goal.position.y - actual.position.y) + kv * (goal.position.y - actual.velocity.y);
        Fdes[0] = goal.acceleration.x * mass + kx * (goal.position.x - actual.position.x) + kv * (goal.position.x - actual.velocity.x) + ki * integrateerror[0];
        Fdes[1] = goal.acceleration.y * mass + kx * (goal.position.y - actual.position.y) + kv * (goal.position.y - actual.velocity.y) + ki * integrateerror[1];

        //cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;
        geometry_msgs::Point fdespoint1;
        fdespoint1.x = actual.position.x + Fdes[0];
        fdespoint1.y = actual.position.y + Fdes[1];
        fdespoint1.z = 0;
        //fdespoint2.x = mag2_actual.position.x + Fdes[2];
        //fdespoint2.y = mag2_actual.position.y + Fdes[3];
        //fdespoint2.z = 0;

        //cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;
        //update fdes
        fdesvis.points.resize(4);
        fdesvis.points[0] = actual.position;
        fdesvis.points[1] = fdespoint1;
        //cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;

        // compute currents to send to roboclaw using nonlinear solver.
        // update magnet variables:
        magnet1.x = actual.position.x;
        magnet1.y = actual.position.y;
        magnet1.Fx = Fdes[0];
        magnet1.Fy = Fdes[1];
        magnet1.Mxmat = Mx(magnet1.x,magnet1.y,coil.R,coil.d);
        magnet1.Mymat = My(magnet1.x,magnet1.y,coil.R,coil.d);
        magnet1.Dxmat = Dx(magnet1.x,magnet1.y,coil.R,coil.d);
        magnet1.Dymat = Dy(magnet1.x,magnet1.y,coil.R,coil.d);
        magnet1.Bmat = computeBmat(magnet1.x,magnet1.y,coil.R,coil.d);

        //solve:
        CoilFunctor functor(coil, magnet1); // functor( ) add arguments here.
        LevenbergMarquardt<CoilFunctor> lm(functor);
        info = lm.minimize(b);   
        //cout << "info: " << info << endl; //LM error: 1: RelativeReductionTooSmall, 2:RelativeErrorTooSmall, 3:RelativeErrorAndReductionTooSmall

// solversoln_msg: ///////////////////
        solversoln_msg.header.stamp = ros::Time::now();
        solversoln_msg.info = info;


        double xtemp[] = {actual.position.x, actual.position.y};
        double dxtemp[] = {goal.position.x - actual.position.x, goal.position.y - actual.position.y};


        solversoln_msg.xactual = vector<double> (xtemp, xtemp+ 2);
        solversoln_msg.dx = vector<double> (dxtemp,dxtemp +2);
        

        solversoln_msg.Fdes = vector<double> (Fdes,Fdes + 2);
        
        VectorXd error(6);
        functor.operator()(b,error);
        //cout << "error: " << error.transpose() << endl;
        solversoln_msg.error = vector<double> (error.data(),error.data() + error.rows() * error.cols());

        //  add notion of error to detemine if solution should be published
        double errorsum = pow(error[0]*error[0] + error[1]*error[1],0.5); // sum only the force
        // error not assigned yet.
        //cout << "errorsum: " << errorsum << endl;
        if(errorsum < 0.00001) // || info == 1)
        {
        
        vector<double> btemp;
        vector<double> currtemp;

        for(int i = 0; i<6; i++)
        {
            btemp.push_back(b[i]);
            if(i<4)
            {
                currtemp.push_back(b[i]); 
            }
        }
        //solversoln_msg.solnvec = vector<double> (btemp,btemp + 6);
        //solversoln_msg.current = vector<double> (currtemp,currtemp + 4);

        solversoln_msg.solnvec = btemp;
        solversoln_msg.current = btemp;

        roboclawCmdDesired.header.stamp = ros::Time::now();
        roboclawCmdDesired.m1 = int(btemp.at(0));
        roboclawCmdDesired.m2 = int(btemp.at(1));
        roboclawCmdDesired.m3 = int(btemp.at(2));
        roboclawCmdDesired.m4 = int(btemp.at(3));

        // publish pwm commands to roboclawCmdDesired
        }
        else
        {
            // if no solution, random start b.
            cout << "re-init b" << endl;
            b = VectorXd::Random(6)*1000; // F = [0.0283, 0]
            // test for large coil 3:
            //b[2] = 1000;
            cout << "b: " << b << endl;
            roboclawCmdDesired.header.stamp = ros::Time::now();
            roboclawCmdDesired.m1 = int(0);
            roboclawCmdDesired.m2 = int(0);
            roboclawCmdDesired.m3 = int(0);
            roboclawCmdDesired.m4 = int(0);
        }
        roboCmdDes_pub.publish(roboclawCmdDesired);
        solversoln_pub.publish(solversoln_msg);
        goalvis_pub.publish(fdesvis);
        goalvis_pub.publish(goalvis);
        ros::spinOnce();
        loop_rate.sleep();

    }
    // set roboclaw to 0:
    roboclawCmdDesired.header.stamp = ros::Time::now();
    roboclawCmdDesired.m1 = int(0);
    roboclawCmdDesired.m2 = int(0);
    roboclawCmdDesired.m3 = int(0);
    roboclawCmdDesired.m4 = int(0);

    // publish pwm commands to roboclawCmdDesired
    roboCmdDes_pub.publish(roboclawCmdDesired);
    return 0;

}
