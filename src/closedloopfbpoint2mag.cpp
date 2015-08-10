#include <ros/ros.h>
#include <iostream>
#include <plane_camera_magnet/PositionCommand.h>
#include <plane_camera_magnet/roboclawCmd.h>
#include <plane_camera_magnet/xyFiltered.h>
#include <plane_camera_magnet/nonlinearsolversoln.h>
#include "currentcompute.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h> //for rvis visualization

using namespace std;

// User input a point and control to stay at that point.

plane_camera_magnet::PositionCommand mag1_actual, mag2_actual;
int numrobot;
bool newmsg;
static visualization_msgs::Marker goalvis;

void xyFilteredcallback1(const plane_camera_magnet::xyFiltered& data)
{

  // unpack subscribed values to vector of keypoints
    numrobot = data.actrobot;
    
    if(!data.xyWorldX.empty())
    {
        mag1_actual.position.x = double(data.xyWorldX.at(0));
        mag1_actual.position.y = double(data.xyWorldY.at(0));
        mag1_actual.velocity.x = double(data.xyWorldXdot.at(0));
        mag1_actual.velocity.y = double(data.xyWorldYdot.at(0));
        newmsg = true;
    }
    

}
void xyFilteredcallback2(const plane_camera_magnet::xyFiltered& data)
{

   // unpack subscribed values to vector of keypoints   
    if(!data.xyWorldX.empty())
    {
        mag2_actual.position.x = double(data.xyWorldX.at(0));
        mag2_actual.position.y = double(data.xyWorldY.at(0));
        mag2_actual.velocity.x = double(data.xyWorldXdot.at(0));
        mag2_actual.velocity.y = double(data.xyWorldYdot.at(0));
        newmsg = true;
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "closedloopfbpoint");
    ros::NodeHandle nh("~");

    std::string param_filename;
    nh.param("param_filename", param_filename, std::string("closedloopfbpoint.yml"));
    ROS_INFO_STREAM("param filename " << param_filename);
    cv::FileStorage fscal(param_filename.c_str(), cv::FileStorage::READ);
    double xgoal1 = (double)fscal["xgoal"];
    double ygoal1 = (double)fscal["ygoal"];
    double kx = (double)fscal["kx"];
    double kv = (double)fscal["kv"];
    double ki = (double)fscal["ki"];
    double xgoal2 = (double)fscal["xgoal2"];
    double ygoal2 = (double)fscal["ygoal2"];
    double kx2 = (double)fscal["kx2"];
    double kv2 = (double)fscal["kv2"];
    double ki2 = (double)fscal["ki2"];

    plane_camera_magnet::PositionCommand goal1, goal2;

    plane_camera_magnet::roboclawCmd roboclawCmdDesired;
    plane_camera_magnet::nonlinearsolversoln solversoln_msg;
    //Setup publisher:
    ros::Publisher roboCmdDes_pub = nh.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1); // always publish newest command
    ros::Publisher solversoln_pub = nh.advertise<plane_camera_magnet::nonlinearsolversoln>("nonlinearsolversoln",1); // always publish newest command
    ros::Publisher goalvis_pub = nh.advertise<visualization_msgs::Marker>("vis_goal", 10);


    ros::Subscriber xyFiltered1_sub_ = nh.subscribe("/kfpose_mag1/magnetactual",1, xyFilteredcallback1);
    ros::Subscriber xyFiltered2_sub_ = nh.subscribe("/kfpose_mag2/magnetactual",1, xyFilteredcallback2);

    // define structs Coil and Magnet to be used in solver:
    Coil coil;
    coil.R = 8900;
    coil.d = 57.5;
    Magnet magnet;
    magnet.gamma = 6500;

    goal1.position.x = xgoal1;
    goal1.position.y = ygoal1;
    goal1.velocity.x = 0;
    goal1.velocity.y = 0;
    goal1.acceleration.x = 0;
    goal1.acceleration.y = 0;

    goal2.position.x = xgoal2;
    goal2.position.y = ygoal2;
    goal2.velocity.x = 0;
    goal2.velocity.y = 0;
    goal2.acceleration.x = 0;
    goal2.acceleration.y = 0;

    // publish goals positions once:
    goalvis.header.frame_id = "/camera_frame";
    goalvis.header.stamp = ros::Time::now();
    goalvis.ns = "closedloopfbpoint2mag";
    goalvis.action = visualization_msgs::Marker::ADD;
    // rotation of frame in quaternions
    goalvis.pose.orientation.x = goalvis.pose.orientation.y = goalvis.pose.orientation.z = 0.0;
    goalvis.pose.orientation.w  = 1.0;

    goalvis.type = visualization_msgs::Marker::SPHERE_LIST;

    goalvis.scale.x = goalvis.scale.y = 0.4;

    goalvis.color.r = 1.0f;
    goalvis.color.a = 1.0;

    goalvis.points.resize(2);
    goalvis.points[0] = goal1.position;
    goalvis.points[1] = goal2.position;
   
    const int n =4; // 4I , 2 lambda
    int info;
    VectorXd b(n);
    //b << 0.004 * pow(10,-3),-0.5 * pow(10,-3),-0.1 * pow(10,-3) ,-0.03 * pow(10,-3),0.5,0.5;
    //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
    //b << 1., 2., 1., 4.; // F = [0.0283, 0]
    b << 1.,1.,1.,20.;
    double freq = 100.;
    ros::Rate loop_rate(freq);

    
    //cout << size.traj() << endl;
    int count = 0;
    double integrateerror[2];
    integrateerror[0] = integrateerror[1] = integrateerror[2] = integrateerror[3] = 0;


    while(ros::ok())
    {
        // take in position of robot
        mag1_actual.acceleration.x = 0;
        mag1_actual.acceleration.y = 0;
        
        double mass = 4*0.0000707;
        //double mass = 1.;

        // compute F desired
        //integrateerror[0] += (goal1.position.x - mag1_actual.position.x)/freq;
        //integrateerror[1] += (goal1.position.y - mag1_actual.position.y)/freq;
        //integrateerror[2] += (goal2.position.x - mag2_actual.position.x)/freq;
        //integrateerror[3] += (goal2.position.y - mag2_actual.position.y)/freq;
        //cout << "integrateerror: " << integrateerror[0] << endl;

        double Fdes[n];

        Fdes[0] = goal1.acceleration.x * mass + kx * (goal1.position.x - mag1_actual.position.x) + kv * (goal1.position.x - mag1_actual.velocity.x) + ki * integrateerror[0];
        Fdes[1] = goal1.acceleration.y * mass + kx * (goal1.position.y - mag1_actual.position.y) + kv * (goal1.position.y - mag1_actual.velocity.y) + ki * integrateerror[1];
        Fdes[2] = goal2.acceleration.x * mass + kx2 * (goal2.position.x - mag2_actual.position.x) + kv2 * (goal2.position.x - mag2_actual.velocity.x) + ki2 * integrateerror[2];
        Fdes[3] = goal2.acceleration.y * mass + kx2 * (goal2.position.y - mag2_actual.position.y) + kv2 * (goal2.position.y - mag2_actual.velocity.y) + ki2 * integrateerror[3];

        //cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;

        // compute currents to send to roboclaw using nonlinear solver.
        // update magnet variables:
        magnet.x = mag1_actual.position.x;
        magnet.y = mag1_actual.position.y;
        magnet.Fx = Fdes[0];
        magnet.Fy = Fdes[1];
        magnet.Mxmat = Mx(magnet.x,magnet.y,coil.R,coil.d);
        magnet.Mymat = My(magnet.x,magnet.y,coil.R,coil.d);
        magnet.Bmat = computeBmat(magnet.x,magnet.y,coil.R,coil.d);

        magnet.x2 = mag2_actual.position.x;
        magnet.y2 = mag2_actual.position.y;
        magnet.Fx2 = Fdes[2];
        magnet.Fy2 = Fdes[3];
        magnet.Mxmat2 = Mx(magnet.x2,magnet.y2,coil.R,coil.d);
        magnet.Mymat2 = My(magnet.x2,magnet.y2,coil.R,coil.d);
        magnet.Bmat2 = computeBmat(magnet.x2,magnet.y2,coil.R,coil.d);
        //solve:
        //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
        CoilFunctor2 functor(coil, magnet); // functor( ) add arguments here.
        LevenbergMarquardt<CoilFunctor2> lm(functor);
        info = lm.minimize(b);   
        //cout << "info: " << info << endl; //LM error: 1: RelativeReductionTooSmall, 2:RelativeErrorTooSmall, 3:RelativeErrorAndReductionTooSmall
        
        // solversoln_msg: ///////////////////
        solversoln_msg.header.stamp = ros::Time::now();
        solversoln_msg.info = info;

        double xtemp[] = {mag1_actual.position.x, mag1_actual.position.y, mag2_actual.position.x, mag2_actual.position.y};
        double dxtemp[] = {goal1.position.x - mag1_actual.position.x, goal1.position.y - mag1_actual.position.y, goal2.position.x - mag2_actual.position.x, goal2.position.y - mag2_actual.position.y};

        solversoln_msg.xactual = vector<double> (xtemp, xtemp+ 4);
        solversoln_msg.dx = vector<double> (dxtemp,dxtemp +4);
        

        solversoln_msg.Fdes = vector<double> (Fdes,Fdes + 4);
        solversoln_msg.info = info;
        // error not assigned yet.

        if(info==2) // || info == 1)
        {
            //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
            VectorXd current(4);
            current = b;
            //cout << "b: " << b.transpose() << endl;
            //cout << "current: \n " << current << endl;
            vector<double> btemp;
            vector<double> currtemp;

            for(int i = 0; i<n; i++)
            {
                btemp.push_back(b[i]);
                currtemp.push_back(current[i]); 
            }
            //solversoln_msg.solnvec = vector<double> (btemp,btemp + 6);
            //solversoln_msg.current = vector<double> (currtemp,currtemp + 4);

            solversoln_msg.solnvec = btemp;
            solversoln_msg.current = currtemp;
        
            roboclawCmdDesired.header.stamp = ros::Time::now();
            roboclawCmdDesired.m1 = int(current[0]);
            roboclawCmdDesired.m2 = int(current[1]);
            roboclawCmdDesired.m3 = int(current[2]);
            roboclawCmdDesired.m4 = int(current[3]);
        
            // publish pwm commands to roboclawCmdDesired
            roboCmdDes_pub.publish(roboclawCmdDesired);
        }
        else
        {
            // if no solution, random start b.
            cout << "re-init b" << endl;
            //b = VectorXd::Random(n)*100; // F = [0.0283, 0]
            b << 1.,1.,1.,20.;
            // test for large coil 3:
            //b[2] = 1000;
            cout << "b: " << b << endl;
        }

        solversoln_pub.publish(solversoln_msg);
        //loop_rate.sleep();
        goalvis_pub.publish(goalvis);

        ros::spinOnce();
        ++count;
    }
    return 0;
}