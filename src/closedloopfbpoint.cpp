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

plane_camera_magnet::PositionCommand actual;
int numrobot;
bool newmsg;
static visualization_msgs::Marker kfpoints, kfline_strip, kfline_list, rawpoints, fdesvis;

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
    double xgoal = (double)fscal["xgoal"];
    double ygoal = (double)fscal["ygoal"];
    double kx = (double)fscal["kx"];
    double kv = (double)fscal["kv"];
    double ki = (double)fscal["ki"];

    plane_camera_magnet::PositionCommand goal;

    plane_camera_magnet::roboclawCmd roboclawCmdDesired;
    plane_camera_magnet::nonlinearsolversoln solversoln_msg;
    //Setup publisher:
    ros::Publisher roboCmdDes_pub = nh.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1); // always publish newest command
    ros::Publisher solversoln_pub = nh.advertise<plane_camera_magnet::nonlinearsolversoln>("nonlinearsolversoln",1); // always publish newest command

    //ros::Subscriber xyFiltered_sub_ = nh.subscribe("/kf_pose/magnetactual",1, xyFilteredcallback);
    ros::Subscriber xyFiltered_sub_ = nh.subscribe("/kfpose_mag1/magnetactual",1, xyFilteredcallback);

    // define structs Coil and Magnet to be used in solver:
    Coil coil;
    coil.R = 8900;
    coil.d = 57.5;
    Magnet magnet1;
    magnet1.gamma = 6500;

    goal.position.x = xgoal;
    goal.position.y = ygoal;
    goal.velocity.x = 0;
    goal.velocity.y = 0;
    goal.acceleration.x = 0;
    goal.acceleration.y = 0;
    
    const int n =6; // 4I , 2 lambda
    int info;
    VectorXd b(n);
    //b << 0.004 * pow(10,-3),-0.5 * pow(10,-3),-0.1 * pow(10,-3) ,-0.03 * pow(10,-3),0.5,0.5;
    //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
    b << 1., 2., 1., 4., -107., 0.; // F = [0.0283, 0]
    double freq = 100;
    ros::Rate loop_rate(freq);

    
    //cout << size.traj() << endl;
    int count = 0;
    double integrateerror[2];
    integrateerror[0] = 0;
    integrateerror[1] = 0;

    while(ros::ok())
    {
        // take in position of robot
        actual.acceleration.x = 0;
        actual.acceleration.y = 0;
        
        double mass = 4*0.0000707;
        //double mass = 1.;

        // compute F desired
        //integrateerror[0] += (goal.position.x - actual.position.x)/freq;
        //integrateerror[1] += (goal.position.y - actual.position.y)/freq;
        //cout << "integrateerror: " << integrateerror[0] <<", " << integrateerror[1] << endl;

        double Fdes[2];

        Fdes[0] = goal.acceleration.x * mass + kx * (goal.position.x - actual.position.x) + kv * (goal.position.x - actual.velocity.x) + ki * integrateerror[0];
        Fdes[1] = goal.acceleration.y * mass + kx * (goal.position.y - actual.position.y) + kv * (goal.position.y - actual.velocity.y) + ki * integrateerror[1];

        //cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;

        // compute currents to send to roboclaw using nonlinear solver.
        // update magnet variables:
        magnet1.x = actual.position.x;
        magnet1.y = actual.position.y;
        magnet1.Fx = Fdes[0];
        magnet1.Fy = Fdes[1];
        magnet1.Mxmat = Mx(magnet1.x,magnet1.y,coil.R,coil.d);
        magnet1.Mymat = My(magnet1.x,magnet1.y,coil.R,coil.d);

        //solve:
        //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
        
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
        solversoln_msg.info = info;

        VectorXd error(6);
        functor.operator()(b,error);
        //cout << "error: " << error.transpose() << endl;
        solversoln_msg.error = vector<double> (error.data(),error.data() + error.rows() * error.cols());

        //  add notion of error to detemine if solution should be published
        double errorsum = pow(error[0]*error[0] + error[1]*error[1],0.5); // sum only the force
        // error not assigned yet.
        cout << "errorsum: " << errorsum << endl;
        if(errorsum < 0.00001) // || info == 1)
        {
        //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
        cout << "Set I" << endl;
        VectorXd current(4);
        VectorXd c = b.head(4);
        MatrixXd Bmat = computeBmat(magnet1.x,magnet1.y,coil.R,coil.d);
        current = c * pow(c.transpose()*Bmat.transpose()*Bmat*c,0.5);
        //cout << "b: " << b.transpose() << endl;
        //cout << "current: \n " << current << endl;
        vector<double> btemp;
        vector<double> currtemp;

        for(int i = 0; i<6; i++)
        {
            btemp.push_back(b[i]);
            if(i<4)
            {
                currtemp.push_back(current[i]); 
            }
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
            b = VectorXd::Random(6)*-1000; // F = [0.0283, 0]
            // test for large coil 3:
            //b[2] = 1000;
            cout << "b: " << b << endl;

        }

        solversoln_pub.publish(solversoln_msg);
        loop_rate.sleep();
        ros::spinOnce();
        ++count;
    }
 
    return 0;

}

