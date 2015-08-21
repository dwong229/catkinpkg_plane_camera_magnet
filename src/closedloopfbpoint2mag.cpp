#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <numeric>
#include <functional>
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
static visualization_msgs::Marker goalvis, fdesvis;
void xyFilteredcallback1(const plane_camera_magnet::xyFiltered& data)
{

  // unpack subscribed values to vector of keypoints
    numrobot = data.actrobot;
    
    if(!data.xyWorldX.empty())
    {
        //cout << "Update1 " <<endl;
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
        //cout << "Update2 " <<endl;
        mag2_actual.position.x = double(data.xyWorldX.at(0));
        mag2_actual.position.y = double(data.xyWorldY.at(0));
        mag2_actual.velocity.x = double(data.xyWorldXdot.at(0));
        mag2_actual.velocity.y = double(data.xyWorldYdot.at(0));
        newmsg = true;
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "closedloopfbpoint2mag");
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
    
    //newmsg = false;

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
    goalvis.points[0] = goal1.position;
    goalvis.points[1] = goal2.position;

    fdesvis.id = 2;
    fdesvis.type = visualization_msgs::Marker::LINE_LIST;
    fdesvis.scale.x = 0.1;

    fdesvis.color.b = 1.0f;
    fdesvis.color.a = 1.0;



    const int n =4; // 4I , 2 lambda
    int info;
    VectorXd b(n);
    //b << 0.004 * pow(10,-3),-0.5 * pow(10,-3),-0.1 * pow(10,-3) ,-0.03 * pow(10,-3),0.5,0.5;
    //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
    //b << 1., 2., 1., 4.; // F = [0.0283, 0]
    b << 1.,1.,100.,1.;
    double freq = 100.;
    ros::Rate loop_rate(freq);

    
    //cout << size.traj() << endl;
    int count = 0;
    double integrateerror[4];
    int errorhistorylength = 100000;
    vector<double> errorx, errory, errorx2, errory2;
    errorx.clear();
    errory.clear();
    errorx2.clear();
    errory2.clear();
    integrateerror[0] = integrateerror[1] = integrateerror[2] = integrateerror[3] = 0;

    
    ros::Duration(1.5).sleep();
    ros::spinOnce();


    while(ros::ok())
        //if(newmsg)
        {
        
        // take in position of robot
        mag1_actual.acceleration.x = 0;
        mag1_actual.acceleration.y = 0;
        
        double mass = 4*0.0000707;
        //double mass = 1.;

        // compute F desired
        errorx.push_back((goal1.position.x - mag1_actual.position.x)/freq);
        errory.push_back((goal1.position.y - mag1_actual.position.y)/freq);
        errorx2.push_back((goal2.position.x - mag2_actual.position.x)/freq);
        errory2.push_back((goal2.position.y - mag2_actual.position.y)/freq);
        //cout << "goal1: " << goal1.position << endl;
        //cout << "mag1: " << mag1_actual.position << endl;
        int errorsize = errorx.size();
        //cout << "Errorx: " << errorx.at(0) << endl;
        //for(int i = 0; i< errorsize; i++)
        

        
        if(errorx.size()>errorhistorylength)
        {
            errorx.erase(errorx.begin(),errorx.begin()+1);
            errory.erase(errory.begin(),errory.begin()+1);
            errorx2.erase(errorx2.begin(),errorx2.begin()+1);
            errory2.erase(errory2.begin(),errory2.begin()+1);
               
        }

        integrateerror[0] = std::accumulate(errorx.begin(),errorx.end(),0.0);
        integrateerror[1] = std::accumulate(errory.begin(),errory.end(),0.0);
        integrateerror[2] = std::accumulate(errorx2.begin(),errorx2.end(),0.0);
        integrateerror[3] = std::accumulate(errory2.begin(),errory2.end(),0.0);
        

        double maxinterror = 500;
        for(int erridx = 0; erridx < 4; erridx++)
            {
                if(abs(integrateerror[erridx]) > maxinterror)
                {
                    integrateerror[erridx] = maxinterror * integrateerror[erridx]/abs(integrateerror[erridx]);
                }
            }
        ROS_INFO_STREAM_THROTTLE(0.1,"integrateerror: " << integrateerror[0] <<", " << integrateerror[1] << ", " << integrateerror[2] << ", " << integrateerror[3] );

        double Fdes[n];

        Fdes[0] = goal1.acceleration.x * mass + kx * (goal1.position.x - mag1_actual.position.x) + kv * (goal1.position.x - mag1_actual.velocity.x) + ki * integrateerror[0];
        Fdes[1] = goal1.acceleration.y * mass + kx * (goal1.position.y - mag1_actual.position.y) + kv * (goal1.position.y - mag1_actual.velocity.y) + ki * integrateerror[1];
        Fdes[2] = goal2.acceleration.x * mass + kx2 * (goal2.position.x - mag2_actual.position.x) + kv2 * (goal2.position.x - mag2_actual.velocity.x) + ki2 * integrateerror[2];
        Fdes[3] = goal2.acceleration.y * mass + kx2 * (goal2.position.y - mag2_actual.position.y) + kv2 * (goal2.position.y - mag2_actual.velocity.y) + ki2 * integrateerror[3];

        geometry_msgs::Point fdespoint1,fdespoint2;
        fdespoint1.x = mag1_actual.position.x + Fdes[0];
        fdespoint1.y = mag1_actual.position.y + Fdes[1];
        fdespoint1.z = 0;
        fdespoint2.x = mag2_actual.position.x + Fdes[2];
        fdespoint2.y = mag2_actual.position.y + Fdes[3];
        fdespoint2.z = 0;

        //cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;
        //update fdes
        fdesvis.points.resize(4);
        fdesvis.points[0] = mag1_actual.position;
        fdesvis.points[1] = fdespoint1;
        fdesvis.points[2] = mag2_actual.position;
        fdesvis.points[3] = fdespoint2;

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
		
		// compute error: F(current) - Fdesired
		VectorXd error(4);
   		functor.operator()(b,error);
   		//cout << "error: " << error.transpose() << endl;
        solversoln_msg.error = vector<double> (error.data(),error.data() + error.rows() * error.cols());

        //  add notion of error to detemine if solution should be published
        double errorsum = error[0] + error[1] + error[2] + error[3];
        //cout << "sum error: " << errorsum << endl;
        
        double maxcurr = abs(b[0]);

        for(int i = 0; i<4; i++)
        {
            if(abs(b[i]) > maxcurr)
            {
                maxcurr = abs(b[i]);
            }
        }


        if(errorsum < 0.00001 && maxcurr < 513. ) // || info == 1)
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
            b = VectorXd::Random(4)*500; 
            //b << 1.,1.,1.,20.;
            // test for large coil 3:
            //b[2] = 1000;
            //cout << "b: " << b << endl;
            roboclawCmdDesired.header.stamp = ros::Time::now();
            roboclawCmdDesired.m1 = 0;
            roboclawCmdDesired.m2 = 0;
            roboclawCmdDesired.m3 = 0;
            roboclawCmdDesired.m4 = 0;
        
            // publish pwm commands to roboclawCmdDesired
            roboCmdDes_pub.publish(roboclawCmdDesired);
        }

        solversoln_pub.publish(solversoln_msg);
        //loop_rate.sleep();
        goalvis_pub.publish(goalvis);
        goalvis_pub.publish(fdesvis);
    
        ros::spinOnce();
        ++count;
    }

    return 0;
}