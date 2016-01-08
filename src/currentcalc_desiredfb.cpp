// Listen to joystick desired position and angle:

#include <ros/ros.h>
#include <plane_camera_magnet/xyPix.h>
#include <plane_camera_magnet/roboclawCmd.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <typeinfo>
#include <Eigen/LU>
//#include <unsupported/Eigen/NonLinearOptimization>
#include "currentcompute.h"

using namespace std;

static ros::Publisher roboclaw_pub;

#define PI 3.14159265

// Listen to xyPix results: data.magx [mm], data.magy [mm], data.angle [rad]

// Port data to x,y,th for computing I

// Compute I


class ComputeCurrent{
public:
    void publish_to_roboclaw() {
        roboclaw_pub_.publish(roboclawDesired);
    }
    void update() {
        if (last_pose_.magx.size() != 0){
        publish_current_update(last_pose_);
        }
    }

    void publish_current_update(const plane_camera_magnet::xyPix &msg) {
        // compute new current (from test_currentI_desiredBF)
        //unpack position and orientation

        magnet.x = msg.magx[0]/1000;  // convert to m
        magnet.y = msg.magy[0]/1000;  // convert to m
        cout << "msg: " << endl;

        magnet.Fx = 0. * pow(10,-8); //need to update this
        magnet.Fy = 0. * pow(10,-8); // need to update this
        th = msg.angle[0];


        coil.d = .0575;
        coil.R = 0.075;

   /*magnet.x =  0.01;
   magnet.y =  0.0;
   magnet.Fx = 1 * pow(10,-8);
   magnet.Fy = 1.* pow(10,-8);*/
        magnet.gamma = 35.8 * pow(10,-6);

        //ROS_INFO_STREAM("X: " << magnet.x);

        double Bx = Bmag * cos(th);
        double By = Bmag * sin(th);

        Vector2d mvec(Bx,By);
        mvec = magnet.gamma/sqrt(Bx * Bx + By * By)*mvec;

        //cout << "mvec" << endl << mvec << endl;

        MatrixXd Bmat = computeBmat(magnet.x,magnet.y,coil.R,coil.d);
        //cout << "Bmat" << endl << Bmat << endl;
        //MatrixXd 

        Matrix4d A;
        //cout << "A" << endl << A << endl;

        A.block(0,0,2,4) = Bmat;
        //cout << "A row 1-2" << endl << A << endl;
        A.block(2,0,1,4) = mvec.transpose() * Dx(magnet.x,magnet.y,coil.R,coil.d);
        A.block(3,0,1,4) = mvec.transpose() * Dy(magnet.x,magnet.y,coil.R,coil.d);
        //cout << "A " << endl << A << endl;

        //cout << "A inverse is:" << endl << A.inverse() << endl;

        Vector4d BF;
        BF << Bx,By,magnet.Fx,magnet.Fy;
        //cout << "BF: " << endl << BF << endl;
        Vector4d Isolve = A.inverse() * BF;
        ROS_INFO_STREAM("I: " << Isolve);
        

        // save current to roboclaw msg
        roboclawDesired.header.stamp = ros::Time::now();
        roboclawDesired.header.seq = msg.header.seq;
        roboclawDesired.m1 = Isolve[0];
        roboclawDesired.m2 = Isolve[1];
        roboclawDesired.m3 = Isolve[2];
        roboclawDesired.m4 = Isolve[3];

        publish_to_roboclaw();
    }

    void xyPixCB(const plane_camera_magnet::xyPix &msg) {
        if (last_pose_.magx.size() == 0){
            last_pose_ = msg;
            cout << "No magx" << endl;
        }
        else {
            cout << "Update lastpose" << endl;

            last_pose_.header = msg.header;
            last_pose_.magx = msg.magx;
            last_pose_.magy = msg.magx;
            last_pose_.angle = msg.angle;
        }
         
    }

    ComputeCurrent() {
        // subscriber and publishers:
        // xyPix: /joyickvisualization/joy_xypix 
        mag_sub_ = n_.subscribe("/joystickvisualization/joy_xypix",20, &ComputeCurrent::xyPixCB, this);

        // publisher:
        // roboclaw
        roboclaw_pub_ = n_.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1);

    }

private:
  ros::NodeHandle n_{"~"};
  ros::Subscriber mag_sub_; //xyPix  
  ros::Publisher roboclaw_pub_; //roboclaw

  plane_camera_magnet::xyPix last_pose_;
  plane_camera_magnet::roboclawCmd roboclawDesired;

  // variables
  Coil coil;
  Magnet magnet;
  
  double th;
  double Bx,By;

  // constants
  const double Bmag = 100. * pow(10,-6);  // Tesla
  
   

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "currentcalc");
    ros::NodeHandle n("~");

    ComputeCurrent cc;   

    ros::Rate r(10.0);
    while (n.ok()) {
        cc.update();
        r.sleep();
        ros::spinOnce();
    }
    return 0;

}
