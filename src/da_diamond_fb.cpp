// Move back and forth along x axis.  Use coil 1 and 3 only.
// Listen to kf_pose/xyPix
// 
//rqt_plot currentcalc_desiredfb/roboclawcmddesired/m1:m2:m3:m4

#include <ros/ros.h>
#include <plane_camera_magnet/xyFiltered.h>
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
        if (last_pose_.xyWorldX.size() != 0){
        publish_current_update(last_pose_);
        }
    }

    void publish_current_update(const plane_camera_magnet::xyFiltered &msg) {
        // compute current based on position
        //unpack position and orientation


        
        magnet.x = msg.xyWorldX[0];  // convert to m
        magnet.y = msg.xyWorldY[0];  // convert to m
        //cout << "msg: " << endl;

        if(magnet.x< -xlimit)
        {
            traj_segment = 2;
            cout << "Move ^->" << endl;
        }
        else if (magnet.y>ylimit)
        {    traj_segment = 3;
            cout << "Move v ->" << endl;
        }
        else if(magnet.x>xlimit)
        {
            traj_segment = 4;
            cout << "Move v <-" << endl;
        }
        else if(magnet.y < -ylimit)
        {
            traj_segment = 1;
            cout << "Move ^ <-" << endl;

        }
        magnet.Fx = 0. * pow(10,-8); //need to update this
        magnet.Fy = 0. * pow(10,-8); // need to update this
        th = msg.xyAnglerad[0];

        
        double Iscale = 10000;
        Vector4d Isolve;
        Isolve << 0,0,0,0;

        if(traj_segment == 4)
        {
            //moving towards -xlimit
            // turn on coil 1
            Isolve[3] = -Iscale;
        }
        else if(traj_segment == 1)
            Isolve[0] = -Iscale;
        else if(traj_segment == 2)
            Isolve[1] = Iscale;
        else
        {
            Isolve[2] = Iscale;
        }
        
        //ROS_INFO_STREAM("I: " << Isolve.transpose());

        // save current to roboclaw msg
        roboclawDesired.header.stamp = ros::Time::now();
        roboclawDesired.header.seq = msg.header.seq;
        roboclawDesired.m1 = Isolve[0];
        roboclawDesired.m2 = Isolve[1];
        roboclawDesired.m3 = Isolve[2];
        roboclawDesired.m4 = Isolve[3];

        publish_to_roboclaw();
    }

    void xyFilteredCB(const plane_camera_magnet::xyFiltered &msg) {
        if (last_pose_.xyWorldX.size() == 0){
            last_pose_ = msg;
            cout << "No msg" << endl;
        }
        else {
            //cout << "Update lastpose" << endl;

            last_pose_.header = msg.header;
            last_pose_.xyWorldX    = msg.xyWorldX; 
            last_pose_.xyWorldY    = msg.xyWorldY;  
            last_pose_.xyAnglerad  = msg.xyAnglerad;
            last_pose_.xyWorldXdot = msg.xyWorldXdot;
            last_pose_.xyWorldYdot = msg.xyWorldYdot;
        }
         
    }

    ComputeCurrent() {
        // subscriber and publishers:
        // xyPix: /joyickvisualization/joy_xypix 
        //mag_sub_ = n_.subscribe("/joystickvisualization/joy_xypix",20, &ComputeCurrent::xyPixCB, this);

        // subscribe to magnet position
        mag_sub_ = n_.subscribe("/kf_pose/magnetactual",20, &ComputeCurrent::xyFilteredCB, this);

        // publisher:
        // roboclaw
        roboclaw_pub_ = n_.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1);

        // Constants
        Bearth << -0.00004,0.; // -40 uT
        coil.d = .0575;
        coil.R = 0.075;
        magnet.gamma = 35.8 * pow(10,-6);    
    }

private:
  ros::NodeHandle n_{"~"};
  ros::Subscriber mag_sub_; //xyFiltered  
  ros::Publisher roboclaw_pub_; //roboclaw

  plane_camera_magnet::xyFiltered last_pose_;
  plane_camera_magnet::roboclawCmd roboclawDesired;

  int traj_segment; // 1 positive x, 2 negative x

  // variables
  Coil coil;
  Magnet magnet;
  
  double th;
  double Bx,By;

  Vector2d Bearth;

  // constants
  const double Bmag = 100. * pow(10,-6);  // Tesla
  const double xlimit = 10; //mm

  const double ylimit = 10; //mm

   

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
