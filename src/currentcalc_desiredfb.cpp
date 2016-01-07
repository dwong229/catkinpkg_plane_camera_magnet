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

// Listen to xyPix results: data.magx [mm], data.magy [mm], data.angle [rad]

// Port data to x,y,th for computing I

// Compute I

class ComputeCurrent{
public:
    void publish_to_roboclaw() {
        // finish
    }

    void publish_current_update(const Vector4d &current) {
        // compute new current (from test_currentI_desiredBF)

        // save current to roboclaw msg
    }

    void xyPixCB(const plane_camera_magnet::xyPix &msg) {
        //unpack position and orientation

    }

    ComputeCurrent() {
        // subscriber and publishers:
        // xyPix: /joyickvisualization/joy_xypix 

        // publisher:
        // roboclaw

    }

private:
  ros::NodeHandle n_{"~"};
  ros::Subscriber _sub_; //xyPix  
  ros::Publisher _pub_; //roboclaw

  plane_camera_magnet::xyPix joyxyPix;

  // constants

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystickvisualization");
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
}