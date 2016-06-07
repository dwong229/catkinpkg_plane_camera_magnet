// Listen to joystick desired position and angle:
//rqt_plot currentcalc_desiredfb/roboclawcmddesired/m1:m2:m3:m4

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <plane_camera_magnet/xyFiltered.h>
#include <plane_camera_magnet/roboclawCmd.h>
#include <geometry_msgs/Pose.h>
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
#include "currentcompute.h"

using namespace std;

static ros::Publisher roboclaw_pub;

#define PI 3.14159265

class ComputeCurrent{
public:
    void publish_to_roboclaw() {
        roboclaw_pub_.publish(roboclawDesired);
    }
    void update() {
        if (last_joy_.buttons.size() != 0) {
            publish_current_update(last_joy_,last_actual_);
        }
    }

    void publish_current_update(const sensor_msgs::Joy &joymsg, const geometry_msgs::Pose &actual) {
         //position of magnet from camera feedback
        magnet_actual.x = actual.position.x/1000;  // convert to m
        magnet_actual.y = actual.position.y/1000;  // convert to m

        //unpack joy: position and orientation
        double joy_orientation;
        //if(std::abs(joymsg.axes.at(axis_Lx))>zerocheck)
            //magnet_joy.Fx = -joymsg.axes.at(axis_Lx) * pow(10,-8); //desired force from joystick
           
        //if(std::abs(joymsg.axes.at(axis_Ly))>zerocheck)
            //magnet_joy.Fy = joymsg.axes.at(axis_Ly) * pow(10,-8);

        magnet_joy.Fx = -alphaF*joymsg.axes.at(axis_crossx) * Fmag + (1-alphaF)*last_F[0]; //desired force from joystick
        magnet_joy.Fy =  alphaF*joymsg.axes.at(axis_crossy) * Fmag + (1-alphaF)*last_F[1]; //desired force from joystick

        if(std::abs(joymsg.axes.at(axis_Rx))>zerocheck ||std::abs(joymsg.axes.at(axis_Ry))>zerocheck){
            joy_orientation = atan2(joymsg.axes.at(axis_Ry),-joymsg.axes.at(axis_Rx));
            

            // determine direction of rotation: + CCW, - CW
            double a,b,c,d,diffang,angstepsize;
            a = cos(magnet_orientation); 
            b = -sin(magnet_orientation); //flip sign for flipped x in joy.
            
            c = cos(joy_orientation); //-msg.axes.at(axis_Rx)
            d = sin(joy_orientation); //msg.axes.at(axis_Ry)
            diffang = atan2(b*c + a*d,a*c-b*d);
            angstepsize = 0.0175*3; //
            if(std::abs(diffang)>angstepsize){
                //ROS_INFO_STREAM("joy_orientation: " << joy_orientation);
                //ROS_INFO_STREAM("diffang: " << diffang);
                magnet_orientation += copysign(angstepsize,diffang); //angstepsize rad per update
            }
            else{
                magnet_orientation = joy_orientation;
            }
            //magnet_orientation += msg.axes.at(axis_Rx);
        }

        double joygain = 1;
        th = magnet_orientation;
        // let orientation of the magnet always be 180.
        th = PI;

        double Bx = Bmag * cos(th) - Bearth[0];
        double By = Bmag * sin(th) - Bearth[1];

        Vector2d mvec(Bx,By);
        mvec = magnet_actual.gamma/sqrt(Bx * Bx + By * By)*mvec;

        //cout << "mvec" << endl << mvec << endl;

        MatrixXd Bmat = computeBmat(magnet_actual.x,magnet_actual.y,coil.R,coil.d);
        //cout << "Bmat" << endl << Bmat << endl;
        //MatrixXd 

        Matrix4d A;
        //cout << "A" << endl << A << endl;

        A.block(0,0,2,4) = Bmat;
        //cout << "A row 1-2" << endl << A << endl;
        A.block(2,0,1,4) = mvec.transpose() * Dx(magnet_actual.x,magnet_actual.y,coil.R,coil.d);
        A.block(3,0,1,4) = mvec.transpose() * Dy(magnet_actual.x,magnet_actual.y,coil.R,coil.d);
        //cout << "A " << endl << A << endl;

        //cout << "A inverse is:" << endl << A.inverse() << endl;

        Vector4d BF;
        BF << Bx,By,magnet_joy.Fx,magnet_joy.Fy;

        //cout << "BF: " << endl << BF << endl;
        Vector4d Isolve = A.inverse() * BF *Iscale;
              
        // check:
        Vector4d bfcheck;
        bfcheck = A*Isolve;

        roboclawDesired.header.stamp = ros::Time::now();
        roboclawDesired.header.seq = joymsg.header.seq;
        // save bf:
        roboclawDesired.bx = Bx;
        roboclawDesired.by = By;
        roboclawDesired.fx = magnet_joy.Fx;
        roboclawDesired.fy = magnet_joy.Fy;
      

        Vector4d tempdI = (last_I - Isolve).cwiseAbs();
        double maxdiffI = tempdI.maxCoeff();

        ROS_INFO_STREAM("bfdesired: " << BF.transpose());
        ROS_INFO_STREAM("I: " << Isolve);


        if( maxdiffI < maxdiffIthresh ) {// large enough diff:{}
        // save current to roboclaw msg
            roboclawDesired.m1 = Isolve[0];
            roboclawDesired.m2 = Isolve[1];
            roboclawDesired.m3 = Isolve[2];
            roboclawDesired.m4 = Isolve[3];

        }
        else{
            cout << "Large current jump, set current to 0" << endl;

            nullroboclawDesired();
        }
        
        //ROS_INFO_STREAM("(x,y): " << magnet_actual.x << " , " << magnet_actual.y);
        //ROS_INFO_STREAM("bfdesired: " << BF.transpose());

        if(magnet_joy.Fx==0 && magnet_joy.Fy ==0 && std::abs(joymsg.axes.at(axis_Rx))<zerocheck && std::abs(joymsg.axes.at(axis_Ry))<zerocheck){
            cout << "No inputs.  Coils off." << endl;
            nullroboclawDesired();
        }
        else{   
        //    ROS_INFO_STREAM("bfcheck: " << bfcheck.transpose());
        //    ROS_INFO_STREAM("I: " << Isolve.transpose());
        }

        publish_to_roboclaw();
        //update last F and last I
        last_F[0] = magnet_joy.Fx;
        last_F[1] = magnet_joy.Fy;
        last_I = Isolve;
    }

    void nullroboclawDesired() {
        roboclawDesired.m1 = 0.0;
        roboclawDesired.m2 = 0.0;
        roboclawDesired.m3 = 0.0;
        roboclawDesired.m4 = 0.0;
    }

    void joyCB(const sensor_msgs::Joy &msg) {
        if (last_joy_.buttons.size() == 0)
            last_joy_ = msg;
        else {
            last_joy_.header = msg.header;
            last_joy_.axes = msg.axes;
        for (int i = 0; i < last_joy_.buttons.size(); i++)
            last_joy_.buttons.at(i) |= msg.buttons.at(i);
        }
    }

    void actualCB(const plane_camera_magnet::xyFiltered &msg) {
        if (msg.xySize[0]> 0.0) {
            //cout << "Update last_actual" << endl;

            last_actual_.position.x = msg.xyWorldX[0];
            last_actual_.position.y = msg.xyWorldY[0];
//            last_actual_.orientation// orientation
        }
        else{
            //last_actual_ = msg;
            cout << "No tri detection" << endl;
        }
    }

    ComputeCurrent() {
        // subscriber and publishers:
        // xyPix: /joyickvisualization/joy_xypix 
        mag_sub_ = n_.subscribe("/joy",20, &ComputeCurrent::joyCB, this);
        //actual_sub_ = n_.subscribe("/filterpose/xyFiltered",1, &ComputeCurrent::actualCB, this);
        actual_sub_ = n_.subscribe("/kf_pose/magnetactual",1, &ComputeCurrent::actualCB, this);

        // publisher:
        // roboclaw
        roboclaw_pub_ = n_.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1);

        // Constants
        Bearth << -0.00004,0.; // -40 uT
        coil.d = .0575;
        coil.R = 0.075;
        magnet_actual.gamma = 35.8 * pow(10,-6);    
    }

private:
  ros::NodeHandle n_{"~"};
  ros::Subscriber mag_sub_; //xyPix  
  ros::Subscriber actual_sub_;
  ros::Publisher roboclaw_pub_; //roboclaw

  geometry_msgs::Pose last_actual_;
  plane_camera_magnet::roboclawCmd roboclawDesired;
  Vector4d last_I;


  // variables
  Coil coil;
  Magnet magnet_actual;
  Magnet magnet_joy;
  
  double last_F[2] = {0,0};
  const double alphaF = 0.5; //0-1

  double th;
  double Bx,By;

  Vector2d Bearth;
  // constants
  const double Bmag = 50. * pow(10,-6);  // Tesla
  const double Fmag = 1 * pow(10,-7); //N
  const double Iscale = 20000; // for current
  const double maxdiffIthresh = 10000;
   // joystick:
  double magnet_orientation{0.0};
  double zerocheck = 0.1;
  sensor_msgs::Joy last_joy_;

  // button mappings
  const int axis_Lx{0};
  const int axis_Ly{1};
  const int axis_Rx{3};
  const int axis_Ry{4};
  const int axis_Lt{2};
  const int axis_Rt{5};
  const int axis_crossy{7};
  const int axis_crossx{6};
  
  const int button_a{0};
  const int button_b{1};
  const int button_x{2};
  const int button_y{3};  
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "currentcalc");
    ros::NodeHandle n("~");

    ComputeCurrent cc;   

    ros::Rate r(20.0);
    while (n.ok()) {
        cc.update();
        r.sleep();
        ros::spinOnce();
    }
    return 0;

}
