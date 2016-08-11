// rosbag play -l sample_dakf_pose_magnetactual.bag 
#include <ros/ros.h>
#include <stdio.h>

#include "currentcompute.h" // contains struct : Coil, Magnet
#include <Eigen/SVD>
#include <Eigen/Geometry> 
#include <plane_camera_magnet/xyFiltered.h>
#include <plane_camera_magnet/roboclawCmd.h>
#include <opencv2/opencv.hpp>


using Eigen::MatrixXd;
using namespace Eigen;
using namespace std;
using std::sqrt;

class CurrentCalc
{
    ros::NodeHandle nh_;
    ros::Subscriber mag_sub_; //xyPix  
    ros::Publisher current_pub_; //roboclaw
    
    plane_camera_magnet::xyFiltered last_pose_;
    plane_camera_magnet::roboclawCmd current_desired;

    // variables
    Coil coil;
    Magnet magnet;

    double amin;
    Vector2d fd;  //assign desired force
    Vector2d m; //magnetic moment

    cv::Point2f goal; //goal in world coordinates
    
public:
    CurrentCalc()
    {
        //subscribe to kfpose output - xyFiltered
        mag_sub_ = nh_.subscribe("/dakf_pose/magnetactual", 1,
            &CurrentCalc::xyCb, this);

        current_pub_ = nh_.advertise<plane_camera_magnet::roboclawCmd>("currentcmddesired",1);

        goal.x = 0.01; // [m]
        goal.y = 0.01;

    }

    ~CurrentCalc()
    {

    }

    void update() {
        if (last_pose_.xyWorldX.size() != 0){
            current_pub_.publish(current_desired);
        }
    }

    void publish_current_update() {

        ROS_INFO_STREAM("magnet xy: " << last_pose_.xyWorldX[0] << last_pose_.xyWorldY[0]);
        magnet.x = last_pose_.xyWorldX[0];
        magnet.y = last_pose_.xyWorldY[0];

        cv::Point2f dgoal;
        double dgoal_norm;

        dgoal.x = goal.x - magnet.x;
        dgoal.y = goal.y - magnet.y;
        
        dgoal_norm = sqrt(pow(dgoal.x,2) + pow(dgoal.y,2));

        current_desired.fx = dgoal.x;
        current_desired.fy = dgoal.y;

        current_desired.xdes.assign(1,goal.x);
        current_desired.ydes.assign(1,goal.y);

        // +/- 5V
        current_desired.m1 = 0;
        current_desired.m2 = 0;
        current_desired.m3 = 0;
        current_desired.m4 = 0;

        double kp = 500;

        double maxvolt = 5;
        double xsignal = dgoal.x * kp;
        double ysignal = dgoal.y * kp;


        if (dgoal.x > 0)
        {
          current_desired.m3 = min(abs(xsignal),maxvolt);
        }
        else
        {
          current_desired.m1 = min(abs(xsignal),maxvolt);
        }

        if (dgoal.y > 0)
        {
          current_desired.m2 = min(abs(ysignal),maxvolt);
        }
        else
        {
          current_desired.m4 = min(abs(ysignal),maxvolt);
        }
        //ROS_INFO_STREAM("current: " << last_pose_.xyWorldX[0] << last_pose_.xyWorldY[0]);

        //publish current
        //update();
    }

    void xyCb(const plane_camera_magnet::xyFiltered& msg) 
    {
        if (last_pose_.xyWorldX.size() == 0){
            last_pose_ = msg;
            cout << "No xyWorldX" << endl;
        }

        else{
            cout << "Update lastpose" << endl;

            last_pose_.header = msg.header;
            last_pose_.xyWorldX = msg.xyWorldX;
            last_pose_.xyWorldY = msg.xyWorldY;
            last_pose_.xyAngledeg = msg.xyAngledeg;

            publish_current_update();
        }

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "da_controller_noskid"); // initialize node: magnet_track
    ros::NodeHandle n("~");

    CurrentCalc cc;

    ros::Rate r(10.0); //controls current command publishing rate
    while (n.ok()) {
        cc.update();
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}