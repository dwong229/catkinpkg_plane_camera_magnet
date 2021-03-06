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

double freq;

class CurrentCalc
{
    ros::NodeHandle nh_;
    ros::Subscriber mag_sub_; //xyPix  
    ros::Subscriber goal_sub_;
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

    bool goalreached; //true is goal reached within goalthresh.  //false 
    double goalthresh; //distance metric to consider goal reached.

    // integral term error:
    vector<double> errorx, errory;
    double integrateerror[2];
    int errorhistorylength = 10000;
    
public:
    CurrentCalc()
    {
        //subscribe to kfpose output - xyFiltered
        mag_sub_ = nh_.subscribe("/dakf_pose/magnetactual", 1,
            &CurrentCalc::xyCb, this);

        goal_sub_ = nh_.subscribe("/image_click/uiGoal", 1,
            &CurrentCalc::goalCb, this);

        current_pub_ = nh_.advertise<plane_camera_magnet::roboclawCmd>("currentcmddesired",1);

        goalthresh = 0.001; //1 mm

        errorx.clear();
        errory.clear();
        integrateerror[0] = integrateerror[1] = 0;
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

        errorx.push_back((dgoal.x)/freq);
        errory.push_back((dgoal.y)/freq);

        int errorsize = errorx.size();

        if(errorx.size()>errorhistorylength)
        {
            errorx.erase(errorx.begin(),errorx.begin()+1);
            errory.erase(errory.begin(),errory.begin()+1);

        }

        // +/- 5V
        current_desired.m1 = 0;
        current_desired.m2 = 0;
        current_desired.m3 = 0;
        current_desired.m4 = 0;

        if (dgoal_norm < goalthresh){
            goalreached = true;
            ROS_INFO_STREAM("~~ GOAL REACHED ~~");
            // leave: current_desired = 0 

            // clear error vectors.
            errorx.clear();
            errory.clear();

        }
        else{
            integrateerror[0] = std::accumulate(errorx.begin(),errorx.end(),0.0);
            integrateerror[1] = std::accumulate(errory.begin(),errory.end(),0.0);

            double maxinterror = 100;
            for(int erridx = 0; erridx < 2; erridx++)
                {
                    if(abs(integrateerror[erridx]) > maxinterror)
                    {
                        integrateerror[erridx] = maxinterror * integrateerror[erridx]/abs(integrateerror[erridx]);
                    }
                }

            ROS_INFO_STREAM_THROTTLE(0.1,"integrateerror: " << integrateerror[0] <<", " << integrateerror[1]);


            current_desired.fx = dgoal.x;
            current_desired.fy = dgoal.y;

            current_desired.xdes.assign(1,goal.x);
            current_desired.ydes.assign(1,goal.y);

            double kp = 600;//24000 for distance 2; //500; works well for rubbertri
            double ki = 50;

            double maxvolt = 5;
            double minvolt = 0;
            double xsignal = pow(dgoal.x,1) * kp + integrateerror[0] * ki;
            double ysignal = pow(dgoal.y,1) * kp + integrateerror[1] * ki;


            if (dgoal.x > 0)
            {
              current_desired.m3 = min(abs(xsignal),maxvolt);
              current_desired.m3 = max(current_desired.m3,minvolt);
            }
            else
            {
              current_desired.m1 = min(abs(xsignal),maxvolt);
              current_desired.m1 = max(current_desired.m1,minvolt);
            }

            if (dgoal.y > 0)
            {
              current_desired.m2 = min(abs(ysignal),maxvolt);
              current_desired.m2 = max(current_desired.m2,minvolt);
            }
            else
            {
              current_desired.m4 = min(abs(ysignal),maxvolt);
              current_desired.m4 = max(current_desired.m4,minvolt);
            }

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

    void goalCb(const plane_camera_magnet::xyFiltered& msg) {
        ROS_INFO_STREAM("goal: " << msg.xyWorldX.at(0) << " , " << msg.xyWorldY.at(0));

        goal.x = (float) msg.xyWorldX.at(0);
        goal.y = (float) msg.xyWorldY.at(0);

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "da_controller_noskid"); // initialize node: magnet_track
    ros::NodeHandle n("~");

    CurrentCalc cc;
    freq = 30.0;

    ros::Rate r(freq); //controls current command publishing rate
    while (n.ok()) {
        cc.update();
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}