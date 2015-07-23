//Filter position using Kalman Filter 
//Position from filter.cpp -> /filterpose/xyFiltered


#include <ros/ros.h>
#include <plane_camera_magnet/xyFiltered.h>
#include "filter.h"
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h> //for rvis visualization
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;

static ros::Publisher kfmarker_pub;
static ros::Publisher rawmarker_pub;
static ros::Publisher kfxy_pub;
static KalmanFilter kf;

static plane_camera_magnet::xyFiltered magnetactual_msg;
static visualization_msgs::Marker kfpoints, kfline_strip, kfline_list, rawpoints;

// calibration file to convert to world
double pix2m;
double centerpixx;
double centerpixy;

static void xyFiltered_callback(const plane_camera_magnet::xyFiltered& data)
{
    //initialize t_last_proc
    static ros::Time t_last_proc = data.header.stamp;

    // compute dt after inialization look
    double dt = (data.header.stamp - t_last_proc).toSec();
    t_last_proc = data.header.stamp; // update last time

    kf.processUpdate(dt);
    const KalmanFilter::Measurement_t meas(data.xyPixX[0],data.xyPixY[0]);

    
    if(data.xySize[0]!=0)
    {
        cout << "~detection~" << endl;
        static ros::Time t_last_meas = data.header.stamp;
        double meas_dt = (data.header.stamp - t_last_meas).toSec();
        t_last_meas = data.header.stamp;
        kf.measurementUpdate(meas, meas_dt);
    }

    const KalmanFilter::State_t state = kf.getState();
    const KalmanFilter::ProcessCov_t proc_noise = kf.getProcessNoise();
    // KF computation end

    //populate odom_msg to be published
    /* // For xyFiltered
    odom_msg.header.seq = data.header.seq;
    odom_msg.header.stamp = data.header.stamp;
    odom_msg.header.frame_id = data.header.frame_id;
    //odom_msg.xyPixX = std::vector<double> {state(0)};
    //odom_msg.xyPixY = std::vector<double> {state(1)};
    
    vector<double> outx (1.0,0.0);
    vector<double> outy (1.0,0.0);

    outx.at(0) = state(0);
    outy.at(0) = state(1);
    
    odom_msg.xyPixX = outx;
    odom_msg.xyPixY = outy;
    */

    // troubleshoot cout:
    cout << "proc noise" << proc_noise << endl;
    cout << "state" << state << endl;
    std::cout << "dt: " << dt << std::endl;
    std::cout << "filterpose: "<< data.xyPixX[0] << "," << data.xyPixY[0] << std::endl;
    std::cout << "xyPixXY: " << state(0) << ", " << state(1) << std::endl;
    std::cout << "Velocity: " << state(2) << ", " << state(3) << std::endl;
    std::cout << "==========" << std::endl;
    
    rawpoints.header.seq = kfpoints.header.seq = kfline_list.header.seq = kfline_strip.header.seq = data.header.seq;
    rawpoints.header.stamp = kfpoints.header.stamp = kfline_list.header.stamp = kfline_strip.header.stamp = data.header.stamp;
    rawpoints.header.frame_id = kfpoints.header.frame_id = kfline_strip.header.frame_id = kfline_list.header.frame_id = data.header.frame_id; //camera id

    kfpoints.id = 0; //robot id?
    kfline_strip.id = 1;
    kfline_list.id = 2;
    rawpoints.id = 3; 

    rawpoints.ns = kfpoints.ns = kfline_strip.ns = kfline_list.ns = "raw_kf_points";
    rawpoints.action = kfpoints.action = kfline_strip.action = kfline_list.action = visualization_msgs::Marker::ADD;
    rawpoints.pose.orientation.w = kfpoints.pose.orientation.w = kfline_strip.pose.orientation.w = kfline_list.pose.orientation.w = 1.0;


    kfpoints.type = visualization_msgs::Marker::POINTS;
    rawpoints.type = visualization_msgs::Marker::POINTS;
    kfline_strip.type = visualization_msgs::Marker::LINE_STRIP;
    kfline_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    kfpoints.scale.x = 0.2;
    kfpoints.scale.y = 0.2;
    rawpoints.scale.x = 0.1;
    rawpoints.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    kfline_strip.scale.x = 0.1;
    kfline_list.scale.x = 0.1;

    // set colors
    kfpoints.color.g = 1.0f;
    kfpoints.color.a = 1.0;
    kfline_list.color.r = 1.0;
    kfline_list.color.a = 1.0;

    rawpoints.color.g = 0.0f;
    rawpoints.color.r = 0.0f;
    rawpoints.color.b = 0.0f;
    rawpoints.color.a = 0.5;    

    //populate xy posn:
    geometry_msgs::Point p;
    p.x = data.xyPixX[0]/100;
    p.y = data.xyPixY[0]/100;
    p.z = 0.0;

    //rawpoints.points.push_back(p);
    rawpoints.points.resize(1);
    rawpoints.points[0] = p;

    p.x = state(0)/100;
    p.y = state(1)/100;

    //kfpoints.points.push_back(p);
    kfpoints.points.resize(1);
    kfpoints.points[0] = p;

    //velocity: kfline_list:
    geometry_msgs::Point v;
    double vmag;
    vmag = pow(pow(state(2),2) + pow(state(3),2),0.5);
    v.x = p.x + state(2)/50;
    v.y = p.y + state(3)/50;
    v.z = 0;

    kfline_list.points.resize(2);
    kfline_list.points[0] = p;
    kfline_list.points[1] = v;

    //convert pix into world coords:
    // 4. Convert to xySorted coords
    vector<double> xyWorldX (data.actrobot,0.0);
    vector<double> xyWorldY (data.actrobot,0.0);
    vector<double> xyWorldXdot (data.actrobot,0.0);
    vector<double> xyWorldYdot (data.actrobot,0.0);
    
    xyWorldX.at(0) = (state(0) - centerpixx)/pix2m;
    xyWorldY.at(0) = (-state(1) + centerpixy)/pix2m;
    xyWorldXdot.at(0) = state(2)/pix2m;
    xyWorldYdot.at(0) = state(3)/pix2m;

        
    //pupulate /kf_pose/magnetactual 
    magnetactual_msg.header.stamp = data.header.stamp;
    magnetactual_msg.header.frame_id = data.header.frame_id;
    magnetactual_msg.xyPixX.assign(1,state(0));
    magnetactual_msg.xyPixY.assign(1,state(1));
    magnetactual_msg.xySize = data.xySize;
    magnetactual_msg.xyAngledeg.assign(1,0);
    magnetactual_msg.xyAnglerad.assign(1,0);
    magnetactual_msg.actrobot = data.actrobot;
    magnetactual_msg.sortidx = data.sortidx;

    magnetactual_msg.xyWorldX = xyWorldX;
    magnetactual_msg.xyWorldY = xyWorldY;
    magnetactual_msg.xyWorldXdot = xyWorldXdot;
    magnetactual_msg.xyWorldYdot = xyWorldYdot;


    //PUBLISH
    kfmarker_pub.publish(kfpoints);
    kfmarker_pub.publish(kfline_list);
    rawmarker_pub.publish(rawpoints);
    kfxy_pub.publish(magnetactual_msg);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kf_pose");

    ros::NodeHandle n("~");


    std::string cal_file;    
    n.param("cal_file", cal_file, std::string("calplane.yml"));
    ROS_INFO_STREAM("cal file " << cal_file);
    cv::FileStorage fscal(cal_file.c_str(), cv::FileStorage::READ);
    centerpixx = (double)fscal["coilavgx"];
    centerpixy = (double)fscal["coilavgy"];
    pix2m = (double)fscal["pix2m"];

    double max_accel;
    //set a max value
    n.param("max_accel",max_accel,5.0); //10pix/s^2

    double dt, camera_fps;
    n.param("camera_fps", camera_fps, 119.0);
    ROS_ASSERT(camera_fps>0.0);
    dt = 1/camera_fps;

    KalmanFilter::State_t proc_noise_diag;
    proc_noise_diag(0) = 0.5*max_accel*dt*dt;
    proc_noise_diag(1) = 0.5*max_accel*dt*dt;
    proc_noise_diag(2) = max_accel*dt;
    proc_noise_diag(3) = max_accel*dt;
    proc_noise_diag = proc_noise_diag.array().square();
    
    KalmanFilter::Measurement_t meas_noise_diag;
    meas_noise_diag(0) = 1e-2; //increase value to smooth velocity.
    meas_noise_diag(1) = 1e-2;
    meas_noise_diag = meas_noise_diag.array().square();
    kf.initialize(KalmanFilter::State_t::Zero(),
                  0.01*KalmanFilter::ProcessCov_t::Identity(),
                  proc_noise_diag.asDiagonal(),
                  meas_noise_diag.asDiagonal());
    
    ros::Subscriber filterpose_sub = n.subscribe("/filterpose/xyFiltered", 10, &xyFiltered_callback);
  
    //odom_pub = n.advertise<plane_camera_magnet::xyFiltered>("odom", 10);
    kfmarker_pub = n.advertise<visualization_msgs::Marker>("visualization_kfmarker", 10);
    rawmarker_pub = n.advertise<visualization_msgs::Marker>("visualization_rawmarker", 10);

    kfxy_pub = n.advertise<plane_camera_magnet::xyFiltered>("magnetactual",1);
  
    ros::spin();
  
    return 0;
  
}