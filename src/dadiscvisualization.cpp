//disc visualization:

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
static visualization_msgs::Marker rawpoints;

class Visualizer
{

public:
    
    void disccb(const plane_camera_magnet::xyPix& msg) {

        discmarker.header.seq = msg.header.seq;
        discmarker.header.stamp = msg.header.stamp;
        discmarker.header.frame_id = "camera_frame";
        discmarker.type = visualization_msgs::Marker::POINTS;
        discmarker.pose.orientation.x = 0.0;
        discmarker.pose.orientation.y = 0.0;
        discmarker.pose.orientation.z = 1.0 * sin(magnet_orientation/2);
        discmarker.pose.orientation.w = cos(magnet_orientation/2);
        discmarker.scale.x = 1;
        discmarker.scale.y = .4;
        discmarker.scale.z = .5;
        discmarker.color.a = 1.0; // Don't forget to set the alpha!
        discmarker.color.r = 0.0;
        discmarker.color.g = 1.0;
        discmarker.color.b = 0.0;



        discmarker_pub_.publish(discmarker);
    }



    Visualizer()  {
        discpose_sub_ = n_.subscribe("/", 10, &Visualizer::disccb, this);
        discmarker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_disc", 1, this);
    }


private:
  ros::NodeHandle n_{"~"};
  ros::Subscriber discpose_sub_;
  ros::Publisher discmarker_pub_;

  visualization_msgs::Marker discmarker;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dadiscvisualization"); // initialize node: 
  ros::NodeHandle n("~");

  Visualizer viz;
  
  ros::spin();
  return 0;
}