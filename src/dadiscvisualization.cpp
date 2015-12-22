//disc visualization:

#include <ros/ros.h>
#include <plane_camera_magnet/xyPix.h>
#include <visualization_msgs/Marker.h> //for rvis visualization
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;

static ros::Publisher discmarker_pub_;

static plane_camera_magnet::xyPix msg;
static visualization_msgs::Marker rawpoints;

double pix2m;
double centerpixx;
double centerpixy;

class Visualizer
{

public:
    
    void disccb(const plane_camera_magnet::xyPix& msg) {

        
        //ROS_INFO_STREAM("msg: " << msg.magx[0] << " , " << msg.magy[0]);
        

        
        //ROS_INFO_STREAM("msg: " << xyWorldX << " , " << xyWorldY);

        discmarker.header.seq = msg.header.seq;
        discmarker.header.stamp = msg.header.stamp;
        discmarker.header.frame_id = "camera_frame";
        discmarker.type = visualization_msgs::Marker::SPHERE_LIST;
        discmarker.pose.orientation.x = 0.0;
        discmarker.pose.orientation.y = 0.0;
        discmarker.pose.orientation.z = 0.0;
        discmarker.pose.orientation.w = 1.0;
        discmarker.scale.x = 1;
        discmarker.scale.y = 1;
        discmarker.scale.z = .3;
        discmarker.color.a = 1.0; // Don't forget to set the alpha!
        discmarker.color.r = 1.0;
        discmarker.color.g = 0.0;
        discmarker.color.b = 0.0;

        discmarker.points.resize(msg.numrobot);
        //convert to mm
        geometry_msgs::Point p;

        for(int i=0; i<msg.numrobot; i++)
        {
          geometry_msgs::Point p;

          p.x = (msg.magx[i] - centerpixx)/pix2m;
          p.y = (-msg.magy[i] + centerpixy)/pix2m;
          p.z = 0;
          discmarker.points[i] = p;
        }
        
        
        discmarker_pub_.publish(discmarker);
    }



    Visualizer()  {
        discpose_sub_ = n_.subscribe("/datrackdisc/xyPix", 10, &Visualizer::disccb, this);
        discmarker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_disc", 1, this);
    }


private:
  ros::NodeHandle n_{"~"};
  ros::Subscriber discpose_sub_;
  ros::Publisher discmarker_pub_;
  visualization_msgs::Marker discmarker;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "dadiscvisualization"); // initialize node: 
  ros::NodeHandle n("~");

  std::string cal_file;    
  n.param("cal_file", cal_file, std::string("calplane.yml"));
  ROS_INFO_STREAM("cal file " << cal_file);
  cv::FileStorage fscal(cal_file.c_str(), cv::FileStorage::READ);
  centerpixx = (double)fscal["coilavgx"];
  centerpixy = (double)fscal["coilavgy"];
  pix2m = (double)fscal["pix2m"];

  Visualizer viz;
  
  ros::spin();
  return 0;
}