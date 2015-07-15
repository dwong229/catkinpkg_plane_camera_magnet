#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//#pragma once
// for ROS - Opencv
#include <ros/ros.h>
#include <image_transport/image_transport.h> 
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/image_encodings.h>
#include "hw4.h"

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

/* magnet_track.cpp : Subscribes to '/camera/image_raw', uses simpleblobdetector to identify magnet.
publishes vector x,y-posn in camera coordinates of magnets to xyReal
TO ADD: 
  - calibration at the beginning of script to identify ROI
  - mask using calibration ROI
  - intelligent filter for position of magnets
*/

//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
  double pixpermm;
  double coilavgx;
  double coilavgy;
  int pt0x;
  int pt0y;
  int pt1x;
  int pt1y;
  int pt2x;
  int pt2y;
  int pt3x;
  int pt3y;
  int pt4x;
  int pt4y;
  int pt5x;
  int pt5y;
  int pt6x;
  int pt6y;
  int pt7x;
  int pt7y;
public:
  ImageConverter()
    : nh_("~"), it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/pg_14434226/image_rect", 1, 
      &ImageConverter::imageCb, this);
    
    std::string cal_file;    
    nh_.param("cal_file", cal_file, std::string("calplane.yml"));
    ROS_INFO_STREAM("cal file " << cal_file);
    FileStorage fs(cal_file.c_str(), FileStorage::READ);
    coilavgx = (double)fs["coilavgx"];
    coilavgy = (double)fs["coilavgy"];
    pt0x = (int)fs["pt0x"]; 
    pt0y = (int)fs["pt0y"];
    pt1x = (int)fs["pt1x"]; 
    pt1y = (int)fs["pt1y"];
    pt2x = (int)fs["pt2x"]; 
    pt2y = (int)fs["pt2y"];
    pt3x = (int)fs["pt3x"]; 
    pt3y = (int)fs["pt3y"];
    pt4x = (int)fs["pt4x"]; 
    pt4y = (int)fs["pt4y"]; 
    pt5x = (int)fs["pt5x"]; 
    pt5y = (int)fs["pt5y"]; 
    pt6x = (int)fs["pt6x"]; 
    pt6y = (int)fs["pt6y"]; 
    pt7x = (int)fs["pt7x"]; 
    pt7y = (int)fs["pt7y"]; 

    pixpermm = (double)fs["pix2m"];

    cout << "TB" << endl;
    cout << "coilavgx " << coilavgx << endl;
    cout << "pt1 " << pt1x << pt1y<<endl;
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // draw crosses
    Point center;
    center.x = coilavgx;
    center.y = coilavgy;

    Point pt1;
    pt1.x = pt1x;
    pt1.y = pt1y;
    Point pt2;
    pt2.x = pt2x;
    pt2.y = pt2y;
    
    Point pt5, pt6;
    pt5.x = pt5x;
    pt5.y = pt5y;
    pt6.x = pt6x;
    pt6.y = pt6y;
    
    circle(cv_ptr->image, center, 5, CV_RGB(0,0,255),3,-1);
    
    
    //drawCross(cv_ptr->image, ymaxpt, 5);
  

    //drawCross(cv_ptr->image, xminpt, 5);
    
    //drawCross(cv_ptr->image, xmaxpt, 5);
    line(cv_ptr->image, pt1,pt2,CV_RGB(255,0,0),2);
    line(cv_ptr->image, pt5,pt6,CV_RGB(255,0,0),2);


    // draw cross for petridish:
    Point dishN, dishS, dishE, dishW;
    double ldist = 89; // size of dish in mm
    //double ldist = 100;

    double lpix = ldist * pixpermm;
    dishN.x = dishS.x = center.x;
    dishN.y = center.y + lpix/2;
    dishS.y = center.y - lpix/2;

    dishE.y = dishW.y = center.y;
    dishE.x = center.x - lpix/2;
    dishW.x = center.x + lpix/2;

    line(cv_ptr->image, dishN,dishS,CV_RGB(0,255,0),1);
    line(cv_ptr->image, dishE,dishW,CV_RGB(0,255,0),1);

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW,  cv_ptr->image);

    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  cout << "======================================" << endl;
  cout << "Calibration check for magnet_track.cpp" << endl;
  cout << "======================================" << endl;
  ros::init(argc, argv, "check_cal_plane_coils"); // initialize node: magnet_track

  
  ImageConverter ic;
  ros::spin();
  return 0;
}