// Calibrate position of coils and workspace by clicking the points
// run image stream first
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <stdio.h>
#include <math.h>
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

static const std::string OPENCV_WINDOW = "image";

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
  image_transport::Publisher image_pub_;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/pg_14434226/image_rect", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/cal_plane_coils/output_video", 1);

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

    // 0. Capture image?
    cout << "Move setup until entire four corners of the platform are in view."<< endl;
    cv::waitKey(0);

    //Mat image(3,3,CV_32F,Scalar(5));
    // 1, Identify y-limits for track
    cout << "Click on coil 1 center of workspace"<< endl;
    Point coil1 = getClick("image",cv_ptr->image);
    cout << "coil1: " << coil1 << endl;
    drawCross(cv_ptr->image, coil1, 5);
    waitKey(10);

    cout << "Click on coil 2 center of workspace"<< endl;
    Point coil2 = getClick("image",cv_ptr->image);
    cout << "coil2: " << coil2 << endl;
    drawCross(cv_ptr->image, coil2, 5);
    waitKey(10);

    // 2. Identify x limits
    cout << "Click on coil 3 center of workspace"<< endl;
    Point coil3 = getClick("image",cv_ptr->image);
    cout << "coil3: " << coil3 << endl;
    drawCross(cv_ptr->image, coil3, 5);
    waitKey(10);

    cout << "Click on coil4 center of workspace"<< endl;
    Point coil4 = getClick("image",cv_ptr->image);
    cout << "coil4: " << coil4 << endl;
    drawCross(cv_ptr->image, coil4, 5);
    waitKey(10);
    

    double d = 115/2;  // half distance between coils

    double coilavgx = (coil1.x + coil2.x + coil3.x + coil4.x)/4;
    double coilavgy = (coil1.y + coil2.y + coil3.y + coil4.y)/4;
    double dist13 = sqrt ( pow(coil1.x - coil3.x,2) + pow(coil1.y - coil3.y,2));
    double dist24 = sqrt ( pow(coil2.x - coil4.x,2) + pow(coil2.y - coil4.y,2));
    double pix2m = (dist13 + dist24)/(2*2*d);

    cout << "d(mm):" << d << endl;
    bool  updateYaml = true;
    // Write Yaml file
    if(updateYaml){
      cout<< "Updating YAML" << endl;

      FileStorage fs("/home/denise/catkin_ws/src/plane_camera_magnet/calib/calplane.yml",FileStorage::WRITE);
      fs << "coil1x" << coil1.x << "coil1y" << coil1.y;
      fs << "coil2x" << coil2.x << "coil2y" << coil2.y;
      fs << "coil3x" << coil3.x << "coil3y" << coil3.y;
      fs << "coil4x" << coil4.x << "coil4y" << coil4.y;
      fs << "coilavgx" << coilavgx << "coilavgy" << coilavgy;
      fs << "dist13" << dist13;
      fs << "dist24" << dist24;
      fs << "pix2m" << pix2m;
      fs << "d" << d ;

      fs.release();
      exit (EXIT_SUCCESS);
    }
    else{
      cout<< "Don't update YAML" << endl;
      
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW,  cv_ptr->image);

    cv::waitKey(3);
    
    //cout<< "xleft: "<< keypoints[1].pt.x << endl;
    //cout<< "xright: "<< keypoints[2].pt.x << endl;
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
  }
};

int main(int argc, char** argv)
{
  cout << "======================================" << endl;
  cout << "Calibration script for plane_camera_track.cpp" << endl;
  cout << "======================================" << endl;
  ros::init(argc, argv, "cal_plane_coils"); // initialize node: magnet_track
  //ros::init(argc, argv, "magnetpose_real");

  ImageConverter ic;
  
  ros::spin();
  return 0;
}

