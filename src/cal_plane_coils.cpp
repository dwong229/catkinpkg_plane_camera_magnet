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

static const std::string OPENCV_WINDOW = "Calibration Image";

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
            // draw grid:
      // assume that mat.type=CV_8UC3

      int dist=20;

      int width=cv_ptr->image.size().width;
      int height=cv_ptr->image.size().height;

/*
      for(int i=0;i<height;i+=dist)
        cv::line(cv_ptr->image,Point(0,i),Point(width,i),cv::Scalar(255,255,255));

      for(int i=0;i<width;i+=dist)
        cv::line(cv_ptr->image,Point(i,0),Point(i,height),cv::Scalar(255,255,255));

      for(int i=0;i<width;i+=dist)
        for(int j=0;j<height;j+=dist)
          cv_ptr->image.at<cv::Vec3b>(i,j)=cv::Scalar(10,10,10); 
          */
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
    int numcorners = 4;
    vector<Point> rectcorners(numcorners);
    double xtotal, ytotal;
    cout << "Click on inner rectangle corner starting from top-left of workspace"<< endl;
    for(int i=0;i<numcorners;i++)
    { 
    
      Point pt = getClick("image",cv_ptr->image);
      rectcorners[i] = pt;

      cout << "point " << i << ":" << pt << endl;
      drawCross(cv_ptr->image, pt, 4);
      waitKey(10);

      xtotal += pt.x;
      ytotal += pt.y;
    }
/*

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
 */   
/* n = 8
    double d = 115.0/2;  // half distance between coils
    double rectd = 75.2; //inner distance between rect corners

    double coilavgx = (xtotal)/numcorners;
    double coilavgy = (ytotal)/numcorners;
    Point centerpt;
    centerpt.x = coilavgx;
    centerpt.y = coilavgy;
    drawCross(cv_ptr->image, centerpt, 3);

    //double pix2m = (dist13 + dist24)/(2*2*d)
    // compute position in pixels of coils:

    double distpt12 = distcalc(rectcorners[1],rectcorners[2]);
    double distpt34 = distcalc(rectcorners[3],rectcorners[4]);
    double distpt56 = distcalc(rectcorners[5],rectcorners[6]);
    double distpt70 = distcalc(rectcorners[7],rectcorners[0]);
    //cout << "pt1: " << rectcorners[1] << " pt2:" << rectcorners[2] << "dist: " << dist12 << endl;
    
    double dist13 = (distpt12 + distpt56)/2;
    double dist24 = (distpt70 + distpt34)/2;

    double pix2m = (dist13 + dist24)/(2*rectd);
*/
    //n = 4
    double d = 15;  // half distance between grid points
    double ddiag = pow(d*d*2,0.5);
    //double rectd = 75.2; //inner distance between rect corners

    double coilavgx = (xtotal)/numcorners;
    double coilavgy = (ytotal)/numcorners;
    Point centerpt;
    centerpt.x = coilavgx;
    centerpt.y = coilavgy;
    drawCross(cv_ptr->image, centerpt, 3);

    //double pix2m = (dist13 + dist24)/(2*2*d)
    // compute position in pixels of coils:

    double distpt12 = distcalc(rectcorners[0],centerpt); //top left [-15,15]
    double distpt34 = distcalc(rectcorners[1],centerpt); //top right [15,15]
    double distpt56 = distcalc(rectcorners[2],centerpt); // bottom right [15,-15]
    double distpt70 = distcalc(rectcorners[3],centerpt); //bottom left [-15, -15]
    //cout << "pt1: " << rectcorners[1] << " pt2:" << rectcorners[2] << "dist: " << dist12 << endl;
    
    double pix2m = (distpt12 + distpt34 + distpt56 + distpt70)/(4*ddiag);
       
    
    cout << "d(mm):" << d << endl;
    
    bool  updateYaml = true;

    // Write Yaml file
    if(updateYaml){
      cout<< "Updating YAML" << endl;

      FileStorage fs("/home/denise/catkin_ws/src/plane_camera_magnet/calib/dacalplane.yml",FileStorage::WRITE);
      /*fs << "coil1x" << coil1.x << "coil1y" << coil1.y;
      fs << "coil2x" << coil2.x << "coil2y" << coil2.y;
      fs << "coil3x" << coil3.x << "coil3y" << coil3.y;
      fs << "coil4x" << coil4.x << "coil4y" << coil4.y;*/
      //fs << "coilcenterx << centerpt.x << "coilcentery" << centerpt.y;
      fs << "coilavgx" << coilavgx << "coilavgy" << coilavgy;
      fs << "pix2m" << pix2m;
      fs << "d" << d ;
      fs << "ddiag" << ddiag;
      fs << "pt0x" << rectcorners[0].x << "pt0y" << rectcorners[0].y;
      fs << "pt1x" << rectcorners[1].x << "pt1y" << rectcorners[1].y;
      fs << "pt2x" << rectcorners[2].x << "pt2y" << rectcorners[2].y;
      fs << "pt3x" << rectcorners[3].x << "pt3y" << rectcorners[3].y;
      /*fs << "pt4x" << rectcorners[4].x << "pt4y" << rectcorners[4].y;
      fs << "pt5x" << rectcorners[5].x << "pt5y" << rectcorners[5].y;
      fs << "pt6x" << rectcorners[6].x << "pt6y" << rectcorners[6].y;
      fs << "pt7x" << rectcorners[7].x << "pt7y" << rectcorners[7].y;
      */
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
  double distcalc(Point pt1, Point pt2)
{
  double dist;
  dist = sqrt ( pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2));
  return dist;
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

