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
#include <opencv2/calib3d/calib3d.hpp>
//#pragma once
// for ROS - Opencv
#include <ros/ros.h>
#include <image_transport/image_transport.h> 
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/image_encodings.h>
#include "hw4.h"

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Live Image";


int numcorners = 4;
vector<Point2f> pixelcorners(numcorners);
vector<Point2f> worldcoords;
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
    //image_pub_ = it_.advertise("/cal_plane_coils/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow("Click Corners");
  }

  Mat clickCorners(Mat img)
  {
    cv::namedWindow("Click Corners");
    cv::imshow("Click Corners", img);


    //Mat image(3,3,CV_32F,Scalar(5));
    // 1, Identify y-limits for track
    //double xtotal, ytotal;
    cout << "Click on inner rectangle corner starting from top-left of workspace"<< endl;
    for(int i=0;i<numcorners;i++)
    { 
    
      Point pt = getClick("Click Corners",img);
      pixelcorners[i] = pt;

      cout << "point " << i << ":" << pt << endl;
      drawCross(img, pt, 4);
      waitKey(10);

      //xtotal += pt.x;
      //ytotal += pt.y;
    }
    cv::imshow("Click Corners", img);
    
    double worldboxsize = 0.03; // units: m
    cout << "Corner coordinates: " << worldboxsize << "m" << endl;
    worldcoords.push_back(Point2f(-worldboxsize,worldboxsize));
    worldcoords.push_back(Point2f(worldboxsize,worldboxsize));
    worldcoords.push_back(Point2f(worldboxsize,-worldboxsize));
    worldcoords.push_back(Point2f(-worldboxsize,-worldboxsize));


    Mat H;

    H = findHomography(pixelcorners, worldcoords, 0);

    return H;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            
      // assume that mat.type=CV_8UC3
      int width=cv_ptr->image.size().width;
      int height=cv_ptr->image.size().height;
      //cv::imshow(OPENCV_WINDOW,  cv_ptr->image);

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

    Mat H = clickCorners(cv_ptr->image);
    cout << "Homography: " << endl << H << endl;

    bool  updateYaml = true;

    // Write Yaml file
    if(updateYaml){
      cout<< "Updating YAML" << endl;
      
      FileStorage fs("/home/denise/catkin_ws/src/plane_camera_magnet/calib/dahomography.yml",FileStorage::WRITE);
      fs << "H" << H;
      fs << "pt0x" << pixelcorners[0].x << "pt0y" << pixelcorners[0].y;
      fs << "pt1x" << pixelcorners[1].x << "pt1y" << pixelcorners[1].y;
      fs << "pt2x" << pixelcorners[2].x << "pt2y" << pixelcorners[2].y;
      fs << "pt3x" << pixelcorners[3].x << "pt3y" << pixelcorners[3].y;
      fs << "world0x" << worldcoords[0].x << "world0y" << worldcoords[0].y;
      fs << "world1x" << worldcoords[1].x << "world1y" << worldcoords[1].y;
      fs << "world2x" << worldcoords[2].x << "world2y" << worldcoords[2].y;
      fs << "world3x" << worldcoords[3].x << "world3y" << worldcoords[3].y;
      fs.release();

      exit (EXIT_SUCCESS);

    }

    else{
      cout<< "Don't update YAML" << endl;
      
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
   
    //cout<< "xleft: "<< keypoints[1].pt.x << endl;
    //cout<< "xright: "<< keypoints[2].pt.x << endl;
    // Output modified video stream
    //    image_pub_.publish(cv_ptr->toImageMsg());

    exit (EXIT_SUCCESS);
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
  cout << "==========================================================" << endl;
  cout << "Calibration script for pixels to world using homography" << endl;
  cout << "==========================================================" << endl;

  ros::init(argc, argv, "homography_calibration"); // initialize node: magnet_track
  //ros::init(argc, argv, "magnetpose_real");

  ImageConverter ic;
  
  ros::spin();
  return 0;
}

