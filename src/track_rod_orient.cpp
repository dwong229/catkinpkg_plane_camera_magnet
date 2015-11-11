#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// for ROS - Opencv
#include <ros/ros.h>
#include <image_transport/image_transport.h> 
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/image_encodings.h>
#include <plane_camera_magnet/xyPix.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "track_rod_orient window";

/* track_rod_orient.cpp : Subscribes to '/camera/image_raw', uses fitellipse to identify magnet and orientation.
publishes vector x,y-posn in camera coordinates (pixels) of magnets to xyPix
- run cal_magnet_track.cpp got calibration a
- calibration values stored in cal.yml (in calib/)
TO ADD: 
  - calibration at the beginning of script to identify ROI
  - mask using calibration ROI
  - intelligent filter for position of magnets
*/

//RNG rng(12345);


//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher xyMagnet_pub_;
  int ymin;
  int ymax;
  int xmin;
  int xmax;
  double L;
  double mperpix;
  double xcenter;
  double ycenter;
  double radius;
  bool visualize;
  //cv::SimpleBlobDetector::Params params;
  int minThreshold;
  int maxThreshold;
  int minArea;
  int maxArea;

public:
  ImageConverter()
    :nh_("~") ,it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    
    image_sub_ = it_.subscribe("/pg_14434226/image_rect", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("output_video", 1);

    //publish vector of magnet positions
    xyMagnet_pub_ = nh_.advertise<plane_camera_magnet::xyPix>("xyPix",1);
    
    // nh_.param("/magnet_track/calib_file", cal_file, std::string("cal.yml"));
    visualize = 0;
    
    std::string cal_file;    
    nh_.param("cal_file", cal_file, std::string("cal.yml"));
    ROS_INFO_STREAM("cal file " << cal_file);
    FileStorage fscal(cal_file.c_str(), FileStorage::READ);
    ymin = (int)fscal["ymin"];
    ymax = (int)fscal["ymax"];
    xmin = (int)fscal["xmin"];
    xmax = (int)fscal["xmax"];
    L = (double)fscal["L"];
    mperpix = L/(xmax-xmin);

    xcenter = (xmax+xmin)/2;
    ycenter = (ymax+ymin)/2;
    radius = xmax-xcenter;

    cout<< "ycenter: " << ycenter << endl;
    cout<< "xcenter: " << xcenter << endl;
    cout<< "radius: " <<radius << endl;
    cout<< "L: " << L << endl;
    cout<< "mperpix: " << mperpix << endl;
  
    waitKey(1);

    std::string param_file;
    nh_.param("param_file", param_file, std::string("ellipseparam.yml"));
    ROS_INFO_STREAM("param file " << param_file);
    FileStorage fs(param_file.c_str(), FileStorage::READ);  

    minThreshold = (int)fs["minThreshold"];
    maxThreshold = (int)fs["maxThreshold"];
    minArea = (int)fs["minArea"]; 
    maxArea = (int)fs["maxArea"];
    ROS_INFO_STREAM("maxArea " << maxArea);


    if(visualize)
          cv::namedWindow(OPENCV_WINDOW);
  }//public
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //initialize tf:
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q;
    q.setRPY(0,0,0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world", "/camera_frame"));
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // process image
    blur( cv_ptr->image , cv_ptr->image , Size(3,3) );

    plane_camera_magnet::xyPix xymsg; //publish
    xymsg.header.stamp = ros::Time::now();
    //xymsg.header.frame_id = msg->header.frame_id;

    xymsg.header.frame_id = "/camera_frame";
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    // Detect edges using Threshold
    threshold( cv_ptr->image, threshold_output, minThreshold, maxThreshold, THRESH_BINARY );

    // Find contours, each contour is stored as a vector of points
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // Find the rotated rectangles and ellipse for each contour
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );
    int count = 0;
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    //cvtColor(cv_ptr->image, drawing, CV_GRAY2RGB);
    cvtColor(threshold_output, drawing, CV_GRAY2RGB);
    
    // loop through each contour, filter, draw
    for( int i = 0; i < contours.size(); i++ )
     { minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > minArea && contours[i].size() < maxArea )
         { minEllipse[i] = fitEllipse( Mat(contours[i]) );
         if (pow((minEllipse[i].center.y-ycenter),2) + pow((minEllipse[i].center.x-xcenter),2) < pow(radius,2.0) && minEllipse[i].size.area() < maxArea)
          {
            count = count + 1;
            float X=minEllipse[i].center.x; 
            float Y=minEllipse[i].center.y;
            //cout << "x = " << X << ", y = " << X << " size = " << minEllipse[i].size << " angle = " << minEllipse[i].angle << endl;
            Point pt;
            pt.x = minEllipse[i].center.x; 
            pt.y = minEllipse[i].center.y; 
            //save to ros msg
            xymsg.magx.push_back(minEllipse[i].center.x);
            xymsg.magy.push_back(minEllipse[i].center.y);
            xymsg.size.push_back(minEllipse[i].size.area());
            xymsg.angle.push_back(minEllipse[i].angle);

            ROS_INFO_STREAM("ID " << i);
            ROS_INFO_STREAM("Area " << contours[i].size());

            //cout << "Ellipse: " << minEllipse[i].center << " size: " << minEllipse[i].size.area() << " angle: " << minEllipse[i].angle << endl;

            //draw
            Scalar color = Scalar(255 , 255 * (count - 1), 0 );

            if (count > 1){
              Scalar color = Scalar(0, 255, 255);
            }
           
            ellipse( drawing, minEllipse[i], color, 2, 8 );
          }
        }
     }
     
     cout<< "===";
     if (count > 2){

      waitKey(10);
     }
    sensor_msgs::ImagePtr imgout;
    // Update GUI Window
    if (visualize){
      cv::imshow( OPENCV_WINDOW, drawing );    
      cv::waitKey(3);
    }
      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());
      

    imgout = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
    image_pub_.publish(imgout);
    
    
    // package position for xyReal.msg:
    xymsg.numrobot = count;

    if (contours.size()<1)
      cout<<"No blobs detected.  Turn on light?" << endl;
    // Output position vector
    xyMagnet_pub_.publish(xymsg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_rod_orient"); // initialize node: magnet_track
  //ros::init(argc, argv, "magnetpose_real");

  ImageConverter ic;
  
  ros::spin();
  return 0;
}

