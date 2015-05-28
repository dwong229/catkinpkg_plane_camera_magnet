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

// for ROS - Opencv
#include <ros/ros.h>
#include <image_transport/image_transport.h> 
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/image_encodings.h>
#include <plane_camera_magnet/xyPix.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Track_rod window";


/*)
int ymin;
int ymax;
int xmin;
int xmax;
double L;
*/
/* magnet_track.cpp : Subscribes to '/camera/image_raw', uses simpleblobdetector to identify magnet.
publishes vector x,y-posn in camera coordinates of magnets to xyReal
- run cal_magnet_track.cpp got calibration a
- calibration values stored in cal.yml (in calib/)
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
  cv::SimpleBlobDetector::Params params;


public:
  ImageConverter()
    :nh_("~") ,it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    
    image_sub_ = it_.subscribe("/pg_14434226/image_rect", 1, 
      &ImageConverter::imageCb, this);
    //image_sub_ = it_.subscribe("/camera/image_raw", 1, 
    //  &ImageConverter::imageCb, this);
    // image_pub_ = it_.advertise("/magnet_track/output_video", 1);
    image_pub_ = it_.advertise("output_video", 1);

    //publish vector of magnet positions
    xyMagnet_pub_ = nh_.advertise<plane_camera_magnet::xyPix>("xyPix",1);
    
    // nh_.param("/magnet_track/calib_file", cal_file, std::string("cal.yml"));
    visualize = 1;
    
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
    
    // blob detector parameters
      // set up and create the detector using the parameters
  /*
    params.minThreshold = 10;
    params.maxThreshold = 50;
    params.thresholdStep = 2;

    params.minArea = 80; 
    params.minConvexity = 0.8;
    params.minInertiaRatio = 0.01;

    params.maxArea = 1000;
    params.maxConvexity = 10;
    //params.maxInertiaRatio = 8;
    params.blobColor = 0;
    params.filterByCircularity = false;*/
    

    std::string param_file;
    nh_.param("param_file", param_file, std::string("blobparam.yml"));
    ROS_INFO_STREAM("param file " << param_file);
    FileStorage fs(param_file.c_str(), FileStorage::READ);  

    params.minThreshold = (int)fs["minThreshold"];
    params.maxThreshold = (int)fs["maxThreshold"];
    params.thresholdStep = (int)fs["thresholdStep"];

    params.minArea = (int)fs["minArea"]; 
    params.minConvexity = (double)fs["minConvexity"];
    params.minInertiaRatio = (double)fs["minInertiaRatio"];

    params.maxArea = (int)fs["maxArea"];
    params.maxConvexity = (int)fs["maxConvexity"];
    //params.maxInertiaRatio = 8;
    params.blobColor = (int)fs["blobColor"];
    params.filterByCircularity = (int)fs["filterByCircularity"];


    if(visualize)
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    
    // Live track magnets using simpleblobdetector
    SimpleBlobDetector blob_detector( params );
    blob_detector.create("SimpleBlob"); 
    // detect!
    vector<KeyPoint> keypoints;
    blob_detector.detect(cv_ptr->image, keypoints); //blob_detector does not allow for angle
    
    // Update rosmsg header
    plane_camera_magnet::xyPix xymsg; //publish
    xymsg.header.stamp = ros::Time::now();

    
    cout << " ===track_rod.cpp=== " << endl;
    int count = 0;

    // cycle through keypoints
    for (int i=0; i<keypoints.size(); i++){
      // determine which keypoints are magnets as filtered by posn
      if (pow((keypoints[i].pt.y-ycenter),2) + pow((keypoints[i].pt.x-xcenter),2) < pow(radius,2.0))
      {
      // candidate keypoints for magnet
        count = count + 1;
        float X=keypoints[i].pt.x; 
        float Y=keypoints[i].pt.y;
        cout << "x = " << keypoints[i].pt.x << ", y = " << keypoints[i].pt.y << " size = " << keypoints[i].size << " angle = " << keypoints[i].angle << endl;
        Point pt;
        pt.x = keypoints[i].pt.x; 
        pt.y = keypoints[i].pt.y; 
        //save to ros msg
        xymsg.magx.push_back(keypoints[i].pt.x);
        xymsg.magy.push_back(keypoints[i].pt.y);
        xymsg.size.push_back(keypoints[i].size);

      if (visualize)
        circle( cv_ptr->image, pt , 10 , CV_RGB(0,0,255), 3 , -1 );
      }
    }
    if (visualize)
      drawKeypoints( cv_ptr->image, keypoints,  cv_ptr->image, CV_RGB(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    //cout << " ==tb==== " << endl;
    cout << "numrobot : " << count << endl;

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    if (visualize){
      cv::imshow(OPENCV_WINDOW,  cv_ptr->image);
      cv::waitKey(3);
      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
    }
    
    // package position for xyReal.msg:
    xymsg.numrobot = count;

    if (keypoints.size()<1)
      cout<<"No blobs detected.  Turn on light?" << endl;
    // Output position vector
    xyMagnet_pub_.publish(xymsg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_rod"); // initialize node: magnet_track
  //ros::init(argc, argv, "magnetpose_real");

  ImageConverter ic;
  
  ros::spin();
  return 0;
}