#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
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
#include <plane_camera_magnet/xyFiltered.h>
#include <plane_camera_magnet/roboclawCmd.h>


using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Visualization Tracked Window";
/*)
int ymin;
int ymax;
int xmin;
int xmax;
double L;
*/
/* magnet_track.cpp : Subscribes to '/camera/image_raw', uses simpleblobdetector to identify magnet.
publishes vector x,y-posn in camera coordinates of magnets to xyFiltered
- run cal_magnet_track.cpp got calibration a
- calibration values stored in cal.yml (in devel/lib/camera_magnet)
TO ADD: 
  - calibration at the beginning of script to identify ROI
  - mask using calibration ROI
  - intelligent filter for position of magnets
*/


vector<KeyPoint> xyFilteredKP; 
vector<double> xyFilteredAng;
vector<KeyPoint> xyDesKP;
int numrobot;
int ymin;
int ymax;
int xmin;
int xmax;
double ycenter;
double xcenter;
double radius;
double L;
double mperpix;
//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  image_transport::Publisher image_pub_;
  
  ros::Subscriber xyFiltered_sub_;


public:
  ImageConverter()
    :nh_("~") ,it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/pg_14434226/image_rect", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("visualization", 1);
    // subscribe to xyFiltered position
    
    // subscribe to desired position
    //xyDes_sub_ = nh_.subscribe("/camera_magnet/xyDes",1);

    /*
    // unpack calibration values
    std::string cal_file;
    // nh_.param("/magnet_track/calib_file", cal_file, std::string("cal.yml"));
    
    nh_.param("cal_file", cal_file, std::string("cal.yml"));
    ROS_INFO_STREAM("cal file " << cal_file);
    FileStorage fs(cal_file.c_str(), FileStorage::READ);
    ymin = (int)fs["ymin"];
    ymax = (int)fs["ymax"];
    xmin = (int)fs["xmin"];
    xmax = (int)fs["xmax"];
    L = (double)fs["L"];
    mperpix = L/(xmax-xmin);
    xcenter = (xmax+xmin)/2;
    ycenter = (ymax+ymin)/2;
    radius = xmax-xcenter;

    cout<< "ycenter: " << ycenter << endl;
    cout<< "xcenter: " << xcenter << endl;
    cout<< "radius: " <<radius << endl;
    cout<< "L: " << L << endl;
    cout<< "mperpix: " << mperpix << endl;
    */
    
    //cout<< "ymin: " << ymin << endl;
    waitKey(0);

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

    // Loop over points:
    

    for (int i = 0; i<numrobot; ++i)
    {
      Point ptTemp, ptLine;
      ptTemp.x = xyFilteredKP[i].pt.x;
      ptTemp.y = xyFilteredKP[i].pt.y;
      double linelength = 10;
      cout << "Ang (rad): " << xyFilteredAng[i] << endl;
      ptLine.x = ptTemp.x + linelength*cos(xyFilteredAng[i]);
      ptLine.y = ptTemp.y + linelength*sin(xyFilteredAng[i]);
      circle( cv_ptr->image, ptTemp , 3 , CV_RGB(0,255*i,255), 3 , -1 );

      cout << ptLine << endl;
      line(cv_ptr->image,ptTemp,ptLine,CV_RGB(0,255*i,255), 1 , 8);
    }
    
    /* // Desired location - need to update!
    Point ptdes1;
    ptdes1.x = xyDesKP[0].pt.x;
    ptdes1.y = ycenter;

    Point ptdes2;
    ptdes2.x = xyDesKP[1].pt.x;
    ptdes2.y = ycenter;

    circle( cv_ptr->image, ptdes1 , 5 , CV_RGB(0,255,0), 3 , -1 );
    circle( cv_ptr->image, ptdes2 , 5 , CV_RGB(0,255,0), 3 , -1 );  
    */
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW,  cv_ptr->image);

    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

  }
};


void xyFilteredcallback(const plane_camera_magnet::xyFiltered& data)
{
  
  // unpack subscribed values to vector of keypoints
    numrobot = data.actrobot;
    xyFilteredKP.reserve(numrobot); // set size of vector
    xyFilteredAng.reserve(numrobot);
    for (int i=0; i<data.xyPixX.size(); ++i)
    {
      xyFilteredKP[i].pt.x = int(data.xyPixX[i]);
      xyFilteredKP[i].pt.y = int(data.xyPixY[i]);
      xyFilteredAng[i] = double(data.xyAnglerad[i]);
    }
}


void roboclawCmdcallback(const plane_camera_magnet::roboclawCmd& data)
{
    cout<<"Update roboclawCmdcallback"<<endl;
// unpack subscribed values to vector of keypoints
    /*
    xyDesKP[0].pt.x = int(data.x1des / mperpix + xmin);
    xyDesKP[0].pt.y = 0; //complete with ymid
    xyDesKP[1].pt.x = int(data.x2des / mperpix + xmin);
    xyDesKP[1].pt.y = 0; //complete with ymid
    */
  }


int main(int argc, char** argv)
{
  xyDesKP.reserve(2);

  ros::init(argc, argv, "visualization"); // initialize node: visualization
  //ros::init(argc, argv, "magnetpose_Pix");
  ros::NodeHandle nh_;
  ros::Subscriber xyFiltered_sub_ = nh_.subscribe("/filterpose/xyFiltered",1, xyFilteredcallback);

  ImageConverter ic;
 
  //ros::Subscriber xyDes_sub_ = nh_.subscribe("/controller_inputvector/roboclawCmd",1,roboclawCmdcallback);
  ros::spin();
  return 0;
}
