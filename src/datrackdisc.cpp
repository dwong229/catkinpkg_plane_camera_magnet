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

static const std::string OPENCV_WINDOW = "detection";

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
#define PI 3.14159265

//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher imageinter_pub_;

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
  int discThreshold;
  int maxThreshold;
  int minArea;
  int maxArea;
  sensor_msgs::ImagePtr imgintermediate;
  int nodetectioncount = 0;

public:
  ImageConverter()
    :nh_("~") ,it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    
    image_sub_ = it_.subscribe("/pg_14434226/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    imageinter_pub_ = it_.advertise("intermediate_vid", 1);
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

    discThreshold = (int)fs["discThreshold"];
    maxThreshold = (int)fs["ceilingThreshold"];
    minArea = (int)fs["minDiscArea"]; 
    maxArea = (int)fs["maxDiscArea"];
    //ROS_INFO_STREAM("maxArea " << maxArea);


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
    
    cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_mono;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      cv_ptr_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    

    // process image
    Mat imghsv = cv_ptr_rgb->image.clone();
    cvtColor( cv_ptr_rgb->image, imghsv, CV_RGB2HSV);
    
    Mat ch1, ch2, ch3;
    // "channels" is a vector of 3 Mat arrays:
    vector<Mat> channels(3);
    // split img:
    split(imghsv, channels);
    // get the channels (dont forget they follow BGR order in OpenCV)
    ch1 = channels[0]; // flourescent beads are dark on this channel!
    ch2 = channels[1];
    ch3 = channels[2];

    //blur( cv_ptr_mono->image , cv_ptr_mono->image , Size(4,4) );

    plane_camera_magnet::xyPix xymsg; //publish
    xymsg.header.stamp = ros::Time::now();
    //xymsg.header.frame_id = msg->header.frame_id;

    xymsg.header.frame_id = "/camera_frame";
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    // Detect edges using Threshold
    threshold( ch1, threshold_output, discThreshold, maxThreshold, THRESH_BINARY_INV );
    if(visualize){
    imgintermediate = cv_bridge::CvImage(std_msgs::Header(), "mono8", ch1).toImageMsg();
    imageinter_pub_.publish(imgintermediate);
  }
    // Find contours, each contour is stored as a vector of points
    blur( threshold_output , threshold_output , Size(5,5) );
    
    threshold( threshold_output, threshold_output, 80, maxThreshold, THRESH_BINARY );

    Mat tmpBinaryImage = threshold_output.clone();
    findContours( tmpBinaryImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );

    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    //cvtColor(cv_ptr->image, drawing, CV_GRAY2RGB);
    if (contours.size() == 0){
      nodetectioncount += 1;
      //cout << "No detection " << nodetectioncount << endl;
      
    }
    
    // Find the rotated rectangles and ellipse for each contour
    //vector<RotatedRect> minRect( contours.size() );
    vector<vector<Point> >hull( contours.size() );
    int count = 0;
    // loop through each contour, filter, draw
    for( int i = 0; i < contours.size(); i++ )
     {// minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > minArea && contours[i].size() < maxArea )
         { 
         //if (pow((minEllipse[i].center.y-ycenter),2) + pow((minEllipse[i].center.x-xcenter),2) < pow(radius,2.0) ) //&& minEllipse[i].size.area() < maxArea)
          {
            count = count + 1;
            Point pt;
            convexHull( Mat(contours[i]), hull[i], false ); //hull stores the hull points

            //center of convex hull
            // Get the moments
            Moments mu;
            mu = moments( hull[i], false );

            ///  Get the mass centers:
            Point2f mc;
            mc= Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
            pt.x = mc.x;
            pt.y = mc.y;
            
            //save to ros msg
            xymsg.magx.push_back(pt.x);
            xymsg.magy.push_back(pt.y);
            xymsg.size.push_back(contourArea(hull[i]));
            xymsg.angle.push_back(0);

            //ROS_INFO_STREAM("ID " << i);
            //ROS_INFO_STREAM("Area " << contours[i].size());

            //cout << "x = " << pt.x << ", y = " << pt.y << " size hull = " << contourArea(hull[i]) <<" angle = " << minEllipse[i].angle << endl;

            //draw
            Scalar color = Scalar(255 , 255 * (count - 1), 0 );

            if (count > 1){
              Scalar color = Scalar(0, 255, 255);
            }
            Scalar colorhull = Scalar(0,255,255);//Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            //ellipse( drawing, minEllipse[i], color, 2, 8 );
            //drawContours( drawing, contours, i, color, CV_FILLED,8,hierarchy);
            drawContours( drawing, hull, i, colorhull, 1, 8, vector<Vec4i>(), 0, Point() );
            circle( drawing, pt, 1, colorhull, 3, 8,0);          
          }
        }
     }


    sensor_msgs::ImagePtr imgout;
    imgout = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
    image_pub_.publish(imgout);
    // Update GUI Window
    if (visualize){
      cv::imshow( OPENCV_WINDOW, drawing );    

      cv::waitKey(3);
    }
    // package position for xyReal.msg:
    xymsg.numrobot = count;

    if (contours.size()<1)
      //cout<<"No blobs detected.  Turn on light?" << endl;
    // Output position vector
    xyMagnet_pub_.publish(xymsg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rawdisc"); // initialize node: 
  //ros::init(argc, argv, "magnetpose_real");

  ImageConverter ic;
  
  ros::spin();
  return 0;
}