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
#include <plane_camera_magnet/xyFiltered.h>
#include <tf/transform_broadcaster.h>
#include "hw4.h"

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
  ros::Publisher xySingleRobot_pub_;

  Mat H;

  double radius;
  bool visualize;
  //cv::SimpleBlobDetector::Params params;
  int minThreshold;
  int maxThreshold;
  int minArea;
  int maxArea;
  sensor_msgs::ImagePtr imgintermediate;
  int nodetectioncount = 0;

  bool firstimg = true;
  // robot characteristics:
  double angle_in_bearth;
  double robothullsize; 
  Point2f robotlastposn;
  plane_camera_magnet::xyFiltered robotonly_xymsglast; //publish


public:
  ImageConverter() //Initialization:, constructor
    :nh_("~") ,it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    
    image_sub_ = it_.subscribe("/pg_14434226/image_rect", 1, 
      &ImageConverter::imageCb, this);
    imageinter_pub_ = it_.advertise("intermediate_vid", 1);
    image_pub_ = it_.advertise("output_video", 1);


    //publish vector of magnet positions
    xyMagnet_pub_ = nh_.advertise<plane_camera_magnet::xyPix>("xyPixAll",1);
    xySingleRobot_pub_ = nh_.advertise<plane_camera_magnet::xyFiltered>("xyPixRobot",1);

    // nh_.param("/magnet_track/calib_file", cal_file, std::string("cal.yml"));
    visualize = 1;
    
    // unpack the calibration files for camera
    std::string cal_file;    
    nh_.param("cal_file", cal_file, std::string("cal.yml"));
    ROS_INFO_STREAM("cal file " << cal_file);
    FileStorage fscal(cal_file.c_str(), FileStorage::READ);
    fscal["H"] >> H;
    cout<< "H: " << H << endl;
  
    waitKey(1);

    // unpack the calibration files for tracking - delete?
    std::string param_file;
    nh_.param("param_file", param_file, std::string("ellipseparam.yml"));
    ROS_INFO_STREAM("param file " << param_file);
    FileStorage fs(param_file.c_str(), FileStorage::READ);  

    minThreshold = (int)fs["Threshold"];
    maxThreshold = (int)fs["ceilingThreshold"];
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

  int initializeRobot( Mat img, vector<vector<Point>> contours)
  {
    cout << "initialize Robot!" << endl;
    cout << "Found: " << contours.size() << endl;

    // display the image with contours identified
    cv::namedWindow("ClickRobot");

    Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
    drawing = img;

    int idx = 0;
    for (; idx < contours.size() ; idx++ )
    {
      Scalar color( rand()&255, rand()&255, rand()&255 );
      drawContours(drawing, contours, idx, color, CV_FILLED, 8);
    }
    

    // show image:
    cv::imshow( "ClickRobot", drawing ); 

    // user to click inside the contour
    Point pt = getClick("ClickRobot",drawing);

    drawCross(img, pt, 5);
    
    // determine which contour is the robot and save the contour stats
    int robotidx = -1;
    int i = 0;
    while(robotidx<0)
    {
      bool measuredist;
      
      if( pointPolygonTest(contours[i], pt, measuredist) > 0)  
      {
        cout << "Found contour: " << i << endl;
        robotidx = i;
      }
      i++;
    }
    firstimg = false;
    cout << "Initialization over." << endl;
    cv::destroyWindow("ClickRobot");
    return robotidx;
  }

  void MyLine( Mat img, Point start, double angle )
  {
    int thickness = 2;
    int lineType = 8;
    Point end;
    int length = 40;
    end.x = start.x + length*sin(angle*PI/180);
    end.y = start.y - length*cos(angle*PI/180);
    line( img,
          start,
          end,
          Scalar( 0, 0, 255 ),
          thickness,
          lineType );
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
      cv_ptr_mono = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // process image
    blur( cv_ptr_mono->image , cv_ptr_mono->image , Size(4,4) );


    plane_camera_magnet::xyPix xymsg; //publish
    xymsg.header.stamp = ros::Time::now();
    //xymsg.header.frame_id = msg->header.frame_id;

    xymsg.header.frame_id = "/camera_frame";

    plane_camera_magnet::xyFiltered robotonly_xymsg; //publish
    robotonly_xymsg.header.stamp = ros::Time::now();
    //xymsg.header.frame_id = msg->header.frame_id;

    robotonly_xymsg.header.frame_id = "/camera_frame";
    

    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<vector<Point> > approxContour;
    vector<double> anglevec;
    vector<double> hullvec;

    vector<Vec4i> hierarchy;
    // Detect edges using Threshold
    threshold( cv_ptr_mono->image, threshold_output, minThreshold, maxThreshold, THRESH_BINARY_INV );

    if(visualize){
      // convert image for transport:
      imgintermediate = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold_output).toImageMsg();
      imageinter_pub_.publish(imgintermediate);
    }

    Mat threshold_outputstatic = threshold_output;
    // Find contours, each contour is stored as a vector of points
    Mat canny_output;
    //int thresh = 100;
    //blur( threshold_output , threshold_output , Size(5,5) );
    
    //threshold( threshold_output, threshold_output, 80, maxThreshold, THRESH_BINARY );

    Mat tmpBinaryImage = threshold_output.clone();
    findContours( tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    //cvtColor(cv_ptr->image, drawing, CV_GRAY2RGB);
    if (contours.size() == 0){
      nodetectioncount += 1;
      ROS_INFO_STREAM("No detection " << nodetectioncount);
    }
    
    // Find the rotated rectangles and ellipse for each contour
    //vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> rotatedrectvec( contours.size() );
    vector<vector<Point> > hull( contours.size() );
    int count = 0;
    
    // loop through each contour, filter, draw
    for( int i = 0; i < contours.size(); i++ )
     {
      

      if( contours[i].size() > minArea && contours[i].size() < maxArea )
      {
        double epsilon;
        epsilon = 5;
        approxContour.resize(contours.size());
        anglevec.resize(contours.size());
        hullvec.resize(contours.size());
        //approxPolyDP(contours[i], approxContour[i], epsilon, true );
        rotatedrectvec.resize(contours.size());
        rotatedrectvec[i] = minAreaRect(contours[i]);
        approxPolyDP(contours[i], approxContour[i], epsilon, true );






      //if (pow((minEllipse[i].center.y-ycenter),2) + pow((minEllipse[i].center.x-xcenter),2) < pow(radius,2.0) ) //&& minEllipse[i].size.area() < maxArea)
      //if(rotatedrectvec[i] == 2 && contours[i].size() > minArea) //check for shape
          
            //cout << " ---- " << i << "----" << endl;
            //cout << "Contours: " << contours[i] << endl;
            //cout << "Rotated Rect: " << rotatedrectvec[i].size << endl;
            
            double rectratio;
            rectratio = rotatedrectvec[i].size.width/rotatedrectvec[i].size.height;

            if(rectratio < 1)
              rectratio = 1/rectratio;

            //cout <<"Ratio: " << rectratio << endl;

            Point pt;
            //convexHull( Mat(contours[i]), hull[i], false ); //hull stores the hull points
            

            //center of convex hull
            pt.x = rotatedrectvec[i].center.x;
            pt.y = rotatedrectvec[i].center.y;
            
            //ROS_INFO_STREAM("Ratio of m11: " << mu.m11);


            //if( pow(pt.x - 640,2) + pow(pt.y - 513,2) < pow(50,2))
            //{
            // compute angle of triangle as perp to longest vertex:
            /*  
            double maxdist;
            double tempdist;
            int maxdistpt[2] = {0,1};
            int tip = 2;
            Point2f pt0(approxContour[i].at(0).x,approxContour[i].at(0).y); // pt 0
            Point2f pt1(approxContour[i].at(1).x,approxContour[i].at(1).y); // pt 1
            Point2f pt2(approxContour[i].at(2).x,approxContour[i].at(2).y); // pt 2

            Point2f pt012[3] = {pt0,pt1,pt2};

            maxdist = norm(pt0 - pt1);
            
            tempdist = norm(pt0-pt2);
            if(tempdist > maxdist){
              maxdist = tempdist;
              tip = maxdistpt[1];
              maxdistpt[1] = 2;

            }

            tempdist = norm(pt1-pt2);
            if(tempdist > maxdist){
              maxdist = tempdist;
              tip = maxdistpt[0];
              maxdistpt[0] = 2;
            }

            //calculate angle:
            double angle = atan2(pt012[maxdistpt[0]].y-pt012[maxdistpt[1]].y,pt012[maxdistpt[0]].x-pt012[maxdistpt[1]].x);
            //cout << "pt one: " << pt012[maxdistpt[0]].x << " , " << pt012[maxdistpt[0]].y << endl;
            //cout << "pt two: " << pt012[maxdistpt[1]].x << " , " << pt012[maxdistpt[1]].y << endl;
            //cout << "angle: " << angle*180/PI<< endl;
            //minEllipse[i] = fitEllipse( Mat(contours[i]) );

            // REMEMBER IMAGE COORDINATES!!! AHHHH!
            double dangle[2] = {sin(angle), -cos(angle)};
            double dtip[2] = {pt012[tip].x - pt.x, pt012[tip].y - pt.y};
            double anglebetween = acos((dangle[0] * dtip[0] + dangle[1]*dtip[1])/(sqrt(dangle[0]*dangle[0] + dangle[1]*dangle[1])*sqrt(dtip[0]*dtip[0] + dtip[1]*dtip[1])));
            //cout << "dangle: " << dangle[0] << " , " << dangle[1] << endl;
            //cout << "dtip: " << dtip[0] << " , " << dtip[1] << endl;
            //cout << "dangle: " << anglebetween << endl;

            if(abs(anglebetween) > PI/2)
              angle = angle + PI;

            //save to ros msg
            xymsg.magx.push_back(pt.x);
            xymsg.magy.push_back(pt.y);
            xymsg.size.push_back(contourArea(hull[i]));
            xymsg.angle.push_back(angle);
            anglevec[i] = angle; // radians
            */
            //ROS_INFO_STREAM("ID " << i);
            ROS_INFO_STREAM("Area " << contours[i].size());
            //cout << "x = " << pt.x << ", y = " << pt.y << " size hull = " << contourArea(hull[i]) <<" angle = " << minEllipse[i].angle << endl;

            //draw
            Scalar color = Scalar(255 , 255, 0 );

            //if (count > 1){
            //  Scalar color = Scalar(0, 255, 255);
            // }
            Scalar colorhull = Scalar(0,255,255);//Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            Point2f vertices[4];
            rotatedrectvec[i].points(vertices);
            //for (int i = 0; i < 4; i++)
            //  line( drawing, vertices[i], vertices[(i+1)], color);
            if(rectratio > 2)
              drawContours( drawing, approxContour, i, color, CV_FILLED,8,hierarchy,1);


            //MyLine(drawing, pt, angle*180/PI);
          
        }
     }


    sensor_msgs::ImagePtr imgout;
    //Run this one time only!
    if(firstimg){
      // Initialize the location of robots with a click:
      int firstrobotidx = initializeRobot(cv_ptr_mono->image, contours);
      angle_in_bearth = anglevec[firstrobotidx];
      
      robothullsize = contours[firstrobotidx].size();



      // center of contour:
      Moments mufirst;
      mufirst = moments(contours[firstrobotidx], false);
      robotlastposn = Point2f( mufirst.m10/mufirst.m00, mufirst.m01/mufirst.m00 );
      cout << "Init Angle: " << angle_in_bearth << endl;
      cout << "Robot size: " << robothullsize << endl;
      cout << "Robot center: " << robotlastposn.x << " , " << robotlastposn.y << endl;
      //cout << "Ratio of m10:m01: " << mufirst.m11 << endl;
      // publish robot stats only:

      // rect:
      RotatedRect firstrect;
      firstrect = minAreaRect(contours[firstrobotidx]);
      cout << "Robot size: " << firstrect.size << endl;





      cv::waitKey();
    }
    else{
      // find index for robot based on last posn
      // publish rosmsg for robotonly_xymsg
      double mindist = 10000;
      int robotidx;
      for(int i = 0; i < xymsg.magx.size() ; i++){
        double tempdist;
        tempdist = pow(xymsg.magx[i]-robotlastposn.x,2) + pow(xymsg.magy[i]-robotlastposn.y,2);
        if(tempdist < mindist){
          robotidx = i;
          mindist = tempdist;
        }
      }

      bool robot_identified = false;


      if( (mindist < 10000))
      {
        robot_identified = true;
      }
      /*else if((abs(robothullsize-xymsg.size[robotidx])/robothullsize < 20))
      {
        robot_identified = true;
      }
      */

      if(robot_identified)
      {
        robotlastposn.x = xymsg.magx[robotidx];
        robotlastposn.y = xymsg.magy[robotidx];

        Mat robotpixMat(3,1, CV_64FC1, 1);
        Mat robotworldMat(3,1, CV_64FC1);
        robotpixMat.at<double>(0,0) = robotlastposn.x ;
        robotpixMat.at<double>(1,0) = robotlastposn.y ;
        robotworldMat = H * robotpixMat;

        //ROS_INFO_STREAM("robotposn (m) : " << robotworldMat);

        ROS_INFO_STREAM("robotposn: [ " << xymsg.magx[robotidx] << " , " << xymsg.magy[robotidx] << " ]");
        //ROS_INFO_STREAM("robot ratio: " );

        double angle_zeroed_rad = xymsg.angle[robotidx] - angle_in_bearth;
        double angle_zeroed_deg = angle_zeroed_rad*180/PI;

        //update robotposn:
        robotonly_xymsg.xyPixX.push_back((int)robotlastposn.x);
        robotonly_xymsg.xyPixY.push_back((int)robotlastposn.y);
        robotonly_xymsg.xySize.push_back(xymsg.size[robotidx]);
        robotonly_xymsg.xyAnglerad.push_back(angle_zeroed_rad);
        robotonly_xymsg.xyAngledeg.push_back(angle_zeroed_deg);

        robotonly_xymsg.xyWorldX.push_back(robotworldMat.at<double>(0,0));
        robotonly_xymsg.xyWorldY.push_back(robotworldMat.at<double>(1,0));
        robotonly_xymsg.actrobot = 1;
        robotonly_xymsg.sortidx.push_back(0);

        Scalar color = Scalar(255, 0, 0);
        drawContours( drawing, approxContour, robotidx, color, CV_FILLED,8,hierarchy,1);
        robotonly_xymsglast = robotonly_xymsg;
      }
      else{
        ROS_INFO_STREAM("Cannot find robot. Post old position");
        robotonly_xymsg = robotonly_xymsglast;
        robotonly_xymsglast.actrobot = 0; //to indicate not updated and using last known location.
      }
    }


      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());
      

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
      cout<<"No blobs detected.  Turn on light?" << endl;
    // Output position vector
    xyMagnet_pub_.publish(xymsg);
    xySingleRobot_pub_.publish(robotonly_xymsg);

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rawrobot"); // initialize node: magnet_track
  //ros::init(argc, argv, "magnetpose_real");

  ImageConverter ic;
  
  ros::spin();
  return 0;
}