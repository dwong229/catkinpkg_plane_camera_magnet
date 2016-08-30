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
#include <plane_camera_magnet/xyFiltered.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Input";

class ImageInput
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    ros::Publisher goal_pub_;


    Mat H;
    Point2f lastclick;

public:
    ImageInput()
        :nh_("~") ,it_(nh_)
    {
        image_sub_ = it_.subscribe("/pg_14434226/image_rect", 1,
            &ImageInput::imageCb, this);

        goal_pub_ = nh_.advertise<plane_camera_magnet::xyFiltered>("uiGoal",1);
   
        cv::namedWindow(OPENCV_WINDOW);
        //set the callback function for any mouse event
        cv::setMouseCallback(OPENCV_WINDOW, &ImageInput::callBackFunc, this);

        //unpack calibration files for camera:
        std::string cal_file;    
        nh_.param("cal_file", cal_file, std::string("cal.yml"));
        ROS_INFO_STREAM("cal file " << cal_file);
        FileStorage fscal(cal_file.c_str(), FileStorage::READ);
        fscal["H"] >> H;
        cout<< "H: " << H << endl;

    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
       
        cv_bridge::CvImagePtr cv_ptr_rbg;

        try
        {
            cv_ptr_rbg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what() );
            return;
        }

        //cv_ptr_rbg ->image

        //draw x at lastclick:

        circle(cv_ptr_rbg->image, lastclick, 3, Scalar(0,0,255),3);

        cv::imshow(OPENCV_WINDOW, cv_ptr_rbg->image );
        
        // setup msg and publish goal: 
        Mat robotpixMat(3,1, CV_64FC1, 1);
        Mat robotworldMat(3,1, CV_64FC1);
        robotpixMat.at<double>(0,0) = lastclick.x ;
        robotpixMat.at<double>(1,0) = lastclick.y ;
        robotworldMat = H * robotpixMat;


        plane_camera_magnet::xyFiltered goal_msg;


        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "camera_frame";
        goal_msg.xyPixX.push_back(lastclick.x);
        goal_msg.xyPixY.push_back(lastclick.y);
        goal_msg.xyWorldX.push_back(robotworldMat.at<double>(0,0));
        goal_msg.xyWorldY.push_back(robotworldMat.at<double>(1,0));
        goal_msg.actrobot = 1;


        // convert to world coords:

        goal_pub_.publish(goal_msg);
        cv::waitKey(1);
        //ROS_INFO_STREAM("Last click: " << lastclick.x << " , " << lastclick.y);
    }

    void assignlastclick(int x, int y)
    {
        lastclick.x = x;
        lastclick.y = y;
    }

    static void callBackFunc(int event, int x, int y, int flags, void* this_)
    {
        if  ( event == EVENT_LBUTTONDOWN )
        {
            cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
            //ImageInput::assignlastclick(x,y);
             static_cast<ImageInput*>(this_)->assignlastclick(x,y);
        }
        else if  ( event == EVENT_RBUTTONDOWN )
        {
            //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        }
        else if  ( event == EVENT_MBUTTONDOWN )
        {
            //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        }
        else if ( event == EVENT_MOUSEMOVE )
        {
            //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

        }

    }


};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_click"); // initialize node: magnet_track
  //ros::init(argc, argv, "magnetpose_real");

  ImageInput ii;
  
 
  ros::spin();
  return 0;
}