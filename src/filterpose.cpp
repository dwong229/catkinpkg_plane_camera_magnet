#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>


// for ROS - Opencv
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <plane_camera_magnet/xyPix.h>
#include <plane_camera_magnet/xyFiltered.h>
#include <cv_bridge/cv_bridge.h> 
#include <sstream>

using namespace std;
using namespace cv;

/* filterpose.cpp : 
    - Subscribes to /track_rod_orient/xyPix
    - Load calplane.yml
    - Publishes to /filterpose/xyPix
                   /filterpose/xyReal
    Given xyPix, determine with magnet is which 
TO ADD: 
    - Kalman filter
*/


#define PI 3.14159265

class FilterPose
    {
    //initialize variables for subscribe:
        ros::NodeHandle nh_;
        ros:: Subscriber xyPix_sub_;
        ros:: Publisher xyFiltered_pub_;
        vector<double> xyPixX;
        vector<double> xyPixY;
        vector<double> xyPixSize;
        vector<double> xyPixAngdeg;
        int numdetections;


        
        // calibration file to convert to world
        double pix2m;
        double centerpixx;
        double centerpixy;

        // lastposition
        vector<double> lastposex;
        vector<double> lastposey;
        vector<double> lastposeang;

        // for calculation
        int actrobot;
        vector<double> sizefirstframe;

public:
    FilterPose()
        :nh_("~") 
    { 
        // load calibration yml.
        std::string cal_file;    
        nh_.param("cal_file", cal_file, std::string("calplane.yml"));
        ROS_INFO_STREAM("cal file " << cal_file);
        FileStorage fscal(cal_file.c_str(), FileStorage::READ);
        centerpixx = (double)fscal["coilavgx"];
        centerpixy = (double)fscal["coilavgy"];
        pix2m = (double)fscal["pix2m"];


        //lastposex[0] = -1;
        cout << "Center (pix):" << centerpixx << centerpixy << endl;
        cout << "Pix2m (pix per mm):" << pix2m << endl;
        

        cout << "Click when ready to Initialize robots" << endl;
        cin.ignore(1);
        // publish to xyFiltered
        xyFiltered_pub_ = nh_.advertise<plane_camera_magnet::xyFiltered>("xyFiltered",1);   
        // subscribe to xyPix
        xyPix_sub_ = nh_.subscribe("/track_rod_orient/xyPix",1,&FilterPose::xyPixcallback, this);
        
    }//public
    // Subscribe to /.../xyPix and Publish filtered pose

void xyPixcallback(const plane_camera_magnet::xyPix& data)
{
    plane_camera_magnet::xyFiltered xymsg; //publish

    xyPixX = data.magx;
    xyPixY = data.magy;
    xyPixSize = data.size;
    xyPixAngdeg = data.angle;
    numdetections = data.numrobot;

    //cout << xyPixX[0] << "," << xyPixX[1] << endl;

    // Initialize lastpose
    //cout << "posex size" << lastposex.size() << endl;
    if(lastposex.size() == 0){
        cout<< "Update lastpose" << endl;
        lastposex = xyPixX;
        lastposey = xyPixY;
        lastposeang = xyPixAngdeg;
        
        actrobot = numdetections;
        sizefirstframe = xyPixSize;

        cout << "Lastposex: " << lastposex[0] << ',' << lastposex[1] << endl;
        cout << "Number of robots: " << actrobot << endl;
    }

    //cout << "Detect and ID" << endl;
    //------- determine detection ID: ---------------
    // 1. compare distance between pairs of points

    //Mat distmat(actrobot,numdetections,CV_8UC1); //[actrobot x numdetection] : dist between actrobot numdetections
    // array
    double distmat [actrobot][numdetections];
    int minidx [actrobot];
    double minval [actrobot];
    // raw sorted values
    double xyPixXraw [actrobot];
    double xyPixYraw [actrobot];
    double xyPixSizeraw [actrobot];
    double xyPixAngdegraw [actrobot];

    // vector<double> xyPixXraw;
    // vector<double> xyPixYraw;
    // vector<double> xyPixSizeraw;
    // vector<double> xyPixAngdegraw;

    cout << "distmat" << endl;
    for( int r = 0; r < actrobot; r++)
        { for( int c = 0; c<numdetections; c ++)
            {
                // distance between actrobot[r] and detection[c]
                
                distmat[r][c] = pow(lastposex[r] - xyPixX[c],2) + pow(lastposey[r] - xyPixY[c],2);

            }
    
            // find min dist idx for each detection
            printarray(distmat[r], numdetections);
            
            // populate value of idx of actrobot that is most likely corresponding to actrobot
            indexofSmallestElement(distmat[r],numdetections,minidx[r],minval[r]);
            cout << minidx[r] << endl;
        }

    cout << "==minidx==" << endl;
    cout << minidx[0] << "," << minidx[1] << endl;
    cout << minval[0] << "," << minval[1] << endl;
    cout << "==minidx==" << endl;
    
    // check for same mapping if 2 detections and values are the same
    if (minidx[0] == minidx[1] && numdetections == 2)
        {
            cout << "same detection" << endl;
            if (numdetections == 2)
                { 
                    if (sizefirstframe[0] > sizefirstframe[1])
                    {
                        if (xyPixSize[0] > xyPixSize[1])
                        {
                            minidx[0] = 0;
                            minidx[1] = 1;
                        }
                        else
                        {
                            minidx[0] = 1;
                            minidx[1] = 0;
                        }
                    }
                    else
                    {
                     if (xyPixSize[0] < xyPixSize[1])
                        {
                            minidx[0] = 0;
                            minidx[1] = 1;
                        }
                        else
                        {
                            minidx[0] = 1;
                            minidx[1] = 0;
                        }   
                    }
                }
        }

    if (numdetections >= actrobot)
    {
        for (int i=0; i<actrobot; i++)
            {
                xyPixXraw[i] = xyPixX[minidx[i]];
                xyPixYraw[i] = xyPixY[minidx[i]];
                xyPixAngdegraw[i] = xyPixAngdeg[minidx[i]];
                xyPixSizeraw[i] = xyPixSize[minidx[i]];
            }
    }
    else // numdetection < actrobot
    {
        cout << "detections less than actrobot" << endl;
        // populate with xyPixXraw with lastpose:
        for (int i = 0; i<actrobot; i++)
        {
            xyPixXraw[i] = lastposex.at(i);
            xyPixYraw[i] = lastposey.at(i);
            xyPixAngdegraw[i] = lastposeang.at(i);
            xyPixSizeraw[i] = 0;
        }

        for (int i=0; i<numdetections; i++)
            {
                if (minval[i] < 20.0)
                {// keep assignment if dist is less than 20.
                    xyPixXraw[i] = xyPixX[minidx[i]];
                    xyPixYraw[i] = xyPixY[minidx[i]];
                    xyPixAngdegraw[i] = xyPixAngdeg[minidx[i]];
                    xyPixSizeraw[i] = xyPixSize[minidx[i]];
                }
                else
                    minidx[i] = -1; // no detection for this robot.
            }
    }

    // 
    vector<double> xyWorldX (actrobot,0.0);
    vector<double> xyWorldY (actrobot,0.0);
    vector<double> xyAnglerad (actrobot,0.0);
    vector<double> xyPixSizeVector (actrobot,0.0);
    vector<int8_t> minidxVector (actrobot,-1);

    for (int i = 0; i<actrobot; i++)
    {
        // 2. Check angle and update
        cout << "update" << endl;
        // 3. Update last pose
        lastposex.at(i) = xyPixXraw[i];
        lastposey.at(i) = xyPixYraw[i];
        lastposeang.at(i) = xyPixAngdegraw[i];
        cout << "lastpose" << endl;
        // 4. Convert to xySorted coords
        xyWorldX.at(i) = (xyPixXraw[i] - centerpixx)/pix2m;
        xyWorldY.at(i) = (-xyPixYraw[i] + centerpixy)/pix2m;
        xyAnglerad.at(i) = xyPixAngdegraw[i] * PI/180.0;
        
        xyPixSizeVector.at(i) = xyPixSizeraw[i];
        minidxVector.at(i) = minidx[i];

    }

    
    // export xyworldX, xyworldY, xyWorldRad, numdetections idx.      
    

    cout << "SortOrder:" << minidx[0] << ',' << minidx[1] << endl;
    
    //save to rosmsg
    xymsg.header.stamp = ros::Time::now();
    xymsg.xyPixX = lastposex;
    xymsg.xyPixY = lastposey;
    xymsg.xySize = xyPixSizeVector;
    xymsg.xyAngledeg = lastposeang;
    xymsg.xyAnglerad = xyAnglerad;
    xymsg.actrobot = actrobot;
    xymsg.sortidx = minidxVector;

    xymsg.xyWorldX = xyWorldX;
    xymsg.xyWorldY = xyWorldY;

    xyFiltered_pub_.publish(xymsg);
}

void printarray (double arg[], int length) {
  for (int n=0; n<length; ++n)
    cout << arg[n] << ' ';
  cout << '\n';
}

void indexofSmallestElement(double array[], int size, int& index, double& value)
// pass arguments out by reference (instead of by value)
{
    index = 0;
    value = array[index];
    for(int i = 1; i < size; i++)
    {
        if(array[i] < array[index])
        {
            index = i;
            value = array[i];      
        }
         
    }
    //cout << "indexofSmallestElement:"<< index << "," << value << endl;
}

};//class;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "filterpose"); // initialize node: filterpose
        
    FilterPose fp;


    ros::spin();
    return 0;
}
