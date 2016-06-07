#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iomanip>
#include <complex>
#include <cmath>
#include <opencv2/core/core.hpp>


// for ROS - Opencv
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <plane_camera_magnet/xyPix.h>
#include <plane_camera_magnet/xyFiltered.h>
#include <cv_bridge/cv_bridge.h> 
#include <sstream>

// Kalman filter
//#include "plane_camera_magnet/filter.h"

using namespace std;
//using namespace std::literals;
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
        vector<double> xyPixAngradraw;
        vector<double> xyPixAngrad;
        //vector<std::complex<double> > angWorldcomplex;
        //vector<std::complex<double> > angWorldcomplexraw;
        vector<double> xyPixAngradcameraframe;

        int numdetections;



        
        // calibration file to convert to world
        double pix2m;
        double centerpixx;
        double centerpixy;

        double th0;
        bool initialflag;
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
        cout << "Center (pix):" << centerpixx << "," << centerpixy << endl;
        cout << "Pix2m (pix per mm):" << pix2m << endl;
        

        cout << "Click when ready to Initialize robots" << endl;
        cin.ignore(1);
        initialflag = true; // to save th0. for single magnet only!
        th0 = 0; //initial angle without applied fields (Bearth)


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
    xyPixAngradcameraframe = data.angle; // - th0;
    numdetections = data.numrobot;

    /*xyPixAngrad.resize(xyPixAngradcameraframe.size());

    for(int k=0; k < xyPixAngradcameraframe.size(); k++)
    {
        cout << "complexraw: " << xyPixAngradcameraframe.at(k) << endl;
        xyPixAngrad.[k] = -1. * xyPixAngradcameraframe.at(k); // zero field alighns with Bearth = -PI
        //angWorldcomplexraw.at(k) = std::exp(1i * -xyPixAngradcameraframe.at(k)); // zero field alighns with Bearth = -PI
        cout << "complexraw: " << xyPixAngradcameraframe.at(k) << endl;
    }

*/    
    // initializte th0:
    if(initialflag == true){
        //th0 = -xyPixAngrad.at(0);
        th0 = -xyPixAngradcameraframe.at(0);
        initialflag = false;
        cout << "th0: " << th0 << endl;
        cin.ignore(1);
    }

    // update xyPixAngrad.  As measured from world frame xaxis
    xyPixAngrad.resize(xyPixAngradcameraframe.size());
    for(int i=0; i < xyPixAngradcameraframe.size(); i++)
    {
        //xyPixAngrad[i] = WrapPosNegPI(xyPixAngradraw[i] - (th0)); // zero field alighns with Bearth = -PI
        xyPixAngrad[i] = -xyPixAngradcameraframe[i] - (th0); // zero field alighns with Bearth = -PI
    }
    // Initialize lastpose
    //cout << "posex size" << lastposex.size() << endl;
    if(lastposex.size() == 0){
        cout<< "Update lastpose" << endl;
        lastposex = xyPixX;
        lastposey = xyPixY;
        lastposeang = xyPixAngrad;
        
        actrobot = numdetections;
        sizefirstframe = xyPixSize;

        //cout << "Lastposex: " << lastposex[0] << ',' << lastposex[1] << endl;
        //cout << "Number of robots: " << actrobot << endl;
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
    double xyPixAngradraw [actrobot];

    // vector<double> xyPixXraw;
    // vector<double> xyPixYraw;
    // vector<double> xyPixSizeraw;
    // vector<double> xyPixAngdegraw;

    //cout << "distmat" << endl;
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
            //cout << minidx[r] << endl;
        }

    /*
    cout << "==minidx==" << endl;
    cout << minidx[0] << "," << minidx[1] << endl;
    cout << minval[0] << "," << minval[1] << endl;
    cout << "==minidx==" << endl;
    */
    // check for same mapping if 2 detections and values are the same
    if (minidx[0] == minidx[1] && numdetections == 2)
        {
            //cout << "same detection" << endl;
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
                xyPixAngradraw[i] = xyPixAngrad[minidx[i]];
                xyPixSizeraw[i] = xyPixSize[minidx[i]];
            }
    }
    else // numdetection < actrobot
    {
        //cout << "detections less than actrobot" << endl;
        // populate with xyPixXraw with lastpose:
        for (int i = 0; i<actrobot; i++)
        {
            xyPixXraw[i] = lastposex.at(i);
            xyPixYraw[i] = lastposey.at(i);
            xyPixAngradraw[i] = lastposeang.at(i);
            xyPixSizeraw[i] = 0;
        }

        for (int i=0; i<numdetections; i++)
            {
                if (minval[i] < 20.0)
                {// keep assignment if dist is less than 20.
                    xyPixXraw[i] = xyPixX[minidx[i]];
                    xyPixYraw[i] = xyPixY[minidx[i]];
                    xyPixAngradraw[i] = xyPixAngrad[minidx[i]];
                    xyPixSizeraw[i] = xyPixSize[minidx[i]];
                }
                else
                    minidx[i] = -1; // no detection for this robot.
            }
    }

    // 
    vector<double> xyWorldX (actrobot,0.0);
    vector<double> xyWorldY (actrobot,0.0);
    vector<double> xyAngledeg (actrobot,0.0);
    vector<double> xyPixSizeVector (actrobot,0.0);
    vector<int8_t> minidxVector (actrobot,-1);

    for (int i = 0; i<actrobot; i++)
    {
        // 2. Check angle and update
        // check if the value jumped.
        
        //cout << lastposeang.at(i) << endl;
        //ROS_INFO_STREAM("th: " << xyPixAngradraw[i] << "," << lastposeang[i]);
        //double dtheta =
        if(std::abs(xyPixAngradraw[i] - lastposeang[i]) > PI/2)
        {
           //cout << "Diff: " << std::abs(xyPixAngradraw[i] - lastposeang[i]) << endl;
            xyPixAngradraw[i] += PI;
        }
        //cout << "update" << endl;
        // 3. Update last pose
        lastposex.at(i) = xyPixXraw[i];
        lastposey.at(i) = xyPixYraw[i];
        // round to +/- PI - use a custom mod function.
        lastposeang.at(i) = WrapPosNegPI(xyPixAngradraw[i]);
        //cout << "lastpose" << endl;
        // 4. Convert to xySorted coords
        xyWorldX.at(i) = (xyPixXraw[i] - centerpixx)/pix2m;
        xyWorldY.at(i) = (-xyPixYraw[i] + centerpixy)/pix2m;
        xyAngledeg.at(i) = lastposeang.at(i) * 180.0/PI;
        
        xyPixSizeVector.at(i) = xyPixSizeraw[i];
        minidxVector.at(i) = minidx[i];

    }

    
    // export xyworldX, xyworldY, xyWorldRad, numdetections idx.      
    

    //cout << "SortOrder:" << minidx[0] << ',' << minidx[1] << endl;
    

    // ********* Filter position *******************
    // Kalman filter, based on constant velocity model

    //1. Prediction:
    
    //2. Update: 

    //save to rosmsg
    xymsg.header.stamp = ros::Time::now();
    xymsg.header.frame_id = data.header.frame_id;
    xymsg.xyPixX = lastposex;
    xymsg.xyPixY = lastposey;
    xymsg.xySize = xyPixSizeVector;
    xymsg.xyAngledeg = xyAngledeg;
    xymsg.xyAnglerad = lastposeang;
    xymsg.actrobot = actrobot;
    xymsg.sortidx = minidxVector;

    xymsg.xyWorldX = xyWorldX;
    xymsg.xyWorldY = xyWorldY;

    xyFiltered_pub_.publish(xymsg);
}


double Mod(double x, double y)
{
    //static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");

    if (0. == y)
        return x;

    double m= x - y * floor(x/y);

    // handle boundary cases resulted from floating-point cut off:

    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;

        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14 
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14 
        }
    }

    return m;
}

// wrap [rad] angle to [-PI..PI)
double WrapPosNegPI(double fAng)
{
    return Mod(fAng + PI, 2*PI) - PI;
}

void printarray (double arg[], int length) {
  //for (int n=0; n<length; ++n)
  //  cout << arg[n] << ' ';
  //cout << '\n';
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
