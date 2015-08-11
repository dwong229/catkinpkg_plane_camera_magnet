#include <ros/ros.h>
#include <iostream>
#include <plane_camera_magnet/PositionCommand.h>
#include <plane_camera_magnet/roboclawCmd.h>
#include <plane_camera_magnet/xyFiltered.h>
#include <plane_camera_magnet/nonlinearsolversoln.h>
#include "currentcompute.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h> //for rvis visualization

using namespace std;

// User input a point and control to stay at that point.

plane_camera_magnet::PositionCommand mag1_actual, mag2_actual;
int numrobot;
bool newmsg;
static visualization_msgs::Marker fdesvis;

void xyFilteredcallback1(const plane_camera_magnet::xyFiltered& data)
{

  // unpack subscribed values to vector of keypoints
    numrobot = data.actrobot;
    
    if(!data.xyWorldX.empty())
    {
        mag1_actual.position.x = double(data.xyWorldX.at(0));
        mag1_actual.position.y = double(data.xyWorldY.at(0));
        mag1_actual.velocity.x = double(data.xyWorldXdot.at(0));
        mag1_actual.velocity.y = double(data.xyWorldYdot.at(0));
        newmsg = true;
    }
    

}
void xyFilteredcallback2(const plane_camera_magnet::xyFiltered& data)
{

   // unpack subscribed values to vector of keypoints   
    if(!data.xyWorldX.empty())
    {
        mag2_actual.position.x = double(data.xyWorldX.at(0));
        mag2_actual.position.y = double(data.xyWorldY.at(0));
        mag2_actual.velocity.x = double(data.xyWorldXdot.at(0));
        mag2_actual.velocity.y = double(data.xyWorldYdot.at(0));
        newmsg = true;
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "keyboard2mag");
    ros::NodeHandle nh("~");

    plane_camera_magnet::roboclawCmd roboclawCmdDesired;
    plane_camera_magnet::nonlinearsolversoln solversoln_msg;
    //Setup publisher:
    ros::Publisher roboCmdDes_pub = nh.advertise<plane_camera_magnet::roboclawCmd>("roboclawcmddesired",1); // always publish newest command
    ros::Publisher solversoln_pub = nh.advertise<plane_camera_magnet::nonlinearsolversoln>("nonlinearsolversoln",1); // always publish newest command
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("vis_goal", 10);

    ros::Subscriber xyFiltered1_sub_ = nh.subscribe("/kfpose_mag1/magnetactual",1, xyFilteredcallback1);
    ros::Subscriber xyFiltered2_sub_ = nh.subscribe("/kfpose_mag2/magnetactual",1, xyFilteredcallback2);

    // define structs Coil and Magnet to be used in solver:
    Coil coil;
    coil.R = 8900;
    coil.d = 57.5;
    Magnet magnet;
    magnet.gamma = 6500;
    
    fdesvis.id = 2;
    fdesvis.type = visualization_msgs::Marker::LINE_LIST;
    fdesvis.scale.x = 0.1;

    fdesvis.color.b = 1.0f;
    fdesvis.color.a = 1.0;

    const int n =4; // 4I , 2 lambda
    int info;
    VectorXd b(n);
    b << 1.,1.,1.,20.;

    while(ros::ok())
    {
        // take in position of robot
        mag1_actual.acceleration.x = 0;
        mag1_actual.acceleration.y = 0;
        
        double mass = 4*0.0000707;
        
        // Wait for input: 
        int input;
        cout << "Enter mode: 1:pull apart 2:push together 3:off" << endl;
        std::cin >> input;


        if(input == 1)
            {   
                cout<< "Pull Apart" << endl;
            }
            else if (input == 2)
            {
                cout << "Push Together" << endl;

            }
            else
            {
                cout << "Coils Off" << endl;
            }


        // compute F desired depending on input:
        double Fdes[n];


        Fdes[0] = 1.0;
        Fdes[1] = 1.0;
        Fdes[2] = 1.0;
        Fdes[3] = 1.0;

        geometry_msgs::Point fdespoint1,fdespoint2;
        fdespoint1.x = Fdes[0];
        fdespoint1.y = Fdes[1];
        fdespoint1.z = 0;
        fdespoint2.x = Fdes[2];
        fdespoint2.y = Fdes[3];
        fdespoint2.z = 0;

        //cout << "F: " << Fdes[0] << ", " << Fdes[1] << endl;
        //update fdes
        fdesvis.points.resize(4);
        fdesvis.points[0] = mag1_actual.position;
        fdesvis.points[1] = fdespoint1;
        fdesvis.points[2] = mag2_actual.position;
        fdesvis.points[3] = fdespoint2;

        // compute currents to send to roboclaw using nonlinear solver.
        // update magnet variables:
        magnet.x = mag1_actual.position.x;
        magnet.y = mag1_actual.position.y;
        magnet.Fx = Fdes[0];
        magnet.Fy = Fdes[1];
        magnet.Mxmat = Mx(magnet.x,magnet.y,coil.R,coil.d);
        magnet.Mymat = My(magnet.x,magnet.y,coil.R,coil.d);
        magnet.Bmat = computeBmat(magnet.x,magnet.y,coil.R,coil.d);

        magnet.x2 = mag2_actual.position.x;
        magnet.y2 = mag2_actual.position.y;
        magnet.Fx2 = Fdes[2];
        magnet.Fy2 = Fdes[3];
        magnet.Mxmat2 = Mx(magnet.x2,magnet.y2,coil.R,coil.d);
        magnet.Mymat2 = My(magnet.x2,magnet.y2,coil.R,coil.d);
        magnet.Bmat2 = computeBmat(magnet.x2,magnet.y2,coil.R,coil.d);
        //solve:
        //b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
        CoilFunctor2 functor(coil, magnet); // functor( ) add arguments here.
        LevenbergMarquardt<CoilFunctor2> lm(functor);
        info = lm.minimize(b);   
        //cout << "info: " << info << endl; //LM error: 1: RelativeReductionTooSmall, 2:RelativeErrorTooSmall, 3:RelativeErrorAndReductionTooSmall
        
        // solversoln_msg: ///////////////////
        solversoln_msg.header.stamp = ros::Time::now();
        solversoln_msg.info = info;

        double xtemp[] = {mag1_actual.position.x, mag1_actual.position.y, mag2_actual.position.x, mag2_actual.position.y};
        //double dxtemp[] = {goal1.position.x - mag1_actual.position.x, goal1.position.y - mag1_actual.position.y, goal2.position.x - mag2_actual.position.x, goal2.position.y - mag2_actual.position.y};

        solversoln_msg.xactual = vector<double> (xtemp, xtemp+ 4);
        solversoln_msg.dx = vector<double> (dxtemp,dxtemp +4);
        

        solversoln_msg.Fdes = vector<double> (Fdes,Fdes + 4);
        solversoln_msg.info = info;
        
        // compute error: F(current) - Fdesired
        VectorXd error(4);
        functor.operator()(b,error);
        cout << "error: " << error.transpose() << endl;
        solversoln_msg.error = vector<double> (error.data(),error.data() + error.rows() * error.cols());

        //  add notion of error to detemine if solution should be published
        if(info==2) // || info == 1)
        {
            //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
            VectorXd current(4);
            current = b;
            //cout << "b: " << b.transpose() << endl;
            //cout << "current: \n " << current << endl;
            vector<double> btemp;
            vector<double> currtemp;

            for(int i = 0; i<n; i++)
            {
                btemp.push_back(b[i]);
                currtemp.push_back(current[i]); 
            }
            //solversoln_msg.solnvec = vector<double> (btemp,btemp + 6);
            //solversoln_msg.current = vector<double> (currtemp,currtemp + 4);

            solversoln_msg.solnvec = btemp;
            solversoln_msg.current = currtemp;
        
            roboclawCmdDesired.header.stamp = ros::Time::now();
            roboclawCmdDesired.m1 = int(current[0]);
            roboclawCmdDesired.m2 = int(current[1]);
            roboclawCmdDesired.m3 = int(current[2]);
            roboclawCmdDesired.m4 = int(current[3]);
        
            // publish pwm commands to roboclawCmdDesired
            roboCmdDes_pub.publish(roboclawCmdDesired);
        }
        else
        {
            // if no solution, random start b.
            cout << "re-init b" << endl;
            b = VectorXd::Random(n)*100; // F = [0.0283, 0]
            //b << 1.,1.,1.,20.;
            // test for large coil 3:
            //b[2] = 1000;
            cout << "b: " << b << endl;
        }

        solversoln_pub.publish(solversoln_msg);
        //loop_rate.sleep();
        vis_pub.publish(fdesvis);

        ros::spinOnce();
    }
    return 0;
}