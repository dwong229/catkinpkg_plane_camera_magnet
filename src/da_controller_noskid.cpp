// rosbag play -l sample_dakf_pose_magnetactual.bag 

#include <stdio.h>

#include "currentcompute.h" // contains struct : Coil, Magnet
#include <Eigen/SVD>
#include <Eigen/Geometry> 


using Eigen::MatrixXd;
using namespace Eigen;
using namespace std;
using std::sqrt;

class CurrentCalc
{
    ros::NodeHandle nh_;
    //
};

int main(int argc, char** argv)
{
    
}

