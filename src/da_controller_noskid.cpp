// rosbag play -l sample_dakf_pose_magnetactual.bag 
#include <ros/ros.h>
#include <stdio.h>

#include "currentcompute.h" // contains struct : Coil, Magnet
#include <Eigen/SVD>
#include <Eigen/Geometry> 
#include <plane_camera_magnet/xyFiltered.h>
#include <plane_camera_magnet/roboclawCmd.h>


using Eigen::MatrixXd;
using namespace Eigen;
using namespace std;
using std::sqrt;

class CurrentCalc
{
    ros::NodeHandle nh_;
    ros::Subscriber mag_sub_; //xyPix  
    ros::Publisher current_pub_; //roboclaw
    
    plane_camera_magnet::xyFiltered last_pose_;
    plane_camera_magnet::roboclawCmd current_desired;

    // variables
    Coil coil;
    Magnet magnet;

    double amin;
    Vector2d fd;  //assign desired force
    Vector2d m; //magnetic moment
    
public:
    CurrentCalc()
    {
        //subscribe to kfpose output - xyFiltered
        mag_sub_ = nh_.subscribe("/dakf_pose/magnetactual", 1,
            &CurrentCalc::xyCb, this);

        current_pub_ = nh_.advertise<plane_camera_magnet::roboclawCmd>("currentcmddesired",1);

        // Constants
        //Bearth << -0.00004,0.; // -40 uT
        coil.d = .0575;
        coil.R = 0.075;
        magnet.gamma = 35.8 * pow(10,-6);       

        fd << 1,0.1;
        fd = fd*pow(10,-6);

        m = -magnet.gamma * fd/fd.norm();

        amin = 0.08;// pow(10,-6);

    }

    ~CurrentCalc()
    {

    }

    void update() {
        if (last_pose_.xyWorldX.size() != 0){
            current_pub_.publish(current_desired);

        }
    }

    void compute_current(){
        // Compute m * Dx, m * Dy
       MatrixXd K(2,4);
       K.row(0) = m.transpose() * magnet.Dxmat;
       K.row(1) = m.transpose() * magnet.Dymat;


       MatrixXd G(4,4);
       G << K, magnet.Bmat;

       // Compute SVD of G:
       JacobiSVD<MatrixXd> svdG(G, ComputeThinU | ComputeThinV);
       FullPivLU<MatrixXd> G_decomp(G);
       auto rankG = G_decomp.rank();
       cout << "rank: " << rankG << endl;

       // determine rank 
       VectorXd f00(4);
       VectorXd zerof(4);
       f00 << fd/fd.norm(), 0.0, 0.0;
       zerof << 0.0, 0.0, fd/fd.norm();

       // compute optimized a:
       //([fd' zerovec'] * U*inv(V)*inv(V) * inv(U)' * [zerovec;fd])/([zerovec' fd'] * U*inv(V)*inv(V) * U' * [zerovec;fd])
       MatrixXd U(4,4);
       MatrixXd V(4,4);

       U = svdG.matrixU(); // Matrix of right eigenvectors

       // Diagonal of singular values
       V = MatrixXd::Identity(4,4);
       VectorXd svdG_sv = svdG.singularValues();
       for(int diag = 0; diag < 4; diag++){
            V(diag,diag) = svdG_sv(diag);
       }

       //V = MatrixXd::Identity(4,4) * svdG.singularValues(); // Matrix of eigenvalues on diagonals arranged in decreasing order
       MatrixXd mattest(4,4);
       MatrixXd a_opt = f00.transpose() * U * V.inverse() * V.inverse() * U.inverse().transpose() * zerof /( zerof.transpose() * U * V.inverse() * V.inverse() * U * zerof);

       cout << "a_opt=  " << a_opt << endl;

       double a = (double) a_opt(0,0);
       // Check rank and sign of a_opt:
       if (a > 0 && rankG == 4){
        cout << "Full rank, use a_opt to compute i." << endl;
       }
       
       else if (rankG < 4){
        // compute d perpendicular to u.
        cout << "G is singular... Compute d perdendicular to u." << endl;
        VectorXd u = U.col(3);
        a = (double) (u.transpose() * f00) / (u.transpose() * zerof) ; 
       }


       if (abs(a) < amin){
        cout << "Use amin" << endl;
        a = amin;
       }

       VectorXd current(4);

       VectorXd falphaf_vec(4);

       falphaf_vec << fd/fd.norm(), -a*fd/fd.norm();

       // compute current:
       current = G.inverse() * falphaf_vec;

       current_desired.m1 = current[0];
       current_desired.m2 = current[1];
       current_desired.m3 = current[2];
       current_desired.m4 = current[3];

       current_desired.fx = fd[0];
       current_desired.fy = fd[1];

       cout << "current: " << current << endl;
       cout << "-----" << endl;

    }

    void publish_current_update(const plane_camera_magnet::xyFiltered &msg) {
        //compute new current (from controllersandbox.cpp)
        cout << "magnet xy: " << last_pose_.xyWorldX[0] << last_pose_.xyWorldY[0] << endl;
        magnet.x = last_pose_.xyWorldX[0];
        magnet.y = last_pose_.xyWorldY[0];
        magnet.Dxmat = Dx(magnet.x,magnet.y,coil.R, coil.d);
        magnet.Dymat = Dy(magnet.x,magnet.y,coil.R, coil.d);
        magnet.Bmat = computeBmat(magnet.x,magnet.y,coil.R, coil.d);

        compute_current();

        //publish current
        update();
    }

    void xyCb(const plane_camera_magnet::xyFiltered& msg) 
    {
        if (last_pose_.xyWorldX.size() == 0){
            last_pose_ = msg;
            cout << "No xyWorldX" << endl;
        }

        else{
            cout << "Update lastpose" << endl;

            last_pose_.header = msg.header;
            last_pose_.xyWorldX = msg.xyWorldX;
            last_pose_.xyWorldY = msg.xyWorldY;
            last_pose_.xyAngledeg = msg.xyAngledeg;
        }

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "da_controller_noskid"); // initialize node: magnet_track
    ros::NodeHandle n("~");

    CurrentCalc cc;

    ros::Rate r(10.0); //controls current command publishing rate
    while (n.ok()) {
        cc.update();
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}