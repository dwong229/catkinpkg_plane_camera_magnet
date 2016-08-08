// Sandbox for testing no-skid controller
#include <stdio.h>

#include "currentcompute.h" // contains struct : Coil, Magnet
#include <Eigen/SVD>
#include <Eigen/Geometry> 


using Eigen::MatrixXd;
using namespace Eigen;
using namespace std;
using std::sqrt;




int main(void)
{
   const int n =2;
   int info;

   VectorXd fd(n);  //assign desired force
   VectorXd p(2); //position

   p << 10,0;
   fd << 1,0.1;

   p = p*pow(10,-3);
   fd = fd*pow(10,-6);

   double amin = 0.08;// pow(10,-6);
   // setup parameters:
   Coil coil;

   coil.R = 0.075;
   coil.d = 0.0571;

   Magnet magnet;
   magnet.gamma = 6500;
   magnet.x = p[0];
   magnet.y = p[1]; 

   VectorXd m(2); //magnetic moment
   m = -magnet.gamma * fd/fd.norm();

   //compute B-matrix and Dx and Dy for position, p:
   magnet.Dxmat = Dx(magnet.x,magnet.y,coil.R, coil.d);
   magnet.Dymat = Dy(magnet.x,magnet.y,coil.R, coil.d);
   magnet.Bmat = computeBmat(magnet.x,magnet.y,coil.R, coil.d);

   cout << "Dx: " << magnet.Dxmat << endl;
   cout << "Dy: " << magnet.Dymat << endl;


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

   cout << "current: " << current << endl;

//   cout << "K: " << K << endl;
//   cout << "G: " << G << endl;

   cout << "position: " << p[0] << " , " << p[1] << " m." << endl;
   cout << "desired force: " << fd << endl;


   
   //

   //cout << "current: " << x[0] << ", " << x[1] << endl;
   //cout << "info: " << info << endl;
   return 0;
}