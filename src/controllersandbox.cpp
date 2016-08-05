// Sandbox for testing no-skid controller
#include <stdio.h>

#include "currentcompute.h" // contains struct : Coil, Magnet

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
   fd << 0.4,0.9;

   p = p*pow(10,-6);

   // setup parameters:
   Coil coil;

   coil.R = 8900;
   coil.d = 57.5;

   Magnet magnet;
   magnet.gamma = 6500;
   magnet.x = p[0];
   magnet.y = p[1]; 

   VectorXd m(2); //magnetic moment
   m = -gamma * fd;

   //compute B-matrix and Dx and Dy for position, p:
   

   // Compute m * Dx, m * Dy

   cout << "position: " << p[0] << " , " << p[1] << " m." << endl;



   
   //

   //cout << "current: " << x[0] << ", " << x[1] << endl;
   //cout << "info: " << info << endl;
   return 0;
}