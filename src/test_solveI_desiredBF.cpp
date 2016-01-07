#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <typeinfo>
#include <Eigen/LU>
//#include <unsupported/Eigen/NonLinearOptimization>
#include "currentcompute.h"

// For directed assembly project
// Compute current based on desired BF
#define PI 3.14159265

int main(void)
{
   const int n =6; // 4I , 2 lambda
   int info;

   // system params
   Coil coil;
   Magnet magnet;
   coil.d = .0575;
   coil.R = 0.075;

   magnet.x =  0.01;
   magnet.y =  0.0;
   magnet.Fx = 1 * pow(10,-8);
   magnet.Fy = 1.* pow(10,-8);
   magnet.gamma = 35.8 * pow(10,-6);
   double th = -19 * PI/180; // convert angle in degrees to radians

   double Bmag = 10. * pow(10,-6);
   double Bx = Bmag * cos(th);
   double By = Bmag * sin(th);

   Vector2d mvec(Bx,By);
   mvec = magnet.gamma/sqrt(Bx * Bx + By * By)*mvec;

   cout << "mvec" << endl << mvec << endl;

   MatrixXd Bmat = computeBmat(magnet.x,magnet.y,coil.R,coil.d);
   cout << "Bmat" << endl << Bmat << endl;
   //MatrixXd 

   Matrix4d A;
   cout << "A" << endl << A << endl;

   A.block(0,0,2,4) = Bmat;
   cout << "A row 1-2" << endl << A << endl;
   A.block(2,0,1,4) = mvec.transpose() * Dx(magnet.x,magnet.y,coil.R,coil.d);
   A.block(3,0,1,4) = mvec.transpose() * Dy(magnet.x,magnet.y,coil.R,coil.d);
   cout << "A " << endl << A << endl;

   cout << "A inverse is:" << endl << A.inverse() << endl;

   Vector4d BF;
   BF << Bx,By,magnet.Fx,magnet.Fy;
   cout << "BF: " << endl << BF << endl;
   Vector4d Isolve = A.inverse() * BF;

   cout << "Isolve: " << endl << Isolve << endl;
   return 0;
}