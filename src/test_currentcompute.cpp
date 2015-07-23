#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include "currentcompute.h"

int main(int argc, char **argv)
{
   //ros::init(argc, argv ,"current");
   //ros::start();
   const int n =6; // 4I , 2 lambda
   int info;
   VectorXd b(n);
   //b << 0.004 * pow(10,-3),-0.5 * pow(10,-3),-0.1 * pow(10,-3) ,-0.03 * pow(10,-3),0.5,0.5;
   b << .2, -2.7, 16, 2.7, 10700, 0.; // F = [0.0283, 0]
   //b << .4,23,-16,-5.0,-500,-30000;
   
   Magnet magnet1;
   Coil coil;
   coil.d = 57.5;
   coil.R = 8900;
   magnet1.x = 10.0;
   magnet1.y = 0.0;
   magnet1.Fx = 0.0283;
   magnet1.Fy = 0.0283;
   magnet1.gamma = 6500;
   magnet1.Mxmat = Mx(magnet1.x,magnet1.y,coil.R,coil.d);
   magnet1.Mymat = My(magnet1.x,magnet1.y,coil.R,coil.d);

   //ros::Time begin = ros::Time::now(); //begin time

   CoilFunctor functor(coil, magnet1); // functor( ) add arguments here.
   LevenbergMarquardt<CoilFunctor> lm(functor);
   info = lm.minimize(b);
   //HybridNonLinearSolver<CoilFunctor> solver(functor);
   //info = solver.solve(b);
   //info = solver.hybrd1(b);
   
  // check return value
   cout << "info: " << info << endl;

   //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
   VectorXd current(4);
   VectorXd c = b.head(4);
   MatrixXd Bmat = computeBmat(magnet1.x,magnet1.y,coil.R,coil.d);
   current = c * pow(c.transpose()*Bmat.transpose()*Bmat*c,0.5);
   cout << "current: \n " << current << endl;

   /* //timing
   ros::Time endtime = ros::Time::now();
   double dt = (endtime - begin).toSec(); 
   //cout << "Time to compute: " << dt << "secs" << endl; //timing */
   return 0;
}