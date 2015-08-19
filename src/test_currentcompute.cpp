#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include "currentcompute.h"

int main(int argc, char **argv)
{
   //One Magnet
   if(1){
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
   magnet1.Fx = 5.;//0.0283;
   magnet1.Fy = 0.0;
   magnet1.gamma = 6500;

   magnet1.Mxmat = Mx(magnet1.x,magnet1.y,coil.R,coil.d);
   //magnet1.Mxmat = Mx(10.0,0.0,coil.R,coil.d);
   magnet1.Mymat = My(magnet1.x,magnet1.y,coil.R,coil.d);

   cout << "magnet1.Mxmat : " << endl << magnet1.Mxmat << endl;
   //ros::Time begin = ros::Time::now(); //begin time

   CoilFunctor functor(coil, magnet1); // functor( ) add arguments here.
   //CoilFunctor2 functor2(coil, magnet1);

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
   }

   if(0)
   {
   // 2 magnet 
   const int n =4; // 4I 
   int info;
   VectorXd b(n);

   //b << -10,20,10,10.0; // x: [10 10 0 0], F = [-20 20 0 0] matches matlab

   b << 100, 1, 100, 0;    
   Magnet magnet2;
   Coil coil;
   coil.d = 57.5;
   coil.R = 8900;
   magnet2.gamma = 6500;
   // xnow
   magnet2.x = -10.0;
   magnet2.y = 0.0;
   magnet2.x2 = 10.0;
   magnet2.y2 = 0.0;
   //fdes
   magnet2.Fx = 1.0;//0.0283;
   magnet2.Fy = 0.0;
   magnet2.Fx2 = -1.0;//0.0283;
   magnet2.Fy2 = 0.0;
   
   magnet2.Mxmat = Mx(magnet2.x,magnet2.y,coil.R,coil.d);
   magnet2.Mymat = My(magnet2.x,magnet2.y,coil.R,coil.d);

   magnet2.Mxmat2 = Mx(magnet2.x2,magnet2.y2,coil.R,coil.d);
   magnet2.Mymat2 = My(magnet2.x2,magnet2.y2,coil.R,coil.d);

   magnet2.Bmat  = computeBmat(magnet2.x,magnet2.y,coil.R,coil.d);
   magnet2.Bmat2 = computeBmat(magnet2.x2,magnet2.y2,coil.R,coil.d);

   CoilFunctor2 functor(coil, magnet2); // functor( ) add arguments here.
   //CoilFunctor2 functor2(coil, magnet1);

   LevenbergMarquardt<CoilFunctor2> lm(functor);
   info = lm.minimize(b);
   //HybridNonLinearSolver<CoilFunctor> solver(functor);
   //info = solver.solve(b);
   //info = solver.hybrd1(b);
   
  // check return value
   cout << " 2 magnet2 " << endl;
   cout << "info: " << info << endl;

   //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
   VectorXd current(4);
   current = b;
   cout << "current: \n " << current << endl;

   VectorXd error(4);
   functor.operator()(b,error);
   cout << "error: " << error.transpose() << endl;

   //vector<double> errorvec;
   vector<double> errorvec(error.data(),error.data() + error.rows() * error.cols());
   cout << errorvec.at(1) << endl;
}
   return 0;
}