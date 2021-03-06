#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include "currentcompute.h"
#include <fstream>
#include <timers.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "lm_trial");
   ros::start();
   // 2 magnet       
   const int n =4; // 4I 
   int info;
   VectorXd b(n);
   VectorXd binit(n);
      
   Magnet magnet2;
   Coil coil;
   coil.d = 57.5;
   coil.R = 8900;
   magnet2.gamma = 6500;
   // xnow
   magnet2.x = -10.0;
   magnet2.y = 0.0;
   magnet2.x2 = 3.0;
   magnet2.y2 = 0.0;
   //fdes
   magnet2.Fx = -3.0;//0.0283;
   magnet2.Fy = -0.0;
   magnet2.Fx2 = -1.0;//0.0283;
   magnet2.Fy2 = -0.0;
   
   magnet2.Mxmat = Mx(magnet2.x,magnet2.y,coil.R,coil.d);
   magnet2.Mymat = My(magnet2.x,magnet2.y,coil.R,coil.d);

   magnet2.Mxmat2 = Mx(magnet2.x2,magnet2.y2,coil.R,coil.d);
   magnet2.Mymat2 = My(magnet2.x2,magnet2.y2,coil.R,coil.d);

   magnet2.Bmat  = computeBmat(magnet2.x,magnet2.y,coil.R,coil.d);
   magnet2.Bmat2 = computeBmat(magnet2.x2,magnet2.y2,coil.R,coil.d);

   magnet2.Dxmat = Dx(magnet2.x,magnet2.y,coil.R,coil.d);
   magnet2.Dymat = Dy(magnet2.x,magnet2.y,coil.R,coil.d);
   magnet2.Dxmat2 = Dx(magnet2.x2,magnet2.y2,coil.R,coil.d);
   magnet2.Dymat2 = Dy(magnet2.x2,magnet2.y2,coil.R,coil.d);

   CoilFunctor2 functor(coil, magnet2); // functor( ) add arguments here.
   //CoilFunctor2 functor2(coil, magnet1);

   // {
   // ScopedTime("this")
   // int a = 3;
   // a *=3;
   // }
   Timer tm;
   tm.tic();

   LevenbergMarquardt<CoilFunctor2> lm(functor);

   int currmin = -500;
   int currmax = 500;
   int db = 200;
   b << currmin, currmin, currmin, currmin;
   binit = b;

   ofstream datafile;
   datafile.open("datafile.csv");
   //first line: x,y,x2,y2,Fx,Fy,Fx2,Fy2
   datafile << "x,y,x2,y2,Fx,Fy,Fx2,Fy2"<< endl;
   datafile << "binit(4), info(1), current(4), error(4)" << endl;
   datafile << magnet2.x << "," << magnet2.y << "," << magnet2.x2 << "," << magnet2.y2 << "," ;
   datafile << magnet2.Fx << "," << magnet2.Fy << "," << magnet2.Fx2 << "," << magnet2.Fy2 << endl;
   while(binit[0] < currmax)
   {
      while(binit[1] < currmax)
      {
         while(binit[2] < currmax)
         {
            while(binit[3] < currmax)
            {
               b = binit;
               //cout << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
               datafile << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << "," ;        
            
               ros::Time begin = ros::Time::now(); //begin time
               info = lm.minimize(b);
               ros::Time endtime = ros::Time::now();
               double dt = (endtime - begin).toSec(); 
               //cout << "Time to compute: " << dt << "secs" << endl; 
               //HybridNonLinearSolver<CoilFunctor> solver(functor);
               //info = solver.solve(b);
               //info = solver.hybrd1(b);

               datafile << info << ",";

               //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
               VectorXd current(4);
               current = b;
               //datafile << b.transpose() << ",";
               datafile << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << "," ;

               VectorXd error(4);
               functor.operator()(b,error);
               //datafile << error.transpose() << endl;
               datafile << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << dt << endl ;
               //vector<double> errorvec;
               vector<double> errorvec(error.data(),error.data() + error.rows() * error.cols());
               binit[3] += db;
               // write file:
            }
            binit[2] += db;
            binit[3] = currmin;
         }

         binit[1] += db;
         binit[2] = currmin;
         binit[3] = currmin;
      }
      binit[0] += db;
      binit[1] = currmin;
      binit[2] = currmin;
      binit[3] = currmin;
   }

   ROS_INFO_STREAM("Total time: " << tm.toc() );
   //timing
   

   datafile.close();

   return 0;
}
