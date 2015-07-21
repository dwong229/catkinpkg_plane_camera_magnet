#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <typeinfo>
#include <unsupported/Eigen/NonLinearOptimization>
#include "currentcompute.h"

struct Coil{
  double R;
  double d;
};

struct Magnet{
  double x;
  double y;
  double Fx;
  double Fy;
  double gamma;
  MatrixXd Mxmat;
  MatrixXd Mymat;
};

//Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

  // you should define that in the subclass :
//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};


struct toy_functor : Functor<double> //inheritence
{
    toy_functor(Coil coil, Magnet magnet) : Functor<double>(6,6) {
        d =coil.d;
        R = coil.R;
        x = magnet.x;
        y = magnet.y;
        Fx = magnet.Fx;
        Fy = magnet.Fy; 
        gamma = magnet.gamma;
        Mxmat = magnet.Mxmat;
        Mymat = magnet.Mymat;
      }
      double d, R, x, y, Fx, Fy, gamma;
      MatrixXd Mxmat, Mymat; 

    int operator()(const VectorXd &b, VectorXd &fvec)
    {
        assert(b.size()==6);
        assert(fvec.size()==6);

        //cout << "Mxmat = \n" << Mxmat << endl;
        //cout << "Mymat = \n" << Mymat << endl;

        VectorXd c; // current components
        VectorXd lambda;
        c = b.head(4);
        lambda = b.tail(2);

        // force constraints
        fvec[0] = gamma * c.transpose() * Mxmat * c - Fx;
        fvec[1] = gamma * c.transpose() * Mymat * c - Fy;

        // current minimizing
        fvec[2] = c[0] + gamma * lambda[0] * Mxmat.row(0)*c + gamma * lambda[1] * Mymat.row(0)*c; 
        fvec[3] = c[1] + gamma * lambda[0] * Mxmat.row(1)*c + gamma * lambda[1] * Mymat.row(1)*c;
        fvec[4] = c[2] + gamma * lambda[0] * Mxmat.row(2)*c + gamma * lambda[1] * Mymat.row(2)*c;
        fvec[5] = c[3] + gamma * lambda[0] * Mxmat.row(3)*c + gamma * lambda[1] * Mymat.row(3)*c;
        
        //cout << "operator: " << fvec.transpose() << endl;
        //cout << "b: " << b.transpose() << endl;
        return 0;
    }
    int df(const VectorXd &b, MatrixXd &fjac)
    {
        assert(b.size()==6);
        assert(fjac.rows()==6);
        assert(fjac.cols()==6);

        VectorXd c; // current components
        VectorXd lambda;
        c = b.head(4);
        lambda = b.tail(2);

        //fjac(equation#, derivative wrt b[i])
        // Force equation derivatives 
        for(int i=0; i<4; i++){
            fjac(0,i) = Mxmat.col(i).transpose() * c; 
            fjac(0,i) += Mxmat.row(i) * c;
            fjac(0,i) *= gamma;

            fjac(1,i) = Mymat.col(i).transpose() * c;
            fjac(1,i) += Mymat.row(i) * c; 
            fjac(1,i) *= gamma;
        }
              

        fjac(0,4) = 0.0;
        fjac(0,5) = 0.0;
        
        // Fy
        fjac(1,4) = 0.0;
        fjac(1,5) = 0.0;
        
        // constraint 1
        for(int eq=2; eq<6; eq++){
          for(int curr = 0; curr < 4; curr++){
            fjac(eq,curr) = gamma*lambda[0]*Mxmat(eq-2,curr) + gamma * lambda[1] * Mymat(eq-2,curr);
            if(eq-2 == curr){
              fjac(eq,curr) += 1.0;
            }
          }
          fjac(eq,4) = gamma * Mxmat.row(eq-2) * c;
          fjac(eq,5) = gamma * Mymat.row(eq-2) * c;
        }
        //cout << "df: " << endl;
        //cout << fjac << endl;

        return 0;
    }
};

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

   toy_functor functor(coil, magnet1); // functor( ) add arguments here.
   LevenbergMarquardt<toy_functor> lm(functor);
   info = lm.minimize(b);
   //HybridNonLinearSolver<toy_functor> solver(functor);
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