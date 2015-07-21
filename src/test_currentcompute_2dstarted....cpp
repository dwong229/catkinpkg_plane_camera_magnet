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


struct toy_functor : Functor<double>
{
    toy_functor(void) : Functor<double>(6,6) {}
    int operator()(const VectorXd &b, VectorXd &fvec)
    {
        assert(b.size()==6);
        assert(fvec.size()==6);

        double d = .05;
        double R = .0286;
        double x = -.03;
        double y = 0.0;
        //double Fx = 0.7316 * pow(10,-7);
        //double Fy = 0.3658 * pow(10,-7);
        double Fx = 0.01;
        double Fy = 0.;
        double gamma = 35.8;

        MatrixXd Mxmat(4,4);
        MatrixXd Mymat(4,4);
        // compute m matrix:
        Mxmat = Mx(x,y,R,d);
        Mymat = My(x,y,R,d);

        //cout << "Mxmat = \n" << Mxmat << endl;
        //cout << "Mymat = \n" << Mymat << endl;

        VectorXd c; // current components
        VectorXd lambda;
        c = b.head(4);
        lambda = b.tail(2);

        // force constraints
        fvec[0] = c.transpose() * Mxmat * c - Fx;
        fvec[1] = c.transpose() * Mymat * c - Fy;

        // current minimizing
        fvec[2] = c[0] + gamma * lambda[0] * Mxmat.row(0)*c + gamma * lambda[1] * Mymat.row(0)*c; 
        fvec[3] = c[1] + gamma * lambda[0] * Mxmat.row(1)*c + gamma * lambda[1] * Mymat.row(1)*c;
        fvec[4] = c[2] + gamma * lambda[0] * Mxmat.row(2)*c + gamma * lambda[1] * Mymat.row(2)*c;
        fvec[5] = c[3] + gamma * lambda[0] * Mxmat.row(3)*c + gamma * lambda[1] * Mymat.row(3)*c;
        
        cout << "operator: \n" << fvec << endl;
        cout << "b:\n " << b << endl;
        return 0;
    }
    int df(const VectorXd &b, MatrixXd &fjac)
    {
        assert(b.size()==6);
        assert(fjac.rows()==6);
        assert(fjac.cols()==6);
        //fjac(equation#, derivative wrt b[i])
        double d = .05;
        double R = .0286;
        double x = -.03;
        double y = 0.0;
        //double Fx = 0.7316 * pow(10,-7);
        //double Fy = 0.3658 * pow(10,-7);
        double Fx = 0.01;
        double Fy = 0.;
        double gamma = 35.8;

        MatrixXd Mxmat(4,4);
        MatrixXd Mymat(4,4);

        Mxmat = Mx(x,y,R,d);
        Mymat = My(x,y,R,d);

        VectorXd c; // current components
        VectorXd lambda;
        c = b.head(4);
        lambda = b.tail(2);

        // Force equation derivatives 
        for(int i=0; i<4; i++){
            fjac(0,i) = Mxmat.col(i).transpose() * c; 
            fjac(0,i) += Mxmat.row(i) * c;
            fjac(0,i) *= gamma;

            fjac(1,i) = Mymat.col(i).transpose() * c;
            fjac(1,i) += Mymat.row(i) * c; 
            fjac(1,i) *= gamma;
        }
              

        fjac(0,4) = 0;
        fjac(0,5) = 0;
        
        // Fy
        fjac(1,4) = 0;
        fjac(1,5) = 0;
        
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


        cout << "df: " << endl;
        cout << fjac << endl;

        return 0;
    }
};

int main(void)
{
   const int n =6; // 4I , 2 lambda
   int info;

   VectorXd b(n);
   //b << 0.004 * pow(10,-3),-0.5 * pow(10,-3),-0.1 * pow(10,-3) ,-0.03 * pow(10,-3),0.5,0.5;
   b << 10, 10, 10, 10 , 0 ,0;
   toy_functor functor;
   LevenbergMarquardt<toy_functor> lm(functor);
   info = lm.minimize(b);

   double d = .05;
   double R = .0286;
   double x = -.03;
   double y = 0.0;
   double Fx = 0.01;
   double Fy = 0.;
   double gamma = 35.8;


   double x2 = 0.;
   double y2 = 0.;
   double Fx2 = 0.;
   double Fy2 = 0.;
   double gamma2 = 35.8;

   //double Fx = 0.7316 * pow(10,-7);
   //double Fy = 0.3658 * pow(10,-7);
   
   //cout << "soln: " << b[0] << ", " << b[1] << ", " << b[2] << ", " << b[3] << endl;
   VectorXd current(4);
   VectorXd c = b.head(4);
   MatrixXd Bmat = computeBmat(x,y,R,d);
   current = c * pow(c.transpose()*Bmat.transpose()*Bmat*c,0.5);
   cout << "current: \n " << current << endl;
   return 0;
}