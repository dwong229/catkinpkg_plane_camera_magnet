#include <Eigen/Dense>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <typeinfo>
#include <iostream>
#include <unsupported/Eigen/NonLinearOptimization>

using Eigen::MatrixXd;

using namespace Eigen;
using namespace std;

// define constants
const double pi = 3.1415926535897;
const double mu0 = 4*pi*pow(10,-7);

// functiosn to help compute Matrices D,B,M:
extern Eigen::MatrixXd Mx(double,double,double,double);
extern Eigen::MatrixXd My(double,double,double,double);
extern Eigen::MatrixXd computeBmat(double, double, double, double);
extern Eigen::MatrixXd Dy(double, double, double, double);
extern Eigen::MatrixXd Dx(double, double, double, double);

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

  // for 2d case
  double x2;
  double y2;
  double Fx2;
  double Fy2;
  MatrixXd Mxmat2;
  MatrixXd Mymat2;
  MatrixXd Bmat;
  MatrixXd Bmat2;
  MatrixXd Dxmat;
  MatrixXd Dymat;
  MatrixXd Dxmat2;
  MatrixXd Dymat2;
};

//Generic Functor
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

//delecation of struct name only (not constructor)
struct CoilFunctor : Functor<double>
{
    CoilFunctor(Coil coil, Magnet magnet); //constructor for CoilFunctor
    
    // declarations:
    double d, R, x, y, Fx, Fy, gamma;
    MatrixXd Mxmat, Mymat, Dxmat, Dymat, Bmat; 
    Vector2d Bearth;

    int operator()(const VectorXd&, VectorXd&);
    int df(const VectorXd&, MatrixXd&);

};

//struct CoilFunctor2 : CoilFunctor {
struct CoilFunctor2 : Functor<double>
{
  CoilFunctor2(Coil c, Magnet m); //: CoilFunctor(c, m) {}

  double d, R, gamma;
  double Fx, Fy, x, y, x2, y2, Fx2, Fy2;
  MatrixXd Mxmat, Mymat, Mxmat2, Mymat2, Bmat, Bmat2, Dxmat, Dymat, Dxmat2, Dymat2; 
  Vector2d Bearth;

  int operator()(const VectorXd&, VectorXd&);
  int df(const VectorXd&, MatrixXd&);

};