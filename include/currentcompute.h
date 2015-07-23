#include <Eigen/Dense>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <typeinfo>
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
    MatrixXd Mxmat, Mymat; 

    int operator()(const VectorXd&, VectorXd&);
    int df(const VectorXd&, MatrixXd&);

};