#include <stdio.h>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <typeinfo>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

using Eigen::MatrixXd;
using namespace Eigen;
using namespace std;
using std::sqrt;

//Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
//template<typename _Scalar, int NX, int NY>
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
    toy_functor(void) : Functor<double>(2,2) {}
    static const double m_x[2];
    static const double m_y[2];
    int operator()(const VectorXd &b, VectorXd &fvec)
    {
        assert(b.size()==2);
        assert(fvec.size()==2);
        //for(int i=0; i<14; i++) {
         //   fvec[i] = b[0]*(1.-exp(-b[1]*m_x[i])) - m_y[i] ;
        //}
        fvec[0] = b[0]*b[0] + 5*b[0]*b[1] + 4*b[1]*b[1] -5.0;
        fvec[1] = 4*b[0]*b[0] - 2*b[0]*b[1] + 4*b[1]*b[1] -3.0;
        cout << "operator: " << fvec[0] << ", " << fvec[1] << endl;
        cout << "b: " << b[0] << ", " << b[1] << endl;
        return 0;
    }
    int df(const VectorXd &b, MatrixXd &fjac)
    {
        assert(b.size()==2);
        assert(fjac.rows()==2);
        assert(fjac.cols()==2);
        //for(int i=0; i<14; i++) {
         //   fjac(i,0) = (1.-exp(-b[1]*m_x[i]));
         //   fjac(i,1) = (b[0]*m_x[i]*exp(-b[1]*m_x[i]));
        //}
        //fjac(equation#, derivative wrt b[i])
        fjac(0,0) = 2*b[0] + 5 * b[1];
        fjac(0,1) = 5*b[0] + 8 *b[1];
        fjac(1,0) = 8*b[0] - 2*b[1];
        fjac(1,1) = -2*b[0] + 8*b[1];
        cout << "df: " << fjac(0,0) << ", " << fjac(0,1) << endl;
        return 0;
    }
};

int main(void)
{
   const int n =2;
   int info;

   VectorXd x(n);
   x << 0.4,0.9;
   toy_functor functor;
   LevenbergMarquardt<toy_functor> lm(functor);
   info = lm.minimize(x);

   cout << "soln: " << x[0] << ", " << x[1] << endl;
   return 0;
}

/* Example from forum
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Mat;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vec;

class Functor {
public: 
    typedef double Scalar;
    enum {
        InputsAtCompileTime = 3,
        ValuesAtCompileTime = Eigen::Dynamic
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    Functor() 
        : m_inputs(InputsAtCompileTime)
        , m_values(10) 
        , xTruth(m_inputs)
        , vTruth(m_values)
    {
        xTruth << 1, 2, -3; // quadradic.
        op(xTruth, vTruth);
    }

    int inputs() { return m_inputs; }
    int values() { return m_values; }
    
    // Evaluate a polynomial over t (range 0 to 1)
    // y = x(0) + x(1) * t + x(2) * t^2
    void op(Vec const & x, Vec & value) const {
        value.resize(m_values);
        for (int i = 0; i < m_values; ++i) {
            Scalar t = static_cast<Scalar>(i) / (m_values - 1);
            Scalar tPower = t;
            Scalar v = x(0); // offset
            for (int k = 1; k < m_inputs; ++k) {
                v += x(k) * tPower;
                tPower *= t;
            }
            value(i) = v;
        }
    }

    int operator()(Vec const & x, Vec & fvec) const {
        op(x, fvec);
        fvec -= vTruth;
        return 0;
    }

private:
    int m_inputs;
    int m_values;
    Vec xTruth;
    Vec vTruth;
};

int main(int argc, char* argv[]) {
    Vec x = Vec::Zero(3);
    Functor functor;
    Eigen::HybridNonLinearSolver<Functor> solver(functor);
    int info = solver.hybrd1(x);
}
/*#include <boost/math/tools/roots.hpp>

using namespace boost;
using namespace math;
using namespace tools;
using namespace std;

typedef double T;

template <class F, class T>
T newton_raphson_iterate(F f, T guess, T min, T max, int digits);

template <class F, class T>
T newton_raphson_iterate(F f, T guess, T min, T max, int digits, boost::uintmax_t& max_iter);

template <class T>
struct cbrt_functor
{
   cbrt_functor(T const& target) : a(target)
   { // Constructor stores value to be 'cube-rooted'.
   }
   boost::math::tuple<T, T> operator()(T const& z)
   { // z is estimate so far.
      return boost::math::make_tuple(
      z*z*z - a, // return both f(x)
      3 * z*z);  // and f'(x)
   }
private:
   T a; // to be 'cube-rooted'.
};

template <class T>
T cbrt(T z)
{
   using namespace std; // for frexp, ldexp, numeric_limits.
   using namespace boost::math::tools;

   int exp;
   frexp(z, &exp); // Get exponent of z (ignore mantissa).
   T min = ldexp(0.5, exp/3);
   T max = ldexp(2.0, exp/3);
   T guess = ldexp(1.0, exp/3); // Rough guess is to divide the exponent by three.
   int digits = std::numeric_limits<T>::digits; // Maximum possible binary digits accuracy for type T.
   return newton_raphson_iterate(cbrt_functor<T>(z), guess, min, max, digits);
};

int main()
{

   T z0 = 0.01;
   T soln;
   soln = cbrt(z0);
   cout << soln << endl;
   return 0;
}

*/


/* Boost example/newton-raphson.cpp
 * Newton iteration for intervals
 *
 * Copyright 2003 Guillaume Melquiond
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or
 * copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/*  //From : http://www.boost.org/doc/libs/1_41_0/libs/numeric/interval/examples/newton-raphson.cpp
#include <boost/numeric/interval.hpp>
#include <vector>
#include <algorithm>
#include <utility>
#include <iostream>
#include <iomanip>

template <class I> I f(const I& x)
{ return x * (x - 1.) * (x - 2.) * (x - 3.) * (x - 4.); }
template <class I> I f_diff(const I& x)
{ return (((5. * x - 40.) * x + 105.) * x - 100.) * x + 24.; }

static const double max_width = 1e-10;
static const double alpha = 0.75;

using namespace boost;
using namespace numeric;
using namespace interval_lib;

// First method: no empty intervals

typedef interval<double> I1_aux;
typedef unprotect<I1_aux>::type I1;

std::vector<I1> newton_raphson(const I1& xs) {
  std::vector<I1> l, res;
  I1 vf, vd, x, x1, x2;
  l.push_back(xs);
  while (!l.empty()) {
    x = l.back();
    l.pop_back();
    bool x2_used;
    double xx = median(x);
    vf = f<I1>(xx);
    vd = f_diff<I1>(x);
    if (zero_in(vf) && zero_in(vd)) {
      x1 = I1::whole();
      x2_used = false;
    } else {
      x1 = xx - division_part1(vf, vd, x2_used);
      if (x2_used) x2 = xx - division_part2(vf, vd);
    }
    if (overlap(x1, x)) x1 = intersect(x, x1);
    else if (x2_used) { x1 = x2; x2_used = false; }
    else continue;
    if (x2_used)
      if (overlap(x2, x)) x2 = intersect(x, x2);
      else x2_used = false;
    if (x2_used && width(x2) > width(x1)) std::swap(x1, x2);
    if (!zero_in(f(x1)))
      if (x2_used) { x1 = x2; x2_used = false; }
      else continue;
    if (width(x1) < max_width) res.push_back(x1);
    else if (width(x1) > alpha * width(x)) {
      std::pair<I1, I1> p = bisect(x);
      if (zero_in(f(p.first))) l.push_back(p.first);
      x2 = p.second;
      x2_used = true;
    } else l.push_back(x1);
    if (x2_used && zero_in(f(x2)))
      if (width(x2) < max_width) res.push_back(x2);
      else l.push_back(x2);
  }
  return res;
}

// Second method: with empty intervals

typedef change_checking<I1_aux, checking_no_nan<double> >::type I2_aux;
typedef unprotect<I2_aux>::type I2;

std::vector<I2> newton_raphson(const I2& xs) {
  std::vector<I2> l, res;
  I2 vf, vd, x, x1, x2;
  l.push_back(xs);
  while (!l.empty()) {
    x = l.back();
    l.pop_back();
    double xx = median(x);
    vf = f<I2>(xx);
    vd = f_diff<I2>(x);
    if (zero_in(vf) && zero_in(vd)) {
      x1 = x;
      x2 = I2::empty();
    } else {
      bool x2_used;
      x1 = intersect(x, xx - division_part1(vf, vd, x2_used));
      x2 = intersect(x, xx - division_part2(vf, vd, x2_used));
    }
    if (width(x2) > width(x1)) std::swap(x1, x2);
    if (empty(x1) || !zero_in(f(x1)))
      if (!empty(x2)) { x1 = x2; x2 = I2::empty(); }
      else continue;
    if (width(x1) < max_width) res.push_back(x1);
    else if (width(x1) > alpha * width(x)) {
      std::pair<I2, I2> p = bisect(x);
      if (zero_in(f(p.first))) l.push_back(p.first);
      x2 = p.second;
    } else l.push_back(x1);
    if (!empty(x2) && zero_in(f(x2)))
      if (width(x2) < max_width) res.push_back(x2);
      else l.push_back(x2);
  }
  return res;
}

template<class T, class Policies>
std::ostream &operator<<(std::ostream &os,
                         const boost::numeric::interval<T, Policies> &x) {
  os << "[" << x.lower() << ", " << x.upper() << "]";
  return os;
}

int main() {
  {
    I1_aux::traits_type::rounding rnd;
    std::vector<I1> res = newton_raphson(I1(-1, 5.1));
    std::cout << "Results: " << std::endl << std::setprecision(12);
    for(std::vector<I1>::const_iterator i = res.begin(); i != res.end(); ++i)
      std::cout << "  " << *i << std::endl;
    std::cout << std::endl;
  }
  {
    I2_aux::traits_type::rounding rnd;
    std::vector<I2> res = newton_raphson(I2(-1, 5.1));
    std::cout << "Results: " << std::endl << std::setprecision(12);
    for(std::vector<I2>::const_iterator i = res.begin(); i != res.end(); ++i)
      std::cout << "  " << *i << std::endl;
    std::cout << std::endl;
  }
}
*/ // works!

