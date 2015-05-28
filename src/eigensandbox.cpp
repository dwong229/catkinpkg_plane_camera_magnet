#include <iostream>
#include <Eigen/Dense>
#include <math.h>

using Eigen::MatrixXd;

using namespace Eigen;
using namespace std;

const double pi = 3.1415926535897;
const double mu0 = 4*pi*pow(10,-7);
const double d = 5;
const double R = 4;
//Utility function to compute B - point dipole model
VectorXd computeB(double x,double y, double R) {
	VectorXd B(2);
	//verified with MATLAB soln
	B(0) = mu0*pow(R,2)/(4*pow(pow(y,2) + pow(x,2),1.5))*(2-3*pow(y,2)/(pow(x,2) + pow(y,2)));
	B(1) = mu0*3*pow(R,2)*x*y/(4*pow(pow(y,2) + pow(x,2),2.5));
	return B;
}

double Bx(double x, double y, double R){
	//verified with MATLAB soln
	double Bx = mu0*pow(R,2)/(4*pow(pow(y,2) + pow(x,2),1.5))*(2-3*pow(y,2)/(pow(x,2) + pow(y,2)));
	return Bx;
}

double By(double x, double y, double R){
	//verified with MATLAB soln
	double By = mu0*3*pow(R,2)*x*y/(4*pow(pow(y,2) + pow(x,2),2.5));
	return By;
}

MatrixXd computeBmat(double x,double y, double R, double d){
	//verified with MATLAB soln
	MatrixXd Bmat(2,4);
	Bmat(0,0) = Bx(x+d,y,R);
	Bmat(1,0) = By(x+d,y,R);
	Bmat(0,1) = -By(y-d,-x,R);
	Bmat(1,1) = Bx(y-d,-x,R);
	Bmat(0,2) = Bx(x-d,y,R);
	Bmat(1,2) = By(x-d,y,R);  
	Bmat(0,3) = -By(y+d,-x,R);
	Bmat(1,3) = Bx(y+d,-x,R);
	return Bmat;
}

/// ~~~~~~~ D ~~~~~~~~~~~~~///
// Define D matrix
double Dx11(double x, double y){
	double Dx = 3* pow(R,2) * (d+x) * pow(y,2) * mu0/(2*pow(pow(d+x,2)+pow(y,2),3.5)) - 3*pow(R,2) * (d+x)*(2-(3*pow(y,2))/(pow(d+x,2)+pow(y,2)))*mu0/(4*pow((pow(d+x,2) + pow(y,2)),2.5));  
	return Dx;
}

double Dx12(double x, double y){
	double Dx = -15*pow(R,2)*pow(x,2)*(-d+y)*mu0/(4*pow((pow(x,2)+pow(-d+y,2)),3.5)) + 3*pow(R,2)*(-d+y)*mu0/(4*pow((pow(x,2) + pow(-d+y,2)),2.5));
	return Dx; 
}

double Dx13(double x, double y){
	double Dx = 3*pow(R,2) *(-d+x)*pow(y,2)*mu0/(2*pow((pow(-d+x,2) +pow(y,2)),3.5)) - 3*pow(R,2)*(-d+x)*(2 - 3*pow(y,2)/(pow(-d+x,2) +pow(y,2)))* mu0/(4*pow((pow(-d+x,2) +pow(y,2) ),2.5));
	return Dx;
}

double Dx14(double x, double y){
	double Dx = -15*pow(R,2)*pow(x,2)*(d+y)*mu0/(4*pow((pow(x,2)+pow(d+y,2)),3.5)) + 3*pow(R,2)*(d+y)*mu0/(4*pow((pow(x,2) + pow(d+y,2)),2.5));
	return Dx;
}

double Dx21(double x, double y){
	double Dx = -15*pow(R,2)*pow(d+x,2)*y*mu0/(4*pow((pow(d+x,2)+pow(y,2)),3.5)) + 3*pow(R,2)*y*mu0/(4*pow((pow(d+x,2)+pow(y,2)),2.5));
	return Dx;
}

double Dx22(double x, double y){
	double Dx =mu0*pow(R,2) * (6*pow(x,3)/pow((pow(x,2)+pow(-d+y,2)),2)-6*x/(pow(x,2)+pow(-d+y,2)))/(4*pow(pow(x,2) + pow(y-d,2),1.5)) - (3*pow(R,2) * x * (2- 3*pow(x,2)/(pow(x,2) + pow(-d+y,2)))*mu0)/(4*pow((pow(x,2)+pow(-d+y,2)),2.5));
	return Dx;
}

double Dx23(double x, double y){
	double Dx =-15*pow(R,2)*pow(-d+x,2)*y*mu0/(4*pow((pow(-d+x,2)+pow(y,2)),3.5)) + 3*pow(R,2)*y*mu0/(4*pow((pow(-d+x,2)+pow(y,2)),2.5));
	return Dx;
}

double Dx24(double x, double y){
	double Dx =mu0*pow(R,2) * (6*pow(x,3)/pow((pow(x,2)+pow(d+y,2)),2)-6*x/(pow(x,2)+pow(d+y,2)))/(4*pow((pow(x,2) + pow(y+d,2)),1.5)) - (3*pow(R,2) * x * (2- 3*pow(x,2)/(pow(x,2) + pow(d+y,2)))*mu0)/(4*pow((pow(x,2)+pow(d+y,2)),2.5));
	return Dx;
}

MatrixXd Dx(double x, double y){
	// Verified in Matlab
	MatrixXd Dmat(2,4);
	Dmat << Dx11(x,y), Dx12(x,y), Dx13(x,y), Dx14(x,y),
  		Dx21(x,y), Dx22(x,y), Dx23(x,y), Dx24(x,y);
  	return Dmat;
}


double Dy11(double x, double y){
    double Dx = mu0*pow(R,2) * (6*pow(y,3)/pow((pow(x+d,2)+pow(y,2)),2)-6*y/(pow(x+d,2)+pow(y,2)))/(4*pow((pow(x+d,2) + pow(y,2)),1.5)) - (3*pow(R,2) * y * (2- 3*pow(y,2)/(pow(d+x,2) + pow(y,2)))*mu0)/(4*pow((pow(d+x,2)+pow(y,2)),2.5));
    return Dx;
}

double Dy12(double x, double y){
    double Dx = -15*pow(R,2)*pow(-d+y,2)*x*mu0/(4*pow((pow(-d+y,2)+pow(x,2)),3.5)) + 3*pow(R,2)*x*mu0/(4*pow((pow(x,2)+pow(y-d,2)),2.5));
    return Dx; 
}

double Dy13(double x, double y){
    double Dx = mu0*pow(R,2) * (6*pow(y,3)/pow((pow(x-d,2)+pow(y,2)),2)-6*y/(pow(x-d,2)+pow(y,2)))/(4*pow(pow(x-d,2) + pow(y,2),1.5)) - (3*pow(R,2) * y * (2- 3*pow(y,2)/(pow(-d+x,2) + pow(y,2)))*mu0)/(4*pow((pow(-d+x,2)+pow(y,2)),2.5));
    return Dx;
}

double Dy14(double x, double y){
    double Dx = -15*pow(R,2)*pow(d+y,2)*x*mu0/(4*pow((pow(d+y,2)+pow(x,2)),3.5)) + 3*pow(R,2)*x*mu0/(4*pow((pow(x,2)+pow(y+d,2)),2.5));
    return Dx;
}

double Dy21(double x, double y){
    double Dx = -15*pow(R,2)*(d+x)*pow(y,2)*mu0/(4*pow((pow(d+x,2)+pow(y,2)),3.5)) + 3*pow(R,2)*(x+d)*mu0/(4*pow((pow(d+x,2)+pow(y,2)),2.5));
    return Dx;
}

double Dy22(double x, double y){
    double Dx = mu0*pow(R,2) * pow(x,2)*(-d+y)/(2*pow((pow(x,2)+pow(-d+y,2)),3.5)) - (3*pow(R,2)*(y-d)*(2-3*pow(x,2)/(pow(x,2)+pow(-d+y,2)))*mu0)/(4*pow((pow(x,2)+pow(y-d,2)),2.5));
    return Dx;
}

double Dy23(double x, double y){
    double Dx = -15*pow(R,2)*(-d+x)*pow(y,2)*mu0/(4*pow((pow(-d+x,2)+pow(y,2)),3.5)) + 3*pow(R,2)*(x-d)*mu0/(4*pow((pow(-d+x,2)+pow(y,2)),2.5));
    return Dx;
}

double Dy24(double x, double y){
    double Dx = mu0*pow(R,2) * pow(x,2)*(d+y)/(2*pow((pow(x,2)+pow(d+y,2)),3.5)) - (3*pow(R,2)*(y+d)*(2-3*pow(x,2)/(pow(x,2)+pow(d+y,2)))*mu0)/(4*pow((pow(x,2)+pow(y+d,2)),2.5));
    return Dx;
}

MatrixXd Dy(double x, double y){
    // Verified in Matlab
    MatrixXd Dmat(2,4);
    Dmat << Dy11(x,y), Dy12(x,y), Dy13(x,y), Dy14(x,y),
        Dy21(x,y), Dy22(x,y), Dy23(x,y), Dy24(x,y);
    return Dmat;
}


int main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  
  double x = 1;
  double y = 2;
  MatrixXd Dmat = Dx(x,y);
  cout << "Dx= " << Dmat << endl;

  Dmat = Dy(x,y);
  cout << "Dy= " << Dmat << endl;  

}