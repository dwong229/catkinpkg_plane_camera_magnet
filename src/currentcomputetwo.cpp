// 2magnets. Jacobian approximation
// 
#include "currentcompute.h"
#include <math.h>

using Eigen::MatrixXd;

using namespace Eigen;
using namespace std;

//definitions:
CoilFunctor::CoilFunctor(Coil coil, Magnet magnet) : Functor<double> {
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

int CoilFunctor::operator()(const VectorXd &b, VectorXd &fvec)
    {
        //int sizeb;
        //sizeb = b.size();

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
int CoilFunctor::df(const VectorXd &b, MatrixXd &fjac)
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
double Dx11(double x, double y, double R, double d){
	double Dx = 3* pow(R,2) * (d+x) * pow(y,2) * mu0/(2*pow(pow(d+x,2)+pow(y,2),3.5)) - 3*pow(R,2) * (d+x)*(2-(3*pow(y,2))/(pow(d+x,2)+pow(y,2)))*mu0/(4*pow((pow(d+x,2) + pow(y,2)),2.5));  
	return Dx;
}

double Dx12(double x, double y, double R, double d){
	double Dx = -15*pow(R,2)*pow(x,2)*(-d+y)*mu0/(4*pow((pow(x,2)+pow(-d+y,2)),3.5)) + 3*pow(R,2)*(-d+y)*mu0/(4*pow((pow(x,2) + pow(-d+y,2)),2.5));
	return Dx; 
}

double Dx13(double x, double y, double R, double d){
	double Dx = 3*pow(R,2) *(-d+x)*pow(y,2)*mu0/(2*pow((pow(-d+x,2) +pow(y,2)),3.5)) - 3*pow(R,2)*(-d+x)*(2 - 3*pow(y,2)/(pow(-d+x,2) +pow(y,2)))* mu0/(4*pow((pow(-d+x,2) +pow(y,2) ),2.5));
	return Dx;
}

double Dx14(double x, double y, double R, double d){
	double Dx = -15*pow(R,2)*pow(x,2)*(d+y)*mu0/(4*pow((pow(x,2)+pow(d+y,2)),3.5)) + 3*pow(R,2)*(d+y)*mu0/(4*pow((pow(x,2) + pow(d+y,2)),2.5));
	return Dx;
}

double Dx21(double x, double y, double R, double d){
	double Dx = -15*pow(R,2)*pow(d+x,2)*y*mu0/(4*pow((pow(d+x,2)+pow(y,2)),3.5)) + 3*pow(R,2)*y*mu0/(4*pow((pow(d+x,2)+pow(y,2)),2.5));
	return Dx;
}

double Dx22(double x, double y, double R, double d){
	double Dx =mu0*pow(R,2) * (6*pow(x,3)/pow((pow(x,2)+pow(-d+y,2)),2)-6*x/(pow(x,2)+pow(-d+y,2)))/(4*pow(pow(x,2) + pow(y-d,2),1.5)) - (3*pow(R,2) * x * (2- 3*pow(x,2)/(pow(x,2) + pow(-d+y,2)))*mu0)/(4*pow((pow(x,2)+pow(-d+y,2)),2.5));
	return Dx;
}

double Dx23(double x, double y, double R, double d){
	double Dx =-15*pow(R,2)*pow(-d+x,2)*y*mu0/(4*pow((pow(-d+x,2)+pow(y,2)),3.5)) + 3*pow(R,2)*y*mu0/(4*pow((pow(-d+x,2)+pow(y,2)),2.5));
	return Dx;
}

double Dx24(double x, double y, double R, double d){
	double Dx =mu0*pow(R,2) * (6*pow(x,3)/pow((pow(x,2)+pow(d+y,2)),2)-6*x/(pow(x,2)+pow(d+y,2)))/(4*pow((pow(x,2) + pow(y+d,2)),1.5)) - (3*pow(R,2) * x * (2- 3*pow(x,2)/(pow(x,2) + pow(d+y,2)))*mu0)/(4*pow((pow(x,2)+pow(d+y,2)),2.5));
	return Dx;
}

MatrixXd Dx(double x, double y, double R, double d){
	// Verified in Matlab
	MatrixXd Dmat(2,4);
	Dmat << Dx11(x,y,R,d), Dx12(x,y,R,d), Dx13(x,y,R,d), Dx14(x,y,R,d),
  		Dx21(x,y,R,d), Dx22(x,y,R,d), Dx23(x,y,R,d), Dx24(x,y,R,d);
  	return Dmat;
}


double Dy11(double x, double y, double R, double d){
    double Dx = mu0*pow(R,2) * (6*pow(y,3)/pow((pow(x+d,2)+pow(y,2)),2)-6*y/(pow(x+d,2)+pow(y,2)))/(4*pow((pow(x+d,2) + pow(y,2)),1.5)) - (3*pow(R,2) * y * (2- 3*pow(y,2)/(pow(d+x,2) + pow(y,2)))*mu0)/(4*pow((pow(d+x,2)+pow(y,2)),2.5));
    return Dx;
}

double Dy12(double x, double y, double R, double d){
    double Dx = -15*pow(R,2)*pow(-d+y,2)*x*mu0/(4*pow((pow(-d+y,2)+pow(x,2)),3.5)) + 3*pow(R,2)*x*mu0/(4*pow((pow(x,2)+pow(y-d,2)),2.5));
    return Dx; 
}

double Dy13(double x, double y, double R, double d){
    double Dx = mu0*pow(R,2) * (6*pow(y,3)/pow((pow(x-d,2)+pow(y,2)),2)-6*y/(pow(x-d,2)+pow(y,2)))/(4*pow(pow(x-d,2) + pow(y,2),1.5)) - (3*pow(R,2) * y * (2- 3*pow(y,2)/(pow(-d+x,2) + pow(y,2)))*mu0)/(4*pow((pow(-d+x,2)+pow(y,2)),2.5));
    return Dx;
}

double Dy14(double x, double y, double R, double d){
    double Dx = -15*pow(R,2)*pow(d+y,2)*x*mu0/(4*pow((pow(d+y,2)+pow(x,2)),3.5)) + 3*pow(R,2)*x*mu0/(4*pow((pow(x,2)+pow(y+d,2)),2.5));
    return Dx;
}

double Dy21(double x, double y, double R, double d){
    double Dx = -15*pow(R,2)*(d+x)*pow(y,2)*mu0/(4*pow((pow(d+x,2)+pow(y,2)),3.5)) + 3*pow(R,2)*(x+d)*mu0/(4*pow((pow(d+x,2)+pow(y,2)),2.5));
    return Dx;
}

double Dy22(double x, double y, double R, double d){
    double Dx = mu0*pow(R,2) * pow(x,2)*(-d+y)/(2*pow((pow(x,2)+pow(-d+y,2)),3.5)) - (3*pow(R,2)*(y-d)*(2-3*pow(x,2)/(pow(x,2)+pow(-d+y,2)))*mu0)/(4*pow((pow(x,2)+pow(y-d,2)),2.5));
    return Dx;
}

double Dy23(double x, double y, double R, double d){
    double Dx = -15*pow(R,2)*(-d+x)*pow(y,2)*mu0/(4*pow((pow(-d+x,2)+pow(y,2)),3.5)) + 3*pow(R,2)*(x-d)*mu0/(4*pow((pow(-d+x,2)+pow(y,2)),2.5));
    return Dx;
}

double Dy24(double x, double y, double R, double d){
    double Dx = mu0*pow(R,2) * pow(x,2)*(d+y)/(2*pow((pow(x,2)+pow(d+y,2)),3.5)) - (3*pow(R,2)*(y+d)*(2-3*pow(x,2)/(pow(x,2)+pow(d+y,2)))*mu0)/(4*pow((pow(x,2)+pow(y+d,2)),2.5));
    return Dx;
}

MatrixXd Dy(double x, double y, double R, double d){
    // Verified in Matlab
    MatrixXd Dmat(2,4);
    Dmat << Dy11(x,y,R,d), Dy12(x,y,R,d), Dy13(x,y,R,d), Dy14(x,y,R,d),
        Dy21(x,y,R,d), Dy22(x,y,R,d), Dy23(x,y,R,d), Dy24(x,y,R,d);
    return Dmat;
}

MatrixXd Mx(double x, double y, double R, double d){
    MatrixXd M;
    MatrixXd Dmat = Dx(x,y,R,d);
    MatrixXd Bmat = computeBmat(x,y,R,d);
    M = 0.5*(Dmat.transpose()*Bmat + Bmat.transpose() * Dmat);
    return M;
}

MatrixXd My(double x, double y, double R, double d){
    MatrixXd M;
    MatrixXd Dmat = Dy(x,y,R,d);
    MatrixXd Bmat = computeBmat(x,y,R,d);
    M = 0.5*(Dmat.transpose()*Bmat + Bmat.transpose() * Dmat);
    return M;
}