#include "currentcompute.h"
#include <math.h>

using Eigen::MatrixXd;

using namespace Eigen;
using namespace std;

CoilFunctor2::CoilFunctor2(Coil coil, Magnet magnet) : Functor<double>(4,4) {
        d = coil.d;
        R = coil.R;
        /*x[0] = magnet.x[0];
        y[0] = magnet.y[0];
        x[1] = magnet.x[1];
        y[1] = magnet.y[1];
        Fx[0] = magnet.Fx[0];
        Fy[0] = magnet.Fy[0]; 
        Fx[1] = magnet.Fx[1];
        Fy[1] = magnet.Fy[1];
        */
        x = magnet.x;
        y = magnet.y;
        Fx = magnet.Fx;
        Fy = magnet.Fy; 
        x2 = magnet.x2;
        y2 = magnet.y2;
        Fx2 = magnet.Fx2;
        Fy2 = magnet.Fy2; 
        
        gamma = magnet.gamma;
        Mxmat = magnet.Mxmat;
        Mymat = magnet.Mymat;
        Mxmat2 = magnet.Mxmat2;
        Mymat2 = magnet.Mymat2;
		Bmat = magnet.Bmat;
		Bmat2 = magnet.Bmat2;   
        Dxmat = magnet.Dxmat;
        Dymat = magnet.Dymat;
        Dxmat2 = magnet.Dxmat2;
        Dymat2 = magnet.Dymat2;

        Bearth << -0.04,0.;
    }

int CoilFunctor2::operator()(const VectorXd &c, VectorXd &fvec)
{
	assert(c.size()==4);
	assert(fvec.size()==4);
    
    // compute B'B
    VectorXd Btotal = Bmat * c + Bearth;
    double Btotalmag = Btotal.norm();
    VectorXd munit = Btotal / Btotalmag;
    fvec[0] = gamma * munit.transpose() * Dxmat * c - Fx;
    fvec[1] = gamma * munit.transpose() * Dymat * c - Fy;

    VectorXd Btotal2 = Bmat2 * c + Bearth;
    double Btotalmag2 = Btotal2.norm();
    VectorXd munit2 = Btotal2 / Btotalmag2;
    fvec[2] = gamma * munit2.transpose() * Dxmat2 * c - Fx2;
    fvec[3] = gamma * munit2.transpose() * Dymat2 * c - Fy2;    
    //cout << "operator: " << fvec.transpose() << endl;
    //cout << "c: " << c.transpose() << endl;
    return 0;
}

int CoilFunctor2::df(const VectorXd &c, MatrixXd &fjac)
{
		// use numerical differentiation
        assert(c.size()==4);
        assert(fjac.rows()==4);
        assert(fjac.cols()==4);

		// 1/sqrt(c' B'B c )
		double BtB1, BtB2;
        //fjac(equation#, derivative wrt b[i])
        // Force equation derivatives 
        double di = .0001; //distance step
        VectorXd ctemp;
        for(int i=0; i<4; i++){
        	ctemp = c;
        	for(int lowhi = 1; lowhi < 3; lowhi++){
        		// compute lower bound first:
        		// ctemp = c[i] + di/2
        		ctemp[i] = c[i] + di/2*pow(-1.0,lowhi);
        		//std::cout << "ctemp: " << ctemp.transpose() << endl;
        		VectorXd Btotal = Bmat * c + Bearth;
                double Btotalmag = Btotal.norm();
                VectorXd munit = Btotal / Btotalmag;
                
                
                VectorXd Btotal2 = Bmat2 * c + Bearth;
                double Btotalmag2 = Btotal2.norm();
                VectorXd munit2 = Btotal2 / Btotalmag2;


        		if(lowhi==1) {
        			fjac(0,i) = -gamma * munit.transpose() * Dxmat * ctemp;
        			fjac(1,i) = -gamma * munit.transpose() * Dymat * ctemp;
					fjac(2,i) = -gamma * munit2.transpose() * Dxmat2 * ctemp;
					fjac(3,i) = -gamma * munit2.transpose() * Dymat2 * ctemp;
        		}

        		else{
        			fjac(0,i) += gamma * munit.transpose() * Dxmat * ctemp;
        			fjac(1,i) += gamma * munit.transpose() * Dymat * ctemp;
        			fjac(2,i) += gamma * munit2.transpose() * Dxmat2 * ctemp;
					fjac(3,i) += gamma * munit2.transpose() * Dymat2 * ctemp;
        			fjac(0,i) *= 1/di;
					fjac(1,i) *= 1/di;
					fjac(2,i) *= 1/di;
					fjac(3,i) *= 1/di;

        		}
        	}
        	
        }
        //cout << "df: " << endl;
        //cout << fjac << endl;

        return 0;
}

//definitions:
CoilFunctor::CoilFunctor(Coil coil, Magnet magnet) : Functor<double>(6,6) {
        d = coil.d;
        R = coil.R;
        /*x = magnet.x[0];
        y = magnet.y[0];
        Fx = magnet.Fx[0];
        Fy = magnet.Fy[0]; 
        */
        x = magnet.x;
        y = magnet.y;
        Fx = magnet.Fx;
        Fy = magnet.Fy; 
        gamma = magnet.gamma;
        Mxmat = magnet.Mxmat;
        Mymat = magnet.Mymat;
        Dxmat = magnet.Dxmat;
        Dymat = magnet.Dymat;
        Bmat = magnet.Bmat;
        Bearth << -0.04,0.;
        //Bearth << -0.06, 0.;

      }

int CoilFunctor::operator()(const VectorXd &b, VectorXd &fvec)
    {   

        assert(b.size()==6);

       // cout << "fvec: " << fvec << endl;
        assert(fvec.size()==6);

        //cout << "Mxmat = \n" << Mxmat << endl;
        //cout << "Mymat = \n" << Mymat << endl;

        VectorXd c; // current components
        VectorXd lambda;
        c = b.head(4);
        lambda = b.tail(2);

        // Considering Bearth:
        
        VectorXd Btotal = Bmat * c + Bearth;
        double Btotalmag = Btotal.norm();
        VectorXd munit = Btotal / Btotalmag;

        //cout << Btotalmag << endl;
        //cout << munit << endl;
        //fvec << 0., 0., 0., 0., 0., 0.;
        // force constraints
        fvec[0] = gamma * munit.transpose() * Dxmat * c - Fx;
        fvec[1] = gamma * munit.transpose() * Dymat * c - Fy;

        // current minimizing
        fvec[2] = c[0] + gamma * lambda[0] * Mxmat.row(0)*c + gamma * lambda[1] * Mymat.row(0)*c; 
        fvec[3] = c[1] + gamma * lambda[0] * Mxmat.row(1)*c + gamma * lambda[1] * Mymat.row(1)*c;
        fvec[4] = c[2] + gamma * lambda[0] * Mxmat.row(2)*c + gamma * lambda[1] * Mymat.row(2)*c;
        fvec[5] = c[3] + gamma * lambda[0] * Mxmat.row(3)*c + gamma * lambda[1] * Mymat.row(3)*c;
        
        //fvec[2] = c[0] + gamma * lambda[0] * munit[0] * Dxmat.row(0)*c + gamma * lambda[1] *Dymat.row(0) * c;
        //fvec[3] = c[1] + gamma * lambda[0] * Mxmat.row(1)*c + gamma * lambda[1] * Dymat.row(1)*c;
        //fvec[4] = c[2] + gamma * lambda[0] * Mxmat.row(2)*c + gamma * lambda[1] * Mymat.row(2)*c;
        //fvec[5] = c[3] + gamma * lambda[0] * Mxmat.row(3)*c + gamma * lambda[1] * Mymat.row(3)*c;
        

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
        // Force equation derivatives exact for no Bearth:
        /*for(int i=0; i<4; i++){
            fjac(0,i) = Mxmat.col(i).transpose() * c; 
            fjac(0,i) += Mxmat.row(i) * c;
            fjac(0,i) *= gamma;

            fjac(1,i) = Mymat.col(i).transpose() * c;
            fjac(1,i) += Mymat.row(i) * c; 
            fjac(1,i) *= gamma;
        }
        */
        // Compute fjac(0:1,0:3)
        double di = 0.0001;
        VectorXd ctemp, Btotal, munit; 
        double Btotalmag; 
        for(int i = 0; i<4;i++){
            ctemp = c;
            for(int lowhi = 1; lowhi < 3; lowhi++){
                // compute lower bound first:
                // ctemp = c[i] + di/2
                ctemp[i] = c[i] + di/2*pow(-1.0,lowhi);
                Btotal = Bmat * ctemp + Bearth;
                Btotalmag = Btotal.norm();
                VectorXd munit = Btotal / Btotalmag;

                if(lowhi == 1) {
                    fjac(0,i) = -gamma * munit.transpose() * Dxmat * ctemp;
                    fjac(1,i) = -gamma * munit.transpose() * Dymat * ctemp;
                }
                else{
                    fjac(0,i) += gamma * munit.transpose() * Dxmat * ctemp;
                    fjac(1,i) += gamma * munit.transpose() * Dymat * ctemp;
                    fjac(0,i) *= 1/di;
                    fjac(1,i) *= 1/di;
                }
            }

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