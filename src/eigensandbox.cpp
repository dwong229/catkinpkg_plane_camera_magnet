#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include "currentcompute.h"

int main()
{
    // constants
    double d = .05;
    double R = .0286;
    double x = -.03;
    double y = 0.0;
    double Fx = 0.0;
    double Fy = 1.0;
    double gamma = 1.0;

    MatrixXd Mxmat(4,4);
    MatrixXd Mymat(4,4);
    VectorXd b(6);
    VectorXd fvec(6);

    b << 1.,2.,3.,4.,5.,6.;
    cout << b << endl;

    // compute m matrix:
    Mxmat = Mx(x,y,R,d);
    cout << Mxmat << endl;

    Mymat = My(x,y,R,d);
    cout << Mymat << endl;

    VectorXd c(4); // current components
    VectorXd lambda(2);
    c = b.head(4);
    lambda = b.tail(2);

    MatrixXd fjac(6,6);

    // force constraints
    fvec[0] = c.transpose() * Mxmat * c - Fx;
    fvec[1] = c.transpose() * Mymat * c - Fy;
    // current minimizing
    
    fvec[2] = c[0] + gamma * lambda[0] * Mxmat.row(0)*c + gamma * lambda[1] * Mymat.row(0)*c; 
    fvec[3] = c[1] + gamma * lambda[0] * Mxmat.row(1)*c + gamma * lambda[1] * Mymat.row(1)*c;
    fvec[4] = c[2] + gamma * lambda[0] * Mxmat.row(2)*c + gamma * lambda[1] * Mymat.row(2)*c;
    fvec[5] = c[3] + gamma * lambda[0] * Mxmat.row(3)*c + gamma * lambda[1] * Mymat.row(3)*c;
    
    
    cout << "operator: " << fvec[0] << ", " << fvec[1] << ", "  << fvec[2] << endl;

            // Fx 
        for(int i=0; i<4; i++){
            fjac(0,i) = Mxmat.col(i).transpose() * c; 
            fjac(0,i) += Mxmat.row(i) * c;
            fjac(0,i) *= gamma;
            fjac(1,i) = Mymat.col(i).transpose() * c;
            fjac(1,i) += Mymat.row(i) * c; 
            fjac(1,i) *= gamma;
        };
              

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
        
}

