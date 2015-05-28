import scipy.optimize
from scipy import optimize
import numpy as np

import math as math

# def q(x):
#     return (3.0-x)*scipy.exp(x)  - 3.0
    
# def qprime(x):
#     return (2.0-x)*scipy.exp(x) 

# print scipy.optimize.newton(q, 5, qprime, maxiter=500)

#def func2(x):
    #y = [x[0]**2 - 2*x[0]*x[1] - x[2], x[0]*x[1] + x[1]**2*x[2], x[2]*x[0] - x[1]*x[2]+ x[0] - 1]
    #np.array([x[0],x[1],x[2],x[3]]) * np.
    #xvec = np.matrix('x[0];x[1]')
    #y = [ xvec.transpose() * np.matrix( '1,2;3,4' ) * xvec - 5]
#    y = [np.matrix('x[0],x[1]') * np.matrix( '1,2;3,4' ) * np.matrix('x[0];x[1]') - 5]
#    y.append(np.matrix('x[0],x[1]') * np.matrix( '4,-1;1,4' ) * np.matrix('x[0];x[1]') - 3)
    #y.append(xvec.transpose() * np.matrix( '4,-1;1,4' ) * xvec - 3)
#    return y

#x0 = scipy.optimize.fsolve(func2, [0, 0])

#x0 = [np.matrix('x[0],x[1]') * np.matrix( '1,2;3,4' ) * np.matrix('x[0];x[1]') - 5]
def func2(x):  
    out = [x[0]*math.cos(x[1]) - 4]  
    out.append(x[1]*x[0] - x[1] - 5)  
    return out  

x02 = scipy.optimize.fsolve(func2,[1,1])
print x02

eq1 = [np.matrix('5,5') * np.matrix( '1,2;3,4' ) * np.matrix('1;1') - 5]
eq2 = np.matrix('5,5') * np.matrix( '1,2;3,4' ) * np.matrix('1;1') - 5
eq1 = eq1[0][0]
eq2 = np.matrix('5,5') * np.matrix( '1,2;3,4' ) * np.matrix('1;1') - 5
eq2 = eq2[0].astype(float)
print eq1
print eq2
vec = [eq1,eq2]
#append(eq2)
print vec
#vectest = eq1.astype(double)
#print vectest

#x0 = np.matrix('x[0],x[1]')
#print x0