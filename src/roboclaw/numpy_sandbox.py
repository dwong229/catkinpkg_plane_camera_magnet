import numpy as np

pwm = np.array([-20, 512, 520, -520])

print "pwm: " 
print pwm

# sign of pwm
pwmsign = np.sign(pwm)

# values greater than 512 limit
pwmlimit = np.absolute(pwm) > 512

# values below 512
pwmsave = np.absolute(pwm)<=512

#pwnnew=pwmnew.astype(int)
pwm = pwm*pwmsave + 512*pwmsign*pwmlimit

pwmlimit = np.absolute(pwm) < 10
pwmsave = np.absolute(pwm)>=10

pwm = pwm*pwmsave + 0*pwmsign*pwmlimit


print pwm