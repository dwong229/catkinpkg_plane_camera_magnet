#  Manual controller for keyboard input to magnet

#  =========== TO ADD =================
# load csv of x, xdot, xdotdot values for a trajectory


#!/usr/bin/env python
import rospy
#from camera_magnet.msg import xyReal
from plane_camera_magnet.msg import roboclawCmd
from std_msgs.msg import String


# for roboclaw : 
import roboclaw
import math
import csv

import sys,tty,termios
import numpy as np

try:
    import numpy
except ImportError:
    print "numpy is not installed"

checksum = 0

baudrate = 38400
roboclaw.Open("/dev/roboclaw0",baudrate)

# Define global values
PI = 3.14159265358979
L = 0.1
m = 2.1841
mu = 0.004
u0 = math.pow(10,-7)*4*PI
R = 0.0286
mass = 0.000282

global pwm, stepsize, toggle
pwm = np.array([0,0,0,0])
maxpwmval = 32767
stepsize = maxpwmval/4;
stepsize = 1000;
toggle = np.array([1.0])

class _Getch:
    # initialize variables
    def __call__(self):
            fd = sys.stdin.fileno() 
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1) # for single variable
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get(): #(pwm,toggle,stepsize):
        global pwm
        print "Press key t to toggle +/-, j/i/l/m for coil left/up/right/down:"
        inkey = _Getch()
        loop = True
        #toggle = 1
        while(loop):
             k=inkey()
             print "Key pressed is " + k 
             if k!='':break
        if k=='t':
            toggle[0] = toggle[0] * (-1)
        
        elif k == "j":
             pwm[0] = pwm[0] + stepsize*toggle[0];
        elif k=='i':
             pwm[1] = pwm[1] + stepsize*toggle[0];
        elif k=='l':
             pwm[2] = pwm[2] + stepsize*toggle[0];
        elif k=='k':
             pwm[3] = pwm[3] + stepsize*toggle[0];
        elif k=='q':
            loop = False
            pwm[0] = 0
            pwm[1] = 0
            pwm[2] = 0
            pwm[3] = 0
        else:
            print "not a registered value!"
        print "pwm"
        print pwm
        # sign of pwm
        pwmsign = np.sign(pwm)

        # values greater than 512 limit
        pwmlimit = np.absolute(pwm) > maxpwmval

        # values below 512
        pwmsave = np.absolute(pwm)<=maxpwmval

        #pwnnew=pwmnew.astype(int)
        pwm = pwm*pwmsave + maxpwmval*pwmsign*pwmlimit

        print "toggle: " + str(toggle[0])
        print stepsize
        #return pwm,toggle,stepsize
        return loop



print "Roboclaw 4 Coil Inputs\r\n"
def talker():
    # publishing to roboclawcommand topic
    pub = rospy.Publisher('/v5roboclaw4keyboard_pub/roboclawCmd', roboclawCmd, queue_size=10)
    #Get version string
    version13 = roboclaw.ReadVersion(0x80)
    version24 = roboclaw.ReadVersion(0x81)
    if version13[0]==False:
        print "roboclaw0: GETVERSION Failed"
    else:
        print repr(version13[1])

    if version24[0]==False:
        print "roboclaw1: GETVERSION Failed"
    else:
        print repr(version24[1])

    #rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(100) #Hz
    msg = roboclawCmd();
    i = 0;
    while not rospy.is_shutdown():
        #m13cur = port13.ReadCurrents(address);
        #print "Current C1: ",m13cur[0]," C3: ",m13cur[1]
        
        # Wait for keyboard input value: 
        # hit t to toggle increase or decrease
        # hit arrows to change value of each coils
        loop = get() #(pwm,toggle,stepsize)
	
        roboclaw.SetM1DutyAccel(65535,pwm[0])
        roboclaw.SetM2DutyAccel(65535,pwm[1])
        roboclaw.SetM3DutyAccel(65535,pwm[2])
        roboclaw.SetM4DutyAccel(65535,pwm[3])
        print ("Coil1: ", pwm[0])
        print ("Coil2: ", pwm[1])
        print ("Coil3: ", pwm[2])
        print ("Coil4: ", pwm[3])

        #print('[m1val m2val]: {0:.3f}, {1:.3f}'.format(m1val,m2val))

        msg.m1 = pwm[0];
        msg.m2 = pwm[1];
        msg.m3 = pwm[2];
        msg.m4 = pwm[3];
        msg.header.stamp =  rospy.Time.now();
        #msg.xdes = -1;
        #msg.ydes = -1;

        if(loop == False):
            print "Exiting loop..."
            break
        #SetM1Speed(m1val)
        #SetM2Speed(m2val)
#        65535 or 1500 accel

        #SetM1DutyAccel(65535,m1val)
        #SetM2DutyAccel(65535,m2val)

        #rospy.loginfo(msg);
        pub.publish(msg);

        rate.sleep()
        
        i = i + 1;
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('controller', anonymous=True)
    #rospy.Subscriber("/magnet_track/xyReal", xyReal, callback)

    # define timer that updates every x:
    
    try: 
        talker()
        #motorupdate()
    except rospy.ROSInterruptException:
        roboclaw.SetM1DutyAccel(1500,0)
        roboclaw.SetM2DutyAccel(1500,0)
        roboclaw.SetM3DutyAccel(1500,0)
        roboclaw.SetM4DutyAccel(1500,0)
        pass

    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
   
   roboclaw.SetM1DutyAccel(1500,0)
   roboclaw.SetM2DutyAccel(1500,0)
   roboclaw.SetM3DutyAccel(1500,0)
   roboclaw.SetM4DutyAccel(1500,0)
   #time.sleep(5)

   listener()
