#!/usr/bin/env python
import rospy
#from camera_magnet.msg import xyReal
from plane_camera_magnet.msg import roboclawCmd
from std_msgs.msg import String

# for roboclaw : 
import roboclaw
import math
import csv
import numpy as np

try:
    import numpy
except ImportError:
    print "numpy is not installed"

roboclaw.Open("/dev/roboclaw0",38400)

checksum = 0

#  Controller that takes in ROS msg roboclawCmdDesired.

# Define global values
PI = 3.14159265358979
L = 0.1
m = 2.1841
mu = 0.004
u0 = math.pow(10,-7)*4*PI
R = 0.0286
mass = 0.000282

print "Roboclaw 4 Coil Inputs\r\n"
def talker(mval):
	# publishing to roboclawcommand topic
    #pub = rospy.Publisher('roboclawcommand',roboclawCmd, queue_size = 10)

    msg = roboclawCmd();
          
    msg.m1 = mval[0];
    msg.m2 = mval[1];
    msg.m3 = mval[2];
    msg.m4 = mval[3];
    msg.header.stamp =  rospy.Time.now();
    pub.publish(msg);
    #print "==============="
        
    
def callback(desiredcmd):
    # unpack msg data
    mval = np.array([desiredcmd.m1, desiredcmd.m2, desiredcmd.m3, desiredcmd.m4])
    #print("Seq: ", desiredcmd.header.seq)
    #print ("Coil1: ", desiredcmd.m1)
    #print ("Coil2: ", desiredcmd.m2)
    #print ("Coil3: ", desiredcmd.m3)
    #print ("Coil4: ", desiredcmd.m4)    
    
    rate = rospy.Rate(10) #Hz

    minval = 2
    maxval = 10000

    # bound value of pwm assignment
    # sign of pwm
    pwmsign = np.sign(mval)

    # values greater than 512 limit
    pwmlimit = np.absolute(mval) > maxval

    # values below 512
    pwmsave = np.absolute(mval)<=maxval

    #pwnnew=pwmnew.astype(int)
    pwm = mval*pwmsave + maxval*pwmsign*pwmlimit

    pwmlimit = np.absolute(pwm) < minval
    pwmsave = np.absolute(pwm)>=minval

    mval = pwm*pwmsave + 0*pwmsign*pwmlimit

    # assign values
    roboclaw.SetM1DutyAccel(65535,mval[0])
    roboclaw.SetM2DutyAccel(65535,mval[1])
    roboclaw.SetM3DutyAccel(65535,mval[2])
    roboclaw.SetM4DutyAccel(65535,mval[3])
    
    #error13 = readerrorstate(port13)
    #error24 = readerrorstate(port24)
    #print "Error State:",repr(readerrorstate(port13))
    #print "Temperature:",readtemperature(port13)/10.0
    #m1cur, m2cur = readcurrents(port13);
    #print "Current C1: ",m1cur/10.0," C3: ",m2cur/10.0
    #print ("Coil1: ", mval[0])
    #print ("Coil2: ", mval[1])
    #print ("Coil3: ", mval[2])
    #print ("Coil4: ", mval[3])    
    
    try:
        talker(mval)
    except rospy.ROSInterruptException:
        roboclaw.SetM1DutyAccel(1500,0)
        roboclaw.SetM2DutyAccel(1500,0)
        roboclaw.SetM3DutyAccel(1500,0)
        roboclaw.SetM4DutyAccel(1500,0)
        pass
    rate.sleep()
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('controller', anonymous=True)


    #rospy.Subscriber("/test_trajreading/roboclawcmddesired", roboclawCmd, callback)
    rospy.Subscriber("/closedloopfbpoint/roboclawcmddesired", roboclawCmd, callback, queue_size=1)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
   
    roboclaw.SetM1DutyAccel(1500,0)
    roboclaw.SetM2DutyAccel(1500,0)
    roboclaw.SetM3DutyAccel(1500,0)
    roboclaw.SetM4DutyAccel(1500,0)
    #time.sleep(5)

    #Get version string
    config = roboclaw.GetConfig(0x80)
    print "13config: ", format(config[1],'02x')

    config = roboclaw.GetConfig(0x81)
    print "24config: ", format(config[1],'02x')


    pub = rospy.Publisher('roboclawcommand',roboclawCmd, queue_size = 10)
    listener()

