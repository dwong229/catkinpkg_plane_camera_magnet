#!/usr/bin/env python
import rospy
#from camera_magnet.msg import xyReal
from plane_camera_magnet.msg import roboclawCmd
from std_msgs.msg import String

# for roboclaw : 
import serial
import struct
import time
import math
import csv
import numpy as np

try:
    import numpy
except ImportError:
    print "numpy is not installed"

checksum = 0

port13 = serial.Serial("/dev/roboclaw0", baudrate=38400, timeout=0.1)
port24 = serial.Serial("/dev/roboclaw1", baudrate=38400, timeout=0.1)
noStr = "nonum"

#  Controller that takes in ROS msg roboclawCmdDesired.

# Define global values
PI = 3.14159265358979
L = 0.1
m = 2.1841
mu = 0.004
u0 = math.pow(10,-7)*4*PI
R = 0.0286
mass = 0.000282
#initialize m1val, m2val

def sendcommand(address,command,port):
	global checksum
	checksum = address
	port.write(chr(address));
	checksum += command
	port.write(chr(command));
	return;

def readbyte(port):
	global checksum
	val = struct.unpack('>B',port.read(1));
	checksum += val[0]
	return val[0];	
def readword(port):
	global checksum
	val = struct.unpack('>H',port.read(2));
	checksum += (val[0]&0xFF)
	checksum += (val[0]>>8)&0xFF
	return val[0];	
def writebyte(val,port):
	global checksum
	checksum += val
	return port.write(struct.pack('>B',val));
def writeword(val,port):
	global checksum
	checksum += val
	checksum += (val>>8)&0xFF
	return port.write(struct.pack('>H',val));
def writesword(val,port):
	global checksum
	checksum += val
	checksum += (val>>8)&0xFF
	return port.write(struct.pack('>h',val));

def readversion(port):
	sendcommand(128,21,port)
	return port.read(32);

def readcurrents(port):
	sendcommand(128,49,port);
	motor1 = readword(port);
	motor2 = readword(port);
	crc = checksum&0x7F
	if crc==readbyte(port):
		return (motor1,motor2);
	return (-1,-1);

def SetM1DutyAccel(accel,duty):
	port = port13
	sendcommand(128,52,port)
	writesword(duty,port)
	writeword(accel,port)
	writebyte(checksum&0x7F,port);
	return;

def SetM2DutyAccel(accel,duty):
	port = port24
	sendcommand(128,52,port)
	writesword(duty,port)
	writeword(accel,port)
	writebyte(checksum&0x7F,port);
	return;

def SetM3DutyAccel(accel,duty):
	port = port13
	sendcommand(128,53,port)
	writesword(duty,port)
	writeword(accel,port)
	writebyte(checksum&0x7F,port);
	return;

def SetM4DutyAccel(accel,duty):
	port = port24
	sendcommand(128,53,port)
	writesword(duty,port)
	writeword(accel,port)
	writebyte(checksum&0x7F,port);
	return;


def readtemperature(port):
	sendcommand(128,82,port);
	val = readword(port)
	crc = checksum&0x7F
	if crc==readbyte(port):
		return val
	return -1

def readerrorstate(port):
	sendcommand(128,90,port);
	val = readbyte(port)
	#val = readword()
	crc = checksum&0x7F
	if crc==readbyte(port):
#	if crc==readword():
		return val
	return -1
# end for roboclaw

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
    print "==============="
        
    
def callback(desiredcmd):
    # unpack msg data
    mval = np.array([desiredcmd.m1, desiredcmd.m2, desiredcmd.m3, desiredcmd.m4])
    print("Seq: ", desiredcmd.header.seq)
    print ("Coil1: ", desiredcmd.m1)
    print ("Coil2: ", desiredcmd.m2)
    print ("Coil3: ", desiredcmd.m3)
    print ("Coil4: ", desiredcmd.m4)    
    
    rate = rospy.Rate(15) #Hz

    minval = 0
    maxval = 512

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
    SetM1DutyAccel(65535,mval[0])
    SetM2DutyAccel(65535,mval[1])
    SetM3DutyAccel(65535,mval[2])
    SetM4DutyAccel(65535,mval[3])
    
    print "Error State:",repr(readerrorstate(port13))
    print "Temperature:",readtemperature(port13)/10.0
    m1cur, m2cur = readcurrents(port13);
    print "Current C1: ",m1cur/10.0," C3: ",m2cur/10.0
    print ("Coil1: ", mval[0])
    print ("Coil2: ", mval[1])
    print ("Coil3: ", mval[2])
    print ("Coil4: ", mval[3])    
    
    try:
        talker(mval)
    except rospy.ROSInterruptException:
        SetM1DutyAccel(1500,0)
        SetM2DutyAccel(1500,0)
        SetM3DutyAccel(1500,0)
        SetM4DutyAccel(1500,0)
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
   
    SetM1DutyAccel(1500,0)
    SetM2DutyAccel(1500,0)
    SetM3DutyAccel(1500,0)
    SetM4DutyAccel(1500,0)
    #time.sleep(5)

    #Get version string
    sendcommand(128,21,port13);
    rcv = port13.read(32)
    print repr(rcv)
    sendcommand(128,21,port24);
    rcv2 = port24.read(32)
    print repr(rcv2)

    pub = rospy.Publisher('roboclawcommand',roboclawCmd, queue_size = 10)
    listener()
