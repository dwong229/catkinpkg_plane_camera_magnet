#  Controller that takes in ROS msg xyReal.
#  Given desired positions along the straw, keep magnet at that position

#  =========== TO ADD =================
# load csv of x, xdot, xdotdot values for a trajectory


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
import numpy
import csv

try:
    import numpy
except ImportError:
    print "numpy is not installed"

checksum = 0
port13 = serial.Serial("/dev/roboclaw0", baudrate=38400, timeout=0.1)
port24 = serial.Serial("/dev/roboclaw1", baudrate=38400, timeout=0.1)
noStr = "nonum"

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
def talker():
	# publishing to roboclawcommand topic
    #pub = rospy.Publisher('roboclawcommand',roboclawCmd, queue_size = 10)
    pub = rospy.Publisher('/roboclaw4loop_pub/roboclawCmd', roboclawCmd, queue_size=10)
    #Get version string
    sendcommand(128,21,port13);
    rcv = port13.read(32)
    print repr(rcv)
    sendcommand(128,21,port24);
    rcv2 = port24.read(32)
    print repr(rcv2)


    #rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(2) #Hz
    msg = roboclawCmd();
    i = 0;
    cnt = 0;
    while not rospy.is_shutdown():
        cnt=cnt+1
        print "Count = ",cnt
        print "Error State:",repr(readerrorstate(port13))
        print "Temperature:",readtemperature(port13)/10.0
        m1cur, m2cur = readcurrents(port13);
        print "Current C1: ",m1cur/10.0," C3: ",m2cur/10.0

        val = 512

        m1val = [-val, 0, -val, 0]
        m2val = [0, 0, 0, 0]
        m3val = [0, -val, 0, -val]
        m4val = [0, 0, 0, 0]

	    # loop through length of m1val
        for i in range(len(m1val)):
            print i
            SetM1DutyAccel(65535,m1val[i])
            SetM2DutyAccel(65535,m2val[i])
            SetM3DutyAccel(65535,m3val[i])
            SetM4DutyAccel(65535,m4val[i])
            
            print ("Coil1: ", m1val[i])
            print ("Coil2: ", m2val[i])
            print ("Coil3: ", m3val[i])
            print ("Coil4: ", m4val[i])    
            msg.m1 = m1val[i];
            msg.m2 = m2val[i];
            msg.m3 = m3val[i];
            msg.m4 = m4val[i];
            msg.header.stamp =  rospy.Time.now();
            pub.publish(msg);
            #time.sleep(1)
            rate.sleep()

        print "==============="        
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
      SetM1DutyAccel(1500,0)
      SetM2DutyAccel(1500,0)
      SetM3DutyAccel(1500,0)
      SetM4DutyAccel(1500,0)
      pass

	
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
   
   SetM1DutyAccel(1500,0)
   SetM2DutyAccel(1500,0)
   SetM3DutyAccel(1500,0)
   SetM4DutyAccel(1500,0)
   #time.sleep(5)

   listener()