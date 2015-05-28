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

port13 = serial.Serial("/dev/ttyUSB1", baudrate=38400, timeout=0.1)
port24 = serial.Serial("/dev/ttyUSB0", baudrate=38400, timeout=0.1)
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
    pub = rospy.Publisher('/roboclaw4input_pub/roboclawCmd', roboclawCmd, queue_size=10)
    #Get version string
    sendcommand(128,21,port13);
    rcv = port13.read(32)
    print repr(rcv)
    sendcommand(128,21,port24);
    rcv2 = port24.read(32)
    print repr(rcv2)


    #rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(0.5) #Hz
    msg = roboclawCmd();
    i = 0;
    while not rospy.is_shutdown():
        m1cur, m2cur = readcurrents(port13);
        print "Current C1: ",m1cur/10.0," C3: ",m2cur/10.0
        rawvalStr = raw_input("Enter duty cycle for all coils (+-512) numbers separated by space:")    
        rawvalStr = rawvalStr or noStr

        if rawvalStr == noStr:
            SetM1DutyAccel(1500,0)
            SetM2DutyAccel(1500,0)
            SetM3DutyAccel(1500,0)
            SetM4DutyAccel(1500,0)
            print "Program ending..."
            time.sleep(1)
            break
        else:
            valStr = map(int,rawvalStr.split())
            print valStr
            SetM1DutyAccel(65535,valStr[0])
            SetM2DutyAccel(65535,valStr[1])
            SetM3DutyAccel(65535,valStr[2])
            SetM4DutyAccel(65535,valStr[3])

            print ("Coil1: ", valStr[0])
            print ("Coil2: ", valStr[1])
            print ("Coil3: ", valStr[2])
            print ("Coil4: ", valStr[3])

            time.sleep(1)


        #print('[m1val m2val]: {0:.3f}, {1:.3f}'.format(m1val,m2val))

        msg.m1 = valStr[0];
        msg.m2 = valStr[1];
        msg.m3 = valStr[2];
        msg.m4 = valStr[3];
        msg.header.stamp =  rospy.Time.now();
        #msg.xdes = -1;
        #msg.ydes = -1;

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