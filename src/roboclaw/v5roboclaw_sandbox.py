import time
import roboclaw

#Windows comport name
#roboclaw.Open("COM3",115200)
#Linux comport name
roboclaw.Open("/dev/roboclaw0",38400)

testversion = 1
commands = 1

# DO NOT USE THESE OPTIONS TO 
# set to multi-unit mode:
# Using this will reset all options each time, so 
# you cannot set more than one option (e.g. address and stat).  
#roboclaw.SetConfig(0x80,0x8000)
#roboclaw.SetConfig(0x81,0x0060)
#time.sleep(0.5)
#roboclaw.SetConfig(0x81,0x8000)
#time.sleep(0.5)

# Instead, use buttons on device.
# Stat1: Set mode to 7 or 8 (depending on address),   
# Stat2: RC and Analog Mode options 4 (TTL Flip and Exp and MCU Enabled)
# Lipo: 1.

# Also ensure you have a good connection to coils.

config = roboclaw.GetConfig(0x80)
print "13config: ", format(config[1],'02x')

config = roboclaw.GetConfig(0x81)
print "24config: ", format(config[1],'02x')

if testversion:
    while 1:
        #Get version string
        version = roboclaw.ReadVersion(0x80)
    
        if version[0]==False:
            print "13 GETVERSION Failed"
        else:
            print "13: ", repr(version[1])
    
        version = roboclaw.ReadVersion(0x81)
        
        if version[0]==False:
            print "24 GETVERSION Failed"
        else:
            print "24: ", repr(version[1])
        time.sleep(1)
if commands:
    print "Commands"
    port13 = 0x80
    port24 = 0x81
    accel = 65535
    duty = 200
    roboclaw.DutyAccelM1(port13, accel, duty)
    time.sleep(1)
    roboclaw.DutyAccelM1(port13, accel, 0)
    roboclaw.DutyAccelM1(port24, accel, duty)
    time.sleep(1)
    roboclaw.DutyAccelM1(port24, accel, 0)

