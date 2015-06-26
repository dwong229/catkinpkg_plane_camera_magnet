#!/usr/bin/env python
 
# this solution will work only in Windows, as msvcrt is a Windows only package

# import thread
# import time
 
# try:
#     from msvcrt import getch  # try to import Windows version
# except ImportError:
#     def getch():   # define non-Windows version
#         import sys, tty, termios
#         fd = sys.stdin.fileno()
#         old_settings = termios.tcgetattr(fd)
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch
 
# char = None
 
# def keypress():
#     global char
#     char = getch()
 
# thread.start_new_thread(keypress, ())
 
# print "Use arrow to change variable arrows, toggle up/down:" 
# while True:
#     if char is not None:
#         print "Key pressed is " + char
#         if char=='t':
#             print "toggle"

#         #break

## Works for arrows only:
#     #print "Program is running"
#     #time.sleep(1)
    #ch = sys.stdin.read(3) # arrows

        # if k=='\x1b[A':
        #      print "up"
        # elif k=='\x1b[B':
        #      print "down"
        # elif k=='\x1b[C':
        #      print "right"
        # elif k=='\x1b[D':
        #      print "left"

# Works for arrows:
import sys,tty,termios
import numpy as np

global pwm, stepsize, toggle
pwm = np.array([0,0,0,0])
stepsize = 10.0
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
        else:
            print "not a registered value!"
        print "pwm"
        print pwm
        print "toggle: " + str(toggle[0])
        print stepsize
        #return pwm,toggle,stepsize
        return loop

def main():
        loop = True
        while (loop):
            loop = get() #(pwm,toggle,stepsize)
            print loop

if __name__=='__main__':
        print pwm;
        print toggle, stepsize
        main()