# This code is copyright Simon Walters under GPL v2
# This code is derived from Pi-Face scratch_handler by Thomas Preston
# This code now hosted on Github thanks to Ben Nuttall
Version =  2.82 # 10Aug13



from array import *
import threading
import socket
import time
import sys
import struct
import datetime as dt
import shlex
import os
import math
#try and inport smbus but don't worry if not installed

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.cleanup()
print "Board Revision" , GPIO.RPI_REVISION

#Set some constants and initialise arrays
STEPPERA=0
STEPPERB=1
STEPPERC=2
stepperInUse = array('b',[False,False,False])
INVERT = False
BIG_NUM = 2123456789

   
turnAStep = 0
turnBStep = 0
turnCStep = 0
stepMode = ['1Coil','2Coil','HalfStep']
stepModeDelay = [0.0025,0.0025,0.0013]
stepType = 2
if stepType == 2:
    step_delay = 0.0013 # use smaller dealy fro halfstep mode
else:
    step_delay = 0.003




PIN_NUM = array('i',[11,12,13,15,16,18,22, 7, 3, 5,24,26,19,21,23, 8,10])
PIN_USE = array('i',[ 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    

#  GPIO_NUM = array('i',[17,18,21,22,23,24,25,4,14,15,8,7,10,9])
PINS = len(PIN_NUM)
PIN_NUM_LOOKUP=[int] * 27

for i in range(PINS):
    PIN_NUM_LOOKUP[PIN_NUM[i]] = i


PWM_OUT = [None] * PINS


def isNumeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False
    
def removeNonAscii(s): return "".join(i for i in s if ord(i)<128)
    
  

def sign(number):return cmp(number,0)


#Procedure to set pin mode for each pin
def SetPinMode():
    for i in range(PINS):
        if (PIN_USE[i] == 1):
            print 'setting pin' , PIN_NUM[i] , ' to out'
            GPIO.setup(PIN_NUM[i],GPIO.OUT)
        elif (PIN_USE[i] == 0):
            print 'setting pin' , PIN_NUM[i] , ' to in'
            GPIO.setup(PIN_NUM[i],GPIO.IN,pull_up_down=GPIO.PUD_UP)

        PIN_NUM_LOOKUP[PIN_NUM[i]] = i
        
#----------------------------- STEPPER CONTROL --------------
class StepperControl(threading.Thread):
    def __init__(self,pinA,pinB,pinC,pinD,step_delay):
        #self.stepper_num = stepper_num # find which stepper a or b
        #self.step_delay = step_delay
        self.stepperSpeed = 0 #stepp speed dset to 0 when thread created
        self.steps = BIG_NUM # default to psuedo infinte number of turns
        self.terminated = False
        self.toTerminate = False
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        self.pins = array("i",[PIN_NUM_LOOKUP[pinA],PIN_NUM_LOOKUP[pinB],PIN_NUM_LOOKUP[pinC],PIN_NUM_LOOKUP[pinD]])
        self.slow_start = self.steps
        self.steps_start = self.steps
        self.paused = False
        self.pause_start_time = dt.datetime.now()


    def start(self):
        self.thread = threading.Thread(None, self.run, None, (), {})
        self.thread.start()


    def stop(self):
        self.toTerminate = True
        while self.terminated == False:
        # Just wait
            time.sleep(0.01)

    def IsTurning(self):
        if self.steps > 0:
            return True
        else:
            return False

    def changeSpeed(self, stepperSpeed,steps):
        self.stepperSpeed = int(stepperSpeed)
        self.steps = int(steps)
        self.steps_start = self.steps
        self.slow_start = self.steps - int(min(64,max(1,int(float(self.steps)*0.8))))
        if self.steps > (BIG_NUM / 2):
            self.slow_start = self.steps - 64       
            


    def physical_pin_update(self, pin_index, value):
        if (PIN_USE[pin_index] == 0):
            PIN_USE[pin_index] = 1
            GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
            print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from input'
        if (PIN_USE[pin_index] == 2):
            PIN_USE[pin_index] = 1
            PWM_OUT[pin_index].stop()
            GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
            print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from PWM'
        if (PIN_USE[pin_index] == 1):
            #print 'setting physical pin %d to %d' % (PIN_NUM[pin_index],value)
            GPIO.output(PIN_NUM[pin_index], value)


    def step_coarse(self,a,b,c,d,delay):
        global stepType
        lstepMode = stepMode[stepType]
        #print stepMode[stepType]
        if lstepMode == '1Coil':
            self.physical_pin_update(d,0)
            self.physical_pin_update(a,1)
            time.sleep(delay)

            
            self.physical_pin_update(b,1)
            self.physical_pin_update(a,0)
            time.sleep(delay)
            
            
            self.physical_pin_update(c,1)
            self.physical_pin_update(b,0)
            time.sleep(delay)
            
            
            self.physical_pin_update(d,1)
            self.physical_pin_update(c,0)
            time.sleep(delay)
            
        elif lstepMode == '2Coil':
            self.physical_pin_update(d,0)
            self.physical_pin_update(c,0)
            self.physical_pin_update(a,1)
            self.physical_pin_update(b,1)

            time.sleep(delay)

            self.physical_pin_update(a,0)
            self.physical_pin_update(c,1)
            time.sleep(delay)
            
            self.physical_pin_update(b,0)
            self.physical_pin_update(d,1)
            time.sleep(delay)
            
            self.physical_pin_update(c,0)
            self.physical_pin_update(a,1)
            time.sleep(delay)
            
        elif lstepMode == 'HalfStep':
            self.physical_pin_update(d,0) 
            self.physical_pin_update(a,1)
            time.sleep(delay)


            self.physical_pin_update(b,1)
            time.sleep(delay)

            self.physical_pin_update(a,0)
            time.sleep(delay)
            
            self.physical_pin_update(c,1)
            time.sleep(delay)

            self.physical_pin_update(b,0)
            time.sleep(delay)

            self.physical_pin_update(d,1)
            time.sleep(delay)

            self.physical_pin_update(c,0)
            time.sleep(delay)
            
            self.physical_pin_update(a,1)
            time.sleep(delay)

    def pause(self):
        self.physical_pin_update(self.pins[0],0)
        self.physical_pin_update(self.pins[1],0)
        self.physical_pin_update(self.pins[2],0)
        self.physical_pin_update(self.pins[3],0)
        self.paused = True
        print PIN_NUM[self.pins[0]], "pause method run"




    def run(self):
        #time.sleep(2) # just wait till board likely to be up and running
        self.pause_start_time = dt.datetime.now()
        while self.toTerminate == False:
            #print self.pins[0],self.pins[1],self.pins[2],self.pins[3]

            if (self.steps > 0):
                self.steps = self.steps - 1
                self.local_stepper_value=self.stepperSpeed # get stepper value in case its changed during this thread
                if self.local_stepper_value != 0: #if stepper_value non-zero
                    self.currentStepDelay = step_delay * 100 / abs(self.local_stepper_value)
                    if self.steps < (self.steps_start - self.slow_start) :
                        self.currentStepDelay = self.currentStepDelay *  (3.0 - (((self.steps) / float(self.steps_start - self.slow_start)))*2.0)
                        #print 2.0 - ((self.steps) / float(self.steps_start - self.slow_start))
                    if (self.slow_start < self.steps):
                        #print 2.0 - ((self.steps_start - self.steps) / float(self.steps_start - self.slow_start))
                        self.currentStepDelay = self.currentStepDelay * (3.0 - (((self.steps_start - self.steps) / float(self.steps_start - self.slow_start)))*2.0)
                    #print self.steps, self.currentStepDelay
                    if self.local_stepper_value > 0: # if positive value
                        self.step_coarse(self.pins[0],self.pins[1],self.pins[2],self.pins[3],self.currentStepDelay) #step forward
                    else:
                        self.step_coarse(self.pins[3],self.pins[2],self.pins[1],self.pins[0],self.currentStepDelay) #step forward
##                    if abs(local_stepper_value) != 100: # Only introduce delay if motor not full speed
##                        time.sleep(10*self.step_delay*((100/abs(local_stepper_value))-1))
                    self.pause_start_time = dt.datetime.now()
                    self.paused = False
                    #print PIN_NUM[self.pins[0]],self.pause_start_time
                else:
                    if ((dt.datetime.now() - self.pause_start_time).seconds > 10) and (self.paused == False):
                        self.pause()
                        #print PIN_NUM[self.pins[0]], "paused inner"
                        #print PIN_NUM[self.pins[0]], self.paused
                    #else:
                        #if self.paused == False:
                            #print PIN_NUM[self.pins[0]], "inner" ,(dt.datetime.now() - self.pause_start_time).seconds
                    time.sleep(0.1) # sleep if stepper value is zero
            else:
                if ((dt.datetime.now() - self.pause_start_time).seconds > 10) and (self.paused == False):
                    self.pause()
                    #print PIN_NUM[self.pins[0]], "paused outer"
                    #print PIN_NUM[self.pins[0]], self.paused
                #else:
                    #if self.paused == False:
                        #print PIN_NUM[self.pins[0]], "outer" ,(dt.datetime.now() - self.pause_start_time).seconds
                time.sleep(0.1) # sleep if stepper value is zero

        self.terminated = True
    ####### end of Stepper Class
                    


class MyError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


 
                         
                                      
 



#                if  '1coil' in dataraw:
#                    print "1coil broadcast"
#                    stepType = 0
#                    print "step mode" ,stepMode[stepType]
#                    step_delay = 0.0025

#                if  '2coil' in dataraw:
#                    print "2coil broadcast"
#                    stepType = 1
#                    print "step mode" ,stepMode[stepType]
#                    step_delay = 0.0025
                    
#                if  'halfstep' in dataraw:
#                    print "halfstep broadcast"
#                    stepType = 2
#                    print "step mode" ,stepMode[stepType]
#                    step_delay = 0.0013


if __name__ == '__main__':
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = "127.0.0.1"
    host = host.replace("'", "")


SetPinMode()
try:
    print "StepperA Starting"
    steppera = StepperControl(11,12,13,15,step_delay)
    steppera.start()
    stepperInUse[STEPPERA] = True
    turnAStep = 0
    steppera.changeSpeed(max(-100,min(100,int(float(0)))),2123456789)

    time.sleep(2)

    if (stepperInUse[STEPPERA] == True):
        sensor_value = "128"
        if isNumeric(sensor_value):
            print "Moving to" , sensor_value
            steppera.changeSpeed(int(100 * sign(int(float(sensor_value)) - turnAStep)),abs(int(float(sensor_value)) - 0))
            turnAStep = int(float(sensor_value))
            print turnAStep

    print turnAStep
    while (steppera.IsTurning() == True):
        print steppera.IsTurning()
        time.sleep(0.5)
        
    if (stepperInUse[STEPPERA] == True):
        sensor_value = "-128"
        if isNumeric(sensor_value):
            print "Moving to" , sensor_value
            steppera.changeSpeed(int(100 * sign(int(float(sensor_value)) - turnAStep)),abs(int(float(sensor_value)) - 0))
            turnAStep = int(float(sensor_value))
            print turnAStep

    print turnAStep
    while (steppera.IsTurning() == True):
        print steppera.IsTurning()
        time.sleep(0.5)


except KeyboardInterrupt:
    print "CTRL-C pressed"


if (stepperInUse[STEPPERA] == True):
    print "stopping stepperA"
    steppera.stop()
    print "stepperA stopped"

GPIO.cleanup()
sys.exit(0)
        
        

