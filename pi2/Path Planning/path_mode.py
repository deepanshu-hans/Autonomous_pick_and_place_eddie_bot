import time
from time import sleep
import RPi.GPIO as GPIO
import serial
import piarm
import random

my_robot = piarm.PiArm()
try:
    my_robot.connect("/dev/ttyUSB1")
except:
    print('Error connecting to PiArm')

GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.IN)

port = serial.Serial("/dev/ttyUSB0",
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 10)

IR_MID = 38
IR_LEFT = 13
IR_RIGHT = 15
ULS_LEFT = 36
ULS_RIGHT = 40

forward = str.encode('GO 1C 1C\r')
reverse = str.encode('GO DF DF\r')
left = str.encode('GO DF 1C\r')
right = str.encode('GO 1C DF\r')
watch = str.encode('WATCH 0\r')
stop = str.encode('STOP 0\r')
reset = str.encode('RST\r')
verbon = str.encode('VERB 1\r')
setspeed = str.encode('GOSPD 1000 1000\r')
port.write(watch)
port.write(setspeed)

GPIO.setup(IR_MID, GPIO.IN)
GPIO.setup(IR_LEFT, GPIO.IN)
GPIO.setup(IR_RIGHT, GPIO.IN)

def run_robotic_arm():
    if my_robot.alive:
        port.write(reverse)
        sleep(1)
        port.write(stop)
        print('PiArm Connected!')
        if True:
            my_robot.servoWrite(1, 200)
            my_robot.servoWrite(2, 0)
            my_robot.servoWrite(3, 0)
            my_robot.servoWrite(4, 0)
            my_robot.servoWrite(5, 0)
            my_robot.servoWrite(6, 0)
            time.sleep(2)
            my_robot.servoWrite(1, 200)
            my_robot.servoWrite(2, 0)
            my_robot.servoWrite(3, 0)
            my_robot.servoWrite(4, 0)
            my_robot.servoWrite(5, 0)
            my_robot.servoWrite(6, 402)
            time.sleep(2)
            my_robot.servoWrite(1, 200)
            my_robot.servoWrite(2, 0)
            my_robot.servoWrite(3, 0)
            my_robot.servoWrite(4, 0)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 402)
            time.sleep(2)
            my_robot.servoWrite(1, 200)
            my_robot.servoWrite(2, 0)
            my_robot.servoWrite(3, 800)
            my_robot.servoWrite(4, 0)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 402)
            time.sleep(2)
            my_robot.servoWrite(1, 200)
            my_robot.servoWrite(2, 100)
            my_robot.servoWrite(3, 700)
            my_robot.servoWrite(4, 200, 999)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 402)
            time.sleep(2)
            my_robot.servoWrite(1, 200)
            my_robot.servoWrite(2, 100)
            my_robot.servoWrite(3, 800)
            my_robot.servoWrite(4, 200)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 400)
            time.sleep(2)
            my_robot.servoWrite(1, 200)
            my_robot.servoWrite(2, 100)
            my_robot.servoWrite(3, 800)
            my_robot.servoWrite(4, 400, 999)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 400)
            time.sleep(2)
            my_robot.servoWrite(1, 600)
            my_robot.servoWrite(2, 100)
            my_robot.servoWrite(3, 800)
            my_robot.servoWrite(4, 400, 999)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 400)
            time.sleep(2)
            my_robot.servoWrite(1, 600)
            my_robot.servoWrite(2, 100)
            my_robot.servoWrite(3, 700, 999)
            my_robot.servoWrite(4, 200, 999)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 400)
            time.sleep(2)
            my_robot.servoWrite(1, 600)
            my_robot.servoWrite(2, 100)
            my_robot.servoWrite(3, 700)
            my_robot.servoWrite(4, 200)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 400)
            time.sleep(2)
            my_robot.servoWrite(1, 200)
            my_robot.servoWrite(2, 100)
            my_robot.servoWrite(3, 700)
            my_robot.servoWrite(4, 200)
            my_robot.servoWrite(5, 500)
            my_robot.servoWrite(6, 400)
            sleep(10)

def CheckForDistance(pin):
    GPIO.setup(pin, GPIO.OUT)  
   
    GPIO.output(pin, 0)  
    time.sleep(0.000002)  
    GPIO.output(pin, 1)
    time.sleep(0.000005)  
    GPIO.output(pin, 0)  
  
    GPIO.setup(pin, GPIO.IN)  
  
  
    while GPIO.input(pin)==0:  
       starttime=time.time()  

  
    while GPIO.input(pin)==1:  
       endtime=time.time()  
        
    duration=endtime-starttime  
    distance=duration*34000/2
    
    if distance < 40:
        return 1
    else:
        return 0
        
    
def CheckForObstacle(pin):
    return GPIO.input(pin)
    
def steerRight(t):
    port.write(stop)
    time.sleep(t)
    port.write(right)
    time.sleep(t)
    port.write(stop)
    
def steerLeft(t):
    port.write(stop)
    time.sleep(t)
    port.write(left)
    time.sleep(t)
    port.write(stop)
    
def reverseAndTurn():
    port.write(stop)
    time.sleep(1)
    port.write(reverse)
    time.sleep(2)
    port.write(stop)
    port.write(right)
    time.sleep(1)
    port.write(stop)
    
def steerForward():
    port.write(forward)
    
def adjust_position():
    port.write(stop)
    print('Object Found!, Starting to Adjust Position!')
    forward = True
    while True:
        time.sleep(2)
        
        if CheckForObstacle(IR_MID):
            print('Robotic Arm Activated!')
            steerForward()
            time.sleep(2)
            port.write(stop)
            run_robotic_arm()
            exit()
        if (not CheckForObstacle(IR_MID)) and (not CheckForDistance(ULS_LEFT)) and (not CheckForDistance(ULS_RIGHT)):
            print("Searching...")
            if forward:
                steerForward()
            else:
                steerForward()
                time.sleep(1)
                port.write(stop)
            time.sleep(0.5)
            port.write(stop)
            time.sleep(2)
        if CheckForDistance(ULS_LEFT):
            print("A little left...")
            forward = False
            steerLeft(0.5)
            time.sleep(1)
        if CheckForDistance(ULS_RIGHT):
            print("A little right...")
            forward = False
            steerRight(0.5)
            time.sleep(1)
        if CheckForObstacle(IR_LEFT):
            print("A little left...")
            forward = False
            steerLeft(0.3)
            time.sleep(1)
        if CheckForObstacle(IR_RIGHT):
            print("A little right...")
            forward = False
            steerRight(0.3)
            time.sleep(1)
        
     
while True:
    
    steerForward()
    
    if CheckForObstacle(IR_MID):
        port.write(stop)
        time.sleep(2)
        if GPIO.input(16):
            adjust_position()
        else:
            reverseAndTurn()
            print('Obstacle in Front! Taking a 180 Turn!')
            time.sleep(0.1)
    if CheckForDistance(ULS_LEFT):
        port.write(stop)
        time.sleep(2)
        if GPIO.input(16):
            adjust_position()
        else:
            steerRight(1)
            print('Obstacle in Left! Taking a Right Turn!')
            time.sleep(0.1)
    if CheckForDistance(ULS_RIGHT):
        port.write(stop)
        time.sleep(2)
        if GPIO.input(16):
            adjust_position()
        else:
            steerLeft(1)
            print('Obstacle in Right! Taking a Left Turn!')
            time.sleep(0.1)
