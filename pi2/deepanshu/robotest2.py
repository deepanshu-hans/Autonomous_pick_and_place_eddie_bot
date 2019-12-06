import piarm
import random
import time
from time import sleep

my_robot = piarm.PiArm()
try:
    my_robot.connect("/dev/ttyUSB0")
except:
    print('Error connecting to PiArm')
    exit()
    
if my_robot.alive:
    print('PiArm Connected!')
    while True:
        my_robot.servoWrite(1, 200)
        my_robot.servoWrite(2, 850)
        my_robot.servoWrite(3, 999)
        my_robot.servoWrite(4, 50)
        my_robot.servoWrite(5, 200)
        my_robot.servoWrite(6, 0)
        time.sleep(0.5)
        my_robot.servoWrite(1, 670)
        my_robot.servoWrite(2, 850)
        my_robot.servoWrite(3, 999)
        my_robot.servoWrite(4, 50)
        my_robot.servoWrite(5, 200)
        my_robot.servoWrite(6, 0)
        time.sleep(0.5)
        my_robot.servoWrite(1, 670)
        my_robot.servoWrite(2, 100)
        my_robot.servoWrite(3, 400)
        my_robot.servoWrite(4, 700)
        my_robot.servoWrite(5, 200)
        my_robot.servoWrite(6, 0)
        time.sleep(0.5)
        my_robot.servoWrite(1, 200)
        my_robot.servoWrite(2, 100)
        my_robot.servoWrite(3, 400)
        my_robot.servoWrite(4, 700)
        my_robot.servoWrite(5, 200)
        my_robot.servoWrite(6, 0)
        time.sleep(0.5)
        my_robot.servoWrite(1, 200)
        my_robot.servoWrite(2, 850)
        my_robot.servoWrite(3, 999)
        my_robot.servoWrite(4, 370)
        my_robot.servoWrite(5, 200)
        my_robot.servoWrite(6, 0)
        sleep(10)