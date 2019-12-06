import piarm
import random
from time import sleep

my_robot = piarm.PiArm()
try:
    my_robot.connect("/dev/ttyS0")
except:
    print('Error connecting to PiArm')
    exit()
    
if my_robot.alive:
    print('PiArm Connected!')
    print('Let the dance begin! ;)')
    while True:
        servo = random.randint(1, 6)
        pos = random.randint(100, 600)
        if servo == 5:
            servo = servo + 1
        my_robot.servoWrite(servo, pos)
        sleep(0.3)