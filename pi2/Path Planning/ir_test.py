import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.IN)
GPIO.setup(38, GPIO.IN)
GPIO.setup(15, GPIO.IN)

while True:
    print('L:', GPIO.input(13), 'M:', GPIO.input(38), 'R:', GPIO.input(15))
    time.sleep(0.2)