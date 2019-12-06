from picamera import PiCamera
from time import sleep
from PIL import Image

camera = PiCamera()

camera.start_preview()
sleep(5)
camera.capture('/home/pi/tensorflow1/models/research/object_detection/test_images/image1.jpg')
camera.stop_preview()
