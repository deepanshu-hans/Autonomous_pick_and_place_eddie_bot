import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import argparse
import sys
import threading
camera_type = 'picamera'
#----------------------------------------------------------
import time
import RPi.GPIO as GPIO
import serial
import random

GPIO.setmode(GPIO.BOARD)

port = serial.Serial("/dev/ttyUSB0",
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 10)

IR_MID = 38
ULS_LEFT = 36
ULS_RIGHT = 40

forward = str.encode('GO 7F 7F\r')
reverse = str.encode('GO 81 81\r')
left = str.encode('GO 81 7F\r')
right = str.encode('GO 7F 81\r')
watch = str.encode('WATCH 0\r')
stop = str.encode('STOP 0\r')
reset = str.encode('RST\r')
verbon = str.encode('VERB 1\r')
setspeed = str.encode('GOSPD 1000 1000\r')
port.write(watch)
port.write(setspeed)

GPIO.setup(IR_MID, GPIO.IN)

obj_detected = False

def CheckForDistance(pin):
    GPIO.setup(pin, GPIO.OUT)  
       
    GPIO.output(pin, 0)  
    time.sleep(0.000002)  
    GPIO.output(pin, 1)
    time.sleep(0.000005)  
    GPIO.output(pin, 0)  
  
    GPIO.setup(pin, GPIO.IN)  
  
    while GPIO.input(pin) == 0:  
       starttime = time.time()  

  
    while GPIO.input(pin) == 1:  
       endtime = time.time()  
        
    duration = endtime - starttime  
    distance = duration * 34000/2
    
    if distance < 50:
        return 1
    else:
        return 0
        
    
def CheckForObstacle(pin):
    return GPIO.input(pin)
    
def steerRight():
    port.write(stop)
    time.sleep(1)
    port.write(right)
    time.sleep(0.5)
    port.write(stop)
    
def steerLeft():
    port.write(stop)
    time.sleep(1)
    port.write(left)
    time.sleep(0.5)
    port.write(stop)
    
def reverseAndTurn():
    port.write(stop)
    time.sleep(1)
    port.write(reverse)
    time.sleep(2)
    port.write(stop)
    port.write(right)
    time.sleep(0.5)
    port.write(stop)
    
def steerForward():
    port.write(forward)
#----------------------------------------------------------

def start_detection():
    global obj_detected
    # Set up camera constants
    IM_WIDTH = 640
    IM_HEIGHT = 480

    parser = argparse.ArgumentParser()
    parser.add_argument('--usbcam', help='Use a USB webcam instead of picamera',
                        action='store_true')
    args = parser.parse_args()
    if args.usbcam:
        camera_type = 'usb'

    # This is needed since the working directory is the object_detection folder.
    sys.path.append('..')

    # Import utilites
    from utils import label_map_util
    from utils import visualization_utils as vis_util

    # Name of the directory containing the object detection module we're using
    MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

    # Grab path to current working directory
    CWD_PATH = os.getcwd()

    # Path to frozen detection graph .pb file, which contains the model that is used
    # for object detection.
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

    # Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,'data','mscoco_label_map.pbtxt')

    # Number of classes the object detector can identify
    NUM_CLASSES = 90

    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    # Load the Tensorflow model into memory.
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        sess = tf.Session(graph=detection_graph)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

    # Number of objects detected
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCapture.truncate(0)
    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        
        if (not CheckForDistance(ULS_LEFT)) and (not CheckForDistance(ULS_RIGHT)) and (not CheckForObstacle(IR_MID)):
            port.write(stop)
            print('Nothing Ahead!')
            steerForward()
            time.sleep(0.1)
        elif CheckForDistance(ULS_LEFT):
            port.write(stop)
            print('Turning Right!')
            time.sleep(5)
            steerRight()
        elif CheckForDistance(ULS_RIGHT):
            port.write(stop)
            print('Turning Left!')
            time.sleep(5)
            steerLeft()
        elif CheckForObstacle(IR_MID):
            print('Getting Back!')
            port.write(stop)
            time.sleep(5)
            reverseAndTurn()
        
        t1 = cv2.getTickCount()
        
        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in the column has the pixel RGB value
        frame = np.copy(frame1.array)
        frame.setflags(write=1)
        frame_expanded = np.expand_dims(frame, axis=0)

        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})

        # Draw the results of the detection (aka 'visulaize the results')
        vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=0.70)

        cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

        # All the results have been drawn on the frame, so it's time to display it.
        cv2.imshow('Object detector', frame)

        t2 = cv2.getTickCount()
        time1 = (t2-t1)/freq
        frame_rate_calc = 1/time1
        
        #Coordinates
        #print(classes)
        index = np.argwhere(classes!=44)
        #print(index)
        new_classes = np.delete(classes, index)
        #print(new_classes)
        if 44 in classes:
            print('Bottle Detected!')
            port.write(stop)
            exit()
        else:
            steerForward()
        #print(boxes)
        new_boxes = np.delete(boxes,index,axis=0)
        #print(new_boxes)
        #f_b=[]
        #for i in new_boxes:
        #    ymin, xmin, ymax, xmax = i
        #    f_b.append([xmin * IM_WIDTH, xmax * IM_WIDTH, ymin * IM_HEIGHT, ymax * IM_HEIGHT])
        #print(f_b)

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

        rawCapture.truncate(0)
    
    camera.close()
    cv2.destroyAllWindows()
    

#Main
if camera_type == 'picamera':
    start_detection()
    

