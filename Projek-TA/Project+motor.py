# Import required libraries
import RPi.GPIO as GPIO
import time
import os
import argparse
import cv2
import numpy as np
import sys
import glob
import importlib.util
import serial
import time
from time import sleep


# --------------------------------------------------------------------
# PINS MAPPING AND SETUP
# --------------------------------------------------------------------

echoPIN = 24
triggerPIN = 23
led = 18
flash = 25

# Pins for Motor Driver Inputs 
Motor1A = 21 #kuning
Motor1B = 20 #orange
kecepatan1 = 16 #merah

Motor2A = 26 #coklat
Motor2B = 13 #ungu
kecepatan2 = 19 #putih

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(echoPIN,GPIO.IN)
GPIO.setup(triggerPIN,GPIO.OUT)
GPIO.setup(led, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(flash, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(Motor1A,GPIO.OUT)  # All pins as Outputs
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(kecepatan1,GPIO.OUT)

GPIO.setup(Motor2A,GPIO.OUT)  # All pins as Outputs
GPIO.setup(Motor2B,GPIO.OUT)
GPIO.setup(kecepatan2,GPIO.OUT)

# --------------------------------------------------------------------
# MAIN FUNCTIONS
# --------------------------------------------------------------------

def Serial() :
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    ser.write(b'1')
    
def Picture() :
    print('[INFO]  Sedang Mengambil Gambar...')
    GPIO.output(flash, GPIO.HIGH)
    cam_port = 0   
    cap = cv2.VideoCapture(cam_port)
    cap.read()
    time.sleep(0.3)
    img_name = "/home/pi/projek/tflite1/Fix/ObjectDetection.jpg"
    ret, img = cap.read() 
    cv2.imwrite(img_name, img) 
    cap.release()
    GPIO.output(flash, GPIO.LOW)
    
    MODEL_NAME = '/home/pi/projek/tflite1/tflite_model'  #/home/pi/projek/tflite1/Sample_TFLite_model
    GRAPH_NAME = 'detect.tflite'
    LABELMAP_NAME = 'labelmap.txt'
    min_conf_threshold = float(0.7)
    print('[INFO]  Sedang Mendeteksi...')

# Parse input image name and directory. 
    IM_NAME = img_name

# Import TensorFlow libraries
# If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
    from tensorflow.lite.python.interpreter import Interpreter

# Get path to current working directory
    CWD_PATH = os.getcwd()

# Define path to images and grab all image filenames

    PATH_TO_IMAGES = os.path.join(CWD_PATH,IM_NAME)
    images = glob.glob(PATH_TO_IMAGES)

# Path to .tflite file, which contains the model that is used for object detection
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

# Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

# Load the label map
    with open(PATH_TO_LABELS, 'r') as f:
        labels = [line.strip() for line in f.readlines()]
    
# Load the Tensorflow Lite model.
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

    interpreter.allocate_tensors()

# Get model details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]

    floating_model = (input_details[0]['dtype'] == np.float32)

    input_mean = 127.5
    input_std = 127.5



# Loop over every image and perform detection
    for image_path in images:

    # Load image and resize to expected shape [1xHxWx3]
        image = cv2.imread(image_path)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        imH, imW, _ = image.shape 
        image_resized = cv2.resize(image_rgb, (width, height))
        input_data = np.expand_dims(image_resized, axis=0)

    # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

    # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()
    
    # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[0]['index'])[0] # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[1]['index'])[0] # Class index of detected objects
        scores = interpreter.get_tensor(output_details[2]['index'])[0] # Confidence of detected objects
        num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)
        
        Nilai_Tertinggi = -1
        for the_num in scores :
            if the_num > Nilai_Tertinggi:
                Nilai_Tertinggi = the_num
                if ((Nilai_Tertinggi > min_conf_threshold) and (Nilai_Tertinggi < 1.0)):
                    print('[INFO]  Terdeteksi')
                    print('[INFO]  Relay OFF')
                    GPIO.output(led, GPIO.LOW)
                                        
                else :
                    Serial()
                    print('[INFO]  Tidak Terdeteksi')
                    print('[INFO]  Relay ON')
                    GPIO.output(led, GPIO.HIGH)
                    
                        
                    
        print('[INFO]  Akurasi : ',Nilai_Tertinggi*100 , ' %')
        print('===============================================')
    # Loop over all detections and draw detection box if confidence is above minimum threshold
       
        for i in range(len(scores)):
           
            if ((scores[i] > min_conf_threshold) and (scores[i] < 1.0)): 
            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))
            
                cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

            # Draw label
                object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(image, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(image, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text
       
       # cv2.imshow('Label-Detector', cv2.resize(image,(320,320)))
       # if cv2.waitKey(5) :
        #    break


def konveyor_off ():
    GPIO.output(kecepatan1,GPIO.LOW)
    GPIO.output(Motor1A,GPIO.LOW)

    GPIO.output(kecepatan2,GPIO.LOW)
    GPIO.output(Motor2A,GPIO.LOW)
    time.sleep(1)
    
def distance ():
    
    distance = 0
    duration = 0
    
# menghidupkan konveyor
    GPIO.output(Motor1A,GPIO.HIGH)
    GPIO.output(Motor1B,GPIO.LOW)
    GPIO.output(kecepatan1,GPIO.HIGH)

    GPIO.output(Motor2A,GPIO.HIGH)
    GPIO.output(Motor2B,GPIO.LOW)
    GPIO.output(kecepatan2,GPIO.HIGH)
    
 # send trigger
    GPIO.output(triggerPIN, 0)
    time.sleep(0.000002)
    GPIO.output(triggerPIN, 1)
    time.sleep(0.000010)
    GPIO.output(triggerPIN, 0)
    time.sleep(0.000002)
    

 # wait for echo reading
    while GPIO.input(echoPIN) == 0: pass
    startT = time.time()
    while GPIO.input(echoPIN) == 1: pass
    feedbackT = time.time()
 # calculating distance
    if feedbackT == startT:
        distance = "N/A"
    else:
        duration = feedbackT - startT
        soundSpeed = 34300 # cm/s
        distance = duration * soundSpeed / 2
        distance = round(distance, 1)
        
        if distance < 7 :
            konveyor_off()
            Picture()
            konveyor_off()
        #else :
         #   cam_port = 0   
          #  cap = cv2.VideoCapture(cam_port)
           # cap.read()
    time.sleep(0.1)
    return distance

# --------------------------------------------------------------------
# MAIN LOOP
# --------------------------------------------------------------------

try:

    while True:
        print ("[INFO]  Distance: " + str(distance())+ " cm  ", end='\r')
            
except KeyboardInterrupt:
    
    print('interrupted!')
    GPIO.cleanup()
    cv2.destroyAllWindows()
    

