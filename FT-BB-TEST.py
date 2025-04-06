"""
Face Tracking with Pan-Tilt Servo Control using Raspberry Pi and OpenCV
------------------------------------------------------------------------

This script captures live video, detects faces in the frame, and controls two servos (pan and tilt)
to move a camera (or object) to face the detected person.

Hardware Requirements:
----------------------
- Raspberry Pi (with GPIO support)
- USB Webcam or Raspberry Pi Camera Module
- 2x Servo Motors (for pan and tilt)
- Pan-Tilt Mount Bracket (optional but recommended)
- External power supply for servos (recommended)
- Jumper wires

GPIO Pin Configuration:
-----------------------
- Servo for Pan: GPIO 18
- Servo for Tilt: GPIO 12

Software Requirements:
-----------------------
- Python 3.x
- OpenCV (cv2)
- NumPy
- RPi.GPIO

Functionality:
--------------
- Uses Haar Cascade Classifier to detect faces in the video frame.
- On detecting a face, sends a signal to both servos to move to a fixed angle (90 degrees).
  (Note: You can improve this by calculating angle based on face position in frame.)
- Displays the live video feed with detected face marked by a rectangle.
- Press 'q' to quit the program.

Notes:
------
- You may need to calibrate angles and duty cycle based on your specific servo motor.
- Make sure the servos have time to move before the signal is stopped.
- Face detection model path may need to be adjusted based on OpenCV install location.
"""



import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

def pan(angle):
    GPIO.setup(18, GPIO.OUT)
    duty = (angle / 18) + 5
    GPIO.output(18, True)
    pwm = GPIO.PWM(18, 50)
    pwm.start(duty)
    time.sleep(1)
    pwm.stop()

def tilt(angle):
    GPIO.setup(12, GPIO.OUT)
    duty = (angle / 18) + 5
    GPIO.output(12, True)
    pwm = GPIO.PWM(12, 50)
    pwm.start(duty)
    time.sleep(1)
    pwm.stop()

GPIO.setmode(GPIO.BCM)

face_cascade = cv2.CascadeClassifier('/home/kushagra/opencv-4.4.0/data/haarcascades/haarcascade_frontalface_default.xml')

video_capture = cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        pan(90)
        tilt(90)
        
    frame = cv2.flip(frame, 0) 

    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()

cv2.destroyAllWindows()

GPIO.cleanup()
