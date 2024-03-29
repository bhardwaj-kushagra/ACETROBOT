import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

def pan(angle):
    duty = (angle / 18) + 2
    GPIO.output(18, True)
    pwm = GPIO.PWM(18, 50)
    pwm.start(duty)
    time.sleep(1)
    pwm.stop()

def tilt(angle):
    duty = (angle / 18) + 2
    GPIO.output(12, True)
    pwm = GPIO.PWM(12, 50)
    pwm.start(duty)
    time.sleep(1)
    pwm.stop()

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)

face_cascade = cv2.CascadeClassifier('/home/kushagra/opencv-4.4.0/data/haarcascades/haarcascade_frontalface_default.xml')

video_capture = cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

    for (x, y, w, h) in faces:
        #(x, y, w, h) = faces[0] #uncomment later
        face_center_x = x + w // 2
        face_center_y = y + h // 2
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        dx = face_center_x - frame_center_x
        dy = face_center_y - frame_center_y

        # Calculate pan and tilt angles based on the differences
        pan_angle = int(90 + dx * 0.1)  # Adjust the multiplier to control sensitivity
        tilt_angle = int(90 + dy * 0.1)  # Adjust the multiplier to control sensitivity
        #cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        pan(pan_angle)
        tilt(tilt_angle)

    frame = cv2.flip(frame, 0)

    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()

cv2.destroyAllWindows()

GPIO.cleanup()