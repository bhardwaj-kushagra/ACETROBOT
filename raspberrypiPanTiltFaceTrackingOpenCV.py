#pigpio module for servo instead of RPi.GPIO in Raspberry pi avoids jittering.
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np 
import pickle
import RPi.GPIO as GPIO
import pigpio
from time import sleep
from numpy import interp
import argparse

args = argparse.ArgumentParser()
args.add_argument('-t', '--trained', default='n')
args = args.parse_args()

if args.trained == 'y':
	recognizer = cv2.face.LBPHFaceRecognizer_create()
	recognizer.read("trainer.yml")
	with open('labels', 'rb') as f:
		dicti = pickle.load(f)
		f.close()

panServo = 2
tiltServo = 3

panPos = 1250
tiltPos = 1250

servo = pigpio.pi()
servo.set_servo_pulsewidth(panServo, panPos)
servo.set_servo_pulsewidth(tiltServo, tiltPos)

minMov = 30
maxMov = 100

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))

faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

def movePanTilt(x, y, w, h):
	global panPos
	global tiltPos
	cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
	if int(x+(w/2)) > 360:
		panPos = int(panPos - interp(int(x+(w/2)), (360, 640), (minMov, maxMov)))
	elif int(x+(w/2)) < 280:
		panPos = int(panPos + interp(int(x+(w/2)), (280, 0), (minMov, maxMov)))
	
	if int(y+(h/2)) > 280:
		tiltPos = int(tiltPos + interp(int(y+(h/2)), (280, 480), (minMov, maxMov)))
	elif int(y+(h/2)) < 200:
		tiltPos = int(tiltPos - interp(int(y+(h/2)), (200, 0), (minMov, maxMov)))
	
	if not panPos > 2500 or not panPos < 500:
		servo.set_servo_pulsewidth(panServo, panPos)
	
	if not tiltPos > 2500 or tiltPos < 500:
		servo.set_servo_pulsewidth(tiltServo, tiltPos)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	frame = frame.array
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	faces = faceCascade.detectMultiScale(gray, scaleFactor = 1.5, minNeighbors = 5)
	
	for (x, y, w, h) in faces:
		if args.trained == "y":
			roiGray = gray[y:y+h, x:x+w]
			id_, conf = recognizer.predict(roiGray)
			conf = int(conf)
			for name, value in dicti.items():
				if value == id_:
					print(name)

			if conf < 70:
				cv2.putText(frame, name + str(conf), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0 ,255), 2,cv2.LINE_AA)
				movePanTilt(x, y, w, h)

		else:
			movePanTilt(x, y, w, h)

	cv2.imshow('frame', frame)
	key = cv2.waitKey(1)

	rawCapture.truncate(0)

	if key == 27:
		break

cv2.destroyAllWindows()