import os
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np 
from PIL import Image 
import pickle
import argparse

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))

faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

args = argparse.ArgumentParser()
args.add_argument('-n', '--name', required = True)
args = args.parse_args()

name = args.name
dirName = "./images/" + name

if not os.path.exists(dirName):
	os.makedirs(dirName)
	print("Directory Created")
	count = 1
	maxCount = 30
else:
	print("Name already exists, Adding more images")
	count = (len([name for name in os.listdir(dirName) if os.path.isfile(os.path.join(dirName, name))])) + 1
	maxCount = count + 29

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	if count > maxCount:
		break
	frame = frame.array
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	faces = faceCascade.detectMultiScale(gray, scaleFactor = 1.5, minNeighbors = 5)
	for (x, y, w, h) in faces:
		roiGray = gray[y:y+h, x:x+w]
		fileName = dirName + "/" + name + str(count) + ".jpg"
		cv2.imwrite(fileName, roiGray)
		cv2.imshow("face", roiGray)
		cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
		count += 1

	cv2.imshow('frame', frame)
	key = cv2.waitKey(1)
	rawCapture.truncate(0)

	if key == 27:
		break

recognizer = cv2.face.LBPHFaceRecognizer_create()

baseDir = os.path.dirname(os.path.abspath(__file__))
imageDir = os.path.join(baseDir, "images")

currentId = 1
labelIds = {}
yLabels = []
xTrain = []

print('Training', end='', flush=True)
for root, dirs, files in os.walk(imageDir):
	for file in files:
		print('.', end='', flush=True)
		if file.endswith("png") or file.endswith("jpg"):
			path = os.path.join(root, file)
			label = os.path.basename(root)

			if not label in labelIds:
				labelIds[label] = currentId
				currentId += 1

			id_ = labelIds[label]
			pilImage = Image.open(path).convert("L")
			imageArray = np.array(pilImage, "uint8")
			faces = faceCascade.detectMultiScale(imageArray, scaleFactor=1.1, minNeighbors=5)

			for (x, y, w, h) in faces:
				roi = imageArray[y:y+h, x:x+w]
				xTrain.append(roi)
				yLabels.append(id_)

with open("labels", "wb") as f:
	pickle.dump(labelIds, f)
	f.close()

recognizer.train(xTrain, np.array(yLabels))
recognizer.save("trainer.yml")
print()
print(labelIds)
print('Done')
cv2.destroyAllWindows()