"""
Red Object Tracking Car with Raspberry Pi and OpenCV
-----------------------------------------------------

This script controls a simple autonomous car that uses computer vision to detect and follow a red object.
The camera captures video frames, processes them to find red-colored objects, and moves the car accordingly.

Hardware Requirements:
----------------------
- Raspberry Pi (any model with GPIO and camera support, e.g., Raspberry Pi 4)
- Raspberry Pi Camera Module or USB Webcam
- L298N Motor Driver Module (or similar)
- 2 DC Motors (one for each side/wheel)
- External Power Supply for Motors
- Jumper Wires and Breadboard (or custom wiring setup)

Motor GPIO Pin Configuration:
-----------------------------
- MOTOR_A_IN1_PIN: GPIO 16 (Left motor input 1)
- MOTOR_A_IN2_PIN: GPIO 18 (Left motor input 2)
- MOTOR_B_IN3_PIN: GPIO 22 (Right motor input 1)
- MOTOR_B_IN4_PIN: GPIO 24 (Right motor input 2)

Software Requirements:
-----------------------
- Python 3.x
- OpenCV (cv2)
- NumPy
- RPi.GPIO

Functionality:
--------------
- Captures video from the camera.
- Detects the largest red object in the frame using HSV color masking.
- Calculates the horizontal offset from the center of the frame.
- Moves the car forward if the object is centered.
- Slightly turns the car left or right if the object is off-center.
- Stops the car if no red object is detected.
- Press 'q' to exit the program.

Note:
-----
Adjust the GPIO pin numbers and HSV red color thresholds as needed to match your hardware and lighting conditions.
"""



import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# Motor control GPIO pins (CHANGE THESE TO MATCH YOUR WIRING)
MOTOR_A_IN1_PIN = 16  # Example
MOTOR_A_IN2_PIN = 18  # Example
MOTOR_B_IN3_PIN = 22  # Example
MOTOR_B_IN4_PIN = 24  # Example

# Camera frame dimensions
FRAME_WIDTH = 640
FRAME_HEIGHT = 480  # ADDED THIS LINE
CENTER_X = FRAME_WIDTH // 2

# Steering threshold (how many pixels off-center to trigger turning)
CENTER_THRESHOLD = 640

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # To prevent "channel already in use" warnings if you run multiple times

# Setup GPIO pins as outputs
GPIO.setup(MOTOR_A_IN1_PIN, GPIO.OUT)
GPIO.setup(MOTOR_A_IN2_PIN, GPIO.OUT)
GPIO.setup(MOTOR_B_IN3_PIN, GPIO.OUT)
GPIO.setup(MOTOR_B_IN4_PIN, GPIO.OUT)

def stop_car():
    GPIO.output(MOTOR_A_IN1_PIN, GPIO.LOW)
    GPIO.output(MOTOR_A_IN2_PIN, GPIO.LOW)
    GPIO.output(MOTOR_B_IN3_PIN, GPIO.LOW)
    GPIO.output(MOTOR_B_IN4_PIN, GPIO.LOW)
    print("Stopping")

def move_forward():
    GPIO.output(MOTOR_A_IN1_PIN, GPIO.HIGH)
    GPIO.output(MOTOR_A_IN2_PIN, GPIO.LOW)
    GPIO.output(MOTOR_B_IN3_PIN, GPIO.HIGH)
    GPIO.output(MOTOR_B_IN4_PIN, GPIO.LOW)
    print("Moving Forward")

def turn_left():
    GPIO.output(MOTOR_A_IN1_PIN, GPIO.LOW)
    GPIO.output(MOTOR_A_IN2_PIN, GPIO.HIGH)
    GPIO.output(MOTOR_B_IN3_PIN, GPIO.HIGH)
    GPIO.output(MOTOR_B_IN4_PIN, GPIO.LOW)
    print("Turning Left")

def turn_right():
    GPIO.output(MOTOR_A_IN1_PIN, GPIO.HIGH)
    GPIO.output(MOTOR_A_IN2_PIN, GPIO.LOW)
    GPIO.output(MOTOR_B_IN3_PIN, GPIO.LOW)
    GPIO.output(MOTOR_B_IN4_PIN, GPIO.HIGH)
    print("Turning Right")

cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        lower_red = np.array([170, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        red_mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y, w, h) = cv2.boundingRect(largest_contour)
            object_x = x + w // 2
            error_x = object_x - CENTER_X

            if abs(error_x) < CENTER_THRESHOLD:
                move_forward()
            
            elif error_x > CENTER_THRESHOLD:
                move_forward()
                time.sleep(0.2)
            else:  # error_x < -CENTER_THRESHOLD
                move_forward()
                time.sleep(0.2)
        else:
            stop_car()

        # Optional: Display the frame with a center line
        cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (0, 255, 0), 1)
        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", red_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopping car and cleaning up GPIO...")
finally:
    cap.release()
    cv2.destroyAllWindows()
    stop_car()
    GPIO.cleanup()
