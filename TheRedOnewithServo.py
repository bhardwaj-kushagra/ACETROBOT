"""
Object Tracking with Raspberry Pi Camera and Pan-Tilt Servos

📌 Description:
Tracks a red-colored object using the Raspberry Pi camera.
Automatically adjusts two servos (pan and tilt) to follow the object in real-time.

🛠️ Requirements:
- Raspberry Pi 3B+ with Camera Module
- Pan-Tilt Servo Mechanism (2x SG90/Servo Motors)
- Libraries: opencv-python, numpy, pigpio, imutils, picamera2
- pigpiod daemon must be running (start with `sudo pigpiod`)

👨‍💻 Author: Kushagra Sharma (@bhardwaj-kushagra)
"""

import cv2
import numpy as np
import pigpio
from picamera2 import Picamera2

# Servo GPIO pins
PAN_PIN = 17  # Horizontal servo
TILT_PIN = 27  # Vertical servo

# Initialize PiCamera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Initialize pigpio for precise servo control
pi = pigpio.pi()
pan_angle = 1500  # Middle position (Servo pulse range: 1000-2000)
tilt_angle = 1500

def set_servo_position(pin, angle):
    """ Move servo to a specific position """
    pi.set_servo_pulsewidth(pin, angle)

# Move servos to center position initially
set_servo_position(PAN_PIN, pan_angle)
set_servo_position(TILT_PIN, tilt_angle)

while True:
    frame = picam2.capture_array()
    frame = cv2.flip(frame, -1)  # Flip for correct orientation
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color range for RED object detection
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours of the detected object
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)  # Largest contour
        (x, y, w, h) = cv2.boundingRect(c)

        # Get center of the object
        obj_center_x = x + w // 2
        obj_center_y = y + h // 2

        # Calculate movement (center of frame is 320x240)
        if obj_center_x < 280:  # Move left
            pan_angle += 10
        elif obj_center_x > 360:  # Move right
            pan_angle -= 10
        
        if obj_center_y < 200:  # Move up
            tilt_angle += 10
        elif obj_center_y > 280:  # Move down
            tilt_angle -= 10
        
        # Limit servo range (1000-2000)
        pan_angle = max(1000, min(2000, pan_angle))
        tilt_angle = max(1000, min(2000, tilt_angle))

        # Move servos
        set_servo_position(PAN_PIN, pan_angle)
        set_servo_position(TILT_PIN, tilt_angle)

    # Show tracking
    cv2.imshow("Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()
pi.stop()
