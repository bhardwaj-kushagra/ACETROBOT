
# We imports the GPIO module
import RPi.GPIO as GPIO
# We import the command sleep from time
from time import sleep

# Stops all warnings from appearing
GPIO.setwarnings(False)
GPIO.cleanup()

# We name all the pins on BOARD mode
GPIO.setmode(GPIO.BOARD)
# Set an output for the PWM Signal
GPIO.setup(18, GPIO.OUT)

# Set up the PWM on pin #16 at 50Hz
pwm = GPIO.PWM(18, 50)
pwm.start(0) # Start the servo with 0 duty cycle ( at 0 deg position )

while(True):
    pwm.ChangeDutyCycle(float(input('pwm ')))
    sleep(0.1)
pwm.stop(0) # Stop the servo with 0 duty cycle ( at 0 deg position )
GPIO.cleanup() # Clean up all the ports we've used.