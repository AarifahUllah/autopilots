import RPi.GPIO as GPIO
import time

# Use BCM mode to match the GPIO numbering
GPIO.setmode(GPIO.BCM)

# Define pin numbers based on BCM
IN1 = 4    # Forward
IN2 = 5    # Backward
PWM = 6    # PWM Speed Control

# Set up GPIO
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)

# Set up PWM on PWM pin at 1000Hz
motorSpeed = GPIO.PWM(PWM, 1000)
motorSpeed.start(0)

def forward(speed):
    print(f"Going forward at {speed}%")
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    motorSpeed.ChangeDutyCycle(speed)

def backward(speed):
    print(f"Going backward at {speed}%")
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    motorSpeed.ChangeDutyCycle(speed)
    
def stop():
    print("Stopping motor")
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    motorSpeed.ChangeDutyCycle(0)
    time.sleep(1)

try:
    forward(50)
    time.sleep(3)
    stop()
    backward(75)
    time.sleep(3)
    stop()
finally:
    motorSpeed.stop()
    GPIO.cleanup()
