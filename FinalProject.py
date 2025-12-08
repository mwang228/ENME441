#!/usr/bin/env python3
"""
Quick motor test - just makes the motor move back and forth
"""

import RPi.GPIO as GPIO
import time

# Pins for vertical motor
IN1 = 24  # GPIO24
IN2 = 25  # GPIO25  
IN3 = 5   # GPIO5
IN4 = 6   # GPIO6

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Stepping sequence
seq = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

def step(step_num):
    """Activate coils for this step"""
    GPIO.output(IN1, seq[step_num][0])
    GPIO.output(IN2, seq[step_num][1]) 
    GPIO.output(IN3, seq[step_num][2])
    GPIO.output(IN4, seq[step_num][3])

try:
    print("Starting motor test...")
    print("Motor should move forward 512 steps, then backward 512 steps")
    print("Press Ctrl+C to stop")
    
    # Move forward
    print("\nMoving FORWARD...")
    for i in range(512):
        step(i % 8)
        time.sleep(0.005)
    
    # Move backward  
    print("\nMoving BACKWARD...")
    for i in range(512):
        step((512 - i) % 8)
        time.sleep(0.005)
    
    # Turn off coils
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 0)
    
    print("\nTest complete!")

except KeyboardInterrupt:
    print("\nTest stopped by user")
finally:
    # Clean up
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 0)
    GPIO.cleanup()
    print("GPIO cleaned up")
