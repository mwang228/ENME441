import RPi.GPIO as GPIO
import time
import math
import threading

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
ledPin = [17, 18, 27, 22, 23]
jumper = 24

f = 0.2 
basef = 500 
theta = math.pi / 5 
direction = 1  # 1 for forward, -1 for reverse
pwm_objects = []
temp = False

# setup LED pins
def setup():
    for pin in ledPin:
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, basef)
        pwm.start(0)  # start with 0% duty cycle
        pwm_objects.append(pwm)
    GPIO.setup(jumper, GPIO.IN)

def read_jumper():
    return GPIO.input(jumper)

def check_direction():
    global direction, temp
    current = read_jumper()
    
    # check if jumper changed
    if current != temp:
        if current:  # connected to 3.3V
            direction = -1  # reverse direction
        else:  # disconnected
            direction = 1   # forward direction   
        temp = current

def brightness(phase_offset):
    current_time = time.time()
    phase = 2 * math.pi * f * current_time + (phase_offset * direction)
    brightness = (math.sin(phase)) ** 2
    duty_cycle = brightness * 100
    return duty_cycle

def update_leds():
    while True:
        check_direction()
        
        for i, pwm in enumerate(pwm_objects):
            phase_offset = i * theta
            duty_cycle = brightness(phase_offset)
            pwm.ChangeDutyCycle(duty_cycle)

def main():
    setup()
    global temp
    temp = read_jumper()
    initial_direction = temp
    update_leds()
    
main()
