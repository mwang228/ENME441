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
pwm_states = [] 
temp = False
pwm_period = 1.0 / basef
last_pwm_time = 0

# setup LED pins
def setup():
    for pin in ledPin:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
        pwm_states.append({'pin': pin, 'duty_cycle': 0, 'on_time': 0})
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

def manual_pwm_update():
    global last_pwm_time
    current_time = time.time()
    
    # Only update at PWM frequency (500 Hz)
    if current_time - last_pwm_time >= pwm_period:
        last_pwm_time = current_time
        
        # Update each LED based on its duty cycle
        for led in pwm_states:
            # Calculate if LED should be on or off for this PWM cycle
            cycle_position = (current_time * basef) % 1.0  # Position in current cycle (0-1)
            
            if cycle_position < (led['duty_cycle'] / 100.0):
                GPIO.output(led['pin'], GPIO.HIGH)  # Turn LED on
            else:
                GPIO.output(led['pin'], GPIO.LOW)   # Turn LED off

def update_leds():
    while True:
        check_direction()
        
        for i, led in enumerate(pwm_states):
            phase_offset = i * theta
            duty_cycle = brightness(phase_offset)
            led['duty_cycle'] = duty_cycle

        manual_pwm_update()

def main():
    setup()
    global temp
    temp = read_jumper()
    initial_direction = temp
    update_leds()
    
main()
