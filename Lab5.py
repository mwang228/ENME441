import RPi.GPIO as GPIO
import time
import math
import threading

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LED pins (using 5 LEDs instead of 12)
LED_PINS = [17, 18, 27, 22, 23]  # Adjust these pins according to your wiring

# Jumper wire pin for direction control
JUMPER_PIN = 24  # Use any free GPIO pin, connect jumper to 3.3V to change direction

# Parameters
FREQUENCY = 0.2  # Hz
BASE_PWM_FREQ = 500  # Hz
PHASE_INCREMENT = math.pi / 5  # rad

# Global variables
direction = 1  # 1 for forward, -1 for reverse
pwm_objects = []
last_jumper_state = False

def setup_gpio():
    """Initialize all GPIO pins"""
    # Setup LED pins
    for pin in LED_PINS:
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, BASE_PWM_FREQ)
        pwm.start(0)  # Start with 0% duty cycle (off)
        pwm_objects.append(pwm)
    
    # Setup jumper pin with pull-down resistor
    GPIO.setup(JUMPER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def read_jumper_state():
    """Read the current state of the jumper wire"""
    return GPIO.input(JUMPER_PIN)  # Returns True (3.3V) or False (0V)

def check_direction_change():
    """Check if jumper state has changed and update direction accordingly"""
    global direction, last_jumper_state
    
    current_state = read_jumper_state()
    
    # If jumper state changed (connected/disconnected to 3.3V)
    if current_state != last_jumper_state:
        if current_state:  # Jumper connected to 3.3V
            direction = -1  # Reverse direction
            print("Direction: REVERSE (jumper connected to 3.3V)")
        else:  # Jumper disconnected (0V)
            direction = 1   # Forward direction
            print("Direction: FORWARD (jumper disconnected)")
        
        last_jumper_state = current_state

def calculate_brightness(phase_offset):
    """Calculate brightness based on time and phase offset"""
    current_time = time.time()
    phase = 2 * math.pi * FREQUENCY * current_time + (phase_offset * direction)
    brightness = (math.sin(phase)) ** 2
    # Convert to duty cycle (0-100)
    duty_cycle = brightness * 100
    return duty_cycle

def update_leds():
    """Update all LED brightness values"""
    while True:
        # Check for direction changes
        check_direction_change()
        
        # Update LED brightness
        for i, pwm in enumerate(pwm_objects):
            # Calculate phase offset for each LED
            phase_offset = i * PHASE_INCREMENT
            duty_cycle = calculate_brightness(phase_offset)
            pwm.ChangeDutyCycle(duty_cycle)
        
        # Small delay to prevent excessive CPU usage
        time.sleep(0.01)

def cleanup():
    """Clean up GPIO resources"""
    for pwm in pwm_objects:
        pwm.stop()
    GPIO.cleanup()

def main():
    try:
        print("Setting up GPIO...")
        setup_gpio()
        print(f"Running LED wave with {len(LED_PINS)} LEDs")
        print("Frequency: 0.2 Hz")
        print(f"Jumper control on GPIO {JUMPER_PIN}")
        print("Direction control:")
        print("  - Jumper disconnected (0V): Forward wave")
        print("  - Jumper connected to 3.3V: Reverse wave")
        print("Press Ctrl+C to exit")
        
        # Initialize jumper state
        global last_jumper_state
        last_jumper_state = read_jumper_state()
        initial_direction = "REVERSE" if last_jumper_state else "FORWARD"
        print(f"Initial direction: {initial_direction}")
        
        # Start the LED update loop
        update_leds()
        
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
