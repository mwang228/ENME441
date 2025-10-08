import RPi.GPIO as GPIO
import time
import math
import threading

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LED pins (using 5 LEDs instead of 12)
LED_PINS = [17, 18, 27, 22, 23]  # Adjust these pins according to your wiring

# Parameters
FREQUENCY = 0.2  # Hz
BASE_PWM_FREQ = 500  # Hz
PHASE_INCREMENT = math.pi / 11  # rad

# Global variables
direction = 1  # 1 for forward, -1 for reverse
pwm_objects = []

def setup_leds():
    """Initialize all LEDs with PWM"""
    for pin in LED_PINS:
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, BASE_PWM_FREQ)
        pwm.start(0)  # Start with 0% duty cycle (off)
        pwm_objects.append(pwm)

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
        print("Setting up LEDs...")
        setup_leds()
        print(f"Running LED wave with {len(LED_PINS)} LEDs")
        print("Frequency: 0.2 Hz")
        print("Press Ctrl+C to exit")
        
        # Start the LED update loop
        update_leds()
        
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
