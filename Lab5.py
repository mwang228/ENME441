import RPi.GPIO as GPIO
import time
import math
import threading

class LEDWaveController:
    def __init__(self):
        # LED GPIO pins (modify these based on your actual wiring)
        self.led_pins = [17, 18, 27, 22, 23, 24, 25, 5, 6, 13, 19, 26]
        self.button_pin = 2  # Modify based on your button wiring
        
        # PWM parameters
        self.pwm_freq = 500  # 500 Hz base frequency
        self.wave_freq = 0.2  # 0.2 Hz wave frequency
        self.phase_increment = math.pi / 11  # π/11 rad phase increment
        
        # Wave direction control
        self.direction = 1  # 1 for forward, -1 for backward
        self.direction_lock = threading.Lock()
        
        # PWM objects and setup
        self.pwm_objects = []
        self.setup_gpio()
        
    def setup_gpio(self):
        """Initialize GPIO pins and PWM objects"""
        GPIO.setmode(GPIO.BCM)
        
        # Setup LED pins as outputs and create PWM objects
        for pin in self.led_pins:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, self.pwm_freq)
            pwm.start(0)  # Start with 0% duty cycle
            self.pwm_objects.append(pwm)
        
        # Setup button pin with pull-down resistor
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
    def calculate_brightness(self, led_index, current_time):
        """Calculate brightness for a specific LED at given time"""
        phase_offset = led_index * self.phase_increment * self.direction
        brightness = (math.sin(2 * math.pi * self.wave_freq * current_time - phase_offset)) ** 2
        # Convert to percentage (0-100)
        return brightness * 100
    
    def update_leds(self):
        """Continuously update LED brightness based on wave pattern"""
        try:
            while True:
                current_time = time.time()
                
                # Update each LED
                for i, pwm in enumerate(self.pwm_objects):
                    brightness = self.calculate_brightness(i, current_time)
                    pwm.ChangeDutyCycle(brightness)
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            self.cleanup()
    
    def button_callback(self, channel):
        """Threaded callback for button press - reverses wave direction"""
        # Debounce - ignore rapid successive presses
        time.sleep(0.05)
        if GPIO.input(self.button_pin):
            with self.direction_lock:
                self.direction *= -1
            print(f"Direction changed to: {'forward' if self.direction == 1 else 'backward'}")
    
    def setup_button_callback(self):
        """Setup threaded callback for button press"""
        GPIO.add_event_detect(self.button_pin, GPIO.RISING, 
                            callback=self.button_callback, 
                            bouncetime=300)
    
    def run(self):
        """Main function to run the LED wave controller"""
        print("LED Wave Controller Started")
        print("Press the button to reverse wave direction")
        print("Press Ctrl+C to exit")
        
        self.setup_button_callback()
        self.update_leds()
    
    def cleanup(self):
        """Clean up GPIO resources"""
        for pwm in self.pwm_objects:
            pwm.stop()
        GPIO.cleanup()
        print("\nGPIO cleaned up. Program exited.")

# Alternative version without classes (simpler approach)
def simple_led_wave():
    """Simpler version without classes"""
    
    # Configuration
    led_pins = [17, 18, 27, 22, 23, 24, 25, 5, 6, 13, 19, 26]
    button_pin = 2
    
    # Global variables for direction control
    direction = 1
    
    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    
    # Setup LEDs
    pwms = []
    for pin in led_pins:
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, 500)
        pwm.start(0)
        pwms.append(pwm)
    
    # Setup button
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    def button_pressed(channel):
        nonlocal direction
        direction *= -1
        print(f"Direction: {'→' if direction == 1 else '←'}")
    
    GPIO.add_event_detect(button_pin, GPIO.RISING, 
                         callback=button_pressed, bouncetime=300)
    
    try:
        print("LED wave running. Press button to change direction. Ctrl+C to exit.")
        while True:
            t = time.time()
            f = 0.2  # 0.2 Hz
            
            for i, pwm in enumerate(pwms):
                phase = i * (math.pi / 11) * direction
                brightness = (math.sin(2 * math.pi * f * t - phase)) ** 2
                pwm.ChangeDutyCycle(brightness * 100)
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        for pwm in pwms:
            pwm.stop()
        GPIO.cleanup()
        print("Program exited.")

if __name__ == "__main__":
    # Choose which version to run:
    
    # Option 1: Class-based version (recommended)
    controller = LEDWaveController()
    controller.run()
    
    # Option 2: Simple version
    # simple_led_wave()
