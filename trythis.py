#!/usr/bin/env python3
"""
QUICK MOTOR TEST
Both motors spin 20° clockwise, pause 2s, then counterclockwise
"""

import RPi.GPIO as GPIO
import time

class QuickMotorTest:
    def __init__(self):
        # GPIO pins for shift register
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS (Pin 14)
        
        # Motor calibration
        self.AZIMUTH_STEPS_PER_REV = 1024    # Your fast motor
        self.ALTITUDE_STEPS_PER_REV = 4096   # Standard motor
        self.ALTITUDE_SPEED_FACTOR = 4       # Moves 4× faster
        
        # Position tracking
        self.azimuth_step = 0
        self.altitude_step = 0
        
        # Simple 4-step sequence (full-step)
        self.STEP_SEQUENCE = [
            0b00010001,  # Phase 1: Az=1, Alt=1
            0b00100010,  # Phase 2: Az=2, Alt=2
            0b01000100,  # Phase 3: Az=4, Alt=4
            0b10001000   # Phase 4: Az=8, Alt=8
        ]
        
        self.setup_gpio()
    
    def setup_gpio(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Initialize to LOW
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        print("✓ GPIO initialized")
    
    def shift_out(self, data_byte):
        """Send 8 bits to shift register"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        for i in range(7, -1, -1):
            bit = (data_byte >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.0001)  # Very short delay
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors with current steps"""
        combined = self.STEP_SEQUENCE[self.azimuth_step % 4] | self.STEP_SEQUENCE[self.altitude_step % 4]
        self.shift_out(combined)
    
    def step_motors(self, az_direction, alt_direction):
        """Take one step for each motor"""
        # Azimuth motor
        if az_direction > 0:
            self.azimuth_step = (self.azimuth_step + 1) % 4
        else:
            self.azimuth_step = (self.azimuth_step - 1) % 4
        
        # Altitude motor (moves faster)
        if alt_direction > 0:
            self.altitude_step = (self.altitude_step + 1) % 4
        else:
            self.altitude_step = (self.altitude_step - 1) % 4
        
        self.update_motors()
    
    def move_degrees(self, az_degrees, alt_degrees, delay=0.005):
        """Move both motors by specified degrees"""
        # Calculate steps needed
        az_steps = int(abs(az_degrees) * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(abs(alt_degrees) * self.ALTITUDE_STEPS_PER_REV / 360)
        
        # Determine directions
        az_dir = 1 if az_degrees >= 0 else -1
        alt_dir = 1 if alt_degrees >= 0 else -1
        
        print(f"Moving: Az={az_degrees:.1f}° ({az_steps} steps), Alt={alt_degrees:.1f}° ({alt_steps} steps)")
        
        # Move both motors - do them separately but quickly
        max_steps = max(az_steps, alt_steps // self.ALTITUDE_SPEED_FACTOR)
        
        az_counter = 0
        alt_counter = 0
        
        for i in range(max_steps):
            if az_counter < az_steps:
                self.step_motors(az_dir, 0)  # Only azimuth
                az_counter += 1
            
            # Altitude moves faster, so check more frequently
            if alt_counter < alt_steps:
                # Move altitude 4 steps for every 1 main loop (since it's 4× faster)
                for _ in range(min(self.ALTITUDE_SPEED_FACTOR, alt_steps - alt_counter)):
                    self.step_motors(0, alt_dir)  # Only altitude
                    alt_counter += 1
            
            time.sleep(delay)
    
    def run_test(self):
        """Run the clockwise/counterclockwise test"""
        print("\n" + "="*60)
        print("QUICK MOTOR TEST")
        print("="*60)
        print("Test sequence:")
        print("1. Both motors 20° CLOCKWISE")
        print("2. Pause 2 seconds")
        print("3. Both motors 20° COUNTERCLOCKWISE")
        print("4. Pause 2 seconds")
        print("5. Repeat 3 times")
        print("="*60)
        
        input("Press Enter to start test...")
        
        test_cycles = 3
        
        for cycle in range(test_cycles):
            print(f"\n--- Test Cycle {cycle + 1}/{test_cycles} ---")
            
            # Clockwise (positive degrees)
            print(f"Moving 20° CLOCKWISE...")
            self.move_degrees(20, 20, 0.003)
            print("Pausing 2 seconds...")
            time.sleep(2)
            
            # Counterclockwise (negative degrees)
            print(f"Moving 20° COUNTERCLOCKWISE...")
            self.move_degrees(-20, -20, 0.003)
            
            if cycle < test_cycles - 1:  # Don't pause after last cycle
                print("Pausing 2 seconds...")
                time.sleep(2)
        
        print(f"\n✓ Test complete! {test_cycles} cycles finished.")
        
        # Turn off motors
        self.shift_out(0b00000000)
        print("Motors turned off.")
    
    def cleanup(self):
        """Clean up GPIO"""
        self.shift_out(0b00000000)
        GPIO.cleanup()
        print("GPIO cleanup complete.")

def main():
    """Main function"""
    print("="*70)
    print("QUICK MOTOR TEST - 20° CLOCKWISE/COUNTERCLOCKWISE")
    print("="*70)
    print("This will make both motors:")
    print("  1. Spin 20° clockwise")
    print("  2. Pause 2 seconds")
    print("  3. Spin 20° counterclockwise")
    print("  4. Pause 2 seconds")
    print("  5. Repeat 3 times")
    print("="*70)
    
    # Quick wiring check
    print("\nWiring check:")
    print("  RPi GPIO 11  →  74HC595 Pin 11 (SH_CP)")
    print("  RPi GPIO 10  →  74HC595 Pin 12 (ST_CP)")
    print("  RPi GPIO 9   →  74HC595 Pin 14 (DS)")
    print("  RPi 5V/3.3V  →  74HC595 Pin 16 (VCC)")
    print("  RPi GND      →  74HC595 Pins 8 & 13 (GND & OE)")
    print("\nMotors should be connected to:")
    print("  Azimuth:  Pins 15, 1, 2, 3")
    print("  Altitude: Pins 4, 5, 6, 7")
    print("="*70)
    
    input("Press Enter when ready (Ctrl+C to cancel)...")
    
    tester = None
    try:
        tester = QuickMotorTest()
        tester.run_test()
        
    except KeyboardInterrupt:
        print("\nTest cancelled by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if tester:
            tester.cleanup()
        print("\nTest ended.")

if __name__ == "__main__":
    main()
