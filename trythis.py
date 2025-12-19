#!/usr/bin/env python3
"""
CORRECTED MOTOR TEST - 90° BOTH DIRECTIONS
Fixed step calculations for both motors
"""

import RPi.GPIO as GPIO
import time

class CorrectMotorTest:
    def __init__(self):
        # GPIO pins for shift register
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS (Pin 14)
        
        # CORRECTED: Motor calibration (from your original code)
        # IMPORTANT: These values determine how many steps for 90°
        self.AZIMUTH_STEPS_PER_REV = 1024    # Fast motor - 1024 steps/rev
        self.ALTITUDE_STEPS_PER_REV = 4096   # Standard motor - 4096 steps/rev
        
        # Position tracking
        self.azimuth_position = 0  # Current step position
        self.altitude_position = 0
        
        # Step sequence for 4-wire bipolar stepper
        self.STEP_SEQUENCE = [
            0b00010001,  # Step 1: Az=0001 (pin 15), Alt=0001 (pin 4)
            0b00100010,  # Step 2: Az=0010 (pin 1),  Alt=0010 (pin 5)
            0b01000100,  # Step 3: Az=0100 (pin 2),  Alt=0100 (pin 6)
            0b10001000,  # Step 4: Az=1000 (pin 3),  Alt=1000 (pin 7)
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
        
        # Initialize motors to OFF
        self.shift_out(0b00000000)
        print("✓ GPIO initialized - Motors OFF")
        print(f"Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
    
    def shift_out(self, data_byte):
        """Send 8 bits to shift register"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        for i in range(7, -1, -1):
            bit = (data_byte >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def test_single_step(self):
        """Test single step movement for both motors"""
        print("\nTesting single step movements...")
        
        # Test azimuth motor
        print("\nAzimuth motor - single step forward:")
        for i in range(4):
            bits = self.STEP_SEQUENCE[i] & 0b00001111  # Azimuth bits only
            print(f"  Step {i+1}: {bits:08b} (should energize one coil)")
            self.shift_out(bits)
            input("  Press Enter for next step...")
        
        # Test altitude motor  
        print("\nAltitude motor - single step forward:")
        for i in range(4):
            bits = self.STEP_SEQUENCE[i] & 0b11110000  # Altitude bits only
            print(f"  Step {i+1}: {bits:08b} (should energize one coil)")
            self.shift_out(bits)
            input("  Press Enter for next step...")
        
        self.shift_out(0b00000000)
        print("✓ Single step test complete")
    
    def calculate_steps_for_degrees(self, degrees, is_altitude=False):
        """Calculate steps needed for given degrees"""
        if is_altitude:
            steps_per_rev = self.ALTITUDE_STEPS_PER_REV
        else:
            steps_per_rev = self.AZIMUTH_STEPS_PER_REV
        
        # Steps = (degrees / 360) × steps_per_revolution
        steps = int((degrees / 360.0) * steps_per_rev)
        return steps
    
    def step_azimuth(self, direction):
        """Take one azimuth step"""
        self.azimuth_position = (self.azimuth_position + direction) % 4
        az_bits = self.STEP_SEQUENCE[self.azimuth_position] & 0b00001111
        # Keep altitude motor in current state
        alt_bits = self.STEP_SEQUENCE[self.altitude_position % 4] & 0b11110000
        self.shift_out(az_bits | alt_bits)
    
    def step_altitude(self, direction):
        """Take one altitude step"""
        self.altitude_position = (self.altitude_position + direction) % 4
        alt_bits = self.STEP_SEQUENCE[self.altitude_position] & 0b11110000
        # Keep azimuth motor in current state
        az_bits = self.STEP_SEQUENCE[self.azimuth_position % 4] & 0b00001111
        self.shift_out(az_bits | alt_bits)
    
    def move_azimuth_degrees(self, degrees, step_delay=0.002):
        """Move azimuth motor by specific degrees"""
        steps_needed = self.calculate_steps_for_degrees(abs(degrees), False)
        direction = 1 if degrees >= 0 else -1
        
        print(f"Azimuth: {degrees:.1f}° = {steps_needed} steps")
        
        for i in range(steps_needed):
            self.step_azimuth(direction)
            time.sleep(step_delay)
            if (i + 1) % 50 == 0:
                print(f"  Step {i+1}/{steps_needed}")
        
        return True
    
    def move_altitude_degrees(self, degrees, step_delay=0.002):
        """Move altitude motor by specific degrees"""
        steps_needed = self.calculate_steps_for_degrees(abs(degrees), True)
        direction = 1 if degrees >= 0 else -1
        
        print(f"Altitude: {degrees:.1f}° = {steps_needed} steps")
        
        for i in range(steps_needed):
            self.step_altitude(direction)
            time.sleep(step_delay)
            if (i + 1) % 100 == 0:
                print(f"  Step {i+1}/{steps_needed}")
        
        return True
    
    def move_both_motors(self, az_degrees, alt_degrees, step_delay=0.002):
        """Move both motors by specific degrees"""
        print(f"\nMoving both motors:")
        print(f"  Azimuth: {az_degrees:.1f}°")
        print(f"  Altitude: {alt_degrees:.1f}°")
        
        # Calculate steps for each motor
        az_steps = self.calculate_steps_for_degrees(abs(az_degrees), False)
        alt_steps = self.calculate_steps_for_degrees(abs(alt_degrees), True)
        
        az_direction = 1 if az_degrees >= 0 else -1
        alt_direction = 1 if alt_degrees >= 0 else -1
        
        print(f"Steps: Azimuth={az_steps}, Altitude={alt_steps}")
        
        # Determine which motor needs more steps
        max_steps = max(az_steps, alt_steps)
        
        # Move both motors with correct timing
        az_completed = 0
        alt_completed = 0
        
        while az_completed < az_steps or alt_completed < alt_steps:
            # Move azimuth if needed
            if az_completed < az_steps:
                self.step_azimuth(az_direction)
                az_completed += 1
            
            # Move altitude if needed
            if alt_completed < alt_steps:
                self.step_altitude(alt_direction)
                alt_completed += 1
            
            # Sleep for step delay
            time.sleep(step_delay)
        
        print(f"✓ Movement complete")
        return True
    
    def run_precise_90_test(self):
        """Run precise 90° test"""
        print("\n" + "="*60)
        print("PRECISE 90° MOTOR TEST")
        print("="*60)
        print("Using CORRECT step calculations:")
        print(f"  90° azimuth = {self.calculate_steps_for_degrees(90, False)} steps")
        print(f"  90° altitude = {self.calculate_steps_for_degrees(90, True)} steps")
        print("\nTest sequence (1 cycle):")
        print("  1. Both motors 90° CLOCKWISE/UP")
        print("  2. Pause 2 seconds")
        print("  3. Both motors 90° COUNTERCLOCKWISE/DOWN")
        print("="*60)
        
        # Optional: Test single steps first
        test_single = input("\nTest single steps first? (y/n): ").strip().lower()
        if test_single == 'y':
            self.test_single_step()
        
        input("\nPress Enter to start 90° test...")
        
        # CLOCKWISE/UP (positive 90°)
        print(f"\n{'='*40}")
        print("MOVING 90° CLOCKWISE/UP")
        print('='*40)
        
        success = self.move_both_motors(90, 90, 0.001)
        
        if not success:
            print("⚠ Movement failed!")
            return
        
        print("⏸ Pausing 2 seconds...")
        time.sleep(2)
        
        # COUNTERCLOCKWISE/DOWN (negative 90°)
        print(f"\n{'='*40}")
        print("MOVING 90° COUNTERCLOCKWISE/DOWN")
        print('='*40)
        
        success = self.move_both_motors(-90, -90, 0.001)
        
        if not success:
            print("⚠ Movement failed!")
            return
        
        print(f"\n{'='*40}")
        print("✓ TEST COMPLETE!")
        print("  Both motors should have moved exactly 90° each way")
        print('='*40)
        
        # Turn off motors
        self.shift_out(0b00000000)
        print("Motors turned off.")
    
    def run_individual_tests(self):
        """Test each motor individually"""
        print("\n" + "="*60)
        print("INDIVIDUAL MOTOR TESTS")
        print("="*60)
        
        print("\n1. Testing AZIMUTH motor only:")
        print("   Moving 45° right, then 45° left")
        input("   Press Enter to start...")
        
        self.move_azimuth_degrees(45, 0.001)
        print("⏸ Pausing 1 second...")
        time.sleep(1)
        self.move_azimuth_degrees(-45, 0.001)
        
        print("\n2. Testing ALTITUDE motor only:")
        print("   Moving 45° up, then 45° down")
        input("   Press Enter to start...")
        
        self.move_altitude_degrees(45, 0.001)
        print("⏸ Pausing 1 second...")
        time.sleep(1)
        self.move_altitude_degrees(-45, 0.001)
        
        print("\n✓ Individual tests complete")
        self.shift_out(0b00000000)
    
    def cleanup(self):
        """Clean up GPIO"""
        self.shift_out(0b00000000)
        GPIO.cleanup()
        print("GPIO cleanup complete.")

def main():
    """Main function"""
    print("="*70)
    print("CORRECTED MOTOR TEST - PRECISE 90° MOVEMENT")
    print("="*70)
    print("This fixes the step calculation issue:")
    print(f"  • 90° azimuth = (90/360) × 1024 = 256 steps")
    print(f"  • 90° altitude = (90/360) × 4096 = 1024 steps")
    print("\nThe altitude motor should move 4× more steps than azimuth")
    print("for the same angular distance (90°).")
    print("="*70)
    
    tester = None
    try:
        tester = CorrectMotorTest()
        
        while True:
            print("\n" + "="*70)
            print("TEST MENU")
            print("="*70)
            print("1. Run precise 90° test (both motors together)")
            print("2. Test motors individually (45° movements)")
            print("3. Test single steps (debug wiring)")
            print("4. Exit")
            
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                tester.run_precise_90_test()
            elif choice == "2":
                tester.run_individual_tests()
            elif choice == "3":
                tester.test_single_step()
            elif choice == "4":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
            
            input("\nPress Enter to continue...")
        
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
