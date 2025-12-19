#!/usr/bin/env python3
"""
FINAL MOTOR TEST - FIXED STEP CALCULATIONS
Matching step counts between motors, fixing overshoot
"""

import RPi.GPIO as GPIO
import time

class FinalMotorTest:
    def __init__(self):
        # GPIO pins for shift register
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS (Pin 14)
        
        # Motor calibration - USING YOUR ORIGINAL VALUES
        self.AZIMUTH_STEPS_PER_REV = 1024    # Fast motor
        self.ALTITUDE_STEPS_PER_REV = 4096   # Standard motor
        
        # IMPORTANT: Our 4-step sequence means each "step" in code
        # moves the motor 4 microsteps. So we need to divide by 4.
        self.AZIMUTH_MICROSTEPS_PER_STEP = 4
        self.ALTITUDE_MICROSTEPS_PER_STEP = 4
        
        # Effective steps for our 4-step sequence
        self.AZIMUTH_EFFECTIVE_STEPS = self.AZIMUTH_STEPS_PER_REV // self.AZIMUTH_MICROSTEPS_PER_STEP
        self.ALTITUDE_EFFECTIVE_STEPS = self.ALTITUDE_STEPS_PER_REV // self.ALTITUDE_MICROSTEPS_PER_STEP
        
        # Position tracking
        self.azimuth_step_idx = 0  # Current position in 4-step sequence
        self.altitude_step_idx = 0
        
        # Simple 4-step sequence
        self.STEP_SEQUENCE = [
            0b00010001,  # Step 0: Az=0001, Alt=0001
            0b00100010,  # Step 1: Az=0010, Alt=0010
            0b01000100,  # Step 2: Az=0100, Alt=0100
            0b10001000,  # Step 3: Az=1000, Alt=1000
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
        print("="*70)
        print("FINAL MOTOR TEST - MATCHED STEP COUNTS")
        print("="*70)
        print(f"Azimuth: {self.AZIMUTH_STEPS_PER_REV} microsteps/rev")
        print(f"        = {self.AZIMUTH_EFFECTIVE_STEPS} effective steps/rev")
        print(f"Altitude: {self.ALTITUDE_STEPS_PER_REV} microsteps/rev")
        print(f"         = {self.ALTITUDE_EFFECTIVE_STEPS} effective steps/rev")
        print("="*70)
    
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
    
    def update_motors(self):
        """Update both motors with current step indices"""
        combined = self.STEP_SEQUENCE[self.azimuth_step_idx] | self.STEP_SEQUENCE[self.altitude_step_idx]
        self.shift_out(combined)
    
    def calculate_effective_steps(self, degrees, is_altitude=False):
        """Calculate effective steps needed for given degrees"""
        if is_altitude:
            effective_steps_per_rev = self.ALTITUDE_EFFECTIVE_STEPS
        else:
            effective_steps_per_rev = self.AZIMUTH_EFFECTIVE_STEPS
        
        # Effective steps = (degrees / 360) × effective_steps_per_rev
        steps = int((degrees / 360.0) * effective_steps_per_rev)
        return steps
    
    def move_azimuth_effective(self, degrees, step_delay=0.002):
        """Move azimuth motor by effective steps"""
        effective_steps = self.calculate_effective_steps(abs(degrees), False)
        direction = 1 if degrees >= 0 else -1
        
        print(f"Azimuth: {degrees:.1f}° = {effective_steps} effective steps")
        print(f"(={effective_steps * self.AZIMUTH_MICROSTEPS_PER_STEP} microsteps)")
        
        for i in range(effective_steps):
            self.azimuth_step_idx = (self.azimuth_step_idx + direction) % 4
            self.update_motors()
            time.sleep(step_delay)
            if (i + 1) % 25 == 0:
                print(f"  Step {i+1}/{effective_steps}")
        
        return True
    
    def move_altitude_effective(self, degrees, step_delay=0.002):
        """Move altitude motor by effective steps"""
        effective_steps = self.calculate_effective_steps(abs(degrees), True)
        direction = 1 if degrees >= 0 else -1
        
        print(f"Altitude: {degrees:.1f}° = {effective_steps} effective steps")
        print(f"(={effective_steps * self.ALTITUDE_MICROSTEPS_PER_STEP} microsteps)")
        
        for i in range(effective_steps):
            self.altitude_step_idx = (self.altitude_step_idx + direction) % 4
            self.update_motors()
            time.sleep(step_delay)
            if (i + 1) % 50 == 0:
                print(f"  Step {i+1}/{effective_steps}")
        
        return True
    
    def run_matched_90_test(self):
        """Run 90° test with matched step counts"""
        print("\n" + "="*60)
        print("MATCHED 90° TEST - BOTH MOTORS USE SAME STEP COUNT")
        print("="*60)
        
        # Calculate steps for 90°
        az_steps_90 = self.calculate_effective_steps(90, False)
        alt_steps_90 = self.calculate_effective_steps(90, True)
        
        print(f"For 90° movement:")
        print(f"  Azimuth: {az_steps_90} effective steps")
        print(f"  Altitude: {alt_steps_90} effective steps")
        print("\nThey should be the SAME number!")
        
        if az_steps_90 != alt_steps_90:
            print(f"⚠ WARNING: Step counts don't match! Check calculations.")
        
        print("\nTest sequence:")
        print("  1. Both motors 90° CLOCKWISE/UP")
        print("  2. Pause 2 seconds")
        print("  3. Both motors 90° COUNTERCLOCKWISE/DOWN")
        print("="*60)
        
        input("Press Enter to start test...")
        
        # CLOCKWISE/UP (positive 90°)
        print(f"\n{'='*40}")
        print("MOVING 90° CLOCKWISE/UP")
        print('='*40)
        
        # Move azimuth
        print("\nMoving azimuth motor...")
        self.move_azimuth_effective(90, 0.001)
        
        # Move altitude
        print("\nMoving altitude motor...")
        self.move_altitude_effective(90, 0.001)
        
        print("⏸ Pausing 2 seconds...")
        time.sleep(2)
        
        # COUNTERCLOCKWISE/DOWN (negative 90°)
        print(f"\n{'='*40}")
        print("MOVING 90° COUNTERCLOCKWISE/DOWN")
        print('='*40)
        
        # Move azimuth back
        print("\nMoving azimuth motor back...")
        self.move_azimuth_effective(-90, 0.001)
        
        # Move altitude back
        print("\nMoving altitude motor back...")
        self.move_altitude_effective(-90, 0.001)
        
        print(f"\n{'='*40}")
        print("✓ TEST COMPLETE!")
        print('='*40)
        
        # Turn off motors
        self.shift_out(0b00000000)
        print("Motors turned off.")
    
    def run_simple_fixed_test(self):
        """Simple fixed test: Use same step count for both motors"""
        print("\n" + "="*60)
        print("SIMPLE FIXED TEST - 256 STEPS FOR BOTH MOTORS")
        print("="*60)
        print("Using 256 steps for 90° movement (simple approach)")
        print("This should make both motors move the same amount")
        print("="*60)
        
        steps_90 = 256  # Simple fixed value
        
        input("Press Enter to start...")
        
        # Move forward
        print(f"\nMoving 90° FORWARD ({steps_90} steps each)...")
        
        # Move azimuth
        print("Azimuth motor...")
        for i in range(steps_90):
            self.azimuth_step_idx = (self.azimuth_step_idx + 1) % 4
            self.update_motors()
            time.sleep(0.001)
        
        # Move altitude
        print("Altitude motor...")
        for i in range(steps_90):
            self.altitude_step_idx = (self.altitude_step_idx + 1) % 4
            self.update_motors()
            time.sleep(0.001)
        
        print("⏸ Pausing 2 seconds...")
        time.sleep(2)
        
        # Move backward
        print(f"\nMoving 90° BACKWARD ({steps_90} steps each)...")
        
        # Move azimuth back
        print("Azimuth motor back...")
        for i in range(steps_90):
            self.azimuth_step_idx = (self.azimuth_step_idx - 1) % 4
            self.update_motors()
            time.sleep(0.001)
        
        # Move altitude back
        print("Altitude motor back...")
        for i in range(steps_90):
            self.altitude_step_idx = (self.altitude_step_idx - 1) % 4
            self.update_motors()
            time.sleep(0.001)
        
        print(f"\n{'='*40}")
        print("✓ TEST COMPLETE!")
        print('='*40)
        
        self.shift_out(0b00000000)
    
    def run_diagnostic_test(self):
        """Diagnostic test to find correct step count"""
        print("\n" + "="*60)
        print("DIAGNOSTIC TEST - FIND CORRECT STEP COUNT")
        print("="*60)
        print("We'll test different step counts to find what works")
        print("="*60)
        
        test_cases = [
            (64, "Very small movement"),
            (128, "Small movement"),
            (256, "Expected 90°"),
            (512, "180°"),
            (1024, "360° (full circle)"),
        ]
        
        for steps, description in test_cases:
            print(f"\n{'='*40}")
            print(f"Testing: {steps} steps ({description})")
            print('='*40)
            
            proceed = input(f"Test {steps} steps? (y/n): ").strip().lower()
            if proceed != 'y':
                continue
            
            print(f"\nMoving azimuth {steps} steps forward...")
            for i in range(steps):
                self.azimuth_step_idx = (self.azimuth_step_idx + 1) % 4
                self.update_motors()
                time.sleep(0.001)
            
            input(f"Azimuth moved {steps} steps. How far did it move? (Press Enter when ready)")
            
            print(f"\nMoving azimuth {steps} steps back...")
            for i in range(steps):
                self.azimuth_step_idx = (self.azimuth_step_idx - 1) % 4
                self.update_motors()
                time.sleep(0.001)
            
            print(f"\nNow testing altitude with {steps} steps...")
            print(f"Moving altitude {steps} steps up...")
            for i in range(steps):
                self.altitude_step_idx = (self.altitude_step_idx + 1) % 4
                self.update_motors()
                time.sleep(0.001)
            
            input(f"Altitude moved {steps} steps. How far did it move? (Press Enter when ready)")
            
            print(f"\nMoving altitude {steps} steps down...")
            for i in range(steps):
                self.altitude_step_idx = (self.altitude_step_idx - 1) % 4
                self.update_motors()
                time.sleep(0.001)
            
            self.shift_out(0b00000000)
            print(f"✓ {steps} step test complete")
    
    def cleanup(self):
        """Clean up GPIO"""
        self.shift_out(0b00000000)
        GPIO.cleanup()
        print("GPIO cleanup complete.")

def main():
    """Main function"""
    print("="*70)
    print("FINAL MOTOR TEST - MATCHING STEP COUNTS")
    print("="*70)
    print("Problem: Altitude moves 2× too far, azimuth barely moves")
    print("Solution: Use same step count for both motors")
    print("="*70)
    
    tester = None
    try:
        tester = FinalMotorTest()
        
        while True:
            print("\n" + "="*70)
            print("TEST MENU")
            print("="*70)
            print("1. Run matched 90° test (calculated steps)")
            print("2. Run simple fixed test (256 steps for both)")
            print("3. Run diagnostic test (find correct step count)")
            print("4. Exit")
            
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                tester.run_matched_90_test()
            elif choice == "2":
                tester.run_simple_fixed_test()
            elif choice == "3":
                tester.run_diagnostic_test()
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
