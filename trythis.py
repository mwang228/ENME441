#!/usr/bin/env python3
"""
AZIMUTH MOTOR DIAGNOSTIC - Find correct steps per revolution
"""

import RPi.GPIO as GPIO
import time

class AzimuthDiagnostic:
    def __init__(self):
        print("="*70)
        print("AZIMUTH MOTOR DIAGNOSTIC")
        print("="*70)
        
        # GPIO pins
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS (Pin 14)
        
        # Current assumption (probably wrong)
        self.AZIMUTH_STEPS_PER_REV = 1024  # Your original value
        
        # Position tracking
        self.azimuth_position = 0  # Steps from home
        self.azimuth_step_idx = 0
        
        # Simple 4-step sequence
        self.STEP_SEQUENCE = [
            0b00000001,  # Only azimuth coil A (pin 15)
            0b00000010,  # Only azimuth coil B (pin 1)
            0b00000100,  # Only azimuth coil C (pin 2)
            0b00001000,  # Only azimuth coil D (pin 3)
        ]
        
        # Setup
        self.setup_gpio()
        
        print(f"Current assumption: {self.AZIMUTH_STEPS_PER_REV} steps/revolution")
        print("If 90° moves <10°, then actual steps/rev is ~10× higher")
        print("="*70)
    
    def setup_gpio(self):
        """Initialize GPIO"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Start with motors off
        self.shift_out(0b00000000)
    
    def shift_out(self, data_byte):
        """Send data to shift register"""
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
    
    def step_azimuth(self, direction):
        """Take one azimuth step"""
        self.azimuth_step_idx = (self.azimuth_step_idx + direction) % 4
        self.azimuth_position += direction
        self.shift_out(self.STEP_SEQUENCE[self.azimuth_step_idx])
    
    def test_step_response(self):
        """Test how the motor responds to different step counts"""
        print("\n" + "="*60)
        print("STEP RESPONSE TEST")
        print("="*60)
        
        print("We'll test different step counts to see how far the motor moves.")
        print("Mark the starting position with tape or a marker.")
        
        test_cases = [
            (100, "Small test"),
            (256, "Should be ~90° if 1024 steps/rev is correct"),
            (512, "Should be ~180° if 1024 steps/rev is correct"),
            (1024, "Should be 360° (full circle)"),
            (2048, "Double current assumption"),
            (4096, "Same as altitude motor"),
        ]
        
        input("\nMark the starting position, then press Enter...")
        
        for steps, description in test_cases:
            print(f"\n{'='*40}")
            print(f"Test: {steps} steps ({description})")
            print('='*40)
            
            if steps == 1024:
                expected = "FULL CIRCLE (360°)"
            else:
                expected_deg = (steps / self.AZIMUTH_STEPS_PER_REV) * 360
                expected = f"~{expected_deg:.0f}°"
            
            print(f"Expected movement if {self.AZIMUTH_STEPS_PER_REV} steps/rev is correct: {expected}")
            
            proceed = input(f"\nRun {steps} step test? (y/n/stop): ").strip().lower()
            if proceed == 'stop':
                break
            elif proceed != 'y':
                continue
            
            print(f"\nMoving FORWARD {steps} steps...")
            for i in range(steps):
                self.step_azimuth(1)
                time.sleep(0.001)  # 1ms per step
                if (i + 1) % 100 == 0:
                    print(f"  Step {i+1}/{steps}")
            
            actual = input(f"\n{steps} steps completed. How many degrees did it actually move? (estimate, e.g., 45, 90, 180, 360): ").strip()
            
            try:
                actual_deg = float(actual)
                # Calculate actual steps per revolution
                if actual_deg > 0:
                    actual_steps_per_rev = (steps / actual_deg) * 360
                    print(f"  Actual movement: {actual_deg}°")
                    print(f"  Calculated steps/rev: {actual_steps_per_rev:.0f}")
                    
                    # Update our assumption
                    self.AZIMUTH_STEPS_PER_REV = int(actual_steps_per_rev)
                    print(f"  Updated assumption: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
            except:
                print(f"  Could not parse: {actual}")
            
            input("\nPress Enter to return to start position...")
            
            print(f"\nMoving BACK {steps} steps...")
            for i in range(steps):
                self.step_azimuth(-1)
                time.sleep(0.001)
                if (i + 1) % 100 == 0:
                    print(f"  Step {i+1}/{steps}")
            
            print(f"✓ Returned to start position")
    
    def find_correct_steps(self):
        """Interactive test to find correct steps/rev"""
        print("\n" + "="*60)
        print("FIND CORRECT STEPS PER REVOLUTION")
        print("="*60)
        
        print("Let's find out how many steps it takes for a full revolution.")
        print("We'll move the motor until it completes one full circle.")
        
        input("\nMark the starting position, then press Enter...")
        
        steps_taken = 0
        print("\nStarting movement...")
        print("Press Enter when the motor returns to the starting position.")
        print("Or type 'stop' and press Enter to stop early.")
        
        try:
            while True:
                # Take 100 steps at a time
                for i in range(100):
                    self.step_azimuth(1)
                    time.sleep(0.001)
                    steps_taken += 1
                
                # Check if user wants to stop
                print(f"  Steps taken: {steps_taken}", end='\r')
                
                # Non-blocking input check
                import select
                import sys
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    user_input = sys.stdin.readline().strip()
                    if user_input.lower() == 'stop' or user_input == '':
                        break
                        
        except KeyboardInterrupt:
            print("\nStopped by user")
        
        print(f"\nTotal steps taken: {steps_taken}")
        
        if steps_taken > 0:
            actual_rev = input("How many complete revolutions did it make? (e.g., 0.5, 1, 1.5): ").strip()
            try:
                revolutions = float(actual_rev)
                if revolutions > 0:
                    actual_steps_per_rev = steps_taken / revolutions
                    print(f"\n✓ CALCULATION COMPLETE!")
                    print(f"  Steps taken: {steps_taken}")
                    print(f"  Revolutions: {revolutions}")
                    print(f"  Actual steps/rev: {actual_steps_per_rev:.0f}")
                    print(f"\nUpdate your code to use: AZIMUTH_STEPS_PER_REV = {int(actual_steps_per_rev)}")
                    self.AZIMUTH_STEPS_PER_REV = int(actual_steps_per_rev)
            except:
                print("Could not parse revolutions")
        
        # Return to start
        print(f"\nReturning to start position ({steps_taken} steps back)...")
        for i in range(steps_taken):
            self.step_azimuth(-1)
            time.sleep(0.001)
            if (i + 1) % 100 == 0:
                print(f"  Step {i+1}/{steps_taken}")
        
        print("✓ Returned to start")
    
    def test_with_new_value(self, new_steps_per_rev):
        """Test with a new steps/rev value"""
        print(f"\n{'='*60}")
        print(f"TESTING WITH {new_steps_per_rev} STEPS/REV")
        print('='*60)
        
        self.AZIMUTH_STEPS_PER_REV = new_steps_per_rev
        
        # Test 90° movement
        steps_90 = int(90 * new_steps_per_rev / 360)
        
        print(f"With {new_steps_per_rev} steps/rev:")
        print(f"  90° should be {steps_90} steps")
        
        input("\nMark start position, press Enter for 90° test...")
        
        print(f"\nMoving {steps_90} steps (should be 90°)...")
        for i in range(steps_90):
            self.step_azimuth(1)
            time.sleep(0.001)
            if (i + 1) % 100 == 0:
                print(f"  Step {i+1}/{steps_90}")
        
        actual = input(f"\n{steps_90} steps completed. How many degrees did it move? ")
        
        try:
            actual_deg = float(actual)
            error = abs(actual_deg - 90)
            print(f"\nResults:")
            print(f"  Expected: 90°")
            print(f"  Actual: {actual_deg}°")
            print(f"  Error: {error:.1f}°")
            
            if error < 10:
                print("  ✓ Good accuracy!")
            else:
                print("  ⚠ Significant error")
        except:
            print("Could not parse result")
        
        # Return
        print(f"\nReturning to start...")
        for i in range(steps_90):
            self.step_azimuth(-1)
            time.sleep(0.001)
    
    def quick_90_test(self):
        """Quick test: Try different multipliers"""
        print("\n" + "="*60)
        print("QUICK 90° TEST WITH DIFFERENT MULTIPLIERS")
        print("="*60)
        
        # Since 90° moved <10°, actual steps/rev is ~10× higher
        multipliers = [5, 8, 10, 12, 15]
        
        original_steps = self.AZIMUTH_STEPS_PER_REV
        
        for mult in multipliers:
            test_steps = original_steps * mult
            steps_90 = int(90 * test_steps / 360)
            
            print(f"\nTesting {test_steps} steps/rev ({mult}× original)")
            print(f"  90° = {steps_90} steps")
            
            proceed = input(f"Test this? (y/n): ").strip().lower()
            if proceed != 'y':
                continue
            
            input("Mark start, press Enter...")
            
            print(f"Moving {steps_90} steps...")
            for i in range(steps_90):
                self.step_azimuth(1)
                time.sleep(0.001)
            
            actual = input(f"How many degrees? ")
            
            try:
                actual_deg = float(actual)
                print(f"  Result: {actual_deg}° (expected 90°)")
                if abs(actual_deg - 90) < 15:
                    print(f"  ✓ {test_steps} steps/rev looks promising!")
            except:
                pass
            
            print("Returning...")
            for i in range(steps_90):
                self.step_azimuth(-1)
                time.sleep(0.001)
    
    def run_diagnostic(self):
        """Main diagnostic menu"""
        while True:
            print(f"\n{'='*70}")
            print(f"AZIMUTH MOTOR DIAGNOSTIC")
            print(f"Current assumption: {self.AZIMUTH_STEPS_PER_REV} steps/revolution")
            print('='*70)
            
            print("\nOptions:")
            print("1. Test step response (find how far different step counts move)")
            print("2. Find correct steps per revolution (measure full circle)")
            print("3. Quick 90° test with different multipliers")
            print("4. Test with custom steps/rev value")
            print("5. Exit")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == "1":
                self.test_step_response()
            elif choice == "2":
                self.find_correct_steps()
            elif choice == "3":
                self.quick_90_test()
            elif choice == "4":
                try:
                    new_value = int(input("Enter new steps/rev value: "))
                    self.test_with_new_value(new_value)
                except:
                    print("Invalid number")
            elif choice == "5":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
    
    def cleanup(self):
        """Cleanup GPIO"""
        self.shift_out(0b00000000)
        GPIO.cleanup()
        print("\n✓ GPIO cleanup complete")

def main():
    """Main function"""
    print("="*70)
    print("AZIMUTH MOTOR DIAGNOSTIC TOOL")
    print("="*70)
    print("Problem: 90° command moves <10° actual")
    print("Solution: Find actual steps per revolution")
    print("\nThe motor likely has MORE steps/rev than we think")
    print("(Probably 8,000-12,000 steps/rev instead of 1,024)")
    print("="*70)
    
    diagnostic = None
    try:
        diagnostic = AzimuthDiagnostic()
        diagnostic.run_diagnostic()
        
    except KeyboardInterrupt:
        print("\nDiagnostic interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if diagnostic:
            diagnostic.cleanup()
        print("\nDiagnostic complete")

if __name__ == "__main__":
    main()
