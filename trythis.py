#!/usr/bin/env python3
"""
ENME441 Laser Turret - INDEPENDENT MOTOR CONTROL
Each motor has its own calibration and timing
"""

import RPi.GPIO as GPIO
import time
import json
import os

class IndependentTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 - INDEPENDENT MOTOR CONTROL")
        print("="*70)
        print("Treating each motor separately")
        print("Azimuth: Likely 200-400 steps/rev")
        print("Altitude: 2400 steps/rev (from earlier)")
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11  # GPIO11 -> Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> Pin 14 (DS)
        
        # Configuration file
        self.CONFIG_FILE = "independent_config.json"
        
        # SEPARATE configurations for each motor
        # Default values based on what we know
        self.AZIMUTH_STEPS_PER_REV = 400    # Common stepper value
        self.ALTITUDE_STEPS_PER_REV = 2400  # From your earlier calibration
        
        self.load_config()
        
        print(f"Loaded INDEPENDENT configuration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # SEPARATE step sequences - simpler 4-step
        self.AZIMUTH_SEQUENCE = [
            0b00000001,  # Coil A
            0b00000010,  # Coil B
            0b00000100,  # Coil C
            0b00001000,  # Coil D
        ]
        
        self.ALTITUDE_SEQUENCE = [
            0b00010000,  # Coil A
            0b00100000,  # Coil B
            0b01000000,  # Coil C
            0b10000000,  # Coil D
        ]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # SEPARATE timing for each motor
        self.AZIMUTH_STEP_DELAY = 0.01      # 10ms for azimuth
        self.ALTITUDE_STEP_DELAY = 0.02     # 20ms for altitude (slower)
        
        # Initialize
        self.setup_gpio()
        
        print(f"\n✓ System initialized with independent control")
        print(f"Azimuth delay: {self.AZIMUTH_STEP_DELAY*1000:.1f}ms")
        print(f"Altitude delay: {self.ALTITUDE_STEP_DELAY*1000:.1f}ms")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print("="*70)
    
    def load_config(self):
        """Load separate configurations"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 400)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 2400)
                print("✓ Loaded independent configuration")
        except:
            print("Using default independent configuration")
    
    def save_config(self):
        """Save separate configurations"""
        try:
            config = {
                'azimuth_steps_per_rev': self.AZIMUTH_STEPS_PER_REV,
                'altitude_steps_per_rev': self.ALTITUDE_STEPS_PER_REV,
                'azimuth_step_delay': self.AZIMUTH_STEP_DELAY,
                'altitude_step_delay': self.ALTITUDE_STEP_DELAY
            }
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            print("✓ Independent configuration saved")
        except:
            print("Warning: Could not save configuration")
    
    def setup_gpio(self):
        """Initialize GPIO"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Clear shift register
        self.send_to_shift_register(0b00000000)
    
    def send_to_shift_register(self, data):
        """Send data to shift register"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.000001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors with their own sequences"""
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth_simple(self):
        """Simple azimuth step - NO delay here"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos + 1) % 4
        self.azimuth_steps += 1
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_azimuth_back_simple(self):
        """Simple azimuth step backwards"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos - 1) % 4
        self.azimuth_steps -= 1
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude_simple(self):
        """Simple altitude step - NO delay here"""
        self.altitude_seq_pos = (self.altitude_seq_pos + 1) % 4
        self.altitude_steps += 1
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude_back_simple(self):
        """Simple altitude step backwards"""
        self.altitude_seq_pos = (self.altitude_seq_pos - 1) % 4
        self.altitude_steps -= 1
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def move_azimuth_manual_calibration(self, target_degrees):
        """
        Manual calibration for azimuth - YOU control the steps
        """
        print(f"\n{'='*60}")
        print("MANUAL AZIMUTH CALIBRATION")
        print(f"{'='*60}")
        
        print(f"Target: Move {target_degrees}°")
        print(f"Current steps/rev: {self.AZIMUTH_STEPS_PER_REV}")
        print(f"Expected steps: {int((target_degrees/360)*self.AZIMUTH_STEPS_PER_REV)}")
        
        print("\nWe'll move step by step.")
        print("Press 'f' for forward, 'b' for backward, 'q' to quit")
        print("Count how many steps it takes to reach the target angle")
        
        start_angle = self.azimuth_angle
        steps_taken = 0
        
        while True:
            print(f"\nCurrent: {self.azimuth_angle:.1f}° from start")
            print(f"Steps taken: {steps_taken}")
            print(f"Remaining to target: {target_degrees - (self.azimuth_angle - start_angle):.1f}°")
            
            cmd = input("Command (f/b/q): ").lower()
            
            if cmd == 'f':
                self.step_azimuth_simple()
                steps_taken += 1
                time.sleep(self.AZIMUTH_STEP_DELAY)
            
            elif cmd == 'b':
                self.step_azimuth_back_simple()
                steps_taken -= 1
                time.sleep(self.AZIMUTH_STEP_DELAY)
            
            elif cmd == 'q':
                print("Calibration cancelled")
                break
            
            else:
                print("Invalid command")
            
            # Check if we reached target
            actual_movement = self.azimuth_angle - start_angle
            if abs(actual_movement - target_degrees) < 1.0:
                print(f"\n✓ Reached target! Actual movement: {actual_movement:.1f}°")
                
                # Calculate new steps/rev
                if actual_movement > 0:
                    new_steps_per_rev = int((360.0 / actual_movement) * steps_taken)
                    print(f"\nCALIBRATION RESULT:")
                    print(f"  Steps taken: {steps_taken}")
                    print(f"  Actual movement: {actual_movement:.1f}°")
                    print(f"  Old steps/rev: {self.AZIMUTH_STEPS_PER_REV}")
                    print(f"  New steps/rev: {new_steps_per_rev}")
                    
                    # Update
                    self.AZIMUTH_STEPS_PER_REV = new_steps_per_rev
                    self.save_config()
                    
                    # Recalculate angle with new value
                    self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
                
                break
        
        return steps_taken
    
    def auto_calibrate_azimuth(self):
        """Automatically calibrate azimuth motor"""
        print(f"\n{'='*60}")
        print("AUTO-CALIBRATE AZIMUTH")
        print(f"{'='*60}")
        
        # First, test with common values
        test_values = [200, 400, 800, 1600, 2400]
        
        print("Testing different steps/revolution values...")
        print("We'll move 90° with each setting and see what works")
        
        # Save current position
        saved_steps = self.azimuth_steps
        saved_seq_pos = self.azimuth_seq_pos
        
        for steps_per_rev in test_values:
            print(f"\nTesting: {steps_per_rev} steps/rev")
            
            # Reset to known state
            self.azimuth_steps = 0
            self.azimuth_seq_pos = 0
            self.AZIMUTH_STEPS_PER_REV = steps_per_rev
            self.update_motors()
            time.sleep(0.5)
            
            # Move 90 degrees
            expected_steps = int((90 / 360) * steps_per_rev)
            print(f"  Expected steps for 90°: {expected_steps}")
            
            input("  Press Enter to test movement...")
            
            start_time = time.time()
            for i in range(expected_steps):
                self.step_azimuth_simple()
                time.sleep(0.01)  # 10ms delay
            
            elapsed = time.time() - start_time
            actual_degrees = self.azimuth_angle
            
            print(f"  Result: Moved {actual_degrees:.1f}° in {elapsed:.1f}s")
            print(f"  Expected: 90°, Actual: {actual_degrees:.1f}°")
            
            response = input("  Did it move approximately 90°? (y/n): ").lower()
            if response == 'y':
                print(f"  ✓ Found working value: {steps_per_rev} steps/rev")
                self.AZIMUTH_STEPS_PER_REV = steps_per_rev
                self.save_config()
                
                # Restore original position
                self.azimuth_steps = saved_steps
                self.azimuth_seq_pos = saved_seq_pos
                self.update_motors()
                return steps_per_rev
        
        # Restore original position
        self.azimuth_steps = saved_steps
        self.azimuth_seq_pos = saved_seq_pos
        self.update_motors()
        
        print("\n✗ No standard value worked")
        print("Try manual calibration instead")
        return None
    
    def test_azimuth_with_timing(self):
        """Test azimuth with different timing"""
        print(f"\n{'='*60}")
        print("AZIMUTH TIMING TEST")
        print(f"{'='*60}")
        
        delays_to_test = [0.005, 0.01, 0.02, 0.03, 0.05]  # 5ms to 50ms
        
        for delay in delays_to_test:
            print(f"\nTesting with {delay*1000:.0f}ms delay...")
            
            # Save current
            saved_steps = self.azimuth_steps
            saved_angle = self.azimuth_angle
            
            # Move 45 degrees
            steps = int((45 / 360) * self.AZIMUTH_STEPS_PER_REV)
            print(f"  Moving {steps} steps (should be 45°)")
            
            start_time = time.time()
            for i in range(steps):
                self.step_azimuth_simple()
                time.sleep(delay)
            
            elapsed = time.time() - start_time
            actual = self.azimuth_angle - saved_angle
            
            print(f"  Result: {actual:.1f}° in {elapsed:.1f}s")
            print(f"  Speed: {actual/elapsed:.1f}°/second")
            
            # Return to start
            for i in range(steps):
                self.step_azimuth_back_simple()
                time.sleep(delay)
            
            response = input("  Keep testing? (y/n): ").lower()
            if response != 'y':
                break
        
        print("\n✓ Timing test complete")
    
    def quick_function_test(self):
        """Quick test to verify both motors work"""
        print(f"\n{'='*60}")
        print("QUICK FUNCTION TEST")
        print(f"{'='*60}")
        
        print("1. Testing azimuth motor (should move ~45°)")
        print(f"   Current: {self.azimuth_angle:.1f}°")
        
        # Save position
        az_start = self.azimuth_angle
        alt_start = self.altitude_angle
        
        # Move azimuth
        steps = int((45 / 360) * self.AZIMUTH_STEPS_PER_REV)
        print(f"   Taking {steps} steps with {self.AZIMUTH_STEP_DELAY*1000:.0f}ms delay")
        
        input("   Press Enter to move azimuth forward...")
        for i in range(steps):
            self.step_azimuth_simple()
            time.sleep(self.AZIMUTH_STEP_DELAY)
        
        az_moved = self.azimuth_angle - az_start
        print(f"   Azimuth moved: {az_moved:.1f}°")
        
        input("\n   Press Enter to move azimuth back...")
        for i in range(steps):
            self.step_azimuth_back_simple()
            time.sleep(self.AZIMUTH_STEP_DELAY)
        
        print(f"   Azimuth returned to: {self.azimuth_angle:.1f}°")
        
        print("\n2. Testing altitude motor (should move ~45°)")
        print(f"   Current: {self.altitude_angle:.1f}°")
        
        # Move altitude
        steps = int((45 / 360) * self.ALTITUDE_STEPS_PER_REV)
        print(f"   Taking {steps} steps with {self.ALTITUDE_STEP_DELAY*1000:.0f}ms delay")
        
        input("   Press Enter to move altitude up...")
        for i in range(steps):
            self.step_altitude_simple()
            time.sleep(self.ALTITUDE_STEP_DELAY)
        
        alt_moved = self.altitude_angle - alt_start
        print(f"   Altitude moved: {alt_moved:.1f}°")
        
        input("\n   Press Enter to move altitude down...")
        for i in range(steps):
            self.step_altitude_back_simple()
            time.sleep(self.ALTITUDE_STEP_DELAY)
        
        print(f"   Altitude returned to: {self.altitude_angle:.1f}°")
        
        print(f"\n✓ Test complete!")
        print(f"Final: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
    
    def interactive_control(self):
        """Interactive control for testing"""
        print(f"\n{'='*60}")
        print("INTERACTIVE CONTROL")
        print(f"{'='*60}")
        
        while True:
            print(f"\nCurrent position:")
            print(f"  Azimuth: {self.azimuth_angle:.1f}°")
            print(f"  Altitude: {self.altitude_angle:.1f}°")
            
            print("\nControls:")
            print("  a/d - Azimuth left/right")
            print("  w/s - Altitude up/down")
            print("  z   - Zero both motors")
            print("  q   - Quit")
            
            cmd = input("\nCommand: ").lower()
            
            if cmd == 'a':  # Azimuth left
                self.step_azimuth_back_simple()
                time.sleep(self.AZIMUTH_STEP_DELAY)
                print(f"Azimuth: {self.azimuth_angle:.1f}°")
            
            elif cmd == 'd':  # Azimuth right
                self.step_azimuth_simple()
                time.sleep(self.AZIMUTH_STEP_DELAY)
                print(f"Azimuth: {self.azimuth_angle:.1f}°")
            
            elif cmd == 'w':  # Altitude up
                self.step_altitude_simple()
                time.sleep(self.ALTITUDE_STEP_DELAY)
                print(f"Altitude: {self.altitude_angle:.1f}°")
            
            elif cmd == 's':  # Altitude down
                self.step_altitude_back_simple()
                time.sleep(self.ALTITUDE_STEP_DELAY)
                print(f"Altitude: {self.altitude_angle:.1f}°")
            
            elif cmd == 'z':  # Zero
                print("Zeroing both motors...")
                # Simple zero - just reset counters
                self.azimuth_steps = 0
                self.altitude_steps = 0
                self.azimuth_angle = 0.0
                self.altitude_angle = 0.0
                print(f"Zeroed: (0°, 0°)")
            
            elif cmd == 'q':
                break
            
            else:
                print("Invalid command")
    
    def cleanup(self):
        """Clean shutdown"""
        print("\nCleaning up...")
        self.send_to_shift_register(0b00000000)
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 - INDEPENDENT MOTOR CONTROL")
    print("="*70)
    print("Key approach:")
    print("• Each motor has SEPARATE calibration")
    print("• Azimuth: Start with 400 steps/rev")
    print("• Altitude: Use 2400 steps/rev")
    print("• Different timing for each")
    print("="*70)
    
    turret = None
    try:
        turret = IndependentTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU - INDEPENDENT CONTROL")
            print("="*60)
            print("1. Quick function test (START HERE)")
            print("2. Manual azimuth calibration (step by step)")
            print("3. Auto-calibrate azimuth (test common values)")
            print("4. Azimuth timing test (find best speed)")
            print("5. Interactive control (manual testing)")
            print("6. Show current configuration")
            print("7. Exit")
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == "1":
                turret.quick_function_test()
            elif choice == "2":
                turret.move_azimuth_manual_calibration(90)
            elif choice == "3":
                turret.auto_calibrate_azimuth()
            elif choice == "4":
                turret.test_azimuth_with_timing()
            elif choice == "5":
                turret.interactive_control()
            elif choice == "6":
                print(f"\nCurrent INDEPENDENT configuration:")
                print(f"  Azimuth steps/rev: {turret.AZIMUTH_STEPS_PER_REV}")
                print(f"  Altitude steps/rev: {turret.ALTITUDE_STEPS_PER_REV}")
                print(f"  Azimuth step delay: {turret.AZIMUTH_STEP_DELAY*1000:.1f}ms")
                print(f"  Altitude step delay: {turret.ALTITUDE_STEP_DELAY*1000:.1f}ms")
                print(f"  Azimuth angle: {turret.azimuth_angle:.1f}°")
                print(f"  Altitude angle: {turret.altitude_angle:.1f}°")
            elif choice == "7":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if turret:
            turret.cleanup()
        print("\nProgram ended.")

if __name__ == "__main__":
    main()
