#!/usr/bin/env python3
"""
ENME441 Laser Turret - CORRECTED CALIBRATION
Based on your actual movement measurements
"""

import RPi.GPIO as GPIO
import time
import json
import os

class CorrectedTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 - CORRECTED CALIBRATION")
        print("="*70)
        print("Based on your measurements:")
        print("Altitude: 60° instead of 45° = 1800 steps/rev")
        print("Azimuth: 20° instead of 45° = 900 steps/rev")
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11  # GPIO11 -> Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> Pin 14 (DS)
        
        # Configuration
        self.CONFIG_FILE = "corrected_config.json"
        
        # CORRECTED VALUES BASED ON YOUR MEASUREMENTS
        self.AZIMUTH_STEPS_PER_REV = 900    # Corrected from 400
        self.ALTITUDE_STEPS_PER_REV = 1800  # Corrected from 2400
        
        self.load_config()
        
        print(f"Loaded CORRECTED configuration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # Simple 4-step sequence
        self.AZIMUTH_SEQUENCE = [0b00000001, 0b00000010, 0b00000100, 0b00001000]
        self.ALTITUDE_SEQUENCE = [0b00010000, 0b00100000, 0b01000000, 0b10000000]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Optimized timing
        self.AZIMUTH_STEP_DELAY = 0.015  # 15ms for azimuth
        self.ALTITUDE_STEP_DELAY = 0.020  # 20ms for altitude
        
        # Initialize
        self.setup_gpio()
        
        print(f"\n✓ System initialized with CORRECTED values")
        print(f"Azimuth delay: {self.AZIMUTH_STEP_DELAY*1000:.1f}ms")
        print(f"Altitude delay: {self.ALTITUDE_STEP_DELAY*1000:.1f}ms")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print("="*70)
    
    def load_config(self):
        """Load configuration"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 900)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 1800)
                    self.AZIMUTH_STEP_DELAY = config.get('azimuth_step_delay', 0.015)
                    self.ALTITUDE_STEP_DELAY = config.get('altitude_step_delay', 0.020)
                print("✓ Loaded corrected configuration")
        except:
            print("Using calculated corrected values")
    
    def save_config(self):
        """Save configuration"""
        try:
            config = {
                'azimuth_steps_per_rev': self.AZIMUTH_STEPS_PER_REV,
                'altitude_steps_per_rev': self.ALTITUDE_STEPS_PER_REV,
                'azimuth_step_delay': self.AZIMUTH_STEP_DELAY,
                'altitude_step_delay': self.ALTITUDE_STEP_DELAY
            }
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            print("✓ Corrected configuration saved")
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
        """Update both motors"""
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth(self, direction=1):
        """Step azimuth motor"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude(self, direction=1):
        """Step altitude motor"""
        self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
        self.altitude_steps += direction
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def move_azimuth_precise(self, degrees):
        """Move azimuth precisely"""
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nAzimuth: Moving {degrees:.1f}°")
        print(f"Steps: {steps}, Delay: {self.AZIMUTH_STEP_DELAY*1000:.1f}ms")
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(self.AZIMUTH_STEP_DELAY)
            
            # Show progress
            if steps > 10 and (i + 1) % (steps // 5) == 0:
                progress = (i + 1) / steps * 100
                print(f"  {progress:.0f}% - Angle: {self.azimuth_angle:.1f}°")
        
        print(f"✓ Complete: {self.azimuth_angle:.1f}°")
        return self.azimuth_angle
    
    def move_altitude_precise(self, degrees):
        """Move altitude precisely"""
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nAltitude: Moving {degrees:.1f}°")
        print(f"Steps: {steps}, Delay: {self.ALTITUDE_STEP_DELAY*1000:.1f}ms")
        
        for i in range(steps):
            self.step_altitude(direction)
            time.sleep(self.ALTITUDE_STEP_DELAY)
            
            # Show progress
            if steps > 10 and (i + 1) % (steps // 5) == 0:
                progress = (i + 1) / steps * 100
                print(f"  {progress:.0f}% - Angle: {self.altitude_angle:.1f}°")
        
        print(f"✓ Complete: {self.altitude_angle:.1f}°")
        return self.altitude_angle
    
    def verify_correction(self):
        """Verify the corrected values work"""
        print("\n" + "="*60)
        print("VERIFY CORRECTED CALIBRATION")
        print("="*60)
        
        print("Testing 45° movement for both motors...")
        
        # Test azimuth
        print("\n1. Testing azimuth (should move 45°)")
        start_az = self.azimuth_angle
        self.move_azimuth_precise(45)
        az_moved = self.azimuth_angle - start_az
        print(f"   Result: Moved {az_moved:.1f}°")
        print(f"   Error: {abs(45 - az_moved):.1f}°")
        
        # Return azimuth to start
        self.move_azimuth_precise(-az_moved)
        
        # Test altitude
        print("\n2. Testing altitude (should move 45°)")
        start_alt = self.altitude_angle
        self.move_altitude_precise(45)
        alt_moved = self.altitude_angle - start_alt
        print(f"   Result: Moved {alt_moved:.1f}°")
        print(f"   Error: {abs(45 - alt_moved):.1f}°")
        
        # Return altitude to start
        self.move_altitude_precise(-alt_moved)
        
        print("\n" + "="*60)
        print("VERIFICATION RESULTS:")
        print("="*60)
        print(f"Azimuth: Target 45°, Actual {az_moved:.1f}°")
        print(f"Altitude: Target 45°, Actual {alt_moved:.1f}°")
        
        if abs(az_moved - 45) < 5 and abs(alt_moved - 45) < 5:
            print("\n✓ CORRECTION SUCCESSFUL!")
            print("Both motors move approximately 45°")
        else:
            print("\n⚠️  Further adjustment needed")
            print(f"Azimuth correction factor: {45/az_moved:.3f}")
            print(f"Altitude correction factor: {45/alt_moved:.3f}")
    
    def fine_tune_calibration(self):
        """Fine-tune calibration based on new measurements"""
        print("\n" + "="*60)
        print("FINE-TUNE CALIBRATION")
        print("="*60)
        
        print("We'll measure actual movement and adjust automatically.")
        
        # Fine-tune azimuth
        print("\n1. Fine-tuning azimuth...")
        start_az = self.azimuth_angle
        target_az = 45.0
        
        self.move_azimuth_precise(target_az)
        actual_az = self.azimuth_angle - start_az
        
        if abs(actual_az) > 1.0:
            correction = target_az / actual_az
            new_az_steps = int(self.AZIMUTH_STEPS_PER_REV * correction)
            
            print(f"\nAzimuth calibration:")
            print(f"  Target: {target_az:.1f}°")
            print(f"  Actual: {actual_az:.1f}°")
            print(f"  Correction: {correction:.3f}")
            print(f"  Old: {self.AZIMUTH_STEPS_PER_REV}")
            print(f"  New: {new_az_steps}")
            
            self.AZIMUTH_STEPS_PER_REV = new_az_steps
        
        # Return to start
        self.move_azimuth_precise(-actual_az)
        
        # Fine-tune altitude
        print("\n2. Fine-tuning altitude...")
        start_alt = self.altitude_angle
        target_alt = 45.0
        
        self.move_altitude_precise(target_alt)
        actual_alt = self.altitude_angle - start_alt
        
        if abs(actual_alt) > 1.0:
            correction = target_alt / actual_alt
            new_alt_steps = int(self.ALTITUDE_STEPS_PER_REV * correction)
            
            print(f"\nAltitude calibration:")
            print(f"  Target: {target_alt:.1f}°")
            print(f"  Actual: {actual_alt:.1f}°")
            print(f"  Correction: {correction:.3f}")
            print(f"  Old: {self.ALTITUDE_STEPS_PER_REV}")
            print(f"  New: {new_alt_steps}")
            
            self.ALTITUDE_STEPS_PER_REV = new_alt_steps
        
        # Return to start
        self.move_altitude_precise(-actual_alt)
        
        # Save new calibration
        self.save_config()
        
        print("\n✓ Fine-tuning complete!")
        print(f"New values saved:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
    
    def test_competition_movements(self):
        """Test movements similar to competition requirements"""
        print("\n" + "="*60)
        print("COMPETITION MOVEMENT TEST")
        print("="*60)
        
        print("Testing movements that might be needed in competition...")
        
        movements = [
            (45, 0),   # Right 45°
            (-45, 0),  # Back to center
            (0, 30),   # Up 30°
            (0, -30),  # Back to center
            (90, 0),   # Right 90°
            (0, 45),   # Up 45°
            (-90, -45) # Back to center
        ]
        
        for az_move, alt_move in movements:
            if az_move != 0:
                print(f"\nMoving azimuth {az_move:+.1f}°...")
                self.move_azimuth_precise(az_move)
            
            if alt_move != 0:
                print(f"Moving altitude {alt_move:+.1f}°...")
                self.move_altitude_precise(alt_move)
            
            time.sleep(0.5)
        
        # Return to center
        print(f"\nReturning to center (0°, 0°)...")
        self.move_azimuth_precise(-self.azimuth_angle)
        self.move_altitude_precise(-self.altitude_angle)
        
        print(f"\n✓ Competition test complete!")
        print(f"Final position: ({self.azimuth_angle:.1f}°, {self.altitude_angle:.1f}°)")
    
    def interactive_precision_test(self):
        """Interactive precision testing"""
        print("\n" + "="*60)
        print("INTERACTIVE PRECISION TEST")
        print("="*60)
        
        while True:
            print(f"\nCurrent: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
            print("\nTest options:")
            print("1. Test azimuth only")
            print("2. Test altitude only")
            print("3. Test both together")
            print("4. Adjust step delays")
            print("5. Return to (0°, 0°)")
            print("6. Back to main menu")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                try:
                    deg = float(input("Azimuth degrees to move: "))
                    self.move_azimuth_precise(deg)
                except:
                    print("Invalid input")
            
            elif choice == "2":
                try:
                    deg = float(input("Altitude degrees to move: "))
                    self.move_altitude_precise(deg)
                except:
                    print("Invalid input")
            
            elif choice == "3":
                try:
                    az_deg = float(input("Azimuth degrees: "))
                    alt_deg = float(input("Altitude degrees: "))
                    print("\nMoving both...")
                    self.move_azimuth_precise(az_deg)
                    self.move_altitude_precise(alt_deg)
                except:
                    print("Invalid input")
            
            elif choice == "4":
                print(f"\nCurrent delays:")
                print(f"  Azimuth: {self.AZIMUTH_STEP_DELAY*1000:.1f}ms")
                print(f"  Altitude: {self.ALTITUDE_STEP_DELAY*1000:.1f}ms")
                try:
                    new_az = float(input("New azimuth delay (ms): ")) / 1000
                    new_alt = float(input("New altitude delay (ms): ")) / 1000
                    self.AZIMUTH_STEP_DELAY = max(0.001, new_az)
                    self.ALTITUDE_STEP_DELAY = max(0.001, new_alt)
                    self.save_config()
                    print("Delays updated and saved")
                except:
                    print("Invalid input")
            
            elif choice == "5":
                print("\nReturning to center...")
                self.move_azimuth_precise(-self.azimuth_angle)
                self.move_altitude_precise(-self.altitude_angle)
                print(f"✓ At center (0°, 0°)")
            
            elif choice == "6":
                break
            
            else:
                print("Invalid choice")
    
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
    print("ENME441 - PRECISION CORRECTED CALIBRATION")
    print("="*70)
    print("CALCULATED CORRECTIONS:")
    print("Altitude: 2400 → 1800 steps/rev (was moving 60° instead of 45°)")
    print("Azimuth: 400 → 900 steps/rev (was moving 20° instead of 45°)")
    print("="*70)
    
    turret = None
    try:
        turret = CorrectedTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU - PRECISION CALIBRATION")
            print("="*60)
            print("1. Verify corrected calibration (START HERE)")
            print("2. Fine-tune calibration (if needed)")
            print("3. Test competition movements")
            print("4. Interactive precision test")
            print("5. Show current configuration")
            print("6. Exit and cleanup")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                turret.verify_correction()
            elif choice == "2":
                turret.fine_tune_calibration()
            elif choice == "3":
                turret.test_competition_movements()
            elif choice == "4":
                turret.interactive_precision_test()
            elif choice == "5":
                print(f"\nCurrent PRECISE configuration:")
                print(f"  Azimuth steps/rev: {turret.AZIMUTH_STEPS_PER_REV}")
                print(f"  Altitude steps/rev: {turret.ALTITUDE_STEPS_PER_REV}")
                print(f"  Azimuth step delay: {turret.AZIMUTH_STEP_DELAY*1000:.1f}ms")
                print(f"  Altitude step delay: {turret.ALTITUDE_STEP_DELAY*1000:.1f}ms")
                print(f"  Azimuth angle: {turret.azimuth_angle:.1f}°")
                print(f"  Altitude angle: {turret.altitude_angle:.1f}°")
            elif choice == "6":
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
