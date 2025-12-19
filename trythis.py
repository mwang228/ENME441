#!/usr/bin/env python3
"""
ENME441 - SWAPPED MOTORS FIXED VERSION
Correcting the issues you observed
"""

import RPi.GPIO as GPIO
import time
import json
import os

class FixedSwappedTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 - SWAPPED MOTORS WITH FIXES")
        print("="*70)
        print("ISSUES IDENTIFIED:")
        print("1. NEW Azimuth: 60° instead of 90° → 2700 steps/rev")
        print("2. NEW Altitude: Direction issues → Fixed sequence")
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        
        # Configuration
        self.CONFIG_FILE = "fixed_swapped_config.json"
        
        # FIXED VALUES based on your observations
        self.AZIMUTH_STEPS_PER_REV = 2700   # Was 1800, corrected for 60°→90°
        self.ALTITUDE_STEPS_PER_REV = 450   # Was 900, HALVED for double movement
        
        self.load_config()
        
        print(f"FIXED configuration loaded:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev (was 1800)")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev (was 900)")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # **CRITICAL FIX: Different sequences for each motor**
        # Azimuth: Standard sequence (works fine)
        self.AZIMUTH_SEQUENCE = [
            0b00000001,  # Coil A
            0b00000010,  # Coil B
            0b00000100,  # Coil C
            0b00001000,  # Coil D
        ]
        
        # **ALTITUDE FIX: Try REVERSED sequence for direction fix**
        # Since it works in negative but not positive direction
        self.ALTITUDE_SEQUENCE_FORWARD = [
            0b00010000,  # Coil A
            0b00100000,  # Coil B
            0b01000000,  # Coil C
            0b10000000,  # Coil D
        ]
        
        self.ALTITUDE_SEQUENCE_REVERSED = [
            0b10000000,  # Coil D (reversed)
            0b01000000,  # Coil C
            0b00100000,  # Coil B
            0b00010000,  # Coil A
        ]
        
        # Start with standard sequence
        self.altitude_sequence = self.ALTITUDE_SEQUENCE_FORWARD
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Direction tracking for altitude
        self.altitude_last_direction = 0  # -1 = negative, 1 = positive, 0 = unknown
        
        # Timing - adjusted for each motor
        self.AZIMUTH_STEP_DELAY = 0.020  # 20ms for azimuth
        self.ALTITUDE_STEP_DELAY = 0.030  # 30ms for altitude (needs more time)
        
        # Initialize
        self.setup_gpio()
        
        print(f"\n✓ Fixed system initialized")
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
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 2700)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 450)
                print("✓ Fixed configuration loaded")
        except:
            print("Using calculated fixed values")
    
    def save_config(self):
        """Save configuration"""
        try:
            config = {
                'azimuth_steps_per_rev': self.AZIMUTH_STEPS_PER_REV,
                'altitude_steps_per_rev': self.ALTITUDE_STEPS_PER_REV
            }
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            print("✓ Fixed configuration saved")
        except:
            print("Warning: Could not save")
    
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
        """Send data with stabilization"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.000001)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.000001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors with current sequences"""
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.altitude_sequence[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth(self, direction):
        """Step azimuth motor"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude_smart(self, direction):
        """
        Smart altitude stepping with direction-dependent sequence
        """
        # Track direction for sequence switching
        if direction != self.altitude_last_direction:
            print(f"  Altitude direction change: {direction}")
            
            # Switch sequence based on what works
            if direction > 0:  # Positive direction was stuttering
                # Try reversed sequence for positive
                self.altitude_sequence = self.ALTITUDE_SEQUENCE_REVERSED
                print("  Using REVERSED sequence for positive direction")
            else:  # Negative direction worked
                self.altitude_sequence = self.ALTITUDE_SEQUENCE_FORWARD
                print("  Using STANDARD sequence for negative direction")
            
            self.altitude_last_direction = direction
        
        # Take the step
        self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
        self.altitude_steps += direction
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        
        self.update_motors()
    
    def move_azimuth_corrected(self, degrees):
        """Move azimuth with corrected steps/rev"""
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nAZIMUTH: Moving {degrees:.1f}°")
        print(f"Expected: {degrees:.1f}°, Steps: {steps}")
        print(f"Based on: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        
        successful_steps = 0
        
        for i in range(steps):
            try:
                self.step_azimuth(direction)
                successful_steps += 1
                time.sleep(self.AZIMUTH_STEP_DELAY)
                
                if steps > 10 and (i + 1) % (steps // 5) == 0:
                    progress = (i + 1) / steps * 100
                    print(f"  {progress:.0f}% - Angle: {self.azimuth_angle:.1f}°")
            
            except Exception as e:
                print(f"  ⚠️ Step {i+1} failed: {e}")
                # Slow down and retry
                time.sleep(self.AZIMUTH_STEP_DELAY * 2)
        
        actual = self.azimuth_angle
        print(f"\n✓ Azimuth complete: {actual:.1f}°")
        print(f"  Steps: {successful_steps}/{steps}")
        
        return actual
    
    def move_altitude_directional(self, degrees):
        """Move altitude with directional intelligence"""
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nALTITUDE: Moving {degrees:.1f}°")
        print(f"Direction: {'UP' if direction > 0 else 'DOWN'}")
        print(f"Steps: {steps} (based on {self.ALTITUDE_STEPS_PER_REV} steps/rev)")
        
        successful_steps = 0
        
        for i in range(steps):
            try:
                self.step_altitude_smart(direction)
                successful_steps += 1
                
                # Longer delay for altitude (it stutters)
                time.sleep(self.ALTITUDE_STEP_DELAY)
                
                if steps > 10 and (i + 1) % (steps // 5) == 0:
                    progress = (i + 1) / steps * 100
                    print(f"  {progress:.0f}% - Angle: {self.altitude_angle:.1f}°")
            
            except Exception as e:
                print(f"  ⚠️ Step {i+1} failed: {e}")
                
                # Try even slower
                time.sleep(self.ALTITUDE_STEP_DELAY * 3)
                
                # Try re-energizing current position
                self.update_motors()
                time.sleep(0.1)
        
        actual = self.altitude_angle
        print(f"\n✓ Altitude complete: {actual:.1f}°")
        print(f"  Steps: {successful_steps}/{steps}")
        
        return actual
    
    def diagnose_altitude_issue(self):
        """Diagnose the altitude motor direction issue"""
        print("\n" + "="*60)
        print("ALTITUDE MOTOR DIAGNOSIS")
        print("="*60)
        
        print("Issue: Works in negative direction, stutters in positive")
        print("Possible causes:")
        print("1. Wrong coil order")
        print("2. Mechanical binding in one direction")
        print("3. Insufficient torque for upward movement")
        print("="*60)
        
        # Test small movements in each direction
        print("\nTesting +10° (UP)...")
        start_up = self.altitude_angle
        actual_up = self.move_altitude_directional(10)
        moved_up = actual_up - start_up
        
        print(f"\nTesting -10° (DOWN)...")
        start_down = self.altitude_angle
        actual_down = self.move_altitude_directional(-10)
        moved_down = actual_down - start_down
        
        print("\n" + "="*60)
        print("DIAGNOSIS RESULTS:")
        print("="*60)
        print(f"UP (+10°): Requested +10°, Actual {moved_up:+.1f}°")
        print(f"DOWN (-10°): Requested -10°, Actual {moved_down:+.1f}°")
        
        if abs(moved_up - 10) > 5 or abs(moved_down + 10) > 5:
            print("\n⚠️ SIGNIFICANT DIRECTIONAL ASYMMETRY")
            print("This suggests WRONG COIL ORDER")
            print("Try swapping coil pairs on the altitude motor")
        
        # Return to approximately original position
        net_movement = moved_up + moved_down
        if abs(net_movement) > 1:
            self.move_altitude_directional(-net_movement)
    
    def test_coil_order_fixes(self):
        """Test different coil orders for altitude"""
        print("\n" + "="*60)
        print("TEST COIL ORDER FIXES")
        print("="*60)
        
        print("Trying different coil sequences...")
        
        sequences = {
            "Standard (A,B,C,D)": [0b00010000, 0b00100000, 0b01000000, 0b10000000],
            "Reversed (D,C,B,A)": [0b10000000, 0b01000000, 0b00100000, 0b00010000],
            "Alternate1 (A,C,B,D)": [0b00010000, 0b01000000, 0b00100000, 0b10000000],
            "Alternate2 (B,A,D,C)": [0b00100000, 0b00010000, 0b10000000, 0b01000000],
        }
        
        for name, seq in sequences.items():
            print(f"\nTesting: {name}")
            self.altitude_sequence = seq
            
            # Reset position
            self.altitude_seq_pos = 0
            self.update_motors()
            time.sleep(0.5)
            
            print("  Testing +20°...")
            start = self.altitude_angle
            try:
                # Try 20 steps (small test)
                for i in range(20):
                    self.altitude_seq_pos = (self.altitude_seq_pos + 1) % 4
                    self.altitude_steps += 1
                    self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
                    self.update_motors()
                    time.sleep(0.05)
                
                moved = self.altitude_angle - start
                print(f"  Result: Moved {moved:.1f}°")
                
                response = input("  Did this work better? (y/n): ").lower()
                if response == 'y':
                    print(f"  ✓ Found working sequence: {name}")
                    self.ALTITUDE_SEQUENCE_FORWARD = seq
                    self.altitude_sequence = seq
                    break
            
            except Exception as e:
                print(f"  Failed: {e}")
        
        # Return to 0
        self.move_altitude_directional(-self.altitude_angle)
        print("\n✓ Coil order test complete")
    
    def verify_corrections(self):
        """Verify all corrections work"""
        print("\n" + "="*60)
        print("VERIFY ALL CORRECTIONS")
        print("="*60)
        
        print("1. Verifying azimuth correction (90° should work now)...")
        start_az = self.azimuth_angle
        self.move_azimuth_corrected(90)
        az_moved = self.azimuth_angle - start_az
        print(f"   Result: {az_moved:.1f}° of 90°")
        
        # Return
        self.move_azimuth_corrected(-az_moved)
        
        print("\n2. Verifying altitude in both directions...")
        
        print("   Testing +30° (UP)...")
        start_up = self.altitude_angle
        self.move_altitude_directional(30)
        up_moved = self.altitude_angle - start_up
        
        print("   Testing -30° (DOWN)...")
        start_down = self.altitude_angle
        self.move_altitude_directional(-30)
        down_moved = self.altitude_angle - start_down
        
        print("\n" + "="*60)
        print("VERIFICATION RESULTS:")
        print("="*60)
        print(f"Azimuth (90° test): {az_moved:.1f}°")
        print(f"Altitude UP (+30°): {up_moved:.1f}°")
        print(f"Altitude DOWN (-30°): {down_moved:.1f}°")
        
        # Return altitude to center
        net_alt = up_moved + down_moved
        if abs(net_alt) > 1:
            self.move_altitude_directional(-net_alt)
        
        print(f"\nFinal: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
    
    def competition_readiness_test(self):
        """Test for competition requirements"""
        print("\n" + "="*60)
        print("COMPETITION READINESS TEST")
        print("="*60)
        
        movements = [
            ("Aim right 45°", 45, 0),
            ("Aim up 30°", 0, 30),
            ("Aim left 90°", -90, 0),
            ("Aim down 45°", 0, -45),
            ("Return to center", -self.azimuth_angle, -self.altitude_angle),
        ]
        
        for name, az_move, alt_move in movements:
            print(f"\n{name}:")
            
            if az_move != 0:
                self.move_azimuth_corrected(az_move)
            
            if alt_move != 0:
                self.move_altitude_directional(alt_move)
            
            time.sleep(0.5)
        
        print(f"\n✓ Competition test complete!")
        print(f"Final position: ({self.azimuth_angle:.1f}°, {self.altitude_angle:.1f}°)")
    
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
    print("ENME441 - SWAPPED MOTORS WITH FIXES")
    print("="*70)
    print("CORRECTIONS APPLIED:")
    print("Azimuth: 1800 → 2700 steps/rev (was 60° instead of 90°)")
    print("Altitude: 900 → 450 steps/rev (was moving double)")
    print("Altitude: Direction-dependent sequence fixing")
    print("="*70)
    
    turret = None
    try:
        turret = FixedSwappedTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU - FIXED VERSION")
            print("="*60)
            print("1. Verify corrections (START HERE)")
            print("2. Diagnose altitude direction issue")
            print("3. Test coil order fixes")
            print("4. Competition readiness test")
            print("5. Interactive control")
            print("6. Show current settings")
            print("7. Exit and cleanup")
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == "1":
                turret.verify_corrections()
            elif choice == "2":
                turret.diagnose_altitude_issue()
            elif choice == "3":
                turret.test_coil_order_fixes()
            elif choice == "4":
                turret.competition_readiness_test()
            elif choice == "5":
                print("\nInteractive control:")
                print("Commands: a/d = azimuth left/right, w/s = altitude up/down")
                print("          z = zero, q = quit")
                while True:
                    cmd = input("Command: ").lower()
                    if cmd == 'a':
                        turret.move_azimuth_corrected(-10)
                    elif cmd == 'd':
                        turret.move_azimuth_corrected(10)
                    elif cmd == 'w':
                        turret.move_altitude_directional(10)
                    elif cmd == 's':
                        turret.move_altitude_directional(-10)
                    elif cmd == 'z':
                        turret.move_azimuth_corrected(-turret.azimuth_angle)
                        turret.move_altitude_directional(-turret.altitude_angle)
                    elif cmd == 'q':
                        break
                    else:
                        print("Invalid command")
            elif choice == "6":
                print(f"\nCurrent FIXED settings:")
                print(f"  Azimuth steps/rev: {turret.AZIMUTH_STEPS_PER_REV}")
                print(f"  Altitude steps/rev: {turret.ALTITUDE_STEPS_PER_REV}")
                print(f"  Azimuth delay: {turret.AZIMUTH_STEP_DELAY*1000:.1f}ms")
                print(f"  Altitude delay: {turret.ALTITUDE_STEP_DELAY*1000:.1f}ms")
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
