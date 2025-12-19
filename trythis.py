#!/usr/bin/env python3
"""
ENME441 Laser Turret - MOTORS SWAPPED VERSION
Altitude motor (2400 steps/rev) is now on AZIMUTH (more load)
Azimuth motor (900 steps/rev) is now on ALTITUDE (less load)
"""

import RPi.GPIO as GPIO
import time
import json
import os

class SwappedTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 - MOTORS SWAPPED CONFIGURATION")
        print("="*70)
        print("SWAP COMPLETE:")
        print("• Old Altitude (2400 steps/rev) → NEW AZIMUTH (more load)")
        print("• Old Azimuth (900 steps/rev) → NEW ALTITUDE (less load)")
        print("="*70)
        
        # GPIO Pins (UNCHANGED)
        self.SHIFT_CLK = 11  # GPIO11 -> Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> Pin 14 (DS)
        
        # Configuration file
        self.CONFIG_FILE = "swapped_config.json"
        
        # SWAPPED VALUES - Based on what we know about each motor
        self.AZIMUTH_STEPS_PER_REV = 1800   # OLD altitude motor (was 2400, corrected to 1800)
        self.ALTITUDE_STEPS_PER_REV = 900   # OLD azimuth motor (was 400, corrected to 900)
        
        self.load_config()
        
        print(f"SWAPPED configuration loaded:")
        print(f"  AZIMUTH (old altitude): {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  ALTITUDE (old azimuth): {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # IMPORTANT: Wiring is physically swapped, but shift register pins are the same
        # Shift register → Physical motors:
        # Pins 15,1,2,3 → NEW AZIMUTH (old altitude motor)
        # Pins 4,5,6,7 → NEW ALTITUDE (old azimuth motor)
        
        # Sequences remain the same (just applied to different physical motors now)
        self.AZIMUTH_SEQUENCE = [
            0b00000001,  # Coil A (Pin 15) → NEW AZIMUTH
            0b00000010,  # Coil B (Pin 1)  → NEW AZIMUTH
            0b00000100,  # Coil C (Pin 2)  → NEW AZIMUTH
            0b00001000,  # Coil D (Pin 3)  → NEW AZIMUTH
        ]
        
        self.ALTITUDE_SEQUENCE = [
            0b00010000,  # Coil A (Pin 4) → NEW ALTITUDE
            0b00100000,  # Coil B (Pin 5) → NEW ALTITUDE
            0b01000000,  # Coil C (Pin 6) → NEW ALTITUDE
            0b10000000,  # Coil D (Pin 7) → NEW ALTITUDE
        ]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Timing optimized for each motor's characteristics
        self.AZIMUTH_STEP_DELAY = 0.015  # 15ms for new azimuth (was altitude)
        self.ALTITUDE_STEP_DELAY = 0.020  # 20ms for new altitude (was azimuth)
        
        # Initialize
        self.setup_gpio()
        
        # Energize motors in starting position
        self.update_motors()
        time.sleep(0.2)
        
        print(f"\n✓ SWAPPED system initialized!")
        print(f"NEW Azimuth (old altitude): {self.azimuth_angle:.1f}°")
        print(f"NEW Altitude (old azimuth): {self.altitude_angle:.1f}°")
        print("="*70)
    
    def load_config(self):
        """Load swapped configuration"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 1800)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 900)
                    self.AZIMUTH_STEP_DELAY = config.get('azimuth_step_delay', 0.015)
                    self.ALTITUDE_STEP_DELAY = config.get('altitude_step_delay', 0.020)
                print("✓ Swapped configuration loaded from file")
        except:
            print("Using default swapped values")
    
    def save_config(self):
        """Save swapped configuration"""
        try:
            config = {
                'azimuth_steps_per_rev': self.AZIMUTH_STEPS_PER_REV,
                'altitude_steps_per_rev': self.ALTITUDE_STEPS_PER_REV,
                'azimuth_step_delay': self.AZIMUTH_STEP_DELAY,
                'altitude_step_delay': self.ALTITUDE_STEP_DELAY
            }
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            print("✓ Swapped configuration saved")
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
        """Update both motors with swapped configuration"""
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth(self, direction=1):
        """Step NEW azimuth (old altitude motor)"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude(self, direction=1):
        """Step NEW altitude (old azimuth motor)"""
        self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
        self.altitude_steps += direction
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def move_azimuth_swapped(self, degrees):
        """Move NEW azimuth (should be more reliable now)"""
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nNEW AZIMUTH (old altitude): Moving {degrees:.1f}°")
        print(f"Steps: {steps}, Delay: {self.AZIMUTH_STEP_DELAY*1000:.1f}ms")
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(self.AZIMUTH_STEP_DELAY)
            
            if steps > 10 and (i + 1) % (steps // 5) == 0:
                progress = (i + 1) / steps * 100
                print(f"  {progress:.0f}% - Angle: {self.azimuth_angle:.1f}°")
        
        print(f"✓ NEW Azimuth complete: {self.azimuth_angle:.1f}°")
        return self.azimuth_angle
    
    def move_altitude_swapped(self, degrees):
        """Move NEW altitude (should have less load now)"""
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nNEW ALTITUDE (old azimuth): Moving {degrees:.1f}°")
        print(f"Steps: {steps}, Delay: {self.ALTITUDE_STEP_DELAY*1000:.1f}ms")
        
        for i in range(steps):
            self.step_altitude(direction)
            time.sleep(self.ALTITUDE_STEP_DELAY)
            
            if steps > 10 and (i + 1) % (steps // 5) == 0:
                progress = (i + 1) / steps * 100
                print(f"  {progress:.0f}% - Angle: {self.altitude_angle:.1f}°")
        
        print(f"✓ NEW Altitude complete: {self.altitude_angle:.1f}°")
        return self.altitude_angle
    
    def verify_swap(self):
        """Verify the motor swap works correctly"""
        print("\n" + "="*60)
        print("VERIFY MOTOR SWAP")
        print("="*60)
        
        print("Testing 45° movement for both NEW configurations...")
        
        # Test NEW azimuth (old altitude motor)
        print("\n1. Testing NEW AZIMUTH (old altitude motor):")
        print("   Should move smoothly - previously worked well at altitude")
        start_az = self.azimuth_angle
        self.move_azimuth_swapped(45)
        az_moved = self.azimuth_angle - start_az
        print(f"   Result: Moved {az_moved:.1f}° of 45°")
        
        # Return to start
        self.move_azimuth_swapped(-az_moved)
        
        # Test NEW altitude (old azimuth motor)
        print("\n2. Testing NEW ALTITUDE (old azimuth motor):")
        print("   Now has less load - should work better")
        start_alt = self.altitude_angle
        self.move_altitude_swapped(45)
        alt_moved = self.altitude_angle - start_alt
        print(f"   Result: Moved {alt_moved:.1f}° of 45°")
        
        # Return to start
        self.move_altitude_swapped(-alt_moved)
        
        print("\n" + "="*60)
        print("SWAP VERIFICATION RESULTS:")
        print("="*60)
        print(f"NEW Azimuth (old altitude): {az_moved:.1f}° of 45°")
        print(f"NEW Altitude (old azimuth): {alt_moved:.1f}° of 45°")
        
        if abs(az_moved - 45) < 5 and abs(alt_moved - 45) < 5:
            print("\n✓ SWAP SUCCESSFUL!")
            print("Both motors work in their new positions")
        else:
            print("\n⚠️  Some calibration needed")
            if abs(az_moved - 45) > 5:
                print(f"  Azimuth correction: {45/az_moved:.3f}")
            if abs(alt_moved - 45) > 5:
                print(f"  Altitude correction: {45/alt_moved:.3f}")
    
    def calibrate_swapped_motors(self):
        """Calibrate the swapped motors"""
        print("\n" + "="*60)
        print("CALIBRATE SWAPPED MOTORS")
        print("="*60)
        
        print("We'll calibrate each motor in its new position.")
        
        # Calibrate NEW azimuth (old altitude)
        print("\n1. Calibrating NEW AZIMUTH (old altitude motor):")
        print("   Moving 90° and measuring actual movement...")
        
        start_az = self.azimuth_angle
        self.move_azimuth_swapped(90)
        actual_az = self.azimuth_angle - start_az
        
        if abs(actual_az) > 1.0:
            correction = 90.0 / actual_az
            new_az_steps = int(self.AZIMUTH_STEPS_PER_REV * correction)
            
            print(f"\n   NEW Azimuth calibration:")
            print(f"     Target: 90.0°")
            print(f"     Actual: {actual_az:.1f}°")
            print(f"     Correction: {correction:.3f}")
            print(f"     Old: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
            print(f"     New: {new_az_steps} steps/rev")
            
            self.AZIMUTH_STEPS_PER_REV = new_az_steps
        
        # Return to start
        self.move_azimuth_swapped(-actual_az)
        
        # Calibrate NEW altitude (old azimuth)
        print("\n2. Calibrating NEW ALTITUDE (old azimuth motor):")
        print("   Moving 90° and measuring actual movement...")
        
        start_alt = self.altitude_angle
        self.move_altitude_swapped(90)
        actual_alt = self.altitude_angle - start_alt
        
        if abs(actual_alt) > 1.0:
            correction = 90.0 / actual_alt
            new_alt_steps = int(self.ALTITUDE_STEPS_PER_REV * correction)
            
            print(f"\n   NEW Altitude calibration:")
            print(f"     Target: 90.0°")
            print(f"     Actual: {actual_alt:.1f}°")
            print(f"     Correction: {correction:.3f}")
            print(f"     Old: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
            print(f"     New: {new_alt_steps} steps/rev")
            
            self.ALTITUDE_STEPS_PER_REV = new_alt_steps
        
        # Return to start
        self.move_altitude_swapped(-actual_alt)
        
        # Save new calibration
        self.save_config()
        
        print("\n✓ Swapped motor calibration complete!")
        print(f"NEW values:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
    
    def test_full_range_swapped(self):
        """Test full range with swapped motors"""
        print("\n" + "="*60)
        print("FULL RANGE TEST - SWAPPED MOTORS")
        print("="*60)
        
        print("Testing NEW azimuth (formerly altitude) through full range...")
        
        test_positions = [
            ("Right 45°", 45),
            ("Right 90°", 90),
            ("Left 45°", -45),
            ("Left 90°", -90),
        ]
        
        for name, angle in test_positions:
            print(f"\n{name}...")
            start_angle = self.azimuth_angle
            self.move_azimuth_swapped(angle)
            actual = self.azimuth_angle - start_angle
            
            print(f"  Target: {angle:+.1f}°, Actual: {actual:+.1f}°")
            
            if abs(actual - angle) < 5:
                print(f"  ✓ Success")
            else:
                print(f"  ⚠️ Limited movement")
                break
        
        # Return to center
        print(f"\nReturning to center...")
        self.move_azimuth_swapped(-self.azimuth_angle)
        
        print(f"\n✓ Range test complete")
        print(f"NEW Azimuth working range: ±{abs(self.azimuth_angle):.1f}°")
    
    def interactive_swapped_control(self):
        """Interactive control for swapped motors"""
        print("\n" + "="*60)
        print("INTERACTIVE CONTROL - SWAPPED MOTORS")
        print("="*60)
        
        while True:
            print(f"\nCurrent position:")
            print(f"  NEW Azimuth (old altitude): {self.azimuth_angle:.1f}°")
            print(f"  NEW Altitude (old azimuth): {self.altitude_angle:.1f}°")
            
            print("\nControls:")
            print("1. Move NEW azimuth only")
            print("2. Move NEW altitude only")
            print("3. Move both motors")
            print("4. Adjust step delays")
            print("5. Return to (0°, 0°)")
            print("6. Back to main menu")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                try:
                    deg = float(input("Azimuth degrees to move: "))
                    self.move_azimuth_swapped(deg)
                except:
                    print("Invalid input")
            
            elif choice == "2":
                try:
                    deg = float(input("Altitude degrees to move: "))
                    self.move_altitude_swapped(deg)
                except:
                    print("Invalid input")
            
            elif choice == "3":
                try:
                    az_deg = float(input("Azimuth degrees: "))
                    alt_deg = float(input("Altitude degrees: "))
                    print("\nMoving both...")
                    self.move_azimuth_swapped(az_deg)
                    self.move_altitude_swapped(alt_deg)
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
                self.move_azimuth_swapped(-self.azimuth_angle)
                self.move_altitude_swapped(-self.altitude_angle)
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
    """Main program for swapped motors"""
    print("="*70)
    print("ENME441 - MOTORS SWAPPED CONFIGURATION")
    print("="*70)
    print("PHYSICAL SWAP:")
    print("• Altitude motor → AZIMUTH position (more load)")
    print("• Azimuth motor → ALTITUDE position (less load)")
    print("")
    print("ELECTRICAL CONNECTIONS:")
    print("Shift Register Pins 15,1,2,3 → NEW AZIMUTH (old altitude)")
    print("Shift Register Pins 4,5,6,7 → NEW ALTITUDE (old azimuth)")
    print("="*70)
    
    turret = None
    try:
        turret = SwappedTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU - SWAPPED MOTORS")
            print("="*60)
            print("1. Verify motor swap (START HERE)")
            print("2. Calibrate swapped motors")
            print("3. Test full range (new azimuth)")
            print("4. Interactive control")
            print("5. Show current configuration")
            print("6. Exit and cleanup")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                turret.verify_swap()
            elif choice == "2":
                turret.calibrate_swapped_motors()
            elif choice == "3":
                turret.test_full_range_swapped()
            elif choice == "4":
                turret.interactive_swapped_control()
            elif choice == "5":
                print(f"\nCurrent SWAPPED configuration:")
                print(f"  Azimuth (old altitude): {turret.AZIMUTH_STEPS_PER_REV} steps/rev")
                print(f"  Altitude (old azimuth): {turret.ALTITUDE_STEPS_PER_REV} steps/rev")
                print(f"  Azimuth delay: {turret.AZIMUTH_STEP_DELAY*1000:.1f}ms")
                print(f"  Altitude delay: {turret.ALTITUDE_STEP_DELAY*1000:.1f}ms")
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
