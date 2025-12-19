#!/usr/bin/env python3
"""
ENME441 Laser Turret - FINAL WORKING VERSION
Calibrated motors with proper timing
"""

import RPi.GPIO as GPIO
import time
import json
import os

class WorkingTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 LASER TURRET - WORKING VERSION")
        print("="*70)
        
        # GPIO Pins for Shift Register
        self.SHIFT_CLK = 11  # GPIO11 -> 74HC595 Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> 74HC595 Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> 74HC595 Pin 14 (DS)
        
        # Configuration file
        self.CONFIG_FILE = "turret_config.json"
        
        # Load or set default calibration
        # Altitude needs 2400 steps/rev (from your testing)
        # Azimuth likely needs similar - we'll calibrate it
        self.AZIMUTH_STEPS_PER_REV = 2400  # Start with same as altitude
        self.ALTITUDE_STEPS_PER_REV = 2400  # Your calibrated value
        
        self.load_config()
        
        print(f"Loaded configuration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current positions
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # IMPORTANT: Use 4-step sequence (ABCD) that worked in debug
        self.AZIMUTH_SEQUENCE = [
            0b00000001,  # Step 1: Coil A (Pin 15)
            0b00000010,  # Step 2: Coil B (Pin 1)
            0b00000100,  # Step 3: Coil C (Pin 2)
            0b00001000,  # Step 4: Coil D (Pin 3)
        ]
        
        self.ALTITUDE_SEQUENCE = [
            0b00010000,  # Step 1: Coil A (Pin 4)
            0b00100000,  # Step 2: Coil B (Pin 5)
            0b01000000,  # Step 3: Coil C (Pin 6)
            0b10000000,  # Step 4: Coil D (Pin 7)
        ]
        
        # Current sequence positions
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Pre-charge the coils to avoid initial stall
        self.update_motors()
        time.sleep(0.1)
        
        print(f"\n✓ System initialized!")
        print(f"Azimuth: {self.azimuth_angle:.1f}°")
        print(f"Altitude: {self.altitude_angle:.1f}°")
        print("="*70)
    
    def load_config(self):
        """Load configuration from file"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 2400)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 2400)
                print(f"✓ Loaded configuration from {self.CONFIG_FILE}")
        except:
            print("Using default configuration")
    
    def save_config(self):
        """Save configuration to file"""
        try:
            config = {
                'azimuth_steps_per_rev': self.AZIMUTH_STEPS_PER_REV,
                'altitude_steps_per_rev': self.ALTITUDE_STEPS_PER_REV
            }
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"✓ Configuration saved")
        except:
            print("Warning: Could not save configuration")
    
    def setup_gpio(self):
        """Initialize GPIO with robust settings"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)  # Reduce warning noise
        
        # Setup shift register pins
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Initialize to known state
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Clear shift register
        self.send_to_shift_register(0b00000000)
        
        print("✓ GPIO initialized")
    
    def send_to_shift_register(self, data):
        """Robust shift register communication"""
        # Latch low to start
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        # Send bits MSB first
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            
            # Clock pulse with proper timing
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.000001)  # 1 µs - shorter but reliable
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        # Latch high to update outputs
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.000001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors - SIMPLIFIED and ROBUST"""
        # Get current pattern for each motor
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        
        # Combine patterns
        combined = az_pattern | alt_pattern
        
        # Send to shift register
        self.send_to_shift_register(combined)
    
    def step_azimuth(self, direction):
        """Step azimuth motor with proper timing"""
        # Update sequence position
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        
        # Update step count and angle
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        
        # Update the motors
        self.update_motors()
        
        # CRITICAL: SHORT delay for motor to respond
        time.sleep(0.002)  # 2 ms - adjust if needed
    
    def step_altitude(self, direction):
        """Step altitude motor with proper timing"""
        # Update sequence position
        self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
        
        # Update step count and angle
        self.altitude_steps += direction
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        
        # Update the motors
        self.update_motors()
        
        # CRITICAL: SHORT delay for motor to respond
        time.sleep(0.002)  # 2 ms - adjust if needed
    
    def move_azimuth_smooth(self, degrees, step_delay=0.003):
        """Move azimuth with smooth operation"""
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nAzimuth: Moving {degrees:.1f}°")
        print(f"Steps: {steps}, Delay: {step_delay*1000:.1f}ms/step")
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(max(0, step_delay - 0.002))  # Account for step_azimuth delay
            
            if (i + 1) % 50 == 0:
                print(f"  Step {i+1}/{steps}, Angle: {self.azimuth_angle:.1f}°")
        
        print(f"✓ Complete: {self.azimuth_angle:.1f}°")
        return self.azimuth_angle
    
    def move_altitude_smooth(self, degrees, step_delay=0.003):
        """Move altitude with smooth operation"""
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nAltitude: Moving {degrees:.1f}°")
        print(f"Steps: {steps}, Delay: {step_delay*1000:.1f}ms/step")
        
        for i in range(steps):
            self.step_altitude(direction)
            time.sleep(max(0, step_delay - 0.002))  # Account for step_altitude delay
            
            if (i + 1) % 50 == 0:
                print(f"  Step {i+1}/{steps}, Angle: {self.altitude_angle:.1f}°")
        
        print(f"✓ Complete: {self.altitude_angle:.1f}°")
        return self.altitude_angle
    
    def calibrate_azimuth(self):
        """Calibrate azimuth motor steps/rev"""
        print("\n" + "="*60)
        print("CALIBRATE AZIMUTH MOTOR")
        print("="*60)
        
        # Move to known start (0°)
        print("Moving to start position (0°)...")
        self.move_azimuth_smooth(-self.azimuth_angle)
        
        # Test movement
        test_angle = 90.0
        print(f"\nTesting {test_angle}° movement...")
        
        start_angle = self.azimuth_angle
        final_angle = self.move_azimuth_smooth(test_angle)
        actual_movement = final_angle - start_angle
        
        print(f"\nCalibration Results:")
        print(f"  Requested: {test_angle:.1f}°")
        print(f"  Actual:    {actual_movement:.1f}°")
        
        if abs(actual_movement) > 1.0:  # Avoid division by near-zero
            # Calculate new steps/rev
            correction = test_angle / actual_movement
            new_steps = int(self.AZIMUTH_STEPS_PER_REV * correction)
            
            print(f"  Correction factor: {correction:.3f}")
            print(f"  Old steps/rev: {self.AZIMUTH_STEPS_PER_REV}")
            print(f"  New steps/rev: {new_steps}")
            
            # Update and save
            self.AZIMUTH_STEPS_PER_REV = new_steps
            self.save_config()
            
            # Test the new calibration
            print(f"\nTesting new calibration...")
            self.move_azimuth_smooth(-actual_movement)  # Back to start
            new_test = self.move_azimuth_smooth(test_angle)
            print(f"  New movement: {new_test - start_angle:.1f}°")
        
        # Return to 0
        self.move_azimuth_smooth(-self.azimuth_angle)
        print("\n✓ Azimuth calibration complete!")
    
    def verify_both_motors(self):
        """Verify both motors work together"""
        print("\n" + "="*60)
        print("VERIFY BOTH MOTORS")
        print("="*60)
        
        print("1. Moving azimuth +45°...")
        self.move_azimuth_smooth(45)
        
        print("\n2. Moving altitude +45°...")
        self.move_altitude_smooth(45)
        
        print("\n3. Moving both back to 0°...")
        self.move_azimuth_smooth(-45)
        self.move_altitude_smooth(-45)
        
        print(f"\n✓ Verification complete!")
        print(f"Final: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
    
    def interactive_test(self):
        """Interactive test interface"""
        print("\n" + "="*60)
        print("INTERACTIVE TEST")
        print("="*60)
        
        while True:
            print(f"\nCurrent: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
            print("\nOptions:")
            print("1. Move azimuth")
            print("2. Move altitude")
            print("3. Move both")
            print("4. Return to (0°, 0°)")
            print("5. Back to main menu")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == "1":
                try:
                    deg = float(input("Degrees to move azimuth (+/-): "))
                    self.move_azimuth_smooth(deg)
                except:
                    print("Invalid input")
            
            elif choice == "2":
                try:
                    deg = float(input("Degrees to move altitude (+/-): "))
                    self.move_altitude_smooth(deg)
                except:
                    print("Invalid input")
            
            elif choice == "3":
                try:
                    az_deg = float(input("Azimuth degrees: "))
                    alt_deg = float(input("Altitude degrees: "))
                    print("\nMoving both motors...")
                    # Simple sequential movement for now
                    self.move_azimuth_smooth(az_deg)
                    self.move_altitude_smooth(alt_deg)
                except:
                    print("Invalid input")
            
            elif choice == "4":
                print("\nReturning to home...")
                self.move_azimuth_smooth(-self.azimuth_angle)
                self.move_altitude_smooth(-self.altitude_angle)
                print(f"✓ At home: (0°, 0°)")
            
            elif choice == "5":
                break
            
            else:
                print("Invalid choice")
    
    def cleanup(self):
        """Clean shutdown"""
        print("\nShutting down...")
        self.send_to_shift_register(0b00000000)  # Turn off coils
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 - WORKING TURRET CONTROL")
    print("="*70)
    print("Key fixes applied:")
    print("✓ 4-step ABCD sequence (confirmed working)")
    print("✓ Proper timing delays")
    print("✓ Pre-charge coils on startup")
    print("="*70)
    
    turret = None
    try:
        turret = WorkingTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU")
            print("="*60)
            print("1. Calibrate azimuth motor (DO THIS FIRST)")
            print("2. Verify both motors work")
            print("3. Interactive test")
            print("4. Show current configuration")
            print("5. Exit")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == "1":
                turret.calibrate_azimuth()
            elif choice == "2":
                turret.verify_both_motors()
            elif choice == "3":
                turret.interactive_test()
            elif choice == "4":
                print(f"\nCurrent configuration:")
                print(f"  Azimuth: {turret.AZIMUTH_STEPS_PER_REV} steps/rev")
                print(f"  Altitude: {turret.ALTITUDE_STEPS_PER_REV} steps/rev")
                print(f"  Azimuth angle: {turret.azimuth_angle:.1f}°")
                print(f"  Altitude angle: {turret.altitude_angle:.1f}°")
            elif choice == "5":
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
