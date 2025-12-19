#!/usr/bin/env python3
"""
ENME441 Laser Turret - Calibrated Motor Control
Automatically calibrates steps per revolution
"""

import RPi.GPIO as GPIO
import time
import json
import os

class CalibratedTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 LASER TURRET - CALIBRATED MOTOR CONTROL")
        print("="*70)
        
        # GPIO Pins for Shift Register
        self.SHIFT_CLK = 11  # GPIO11 -> 74HC595 Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> 74HC595 Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> 74HC595 Pin 14 (DS)
        
        # Configuration file for saving calibrated values
        self.CONFIG_FILE = "turret_calibration.json"
        
        # Try to load calibration from file
        self.AZIMUTH_STEPS_PER_REV = 400  # Default starting value
        self.ALTITUDE_STEPS_PER_REV = 400
        
        self.load_calibration()
        
        print(f"Loaded calibration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current positions (in steps)
        self.azimuth_steps = 0
        self.altitude_steps = 0
        
        # Current angles (in degrees)
        self.azimuth_angle = 0.0   # 0 = pointing forward
        self.altitude_angle = 0.0  # 0 = horizontal
        
        # Step sequences for each motor
        self.AZIMUTH_SEQUENCE = [
            0b00000001,  # Q0 (Pin 15) - Azimuth coil A
            0b00000010,  # Q1 (Pin 1)  - Azimuth coil B
            0b00000100,  # Q2 (Pin 2)  - Azimuth coil C
            0b00001000,  # Q3 (Pin 3)  - Azimuth coil D
        ]
        
        self.ALTITUDE_SEQUENCE = [
            0b00010000,  # Q4 (Pin 4)  - Altitude coil A
            0b00100000,  # Q5 (Pin 5)  - Altitude coil B
            0b01000000,  # Q6 (Pin 6)  - Altitude coil C
            0b10000000,  # Q7 (Pin 7)  - Altitude coil D
        ]
        
        # Current step in sequence
        self.azimuth_sequence_pos = 0
        self.altitude_sequence_pos = 0
        
        # Initialize GPIO
        self.setup_gpio()
        
        print(f"\nSystem initialized!")
        print(f"Azimuth position: {self.azimuth_angle:.1f}°")
        print(f"Altitude position: {self.altitude_angle:.1f}°")
        print("="*70)
    
    def load_calibration(self):
        """Load calibration values from file"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 400)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 400)
                print(f"✓ Loaded calibration from {self.CONFIG_FILE}")
        except Exception as e:
            print(f"Note: Could not load calibration: {e}")
            print("Using default values")
    
    def save_calibration(self):
        """Save calibration values to file"""
        try:
            config = {
                'azimuth_steps_per_rev': self.AZIMUTH_STEPS_PER_REV,
                'altitude_steps_per_rev': self.ALTITUDE_STEPS_PER_REV
            }
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"✓ Calibration saved to {self.CONFIG_FILE}")
        except Exception as e:
            print(f"Warning: Could not save calibration: {e}")
    
    def setup_gpio(self):
        """Initialize all GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        
        # Setup shift register pins
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Initialize pins to LOW
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Turn off all motors initially
        self.send_to_shift_register(0b00000000)
        
        print("✓ GPIO pins initialized")
        print("✓ Shift register ready")
        print("✓ Motors OFF")
    
    def send_to_shift_register(self, data):
        """
        Send 8 bits to the 74HC595 shift register
        """
        # Pull latch low to start sending data
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        # Send each bit, MSB first
        for i in range(7, -1, -1):
            # Get the bit value
            bit = (data >> i) & 0x01
            
            # Set data pin
            GPIO.output(self.DATA_PIN, bit)
            
            # Pulse the clock pin
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.00001)  # 10 microseconds
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        # Pull latch high to update outputs
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors with current sequence positions"""
        # Get azimuth bits
        azimuth_bits = self.AZIMUTH_SEQUENCE[self.azimuth_sequence_pos]
        
        # Get altitude bits
        altitude_bits = self.ALTITUDE_SEQUENCE[self.altitude_sequence_pos]
        
        # Combine and send to shift register
        combined = azimuth_bits | altitude_bits
        self.send_to_shift_register(combined)
    
    def step_azimuth(self, direction):
        """
        Move azimuth motor one step
        direction: 1 for clockwise, -1 for counterclockwise
        """
        # Update sequence position
        self.azimuth_sequence_pos = (self.azimuth_sequence_pos + direction) % 4
        
        # Update step count
        self.azimuth_steps += direction
        
        # Update angle
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        
        # Update motors
        self.update_motors()
        
        # Small delay for motor to respond
        time.sleep(0.001)
    
    def step_altitude(self, direction):
        """
        Move altitude motor one step
        direction: 1 for up, -1 for down
        """
        # Update sequence position
        self.altitude_sequence_pos = (self.altitude_sequence_pos + direction) % 4
        
        # Update step count
        self.altitude_steps += direction
        
        # Update angle
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        
        # Update motors
        self.update_motors()
        
        # Small delay for motor to respond
        time.sleep(0.001)
    
    def move_azimuth_degrees(self, degrees, step_delay=0.003):
        """
        Move azimuth motor by specified degrees
        Returns the actual angle moved
        """
        # Save starting position
        start_angle = self.azimuth_angle
        
        # Calculate number of steps needed
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nMoving azimuth {degrees:.1f}°")
        print(f"Starting at: {start_angle:.1f}°")
        print(f"Calculated steps: {steps} (based on {self.AZIMUTH_STEPS_PER_REV} steps/rev)")
        
        # Take the steps
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(step_delay)
        
        # Calculate actual movement
        actual_movement = self.azimuth_angle - start_angle
        print(f"Movement complete:")
        print(f"  Requested: {degrees:.1f}°")
        print(f"  Actual:    {actual_movement:.1f}°")
        print(f"  Error:     {abs(degrees - actual_movement):.1f}°")
        print(f"  Final position: {self.azimuth_angle:.1f}°")
        
        return actual_movement
    
    def move_altitude_degrees(self, degrees, step_delay=0.003):
        """
        Move altitude motor by specified degrees
        Returns the actual angle moved
        """
        # Save starting position
        start_angle = self.altitude_angle
        
        # Calculate number of steps needed
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nMoving altitude {degrees:.1f}°")
        print(f"Starting at: {start_angle:.1f}°")
        print(f"Calculated steps: {steps} (based on {self.ALTITUDE_STEPS_PER_REV} steps/rev)")
        
        # Take the steps
        for i in range(steps):
            self.step_altitude(direction)
            time.sleep(step_delay)
        
        # Calculate actual movement
        actual_movement = self.altitude_angle - start_angle
        print(f"Movement complete:")
        print(f"  Requested: {degrees:.1f}°")
        print(f"  Actual:    {actual_movement:.1f}°")
        print(f"  Error:     {abs(degrees - actual_movement):.1f}°")
        print(f"  Final position: {self.altitude_angle:.1f}°")
        
        return actual_movement
    
    def calibrate_motor(self, motor_type, test_angle=90.0):
        """
        Calibrate steps per revolution for a specific motor
        motor_type: 'azimuth' or 'altitude'
        test_angle: angle to test movement (degrees)
        """
        print(f"\n{'='*60}")
        print(f"CALIBRATING {motor_type.upper()} MOTOR")
        print(f"{'='*60}")
        
        # Move to a known starting position
        if motor_type == 'azimuth':
            self.move_azimuth_degrees(-self.azimuth_angle)  # Return to 0
            print(f"\nStarting calibration for azimuth motor...")
            print(f"Testing movement of {test_angle}°")
            
            # Perform test movement
            actual_movement = self.move_azimuth_degrees(test_angle)
            
            # Calculate correction factor
            if abs(actual_movement) > 0.1:  # Avoid division by zero
                correction_factor = test_angle / actual_movement
                old_value = self.AZIMUTH_STEPS_PER_REV
                self.AZIMUTH_STEPS_PER_REV = int(old_value * correction_factor)
                
                print(f"\nCalibration results:")
                print(f"  Requested: {test_angle:.1f}°")
                print(f"  Actual:    {actual_movement:.1f}°")
                print(f"  Correction factor: {correction_factor:.3f}")
                print(f"  Old steps/rev: {old_value}")
                print(f"  New steps/rev: {self.AZIMUTH_STEPS_PER_REV}")
                
                # Save calibration
                self.save_calibration()
                
                # Return to test the new calibration
                print(f"\nTesting new calibration...")
                self.move_azimuth_degrees(-actual_movement)  # Return to start
                new_actual = self.move_azimuth_degrees(test_angle)
                print(f"  New actual movement: {new_actual:.1f}°")
                print(f"  New error: {abs(test_angle - new_actual):.1f}°")
            else:
                print("Error: Motor didn't move enough for calibration")
        
        elif motor_type == 'altitude':
            self.move_altitude_degrees(-self.altitude_angle)  # Return to 0
            print(f"\nStarting calibration for altitude motor...")
            print(f"Testing movement of {test_angle}°")
            
            # Perform test movement
            actual_movement = self.move_altitude_degrees(test_angle)
            
            # Calculate correction factor
            if abs(actual_movement) > 0.1:  # Avoid division by zero
                correction_factor = test_angle / actual_movement
                old_value = self.ALTITUDE_STEPS_PER_REV
                self.ALTITUDE_STEPS_PER_REV = int(old_value * correction_factor)
                
                print(f"\nCalibration results:")
                print(f"  Requested: {test_angle:.1f}°")
                print(f"  Actual:    {actual_movement:.1f}°")
                print(f"  Correction factor: {correction_factor:.3f}")
                print(f"  Old steps/rev: {old_value}")
                print(f"  New steps/rev: {self.ALTITUDE_STEPS_PER_REV}")
                
                # Save calibration
                self.save_calibration()
                
                # Return to test the new calibration
                print(f"\nTesting new calibration...")
                self.move_altitude_degrees(-actual_movement)  # Return to start
                new_actual = self.move_altitude_degrees(test_angle)
                print(f"  New actual movement: {new_actual:.1f}°")
                print(f"  New error: {abs(test_angle - new_actual):.1f}°")
            else:
                print("Error: Motor didn't move enough for calibration")
        
        print(f"\n✓ {motor_type.capitalize()} motor calibration complete!")
    
    def auto_calibrate_both(self):
        """Automatically calibrate both motors"""
        print("\n" + "="*70)
        print("AUTO-CALIBRATION FOR BOTH MOTORS")
        print("="*70)
        print("This will calibrate each motor using a 90° test movement.")
        print("Make sure the turret has clear space to move!")
        
        input("\nPress Enter to start calibration...")
        
        # Calibrate azimuth first
        self.calibrate_motor('azimuth', 90.0)
        
        input("\nPress Enter to calibrate altitude motor...")
        
        # Calibrate altitude
        self.calibrate_motor('altitude', 90.0)
        
        print("\n" + "="*70)
        print("CALIBRATION COMPLETE!")
        print("="*70)
        print(f"Final calibration values:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/revolution")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/revolution")
        print(f"\nCalibration saved to: {self.CONFIG_FILE}")
    
    def test_accuracy(self, test_angles=[45, 90, 180]):
        """Test accuracy with various angles"""
        print("\n" + "="*70)
        print("ACCURACY TEST")
        print("="*70)
        
        print(f"Current calibration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Test azimuth
        print("\n" + "-"*40)
        print("AZIMUTH ACCURACY TEST")
        print("-"*40)
        
        # Return to 0 first
        self.move_azimuth_degrees(-self.azimuth_angle)
        
        for angle in test_angles:
            print(f"\nTesting {angle}° movement:")
            actual = self.move_azimuth_degrees(angle)
            error = abs(angle - actual)
            print(f"  Error: {error:.1f}° ({error/angle*100:.1f}%)")
        
        # Test altitude
        print("\n" + "-"*40)
        print("ALTITUDE ACCURACY TEST")
        print("-"*40)
        
        # Return to 0 first
        self.move_altitude_degrees(-self.altitude_angle)
        
        for angle in test_angles:
            print(f"\nTesting {angle}° movement:")
            actual = self.move_altitude_degrees(angle)
            error = abs(angle - actual)
            print(f"  Error: {error:.1f}° ({error/angle*100:.1f}%)")
        
        # Return both to 0
        self.move_azimuth_degrees(-self.azimuth_angle)
        self.move_altitude_degrees(-self.altitude_angle)
        
        print(f"\n✓ Accuracy test complete!")
        print(f"Final position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
    
    def cleanup(self):
        """Clean up GPIO and turn off motors"""
        print("\nCleaning up...")
        self.send_to_shift_register(0b00000000)  # Turn off all coils
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ GPIO cleanup complete")
        print("✓ Motors turned off")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 LASER TURRET - CALIBRATION TOOL")
    print("="*70)
    print("This tool will help you calibrate your motors.")
    print("If motors move half the requested distance,")
    print("this will automatically correct it!")
    print("="*70)
    
    turret = None
    try:
        # Initialize the turret
        turret = CalibratedTurret()
        
        while True:
            print("\n" + "="*70)
            print("MAIN MENU")
            print("="*70)
            print("1. Auto-calibrate both motors (RECOMMENDED FIRST)")
            print("2. Test accuracy with various angles")
            print("3. Manually set steps per revolution")
            print("4. Quick test: Move 90° each axis")
            print("5. Return to home (0°, 0°)")
            print("6. Exit and cleanup")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                turret.auto_calibrate_both()
            elif choice == "2":
                turret.test_accuracy()
            elif choice == "3":
                print(f"\nCurrent values:")
                print(f"  Azimuth: {turret.AZIMUTH_STEPS_PER_REV}")
                print(f"  Altitude: {turret.ALTITUDE_STEPS_PER_REV}")
                try:
                    new_az = int(input("\nEnter new azimuth steps/rev: "))
                    new_alt = int(input("Enter new altitude steps/rev: "))
                    turret.AZIMUTH_STEPS_PER_REV = new_az
                    turret.ALTITUDE_STEPS_PER_REV = new_alt
                    turret.save_calibration()
                    print("✓ Values updated and saved!")
                except ValueError:
                    print("Invalid input. Using current values.")
            elif choice == "4":
                print("\nQuick test: Moving 90° on each axis...")
                turret.move_azimuth_degrees(90)
                turret.move_altitude_degrees(90)
                print(f"\nFinal position: Az={turret.azimuth_angle:.1f}°, Alt={turret.altitude_angle:.1f}°")
            elif choice == "5":
                print("\nReturning to home...")
                turret.move_azimuth_degrees(-turret.azimuth_angle)
                turret.move_altitude_degrees(-turret.altitude_angle)
                print(f"✓ At home: Az={turret.azimuth_angle:.1f}°, Alt={turret.altitude_angle:.1f}°")
            elif choice == "6":
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please enter 1-6.")
    
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if turret:
            turret.cleanup()
        print("\nProgram ended.")

if __name__ == "__main__":
    main()
