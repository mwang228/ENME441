#!/usr/bin/env python3
"""
ENME441 Laser Turret - Manual Motor Control (Feature 2)
Basic implementation - starting fresh
"""

import RPi.GPIO as GPIO
import time

class BasicTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 LASER TURRET - BASIC MANUAL CONTROL")
        print("="*70)
        print("Feature 2: Manual control of motor angles")
        print("="*70)
        
        # GPIO Pins for Shift Register
        self.SHIFT_CLK = 11  # GPIO11 -> 74HC595 Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> 74HC595 Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> 74HC595 Pin 14 (DS)
        
        # Motor Wiring (from shift register to motors):
        # Azimuth Motor (horizontal): Pins 15(Q0), 1(Q1), 2(Q2), 3(Q3)
        # Altitude Motor (vertical):  Pins 4(Q4), 5(Q5), 6(Q6), 7(Q7)
        
        # Motor Specifications (to be determined through testing)
        self.AZIMUTH_STEPS_PER_REV = 200     # Common stepper value - we'll adjust
        self.ALTITUDE_STEPS_PER_REV = 200    # Same assumption for now
        
        # Current positions (in steps)
        self.azimuth_steps = 0
        self.altitude_steps = 0
        
        # Current angles (in degrees)
        self.azimuth_angle = 0.0   # 0 = pointing forward
        self.altitude_angle = 0.0  # 0 = horizontal
        
        # Step sequence for 4-wire bipolar stepper
        # Each step energizes two coils for more torque
        self.STEP_SEQUENCE = [
            0b00010001,  # Az: coil A, Alt: coil A
            0b00100010,  # Az: coil B, Alt: coil B  
            0b01000100,  # Az: coil C, Alt: coil C
            0b10001000,  # Az: coil D, Alt: coil D
        ]
        
        # Current step in sequence
        self.azimuth_sequence_pos = 0
        self.altitude_sequence_pos = 0
        
        # Initialize GPIO
        self.setup_gpio()
        
        print("System initialized successfully!")
        print(f"Azimuth position: {self.azimuth_angle:.1f}°")
        print(f"Altitude position: {self.altitude_angle:.1f}°")
        print("="*70)
    
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
        Bit mapping:
          Bit 0 (LSB): Azimuth coil A (Pin 15)
          Bit 1:       Azimuth coil B (Pin 1)
          Bit 2:       Azimuth coil C (Pin 2)
          Bit 3:       Azimuth coil D (Pin 3)
          Bit 4:       Altitude coil A (Pin 4)
          Bit 5:       Altitude coil B (Pin 5)
          Bit 6:       Altitude coil C (Pin 6)
          Bit 7:       Altitude coil D (Pin 7)
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
        # Get azimuth bits (lower 4 bits)
        azimuth_bits = self.STEP_SEQUENCE[self.azimuth_sequence_pos] & 0b00001111
        
        # Get altitude bits (upper 4 bits)
        altitude_bits = self.STEP_SEQUENCE[self.altitude_sequence_pos] & 0b11110000
        
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
    
    def move_azimuth_degrees(self, degrees, step_delay=0.01):
        """
        Move azimuth motor by specified degrees
        Positive degrees = clockwise, Negative = counterclockwise
        """
        # Calculate number of steps needed
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"Moving azimuth {degrees:.1f}°")
        print(f"This requires {steps} steps")
        print(f"Direction: {'clockwise' if direction > 0 else 'counterclockwise'}")
        
        # Take the steps
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(step_delay)
            
            # Show progress every 10 steps
            if (i + 1) % 10 == 0:
                print(f"  Step {i+1}/{steps}, Current angle: {self.azimuth_angle:.1f}°")
        
        print(f"✓ Azimuth movement complete: {self.azimuth_angle:.1f}°")
    
    def move_altitude_degrees(self, degrees, step_delay=0.01):
        """
        Move altitude motor by specified degrees
        Positive degrees = up, Negative = down
        """
        # Calculate number of steps needed
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"Moving altitude {degrees:.1f}°")
        print(f"This requires {steps} steps")
        print(f"Direction: {'up' if direction > 0 else 'down'}")
        
        # Take the steps
        for i in range(steps):
            self.step_altitude(direction)
            time.sleep(step_delay)
            
            # Show progress every 10 steps
            if (i + 1) % 10 == 0:
                print(f"  Step {i+1}/{steps}, Current angle: {self.altitude_angle:.1f}°")
        
        print(f"✓ Altitude movement complete: {self.altitude_angle:.1f}°")
    
    def move_both_degrees(self, az_degrees, alt_degrees, step_delay=0.01):
        """
        Move both motors by specified degrees simultaneously
        """
        # Calculate steps for each motor
        az_steps = int((az_degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        alt_steps = int((alt_degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        
        az_direction = 1 if az_steps > 0 else -1
        alt_direction = 1 if alt_steps > 0 else -1
        
        az_steps = abs(az_steps)
        alt_steps = abs(alt_steps)
        
        print(f"Moving both motors:")
        print(f"  Azimuth: {az_degrees:.1f}° ({az_steps} steps)")
        print(f"  Altitude: {alt_degrees:.1f}° ({alt_steps} steps)")
        
        # Find which motor needs more steps
        max_steps = max(az_steps, alt_steps)
        
        # Move both motors
        az_completed = 0
        alt_completed = 0
        
        for i in range(max_steps):
            # Move azimuth if needed
            if az_completed < az_steps:
                self.step_azimuth(az_direction)
                az_completed += 1
            
            # Move altitude if needed
            if alt_completed < alt_steps:
                self.step_altitude(alt_direction)
                alt_completed += 1
            
            time.sleep(step_delay)
            
            # Show progress every 20 combined steps
            if (i + 1) % 20 == 0:
                print(f"  Progress: Az {az_completed}/{az_steps}, Alt {alt_completed}/{alt_steps}")
        
        print(f"✓ Both movements complete!")
        print(f"  Azimuth: {self.azimuth_angle:.1f}°")
        print(f"  Altitude: {self.altitude_angle:.1f}°")
    
    def set_azimuth_angle(self, angle):
        """Set azimuth motor to specific angle"""
        current_angle = self.azimuth_angle
        degrees_to_move = angle - current_angle
        
        print(f"Setting azimuth from {current_angle:.1f}° to {angle:.1f}°")
        print(f"Need to move {degrees_to_move:.1f}°")
        
        self.move_azimuth_degrees(degrees_to_move)
    
    def set_altitude_angle(self, angle):
        """Set altitude motor to specific angle"""
        current_angle = self.altitude_angle
        degrees_to_move = angle - current_angle
        
        print(f"Setting altitude from {current_angle:.1f}° to {angle:.1f}°")
        print(f"Need to move {degrees_to_move:.1f}°")
        
        self.move_altitude_degrees(degrees_to_move)
    
    def set_both_angles(self, az_angle, alt_angle):
        """Set both motors to specific angles"""
        print(f"Setting motors to:")
        print(f"  Azimuth: {az_angle:.1f}° (currently {self.azimuth_angle:.1f}°)")
        print(f"  Altitude: {alt_angle:.1f}° (currently {self.altitude_angle:.1f}°)")
        
        az_move = az_angle - self.azimuth_angle
        alt_move = alt_angle - self.altitude_angle
        
        self.move_both_degrees(az_move, alt_move)
    
    def test_basic_movement(self):
        """Basic test to verify motors work"""
        print("\n" + "="*60)
        print("BASIC MOTOR TEST")
        print("="*60)
        
        print("This will test each motor with small movements.")
        print("Mark the starting position so you can see movement.")
        
        input("\nPress Enter to start azimuth test...")
        
        # Test azimuth
        print("\n1. Testing azimuth motor (+45°, then -45°)")
        self.move_azimuth_degrees(45, 0.02)
        time.sleep(1)
        self.move_azimuth_degrees(-45, 0.02)
        
        input("\nPress Enter to start altitude test...")
        
        # Test altitude
        print("\n2. Testing altitude motor (+45°, then -45°)")
        self.move_altitude_degrees(45, 0.02)
        time.sleep(1)
        self.move_altitude_degrees(-45, 0.02)
        
        print("\n✓ Basic test complete!")
        print(f"Final position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
    
    def manual_control_interface(self):
        """Main manual control interface"""
        print("\n" + "="*70)
        print("MANUAL MOTOR CONTROL INTERFACE")
        print("="*70)
        
        while True:
            print(f"\nCurrent Position:")
            print(f"  Azimuth: {self.azimuth_angle:.1f}°")
            print(f"  Altitude: {self.altitude_angle:.1f}°")
            
            print("\nOptions:")
            print("1. Move azimuth motor only")
            print("2. Move altitude motor only")
            print("3. Move both motors")
            print("4. Set exact angles for both motors")
            print("5. Test basic movement (45° each way)")
            print("6. Return to home (0°, 0°)")
            print("7. Exit manual control")
            
            choice = input("\nEnter your choice (1-7): ").strip()
            
            if choice == "1":
                self.control_azimuth_only()
            elif choice == "2":
                self.control_altitude_only()
            elif choice == "3":
                self.control_both_motors()
            elif choice == "4":
                self.set_exact_angles()
            elif choice == "5":
                self.test_basic_movement()
            elif choice == "6":
                self.return_to_home()
            elif choice == "7":
                print("Exiting manual control...")
                break
            else:
                print("Invalid choice. Please enter 1-7.")
    
    def control_azimuth_only(self):
        """Control azimuth motor only"""
        print("\n--- Azimuth Motor Control ---")
        print("Positive degrees = clockwise (right)")
        print("Negative degrees = counterclockwise (left)")
        
        try:
            degrees = float(input("Enter degrees to move: "))
            self.move_azimuth_degrees(degrees, 0.01)
        except ValueError:
            print("Invalid input. Please enter a number.")
    
    def control_altitude_only(self):
        """Control altitude motor only"""
        print("\n--- Altitude Motor Control ---")
        print("Positive degrees = up")
        print("Negative degrees = down")
        
        try:
            degrees = float(input("Enter degrees to move: "))
            self.move_altitude_degrees(degrees, 0.01)
        except ValueError:
            print("Invalid input. Please enter a number.")
    
    def control_both_motors(self):
        """Control both motors"""
        print("\n--- Both Motors Control ---")
        
        try:
            az_degrees = float(input("Enter azimuth degrees to move: "))
            alt_degrees = float(input("Enter altitude degrees to move: "))
            self.move_both_degrees(az_degrees, alt_degrees, 0.01)
        except ValueError:
            print("Invalid input. Please enter numbers.")
    
    def set_exact_angles(self):
        """Set exact angles for both motors"""
        print("\n--- Set Exact Angles ---")
        
        try:
            az_angle = float(input("Enter target azimuth angle (degrees): "))
            alt_angle = float(input("Enter target altitude angle (degrees): "))
            self.set_both_angles(az_angle, alt_angle)
        except ValueError:
            print("Invalid input. Please enter numbers.")
    
    def return_to_home(self):
        """Return both motors to home position (0°, 0°)"""
        print("\nReturning to home position (0°, 0°)...")
        self.set_both_angles(0, 0)
        print("✓ At home position")
    
    def adjust_steps_per_revolution(self):
        """Adjust steps per revolution if movement is wrong"""
        print("\n" + "="*60)
        print("ADJUST STEPS PER REVOLUTION")
        print("="*60)
        
        print(f"Current values:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/revolution")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/revolution")
        
        print("\nIf motors don't move the expected distance,")
        print("you may need to adjust these values.")
        
        try:
            new_az = int(input("\nEnter new azimuth steps/rev (or press Enter to keep current): ") or self.AZIMUTH_STEPS_PER_REV)
            new_alt = int(input("Enter new altitude steps/rev (or press Enter to keep current): ") or self.ALTITUDE_STEPS_PER_REV)
            
            self.AZIMUTH_STEPS_PER_REV = new_az
            self.ALTITUDE_STEPS_PER_REV = new_alt
            
            print(f"\n✓ Updated values:")
            print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/revolution")
            print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/revolution")
            
            # Recalculate current angles with new values
            self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
            self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
            
            print(f"Recalculated position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
            
        except ValueError:
            print("Invalid input. Using current values.")
    
    def cleanup(self):
        """Clean up GPIO and turn off motors"""
        print("\nCleaning up...")
        self.send_to_shift_register(0b00000000)  # Turn off all coils
        GPIO.cleanup()
        print("✓ GPIO cleanup complete")
        print("✓ Motors turned off")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 LASER TURRET - MANUAL MOTOR CONTROL")
    print("="*70)
    print("Starting from scratch - basic implementation")
    print("\nWiring Check:")
    print("  RPi GPIO 11 → 74HC595 Pin 11 (SH_CP)")
    print("  RPi GPIO 10 → 74HC595 Pin 12 (ST_CP)")
    print("  RPi GPIO 9  → 74HC595 Pin 14 (DS)")
    print("  3.3V/5V     → 74HC595 Pin 16 (VCC)")
    print("  GND         → 74HC595 Pins 8 & 13 (GND & OE)")
    print("\nMotor Connections:")
    print("  Azimuth Motor → Pins 15, 1, 2, 3")
    print("  Altitude Motor → Pins 4, 5, 6, 7")
    print("="*70)
    
    turret = None
    try:
        # Initialize the turret
        turret = BasicTurret()
        
        # Main loop
        while True:
            print("\n" + "="*70)
            print("MAIN MENU")
            print("="*70)
            print("1. Manual motor control (Feature 2)")
            print("2. Test basic movement")
            print("3. Adjust steps per revolution")
            print("4. Return to home position")
            print("5. Exit and cleanup")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == "1":
                turret.manual_control_interface()
            elif choice == "2":
                turret.test_basic_movement()
            elif choice == "3":
                turret.adjust_steps_per_revolution()
            elif choice == "4":
                turret.return_to_home()
            elif choice == "5":
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please enter 1-5.")
    
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
