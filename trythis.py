#!/usr/bin/env python3
"""
ENME441 Laser Turret - Fixed Motor Control
Corrected step sequence and motor handling
"""

import RPi.GPIO as GPIO
import time

class FixedTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 LASER TURRET - FIXED MOTOR CONTROL")
        print("="*70)
        
        # GPIO Pins for Shift Register
        self.SHIFT_CLK = 11  # GPIO11 -> 74HC595 Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> 74HC595 Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> 74HC595 Pin 14 (DS)
        
        # Motor Wiring (from shift register to motors):
        # Azimuth Motor (horizontal): Pins 15(Q0), 1(Q1), 2(Q2), 3(Q3)
        # Altitude Motor (vertical):  Pins 4(Q4), 5(Q5), 6(Q6), 7(Q7)
        
        # CORRECTED: Typical bipolar stepper values
        # Try 400 if motors have half-stepping, 200 if full-step
        self.AZIMUTH_STEPS_PER_REV = 400    # Try 200 or 400
        self.ALTITUDE_STEPS_PER_REV = 400   # Try 200 or 400
        
        # Current positions (in steps)
        self.azimuth_steps = 0
        self.altitude_steps = 0
        
        # Current angles (in degrees)
        self.azimuth_angle = 0.0   # 0 = pointing forward
        self.altitude_angle = 0.0  # 0 = horizontal
        
        # CORRECTED: Separate step sequences for each motor
        # Standard 4-step sequence for bipolar stepper (wave drive)
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
    
    def move_azimuth_degrees(self, degrees, step_delay=0.005):
        """
        Move azimuth motor by specified degrees
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
    
    def move_altitude_degrees(self, degrees, step_delay=0.005):
        """
        Move altitude motor by specified degrees
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
    
    def test_diagnostics(self):
        """Diagnostic test to check motors"""
        print("\n" + "="*60)
        print("DIAGNOSTIC MOTOR TEST")
        print("="*60)
        
        print("This will move each motor through one full sequence (4 steps)")
        print("to verify wiring and step sequence.")
        
        input("\nPress Enter to start azimuth diagnostic...")
        
        print("\n1. Testing azimuth motor sequence (4 steps forward):")
        for i in range(4):
            print(f"  Step {i+1}: Energizing coil {self.azimuth_sequence_pos}")
            self.update_motors()
            time.sleep(1)
            self.azimuth_sequence_pos = (self.azimuth_sequence_pos + 1) % 4
        
        input("\nPress Enter to start altitude diagnostic...")
        
        print("\n2. Testing altitude motor sequence (4 steps forward):")
        for i in range(4):
            print(f"  Step {i+1}: Energizing coil {self.altitude_sequence_pos + 4}")
            self.update_motors()
            time.sleep(1)
            self.altitude_sequence_pos = (self.altitude_sequence_pos + 1) % 4
        
        print("\n✓ Diagnostic complete!")
        print("Check that each motor stepped 4 times (one full rotation of the magnetic field)")
    
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
    print("ENME441 LASER TURRET - DIAGNOSTIC VERSION")
    print("="*70)
    
    turret = None
    try:
        # Initialize the turret
        turret = FixedTurret()
        
        # Diagnostic test first
        turret.test_diagnostics()
        
        # Then test with small movements
        print("\n" + "="*60)
        print("SMALL MOVEMENT TEST")
        print("="*60)
        
        print("Testing 90° movement for each motor...")
        
        input("\nPress Enter to test azimuth motor (90° right)...")
        turret.move_azimuth_degrees(90, 0.005)
        
        input("\nPress Enter to test altitude motor (90° up)...")
        turret.move_altitude_degrees(90, 0.005)
        
        print(f"\nFinal position:")
        print(f"  Azimuth: {turret.azimuth_angle:.1f}°")
        print(f"  Altitude: {turret.altitude_angle:.1f}°")
        
        # If movement is wrong, adjust steps per revolution
        actual_az_move = turret.azimuth_angle
        actual_alt_move = turret.altitude_angle
        
        if abs(actual_az_move - 90) > 5:  # More than 5° error
            print(f"\n⚠️  Azimuth error: Requested 90°, got {actual_az_move:.1f}°")
            print(f"   Current steps/rev: {turret.AZIMUTH_STEPS_PER_REV}")
            new_az = int(turret.AZIMUTH_STEPS_PER_REV * (90 / actual_az_move))
            print(f"   Suggested new value: {new_az}")
            turret.AZIMUTH_STEPS_PER_REV = new_az
        
        if abs(actual_alt_move - 90) > 5:  # More than 5° error
            print(f"\n⚠️  Altitude error: Requested 90°, got {actual_alt_move:.1f}°")
            print(f"   Current steps/rev: {turret.ALTITUDE_STEPS_PER_REV}")
            new_alt = int(turret.ALTITUDE_STEPS_PER_REV * (90 / actual_alt_move))
            print(f"   Suggested new value: {new_alt}")
            turret.ALTITUDE_STEPS_PER_REV = new_alt
    
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
