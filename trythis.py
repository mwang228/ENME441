#!/usr/bin/env python3
"""
STANDALONE MOTOR TEST - Manual Angle Adjustment Only
Tests exactly what Option 2 should do
"""

import RPi.GPIO as GPIO
import time

class MotorTestOnly:
    def __init__(self):
        print("="*70)
        print("STANDALONE MOTOR TEST - Manual Angle Adjustment")
        print("="*70)
        
        # GPIO pins
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS (Pin 14)
        
        # Motor specs from your project
        self.AZIMUTH_STEPS_PER_REV = 1024    # Fast motor
        self.ALTITUDE_STEPS_PER_REV = 4096   # Standard motor
        
        # IMPORTANT: Let's track what's actually happening
        self.azimuth_position = 0  # Steps from home
        self.altitude_position = 0
        
        # Current angles
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # Motor limits
        self.MAX_AZIMUTH_LEFT = -120
        self.MAX_AZIMUTH_RIGHT = 120
        
        # Step sequence - Let's try DIFFERENT approaches
        print("\nTesting different step sequences...")
        
        # Approach 1: Simple 4-step (what we've been using)
        self.STEP_SEQUENCE_4 = [
            0b00010001,  # Az=0001, Alt=0001
            0b00100010,  # Az=0010, Alt=0010
            0b01000100,  # Az=0100, Alt=0100
            0b10001000,  # Az=1000, Alt=1000
        ]
        
        # Approach 2: Wave drive (simpler)
        self.STEP_SEQUENCE_WAVE = [
            0b00010001,  # Only coil A on
            0b00100010,  # Only coil B on
            0b01000100,  # Only coil C on
            0b10001000,  # Only coil D on
        ]
        
        # Approach 3: Two-phase (both coils always on)
        self.STEP_SEQUENCE_2PHASE = [
            0b00110011,  # Coils A+B
            0b01100110,  # Coils B+C
            0b11001100,  # Coils C+D
            0b10011001,  # Coils D+A
        ]
        
        # Current approach
        self.current_sequence = self.STEP_SEQUENCE_4
        self.sequence_name = "4-step"
        
        # Step indices
        self.azimuth_step_idx = 0
        self.altitude_step_idx = 0
        
        # Setup
        self.setup_gpio()
        print("Ready for testing!")
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
    
    def update_motors(self):
        """Update both motors"""
        combined = (self.current_sequence[self.azimuth_step_idx % 4] & 0b00001111) | \
                   (self.current_sequence[self.altitude_step_idx % 4] & 0b11110000)
        self.shift_out(combined)
    
    def test_direct_control(self):
        """Test motors with direct step commands"""
        print("\n" + "="*60)
        print("DIRECT MOTOR CONTROL TEST")
        print("="*60)
        
        print(f"Using sequence: {self.sequence_name}")
        print(f"Current position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
        
        print("\n1. Test AZIMUTH motor only")
        print("2. Test ALTITUDE motor only")
        print("3. Test BOTH motors")
        print("4. Change step sequence")
        
        choice = input("\nEnter choice (1-4): ").strip()
        
        if choice == "1":
            self.test_azimuth_only()
        elif choice == "2":
            self.test_altitude_only()
        elif choice == "3":
            self.test_both_motors()
        elif choice == "4":
            self.change_sequence()
    
    def test_azimuth_only(self):
        """Test azimuth motor with direct control"""
        print("\n" + "="*40)
        print("AZIMUTH MOTOR TEST")
        print("="*40)
        
        print("Options:")
        print("1. Move +90° (clockwise)")
        print("2. Move -90° (counterclockwise)")
        print("3. Move +45°")
        print("4. Move -45°")
        print("5. Custom angle")
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            degrees = 90
        elif choice == "2":
            degrees = -90
        elif choice == "3":
            degrees = 45
        elif choice == "4":
            degrees = -45
        elif choice == "5":
            try:
                degrees = float(input("Enter angle in degrees: "))
            except:
                print("Invalid input")
                return
        else:
            print("Invalid choice")
            return
        
        # Calculate steps
        steps = int(abs(degrees) * self.AZIMUTH_STEPS_PER_REV / 360)
        direction = 1 if degrees > 0 else -1
        
        print(f"\nMoving azimuth {degrees:.1f}°")
        print(f"That's {steps} steps ({steps/4} full sequence cycles)")
        print(f"Direction: {'clockwise' if direction > 0 else 'counterclockwise'}")
        
        input("Press Enter to start movement...")
        
        start_angle = self.azimuth_angle
        
        for i in range(steps):
            self.azimuth_step_idx = (self.azimuth_step_idx + direction) % 4
            self.azimuth_position += direction
            self.update_motors()
            time.sleep(0.001)  # 1ms per step
            
            # Update angle display
            self.azimuth_angle = self.azimuth_position * 360.0 / self.AZIMUTH_STEPS_PER_REV
            if i % 50 == 0:
                print(f"  Step {i+1}/{steps}, Angle: {self.azimuth_angle:.1f}°")
        
        print(f"\n✓ Movement complete!")
        print(f"Started at: {start_angle:.1f}°")
        print(f"Ended at: {self.azimuth_angle:.1f}°")
        print(f"Actual movement: {self.azimuth_angle - start_angle:.1f}°")
    
    def test_altitude_only(self):
        """Test altitude motor with direct control"""
        print("\n" + "="*40)
        print("ALTITUDE MOTOR TEST")
        print("="*40)
        
        print("Options:")
        print("1. Move +90° (up)")
        print("2. Move -90° (down)")
        print("3. Move +45°")
        print("4. Move -45°")
        print("5. Custom angle")
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            degrees = 90
        elif choice == "2":
            degrees = -90
        elif choice == "3":
            degrees = 45
        elif choice == "4":
            degrees = -45
        elif choice == "5":
            try:
                degrees = float(input("Enter angle in degrees: "))
            except:
                print("Invalid input")
                return
        else:
            print("Invalid choice")
            return
        
        # Calculate steps - IMPORTANT: Let's try HALF the steps
        # Since altitude was moving double, maybe steps calculation is wrong
        steps_full = int(abs(degrees) * self.ALTITUDE_STEPS_PER_REV / 360)
        steps_half = steps_full // 2  # Try half steps
        
        print("\nWhich step count to use?")
        print(f"1. Full steps: {steps_full} steps (might move double)")
        print(f"2. Half steps: {steps_half} steps (trying to fix overshoot)")
        
        step_choice = input("Enter choice (1-2): ").strip()
        
        if step_choice == "1":
            steps = steps_full
            step_type = "full"
        else:
            steps = steps_half
            step_type = "half"
        
        direction = 1 if degrees > 0 else -1
        
        print(f"\nMoving altitude {degrees:.1f}° using {step_type} steps")
        print(f"That's {steps} steps ({steps/4} full sequence cycles)")
        
        input("Press Enter to start movement...")
        
        start_angle = self.altitude_angle
        
        for i in range(steps):
            self.altitude_step_idx = (self.altitude_step_idx + direction) % 4
            self.altitude_position += direction
            self.update_motors()
            time.sleep(0.001)
            
            # Update angle
            if step_type == "full":
                self.altitude_angle = self.altitude_position * 360.0 / self.ALTITUDE_STEPS_PER_REV
            else:
                # If using half steps, need to adjust calculation
                self.altitude_angle = self.altitude_position * 360.0 / (self.ALTITUDE_STEPS_PER_REV // 2)
            
            if i % 100 == 0:
                print(f"  Step {i+1}/{steps}, Angle: {self.altitude_angle:.1f}°")
        
        print(f"\n✓ Movement complete!")
        print(f"Started at: {start_angle:.1f}°")
        print(f"Ended at: {self.altitude_angle:.1f}°")
        print(f"Actual movement: {self.altitude_angle - start_angle:.1f}°")
        print(f"Expected: {degrees:.1f}°")
    
    def test_both_motors(self):
        """Test both motors together"""
        print("\n" + "="*40)
        print("BOTH MOTORS TEST")
        print("="*40)
        
        print("Enter angles for both motors")
        
        try:
            az_degrees = float(input("Azimuth angle change (degrees): "))
            alt_degrees = float(input("Altitude angle change (degrees): "))
        except:
            print("Invalid input")
            return
        
        # Calculate steps
        az_steps = int(abs(az_degrees) * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(abs(alt_degrees) * self.ALTITUDE_STEPS_PER_REV / 360)
        
        # Try HALF steps for altitude
        alt_steps = alt_steps // 2
        
        az_dir = 1 if az_degrees > 0 else -1
        alt_dir = 1 if alt_degrees > 0 else -1
        
        print(f"\nMoving:")
        print(f"  Azimuth: {az_degrees:.1f}° = {az_steps} steps")
        print(f"  Altitude: {alt_degrees:.1f}° = {alt_steps} steps (using half steps)")
        
        input("Press Enter to start...")
        
        az_start = self.azimuth_angle
        alt_start = self.altitude_angle
        
        # Determine max steps
        max_steps = max(az_steps, alt_steps)
        
        az_completed = 0
        alt_completed = 0
        
        for i in range(max_steps):
            # Move azimuth if needed
            if az_completed < az_steps:
                self.azimuth_step_idx = (self.azimuth_step_idx + az_dir) % 4
                self.azimuth_position += az_dir
                az_completed += 1
            
            # Move altitude if needed
            if alt_completed < alt_steps:
                self.altitude_step_idx = (self.altitude_step_idx + alt_dir) % 4
                self.altitude_position += alt_dir
                alt_completed += 1
            
            # Update motors
            self.update_motors()
            time.sleep(0.001)
            
            # Update angles
            self.azimuth_angle = self.azimuth_position * 360.0 / self.AZIMUTH_STEPS_PER_REV
            self.altitude_angle = self.altitude_position * 360.0 / (self.ALTITUDE_STEPS_PER_REV // 2)
            
            if i % 50 == 0:
                print(f"  Progress: Az {az_completed}/{az_steps}, Alt {alt_completed}/{alt_steps}")
        
        print(f"\n✓ Movement complete!")
        print(f"Azimuth: {az_start:.1f}° → {self.azimuth_angle:.1f}° (Δ={self.azimuth_angle - az_start:.1f}°)")
        print(f"Altitude: {alt_start:.1f}° → {self.altitude_angle:.1f}° (Δ={self.altitude_angle - alt_start:.1f}°)")
    
    def change_sequence(self):
        """Change step sequence"""
        print("\n" + "="*40)
        print("CHANGE STEP SEQUENCE")
        print("="*40)
        
        print("Current sequence: " + self.sequence_name)
        print("\nAvailable sequences:")
        print("1. 4-step sequence (default)")
        print("2. Wave drive (one coil at a time)")
        print("3. Two-phase (two coils always on)")
        
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == "1":
            self.current_sequence = self.STEP_SEQUENCE_4
            self.sequence_name = "4-step"
        elif choice == "2":
            self.current_sequence = self.STEP_SEQUENCE_WAVE
            self.sequence_name = "wave drive"
        elif choice == "3":
            self.current_sequence = self.STEP_SEQUENCE_2PHASE
            self.sequence_name = "two-phase"
        else:
            print("Invalid choice")
            return
        
        print(f"\n✓ Changed to {self.sequence_name} sequence")
        self.update_motors()
    
    def run_manual_adjustment_test(self):
        """Test manual angle adjustment like Option 2"""
        print("\n" + "="*70)
        print("MANUAL ANGLE ADJUSTMENT TEST")
        print("="*70)
        
        while True:
            print(f"\nCurrent position:")
            print(f"  Azimuth: {self.azimuth_angle:.1f}°")
            print(f"  Altitude: {self.altitude_angle:.1f}°")
            print(f"  Step sequence: {self.sequence_name}")
            
            print("\nOptions:")
            print("1. Set exact angles")
            print("2. Move relative amounts")
            print("3. Test individual motors")
            print("4. Change step sequence")
            print("5. Return to home (0°, 0°)")
            print("6. Exit test")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                self.set_exact_angles()
            elif choice == "2":
                self.move_relative()
            elif choice == "3":
                self.test_direct_control()
            elif choice == "4":
                self.change_sequence()
            elif choice == "5":
                self.return_to_home()
            elif choice == "6":
                print("Exiting test...")
                break
            else:
                print("Invalid choice")
    
    def set_exact_angles(self):
        """Set motors to exact angles"""
        try:
            new_az = float(input(f"Enter azimuth angle ({self.MAX_AZIMUTH_LEFT} to {self.MAX_AZIMUTH_RIGHT}): "))
            new_alt = float(input("Enter altitude angle: "))
            
            if new_az < self.MAX_AZIMUTH_LEFT or new_az > self.MAX_AZIMUTH_RIGHT:
                print(f"Azimuth must be between {self.MAX_AZIMUTH_LEFT}° and {self.MAX_AZIMUTH_RIGHT}°")
                return
            
            print(f"\nMoving to: Azimuth={new_az:.1f}°, Altitude={new_alt:.1f}°")
            
            # Calculate movement needed
            az_move = new_az - self.azimuth_angle
            alt_move = new_alt - self.altitude_angle
            
            # Use the individual test functions
            if az_move != 0:
                steps = int(abs(az_move) * self.AZIMUTH_STEPS_PER_REV / 360)
                direction = 1 if az_move > 0 else -1
                
                print(f"Azimuth: {az_move:.1f}° = {steps} steps")
                for i in range(steps):
                    self.azimuth_step_idx = (self.azimuth_step_idx + direction) % 4
                    self.azimuth_position += direction
                    self.update_motors()
                    time.sleep(0.001)
            
            if alt_move != 0:
                steps = int(abs(alt_move) * self.ALTITUDE_STEPS_PER_REV / 360) // 2  # Half steps
                direction = 1 if alt_move > 0 else -1
                
                print(f"Altitude: {alt_move:.1f}° = {steps} steps (half)")
                for i in range(steps):
                    self.altitude_step_idx = (self.altitude_step_idx + direction) % 4
                    self.altitude_position += direction
                    self.update_motors()
                    time.sleep(0.001)
            
            # Update angles
            self.azimuth_angle = self.azimuth_position * 360.0 / self.AZIMUTH_STEPS_PER_REV
            self.altitude_angle = self.altitude_position * 360.0 / (self.ALTITUDE_STEPS_PER_REV // 2)
            
            print(f"\n✓ Moved to: Azimuth={self.azimuth_angle:.1f}°, Altitude={self.altitude_angle:.1f}°")
            
        except ValueError:
            print("Invalid input")
    
    def move_relative(self):
        """Move motors by relative amounts"""
        try:
            az_move = float(input("Enter azimuth change (degrees): "))
            alt_move = float(input("Enter altitude change (degrees): "))
            
            print(f"\nMoving: ΔAz={az_move:.1f}°, ΔAlt={alt_move:.1f}°")
            
            # Calculate new target angles
            new_az = self.azimuth_angle + az_move
            new_alt = self.altitude_angle + alt_move
            
            # Check azimuth limits
            if new_az < self.MAX_AZIMUTH_LEFT or new_az > self.MAX_AZIMUTH_RIGHT:
                print(f"Azimuth would exceed limits ({self.MAX_AZIMUTH_LEFT}° to {self.MAX_AZIMUTH_RIGHT}°)")
                return
            
            # Call set_exact_angles with the new values
            self.azimuth_angle = new_az
            self.altitude_angle = new_alt
            self.set_exact_angles()
            
        except ValueError:
            print("Invalid input")
    
    def return_to_home(self):
        """Return to home position (0°, 0°)"""
        print("\nReturning to home position (0°, 0°)...")
        
        # Calculate steps to return
        az_steps_needed = -self.azimuth_position
        alt_steps_needed = -self.altitude_position
        
        # Move azimuth back
        if az_steps_needed != 0:
            direction = 1 if az_steps_needed > 0 else -1
            steps = abs(az_steps_needed)
            
            print(f"Azimuth: {steps} steps")
            for i in range(steps):
                self.azimuth_step_idx = (self.azimuth_step_idx + direction) % 4
                self.azimuth_position += direction
                self.update_motors()
                time.sleep(0.001)
        
        # Move altitude back
        if alt_steps_needed != 0:
            direction = 1 if alt_steps_needed > 0 else -1
            steps = abs(alt_steps_needed)
            
            print(f"Altitude: {steps} steps")
            for i in range(steps):
                self.altitude_step_idx = (self.altitude_step_idx + direction) % 4
                self.altitude_position += direction
                self.update_motors()
                time.sleep(0.001)
        
        # Reset angles
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        print("✓ At home position (0°, 0°)")
    
    def cleanup(self):
        """Cleanup GPIO"""
        self.shift_out(0b00000000)
        GPIO.cleanup()
        print("\n✓ GPIO cleanup complete")

def main():
    """Main function"""
    print("="*70)
    print("STANDALONE MOTOR TEST - DEBUG VERSION")
    print("="*70)
    print("This tests ONLY manual motor adjustment")
    print("It will help us debug:")
    print("  1. Why azimuth isn't turning")
    print("  2. Why altitude moves double")
    print("="*70)
    
    tester = None
    try:
        tester = MotorTestOnly()
        tester.run_manual_adjustment_test()
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if tester:
            tester.cleanup()
        print("\nTest ended")

if __name__ == "__main__":
    main()
