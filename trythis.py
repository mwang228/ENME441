#!/usr/bin/env python3
"""
ENME441 Laser Turret - OPTIMIZED TIMING VERSION
With proper motor timing and speed control
"""

import RPi.GPIO as GPIO
import time
import json
import os

class OptimizedTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 LASER TURRET - OPTIMIZED TIMING")
        print("="*70)
        
        # GPIO Pins for Shift Register
        self.SHIFT_CLK = 11  # GPIO11 -> 74HC595 Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> 74HC595 Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> 74HC595 Pin 14 (DS)
        
        # Configuration
        self.CONFIG_FILE = "turret_config.json"
        
        # Load configuration
        self.AZIMUTH_STEPS_PER_REV = 2400  # Will calibrate
        self.ALTITUDE_STEPS_PER_REV = 2400  # Your calibrated value
        
        self.load_config()
        
        print(f"Configuration loaded:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # Step sequences (4-step wave drive)
        self.AZIMUTH_SEQUENCE = [0b00000001, 0b00000010, 0b00000100, 0b00001000]
        self.ALTITUDE_SEQUENCE = [0b00010000, 0b00100000, 0b01000000, 0b10000000]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # TIMING PARAMETERS - ADJUST THESE!
        self.STEP_DELAY = 0.01      # 10ms between steps (start slow)
        self.MIN_STEP_DELAY = 0.002  # 2ms minimum (fastest)
        self.MAX_STEP_DELAY = 0.1    # 100ms maximum (slowest)
        
        # Acceleration profile
        self.ACCELERATION_STEPS = 20  # Steps to accelerate/decelerate
        
        # Initialize
        self.setup_gpio()
        
        # Energize first position
        self.update_motors()
        time.sleep(0.1)
        
        print(f"\n✓ System initialized")
        print(f"Step delay: {self.STEP_DELAY*1000:.1f}ms")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print("="*70)
    
    def load_config(self):
        """Load configuration from file"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 2400)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 2400)
                print("✓ Configuration loaded from file")
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
            print("✓ Configuration saved")
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
        
        # Initialize shift register to all zeros
        self.send_to_shift_register(0b00000000)
    
    def send_to_shift_register(self, data):
        """Optimized shift register communication"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        # Send bits MSB first
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            
            # Clock pulse - minimum reliable timing
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            # NO DELAY - just toggle as fast as Python can
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        # Tiny delay to ensure latch is seen
        time.sleep(0.000001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors"""
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth(self, direction, step_delay=None):
        """Step azimuth motor with adjustable delay"""
        if step_delay is None:
            step_delay = self.STEP_DELAY
        
        # Update position
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        
        # Update hardware
        self.update_motors()
        
        # Delay for motor to respond
        if step_delay > 0:
            time.sleep(step_delay)
    
    def step_altitude(self, direction, step_delay=None):
        """Step altitude motor with adjustable delay"""
        if step_delay is None:
            step_delay = self.STEP_DELAY
        
        # Update position
        self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
        self.altitude_steps += direction
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        
        # Update hardware
        self.update_motors()
        
        # Delay for motor to respond
        if step_delay > 0:
            time.sleep(step_delay)
    
    def move_azimuth_with_speed(self, degrees, speed_percent=50):
        """
        Move azimuth with speed control
        speed_percent: 1-100% (1=slowest, 100=fastest)
        """
        # Calculate steps
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        # Calculate step delay based on speed percentage
        delay_range = self.MAX_STEP_DELAY - self.MIN_STEP_DELAY
        step_delay = self.MAX_STEP_DELAY - (delay_range * speed_percent / 100)
        step_delay = max(self.MIN_STEP_DELAY, min(self.MAX_STEP_DELAY, step_delay))
        
        print(f"\nAzimuth: Moving {degrees:.1f}°")
        print(f"Steps: {steps}, Speed: {speed_percent}%, Delay: {step_delay*1000:.1f}ms")
        
        # Move with constant speed
        for i in range(steps):
            self.step_azimuth(direction, step_delay)
            
            # Progress indicator
            if steps > 20 and (i + 1) % (steps // 10) == 0:
                progress = (i + 1) / steps * 100
                print(f"  {progress:.0f}% complete, Angle: {self.azimuth_angle:.1f}°")
        
        print(f"✓ Complete: {self.azimuth_angle:.1f}°")
        return self.azimuth_angle
    
    def move_altitude_with_speed(self, degrees, speed_percent=50):
        """
        Move altitude with speed control
        speed_percent: 1-100% (1=slowest, 100=fastest)
        """
        # Calculate steps
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        # Calculate step delay based on speed percentage
        delay_range = self.MAX_STEP_DELAY - self.MIN_STEP_DELAY
        step_delay = self.MAX_STEP_DELAY - (delay_range * speed_percent / 100)
        step_delay = max(self.MIN_STEP_DELAY, min(self.MAX_STEP_DELAY, step_delay))
        
        print(f"\nAltitude: Moving {degrees:.1f}°")
        print(f"Steps: {steps}, Speed: {speed_percent}%, Delay: {step_delay*1000:.1f}ms")
        
        # Move with constant speed
        for i in range(steps):
            self.step_altitude(direction, step_delay)
            
            # Progress indicator
            if steps > 20 and (i + 1) % (steps // 10) == 0:
                progress = (i + 1) / steps * 100
                print(f"  {progress:.0f}% complete, Angle: {self.altitude_angle:.1f}°")
        
        print(f"✓ Complete: {self.altitude_angle:.1f}°")
        return self.altitude_angle
    
    def find_optimal_speed(self, motor_type='azimuth', test_angle=45):
        """
        Find the fastest reliable speed for a motor
        """
        print(f"\n{'='*60}")
        print(f"FINDING OPTIMAL SPEED FOR {motor_type.upper()}")
        print(f"{'='*60}")
        
        speeds_to_test = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
        working_speeds = []
        
        # Return to known position
        if motor_type == 'azimuth':
            self.move_azimuth_with_speed(-self.azimuth_angle, 10)  # Slow return to 0
        else:
            self.move_altitude_with_speed(-self.altitude_angle, 10)
        
        for speed in speeds_to_test:
            print(f"\nTesting speed: {speed}%...")
            
            try:
                if motor_type == 'azimuth':
                    start_angle = self.azimuth_angle
                    self.move_azimuth_with_speed(test_angle, speed)
                    actual = self.azimuth_angle - start_angle
                    
                    # Return to start for next test
                    self.move_azimuth_with_speed(-actual, 10)
                else:
                    start_angle = self.altitude_angle
                    self.move_altitude_with_speed(test_angle, speed)
                    actual = self.altitude_angle - start_angle
                    
                    # Return to start for next test
                    self.move_altitude_with_speed(-actual, 10)
                
                # Check if movement was successful
                if abs(actual - test_angle) < 5:  # Within 5 degrees
                    working_speeds.append(speed)
                    print(f"  ✓ Speed {speed}% works")
                else:
                    print(f"  ✗ Speed {speed}% failed (moved {actual:.1f}°)")
            
            except Exception as e:
                print(f"  ✗ Speed {speed}% failed with error: {e}")
        
        if working_speeds:
            optimal_speed = max(working_speeds)
            print(f"\n✓ Optimal speed for {motor_type}: {optimal_speed}%")
            return optimal_speed
        else:
            print(f"\n✗ No working speeds found for {motor_type}")
            return 10  # Default to slow
    
    def test_both_motors(self):
        """Test both motors with interactive speed control"""
        print("\n" + "="*60)
        print("INTERACTIVE SPEED TEST")
        print("="*60)
        
        while True:
            print(f"\nCurrent position:")
            print(f"  Azimuth: {self.azimuth_angle:.1f}°")
            print(f"  Altitude: {self.altitude_angle:.1f}°")
            
            print("\nTest options:")
            print("1. Test azimuth motor")
            print("2. Test altitude motor")
            print("3. Find optimal speed for azimuth")
            print("4. Find optimal speed for altitude")
            print("5. Return both to 0°")
            print("6. Back to main menu")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                try:
                    deg = float(input("Degrees to move: "))
                    speed = int(input("Speed % (1-100, start with 30): "))
                    self.move_azimuth_with_speed(deg, speed)
                except:
                    print("Invalid input")
            
            elif choice == "2":
                try:
                    deg = float(input("Degrees to move: "))
                    speed = int(input("Speed % (1-100, start with 30): "))
                    self.move_altitude_with_speed(deg, speed)
                except:
                    print("Invalid input")
            
            elif choice == "3":
                self.find_optimal_speed('azimuth', 45)
            
            elif choice == "4":
                self.find_optimal_speed('altitude', 45)
            
            elif choice == "5":
                print("\nReturning to home...")
                self.move_azimuth_with_speed(-self.azimuth_angle, 30)
                self.move_altitude_with_speed(-self.altitude_angle, 30)
                print(f"✓ At home: (0°, 0°)")
            
            elif choice == "6":
                break
            
            else:
                print("Invalid choice")
    
    def calibrate_azimuth(self):
        """Calibrate azimuth motor steps/revolution"""
        print("\n" + "="*60)
        print("CALIBRATE AZIMUTH MOTOR")
        print("="*60)
        
        print("First, find a good working speed...")
        optimal_speed = self.find_optimal_speed('azimuth', 45)
        
        print(f"\nUsing speed: {optimal_speed}%")
        print("Moving to start position (0°)...")
        self.move_azimuth_with_speed(-self.azimuth_angle, optimal_speed)
        
        # Test 90 degree movement
        test_angle = 90.0
        print(f"\nTesting {test_angle}° movement...")
        
        start_angle = self.azimuth_angle
        self.move_azimuth_with_speed(test_angle, optimal_speed)
        actual_movement = self.azimuth_angle - start_angle
        
        print(f"\nCalibration results:")
        print(f"  Requested: {test_angle:.1f}°")
        print(f"  Actual:    {actual_movement:.1f}°")
        
        if abs(actual_movement) > 1.0:
            correction = test_angle / actual_movement
            old_value = self.AZIMUTH_STEPS_PER_REV
            new_value = int(old_value * correction)
            
            print(f"  Correction factor: {correction:.3f}")
            print(f"  Old: {old_value} steps/rev")
            print(f"  New: {new_value} steps/rev")
            
            # Update configuration
            self.AZIMUTH_STEPS_PER_REV = new_value
            self.save_config()
            
            # Test new calibration
            print(f"\nTesting new calibration...")
            self.move_azimuth_with_speed(-actual_movement, optimal_speed)
            new_test = self.move_azimuth_with_speed(test_angle, optimal_speed)
            print(f"  New error: {abs(test_angle - (new_test - start_angle)):.1f}°")
        
        # Return to 0
        self.move_azimuth_with_speed(-self.azimuth_angle, optimal_speed)
        print("\n✓ Calibration complete!")
    
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
    print("ENME441 - OPTIMIZED MOTOR CONTROL")
    print("="*70)
    print("Key features:")
    print("• Adjustable speed control (1-100%)")
    print("• Automatic speed optimization")
    print("• Proper timing for reliable operation")
    print("="*70)
    
    turret = None
    try:
        turret = OptimizedTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU")
            print("="*60)
            print("1. Test motors with speed control (START HERE)")
            print("2. Calibrate azimuth motor steps/rev")
            print("3. Find optimal speed for both motors")
            print("4. Show current status")
            print("5. Exit")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == "1":
                turret.test_both_motors()
            elif choice == "2":
                turret.calibrate_azimuth()
            elif choice == "3":
                print("\nFinding optimal speed for azimuth...")
                turret.find_optimal_speed('azimuth', 45)
                print("\nFinding optimal speed for altitude...")
                turret.find_optimal_speed('altitude', 45)
            elif choice == "4":
                print(f"\nCurrent status:")
                print(f"  Azimuth: {turret.azimuth_angle:.1f}°")
                print(f"  Altitude: {turret.altitude_angle:.1f}°")
                print(f"  Azimuth steps/rev: {turret.AZIMUTH_STEPS_PER_REV}")
                print(f"  Altitude steps/rev: {turret.ALTITUDE_STEPS_PER_REV}")
                print(f"  Current step delay: {turret.STEP_DELAY*1000:.1f}ms")
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
