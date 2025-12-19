#!/usr/bin/env python3
"""
ENME441 LASER TURRET - FIXED MOTOR CONTROL VERSION
Consistent bit assignments for both motors
"""

import RPi.GPIO as GPIO
import time
import json
import requests
import math
import sys
import atexit

class FixedTurret:
    def __init__(self):
        # ========== COMPETITION CONFIGURATION ==========
        self.SERVER_IP = "192.168.1.254"  # ENME441 WiFi router IP
        self.SERVER_URL = f"http://{self.SERVER_IP}:8000/positions.json"
        
        # Competition laser firing duration: 3 seconds (per rules)
        self.FIRING_DURATION = 3.0
        
        # Team number (will be asked later)
        self.team_number = None
        
        # Motor calibration - USING YOUR VALUES
        self.AZIMUTH_STEPS_PER_REV = 1024    # Fast motor
        self.ALTITUDE_STEPS_PER_REV = 4096   # Standard motor
        
        # IMPORTANT: Azimuth orientation
        self.MAX_AZIMUTH_LEFT = -120    # degrees (negative = left from center)
        self.MAX_AZIMUTH_RIGHT = 120    # degrees (positive = right from center)
        
        # Mechanical offsets (mm)
        self.LASER_HEIGHT = 43.0
        self.LASER_OFFSET = 23.5
        
        # Position tracking
        self.azimuth_angle = 0.0    # Current azimuth angle (0 = pointing to center)
        self.altitude_angle = 0.0   # Current altitude angle (0 = horizontal)
        self.azimuth_position = 0   # Steps from home
        self.altitude_position = 0  # Steps from home
        
        # Store original/home position
        self.home_azimuth_angle = 0.0
        self.home_altitude_angle = 0.0
        self.home_azimuth_position = 0
        self.home_altitude_position = 0
        
        # Target tracking
        self.competition_data = None
        self.my_position = None
        self.targets_hit = set()
        
        # GPIO pins
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS (Pin 14)
        self.LASER_PIN = 26  # GPIO26 (HIGH = ON, LOW = OFF)
        
        # FIXED: Consistent bit assignments
        # Azimuth motor: Bits 0-3 (pins 15, 1, 2, 3)
        # Altitude motor: Bits 4-7 (pins 4, 5, 6, 7)
        self.AZIMUTH_BITS = 0b00001111
        self.ALTITUDE_BITS = 0b11110000
        
        # Step sequence for 4-wire bipolar stepper
        self.STEP_SEQUENCE = [
            0b00010001,  # Step 0: Az=0001, Alt=0001
            0b00100010,  # Step 1: Az=0010, Alt=0010  
            0b01000100,  # Step 2: Az=0100, Alt=0100
            0b10001000,  # Step 3: Az=1000, Alt=1000
        ]
        
        # Current step indices
        self.azimuth_step_idx = 0
        self.altitude_step_idx = 0
        
        # State
        self.laser_state = False
        self.gpio_initialized = False
        
        self.setup_gpio()
        
        # Register auto-return function
        atexit.register(self.auto_return_to_home)
    
    def setup_gpio(self):
        """Initialize all GPIO pins"""
        if self.gpio_initialized:
            return
            
        GPIO.setmode(GPIO.BCM)
        
        # Shift register pins
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Laser pin
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        # Initialize
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        self.gpio_initialized = True
        print("✓ GPIO initialized - Motors OFF, Laser OFF")
    
    def ensure_gpio(self):
        """Ensure GPIO is initialized before using it"""
        if not self.gpio_initialized:
            self.setup_gpio()
    
    # ========== FIXED MOTOR CONTROL ==========
    
    def shift_out(self, data_byte):
        """Send 8 bits to shift register"""
        self.ensure_gpio()
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
        """Update both motors with current step indices"""
        combined = (self.STEP_SEQUENCE[self.azimuth_step_idx] & self.AZIMUTH_BITS) | \
                   (self.STEP_SEQUENCE[self.altitude_step_idx] & self.ALTITUDE_BITS)
        self.shift_out(combined)
    
    def step_azimuth(self, direction):
        """Take one azimuth step"""
        self.azimuth_step_idx = (self.azimuth_step_idx + direction) % 4
        self.azimuth_position += direction
        self.azimuth_angle = self.azimuth_position * 360.0 / self.AZIMUTH_STEPS_PER_REV
        self.update_motors()
    
    def step_altitude(self, direction):
        """Take one altitude step"""
        self.altitude_step_idx = (self.altitude_step_idx + direction) % 4
        self.altitude_position += direction
        self.altitude_angle = self.altitude_position * 360.0 / self.ALTITUDE_STEPS_PER_REV
        self.update_motors()
    
    def move_motors_direct(self, az_steps, alt_steps, step_delay=0.001):
        """
        Direct motor movement - most reliable
        Returns True if successful
        """
        if az_steps == 0 and alt_steps == 0:
            return True
        
        az_direction = 1 if az_steps > 0 else -1
        alt_direction = 1 if alt_steps > 0 else -1
        
        az_steps_abs = abs(az_steps)
        alt_steps_abs = abs(alt_steps)
        
        # Move both motors
        az_completed = 0
        alt_completed = 0
        
        while az_completed < az_steps_abs or alt_completed < alt_steps_abs:
            current_time = time.time()
            
            # Move azimuth if needed
            if az_completed < az_steps_abs:
                # Check limits for azimuth
                new_angle = self.azimuth_angle + (az_direction * 360.0 / self.AZIMUTH_STEPS_PER_REV)
                if new_angle < self.MAX_AZIMUTH_LEFT or new_angle > self.MAX_AZIMUTH_RIGHT:
                    print(f"⚠ Azimuth limit reached: {new_angle:.1f}°")
                    return False
                
                self.step_azimuth(az_direction)
                az_completed += 1
            
            # Move altitude if needed
            if alt_completed < alt_steps_abs:
                self.step_altitude(alt_direction)
                alt_completed += 1
            
            time.sleep(step_delay)
        
        return True
    
    def move_motors_degrees(self, az_degrees, alt_degrees, step_delay=0.001):
        """Move by degrees"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        
        print(f"Moving: Az={az_degrees:.1f}° ({az_steps} steps), Alt={alt_degrees:.1f}° ({alt_steps} steps)")
        return self.move_motors_direct(az_steps, alt_steps, step_delay)
    
    # ========== MANUAL CONTROLS ==========
    
    def manual_toggle_laser(self):
        """Manually toggle laser on/off"""
        print("\n" + "="*60)
        print("MANUAL LASER CONTROL")
        print("="*60)
        
        print(f"Current laser state: {'ON' if self.laser_state else 'OFF'}")
        print("\nOptions:")
        print("1. Turn laser ON")
        print("2. Turn laser OFF")
        print("3. Test fire (3 seconds)")
        print("4. Back to main menu")
        
        choice = input("\nEnter choice (1-4): ").strip()
        
        if choice == "1":
            self.laser_on()
            print("Laser turned ON")
        elif choice == "2":
            self.laser_off()
            print("Laser turned OFF")
        elif choice == "3":
            print(f"Firing laser for {self.FIRING_DURATION} seconds...")
            self.laser_on()
            time.sleep(self.FIRING_DURATION)
            self.laser_off()
            print("Laser fired.")
    
    def manual_adjust_motors_fixed(self):
        """FIXED: Manual motor adjustment with better controls"""
        print("\n" + "="*60)
        print("MANUAL MOTOR ADJUSTMENT - FIXED")
        print("="*60)
        
        print(f"Current position: Azimuth={self.azimuth_angle:.1f}°, Altitude={self.altitude_angle:.1f}°")
        print(f"Azimuth limits: {self.MAX_AZIMUTH_LEFT}° to {self.MAX_AZIMUTH_RIGHT}°")
        print("\nOptions:")
        print("1. Set exact angles")
        print("2. Move relative amounts")
        print("3. Test individual motors")
        print("4. Back to menu")
        
        choice = input("\nEnter choice (1-4): ").strip()
        
        if choice == "1":
            # Set exact angles
            try:
                az_input = input(f"Enter azimuth angle ({self.MAX_AZIMUTH_LEFT} to {self.MAX_AZIMUTH_RIGHT}, current={self.azimuth_angle:.1f}): ").strip()
                alt_input = input(f"Enter altitude angle (current={self.altitude_angle:.1f}): ").strip()
                
                if az_input:
                    new_az = float(az_input)
                    if new_az < self.MAX_AZIMUTH_LEFT or new_az > self.MAX_AZIMUTH_RIGHT:
                        print(f"Azimuth must be between {self.MAX_AZIMUTH_LEFT}° and {self.MAX_AZIMUTH_RIGHT}°")
                        return
                else:
                    new_az = self.azimuth_angle
                
                if alt_input:
                    new_alt = float(alt_input)
                else:
                    new_alt = self.altitude_angle
                
                az_move = new_az - self.azimuth_angle
                alt_move = new_alt - self.altitude_angle
                
                print(f"\nMoving: ΔAz={az_move:.1f}°, ΔAlt={alt_move:.1f}°")
                success = self.move_motors_degrees(az_move, alt_move, 0.001)
                
                if success:
                    print(f"✓ Moved to: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
                else:
                    print("⚠ Movement failed (azimuth limits?)")
                    
            except ValueError:
                print("Invalid input. Please enter numbers.")
        
        elif choice == "2":
            # Move relative amounts
            try:
                az_move = float(input("Enter azimuth change (degrees): ").strip())
                alt_move = float(input("Enter altitude change (degrees): ").strip())
                
                print(f"\nMoving: ΔAz={az_move:.1f}°, ΔAlt={alt_move:.1f}°")
                success = self.move_motors_degrees(az_move, alt_move, 0.001)
                
                if success:
                    print(f"✓ New position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
                else:
                    print("⚠ Movement failed (azimuth limits?)")
                    
            except ValueError:
                print("Invalid input. Please enter numbers.")
        
        elif choice == "3":
            # Test individual motors
            print("\nTesting individual motors:")
            print("1. Test azimuth motor (+30°, -30°)")
            print("2. Test altitude motor (+30°, -30°)")
            
            test_choice = input("Enter choice (1-2): ").strip()
            
            if test_choice == "1":
                print("Testing azimuth motor...")
                print("Moving +30°...")
                if self.move_motors_degrees(30, 0, 0.001):
                    time.sleep(1)
                    print("Moving -30°...")
                    self.move_motors_degrees(-30, 0, 0.001)
                    print("✓ Azimuth test complete")
                else:
                    print("⚠ Azimuth test failed")
            
            elif test_choice == "2":
                print("Testing altitude motor...")
                print("Moving +30°...")
                if self.move_motors_degrees(0, 30, 0.001):
                    time.sleep(1)
                    print("Moving -30°...")
                    self.move_motors_degrees(0, -30, 0.001)
                    print("✓ Altitude test complete")
                else:
                    print("⚠ Altitude test failed")
    
    def motor_calibration(self):
        """Manually set motors to starting point"""
        print("\n" + "="*60)
        print("MOTOR CALIBRATION")
        print("="*60)
        
        print("IMPORTANT: Align turret so that:")
        print("1. Laser points to CENTER of competition ring")
        print("2. This is your 'forward' direction (0°)")
        print("\nPress Enter when aligned...")
        input()
        
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        self.azimuth_position = 0
        self.altitude_position = 0
        self.azimuth_step_idx = 0
        self.altitude_step_idx = 0
        
        self.home_azimuth_angle = self.azimuth_angle
        self.home_altitude_angle = self.altitude_angle
        self.home_azimuth_position = self.azimuth_position
        self.home_altitude_position = self.altitude_position
        
        self.update_motors()
        
        print("✓ Calibration complete!")
        print(f"Home position: Azimuth=0°, Altitude=0°")
    
    def json_file_reading(self):
        """Read JSON file and set team number"""
        print("\n" + "="*60)
        print("JSON FILE READING")
        print("="*60)
        
        if self.team_number:
            print(f"Current team: {self.team_number}")
            change = input("Change team? (y/n): ").strip().lower()
            if change != 'y':
                return
        
        team = input("Enter your team number: ").strip()
        if not team:
            print("No team number entered.")
            return
        
        self.team_number = team
        print(f"\nFetching competition data for Team {team}...")
        print(f"Server: {self.SERVER_URL}")
        
        try:
            response = requests.get(self.SERVER_URL, timeout=10)
            if response.status_code == 200:
                self.competition_data = response.json()
                
                if self.team_number in self.competition_data["turrets"]:
                    self.my_position = self.competition_data["turrets"][self.team_number]
                    print(f"✓ Success! Team {self.team_number} position:")
                    print(f"  r = {self.my_position['r']} cm")
                    print(f"  θ = {self.my_position['theta']} rad ({math.degrees(self.my_position['theta']):.1f}°)")
                    
                else:
                    print(f"✗ Team {self.team_number} not found")
                    self.competition_data = None
                    self.my_position = None
            else:
                print(f"✗ Server error: HTTP {response.status_code}")
                
        except requests.exceptions.ConnectionError:
            print(f"✗ Cannot connect to server")
        except Exception as e:
            print(f"✗ Error: {e}")
    
    def motor_diagnostic(self):
        """Run motor diagnostic test"""
        print("\n" + "="*60)
        print("MOTOR DIAGNOSTIC TEST")
        print("="*60)
        print("Testing both motors with simple movements")
        
        print("\n1. Testing azimuth motor (+45°, -45°)")
        input("Press Enter to start...")
        
        print("Moving +45°...")
        if self.move_motors_degrees(45, 0, 0.001):
            print("Pausing 1 second...")
            time.sleep(1)
            print("Moving -45°...")
            self.move_motors_degrees(-45, 0, 0.001)
            print("✓ Azimuth test complete")
        else:
            print("⚠ Azimuth test failed")
        
        print("\n2. Testing altitude motor (+45°, -45°)")
        input("Press Enter to start...")
        
        print("Moving +45°...")
        if self.move_motors_degrees(0, 45, 0.001):
            print("Pausing 1 second...")
            time.sleep(1)
            print("Moving -45°...")
            self.move_motors_degrees(0, -45, 0.001)
            print("✓ Altitude test complete")
        else:
            print("⚠ Altitude test failed")
        
        print("\n3. Testing both motors together (+30°, -30°)")
        input("Press Enter to start...")
        
        print("Moving both +30°...")
        if self.move_motors_degrees(30, 30, 0.001):
            print("Pausing 1 second...")
            time.sleep(1)
            print("Moving both -30°...")
            self.move_motors_degrees(-30, -30, 0.001)
            print("✓ Both motors test complete")
        else:
            print("⚠ Both motors test failed")
        
        print("\n✓ Diagnostic complete")
    
    def initiate_firing_sequence(self):
        """Fire at targets in sequence"""
        print("\n" + "="*60)
        print("FIRING SEQUENCE")
        print("="*60)
        print("Note: This requires competition data")
        print("Run JSON file reading first (Option 4)")
        print("="*60)
        
        if not self.team_number or not self.competition_data:
            print("Please set team number and fetch competition data first")
            return
        
        print("Firing sequence would run here")
        print("(Implementation depends on competition setup)")
    
    # ========== UTILITY FUNCTIONS ==========
    
    def laser_on(self):
        """Turn laser ON"""
        self.ensure_gpio()
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_state = True
        print("LASER: ON")
    
    def laser_off(self):
        """Turn laser OFF"""
        self.ensure_gpio()
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_state = False
        print("LASER: OFF")
    
    def go_to_home(self):
        """Return to home position"""
        print("Returning to home position...")
        
        # Calculate steps needed
        az_steps_needed = self.home_azimuth_position - self.azimuth_position
        alt_steps_needed = self.home_altitude_position - self.altitude_position
        
        self.move_motors_direct(az_steps_needed, alt_steps_needed, 0.001)
        print("✓ At home position")
    
    def auto_return_to_home(self):
        """Automatically return to home position (called on exit)"""
        print("\nAuto-returning to home position...")
        
        try:
            if not self.gpio_initialized:
                self.setup_gpio()
            
            self.go_to_home()
            print("✓ Returned to home position")
                
        except Exception as e:
            print(f"⚠ Error during auto-return: {e}")
        
        try:
            self.shift_out(0b00000000)
            self.laser_off()
        except:
            pass
    
    def cleanup(self):
        """Force cleanup"""
        print("\nCleanup initiated...")
        self.auto_return_to_home()
        if self.gpio_initialized:
            GPIO.cleanup()
            self.gpio_initialized = False
        print("GPIO cleanup complete.")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 LASER TURRET - FIXED MOTOR CONTROL")
    print("="*70)
    print("Features:")
    print("  • Fixed motor bit assignments")
    print("  • Better manual controls")
    print("  • Motor diagnostic test")
    print("  • Consistent step calculations")
    print("="*70)
    
    turret = None
    try:
        turret = FixedTurret()
        
        while True:
            print("\n" + "="*70)
            print("MAIN MENU")
            print("="*70)
            print("1. Manual toggle the laser")
            print("2. Manual adjust motors (FIXED)")
            print("3. Motor calibration")
            print("4. JSON file reading")
            print("5. Motor diagnostic test")
            print("6. Initiate firing sequence")
            print("7. Return to home position")
            print("8. Force cleanup & exit")
            print("9. Exit (Auto-return to home)")
            
            choice = input("\nEnter choice (1-9): ").strip()
            
            if choice == "1":
                turret.manual_toggle_laser()
            elif choice == "2":
                turret.manual_adjust_motors_fixed()
            elif choice == "3":
                turret.motor_calibration()
            elif choice == "4":
                turret.json_file_reading()
            elif choice == "5":
                turret.motor_diagnostic()
            elif choice == "6":
                turret.initiate_firing_sequence()
            elif choice == "7":
                turret.go_to_home()
            elif choice == "8":
                print("Force cleanup...")
                turret.cleanup()
                print("Exiting...")
                break
            elif choice == "9":
                print("Exiting with auto-return to home...")
                break
            else:
                print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if turret:
            turret.cleanup()
        print("\nProgram ended.")

if __name__ == "__main__":
    # Check for required packages
    try:
        import requests
    except ImportError:
        print("Installing 'requests' package...")
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "requests"])
        import requests
    
    main()
