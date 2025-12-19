#!/usr/bin/env python3
"""
ENME441 - SIMPLE COMPETITION TURRET
No recursion, just basic motor control
"""

import RPi.GPIO as GPIO
import time
import math
import atexit

class SimpleTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 SIMPLE TURRET")
        print("="*70)
        
        # YOUR CALIBRATED VALUES
        self.AZ_STEPS_REV = 2200     # Azimuth steps per revolution
        self.ALT_STEPS_REV = 450     # Altitude steps per revolution
        
        # YOUR MEASURED LIMITS
        self.MAX_ALT_UP = -60.0      # Max UP (negative)
        self.MAX_ALT_DOWN = 45.0     # Max DOWN (positive)
        self.MAX_AZ_LEFT = -180      # Max left
        self.MAX_AZ_RIGHT = 180      # Max right
        
        # Current angles (0° = pointing forward, horizontal)
        self.az_angle = 0.0
        self.alt_angle = 0.0
        
        # Current steps
        self.az_steps = 0
        self.alt_steps = 0
        
        # GPIO Pins
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        self.LASER_PIN = 26
        
        # Step sequence
        self.STEP_SEQ = [
            0b00010001,  # Step 0
            0b00100010,  # Step 1
            0b01000100,  # Step 2
            0b10001000,  # Step 3
        ]
        
        self.az_step_idx = 0
        self.alt_step_idx = 0
        
        # Setup
        self.setup_gpio()
        atexit.register(self.cleanup)
        
        print(f"✓ Azimuth: {self.AZ_STEPS_REV} steps/rev")
        print(f"✓ Altitude: {self.ALT_STEPS_REV} steps/rev")
        print(f"✓ Altitude limits: {self.MAX_ALT_UP}° to {self.MAX_ALT_DOWN}°")
        print("="*70)
    
    def setup_gpio(self):
        """Simple GPIO setup"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        self.shift_out(0b00000000)
    
    def shift_out(self, data):
        """Send data to shift register"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors"""
        combined = (self.STEP_SEQ[self.az_step_idx] & 0b00001111) | \
                   (self.STEP_SEQ[self.alt_step_idx] & 0b11110000)
        self.shift_out(combined)
    
    def step_azimuth(self, direction):
        """Step azimuth motor"""
        self.az_step_idx = (self.az_step_idx + direction) % 4
        self.az_steps += direction
        self.az_angle = self.az_steps * 360.0 / self.AZ_STEPS_REV
        self.update_motors()
    
    def step_altitude(self, direction):
        """Step altitude motor"""
        self.alt_step_idx = (self.alt_step_idx + direction) % 4
        self.alt_steps += direction
        self.alt_angle = self.alt_steps * 360.0 / self.ALT_STEPS_REV
        self.update_motors()
    
    def move_direct(self, az_steps, alt_steps, delay=0.001):
        """Move motors directly by steps"""
        az_dir = 1 if az_steps > 0 else -1
        alt_dir = 1 if alt_steps > 0 else -1
        
        az_steps = abs(az_steps)
        alt_steps = abs(alt_steps)
        
        az_done = 0
        alt_done = 0
        
        while az_done < az_steps or alt_done < alt_steps:
            if az_done < az_steps:
                # Check azimuth limits
                new_az = self.az_angle + (az_dir * 360.0 / self.AZ_STEPS_REV)
                if self.MAX_AZ_LEFT <= new_az <= self.MAX_AZ_RIGHT:
                    self.step_azimuth(az_dir)
                    az_done += 1
            
            if alt_done < alt_steps:
                # Check altitude limits
                new_alt = self.alt_angle + (alt_dir * 360.0 / self.ALT_STEPS_REV)
                if self.MAX_ALT_UP <= new_alt <= self.MAX_ALT_DOWN:
                    self.step_altitude(alt_dir)
                    alt_done += 1
            
            time.sleep(delay)
        
        return True
    
    def move_degrees(self, az_deg, alt_deg, delay=0.001):
        """Move by degrees - NO RECURSION"""
        print(f"Moving: Az={az_deg:+.1f}°, Alt={alt_deg:+.1f}°")
        
        # Calculate steps
        az_steps = int(az_deg * self.AZ_STEPS_REV / 360.0)
        alt_steps = int(alt_deg * self.ALT_STEPS_REV / 360.0)
        
        # Move directly
        self.move_direct(az_steps, alt_steps, delay)
        
        print(f"Now at: Az={self.az_angle:+.1f}°, Alt={self.alt_angle:+.1f}°")
        return True
    
    def move_to_angle(self, target_az, target_alt, delay=0.001):
        """Move to specific angle - NO RECURSION"""
        az_move = target_az - self.az_angle
        alt_move = target_alt - self.alt_angle
        
        return self.move_degrees(az_move, alt_move, delay)
    
    def laser_on(self):
        """Turn laser ON"""
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        print("LASER ON")
    
    def laser_off(self):
        """Turn laser OFF"""
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        print("LASER OFF")
    
    def fire_laser(self, duration=3.0):
        """Fire laser for duration"""
        print(f"Firing laser for {duration}s...")
        self.laser_on()
        time.sleep(duration)
        self.laser_off()
        print("Fired")
    
    def calibrate(self):
        """Set home position"""
        print("\nPoint laser to field center, then press Enter...")
        input()
        
        self.az_angle = 0.0
        self.alt_angle = 0.0
        self.az_steps = 0
        self.alt_steps = 0
        self.az_step_idx = 0
        self.alt_step_idx = 0
        
        self.update_motors()
        print("✓ Calibrated to (0°, 0°)")
    
    def test_limits(self):
        """Test your working range"""
        print("\nTesting your limits:")
        
        tests = [
            ("Center", 0, 0),
            ("Max UP", 0, self.MAX_ALT_UP),
            ("Max DOWN", 0, self.MAX_ALT_DOWN),
            ("Right 45°, Mid", 45, (self.MAX_ALT_UP + self.MAX_ALT_DOWN)/2),
            ("Left 45°, Mid", -45, (self.MAX_ALT_UP + self.MAX_ALT_DOWN)/2),
        ]
        
        for name, az, alt in tests:
            print(f"\n{name} (Az={az:+.1f}°, Alt={alt:+.1f}°)")
            self.move_to_angle(az, alt, 0.001)
            time.sleep(1)
        
        # Return to center
        self.move_to_angle(0, 0, 0.001)
        print("\n✓ Limit test complete")
    
    def manual_control(self):
        """Simple manual control"""
        print("\n" + "="*60)
        print("MANUAL CONTROL")
        print("="*60)
        
        while True:
            print(f"\nPosition: Az={self.az_angle:+.1f}°, Alt={self.alt_angle:+.1f}°")
            print("Commands: a/d=azimuth, w/s=altitude, f=fire, z=zero, q=quit")
            
            cmd = input("> ").lower()
            
            if cmd == 'a':
                self.move_degrees(-10, 0)
            elif cmd == 'd':
                self.move_degrees(10, 0)
            elif cmd == 'w':
                self.move_degrees(0, 10)
            elif cmd == 's':
                self.move_degrees(0, -10)
            elif cmd == 'f':
                self.fire_laser(3)
            elif cmd == 'z':
                self.move_to_angle(0, 0)
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def calculate_target(self, target_r, target_theta, target_z, our_theta):
        """Calculate angles to hit a target"""
        # Target position
        target_x = target_r * math.cos(target_theta)
        target_y = target_r * math.sin(target_theta)
        
        # Our position (we're at radius r, angle our_theta)
        our_x = 300 * math.cos(our_theta)  # 3m radius
        our_y = 300 * math.sin(our_theta)
        
        # Relative position
        dx = target_x - our_x
        dy = target_y - our_y
        
        # Calculate angles
        azimuth = math.degrees(math.atan2(dy, dx) - our_theta)
        
        # Altitude calculation
        distance = math.sqrt(dx*dx + dy*dy)
        if distance > 0:
            altitude = math.degrees(math.atan2(target_z, distance))
        else:
            altitude = 90 if target_z > 0 else -90
        
        return azimuth, altitude
    
    def auto_competition(self):
        """Simple competition routine"""
        print("\n" + "="*60)
        print("AUTO COMPETITION MODE")
        print("="*60)
        
        # Example competition data
        our_team = input("Your team number: ").strip()
        our_theta = float(input("Your theta angle (radians): ").strip())
        
        # Example targets (you'd get these from JSON)
        targets = [
            ("Turret 1", 300, 1.5, 0),
            ("Turret 2", 300, 3.0, 0),
            ("Globe 1", 300, 0.5, 30),
            ("Globe 2", 300, 2.5, 20),
        ]
        
        print(f"\nFound {len(targets)} targets")
        
        for name, r, theta, z in targets:
            print(f"\nTargeting {name}...")
            
            # Calculate angles
            az, alt = self.calculate_target(r, theta, z, our_theta)
            print(f"  Required: Az={az:.1f}°, Alt={alt:.1f}°")
            
            # Check if in range
            if self.MAX_ALT_UP <= alt <= self.MAX_ALT_DOWN:
                # Move and fire
                self.move_to_angle(az, alt, 0.001)
                self.fire_laser(3)
                print(f"  ✓ Hit {name}")
            else:
                print(f"  ✗ Out of range (altitude {alt:.1f}°)")
        
        # Return home
        self.move_to_angle(0, 0, 0.001)
        print("\n✓ Competition complete")
    
    def cleanup(self):
        """Clean shutdown"""
        print("\nCleaning up...")
        self.laser_off()
        self.move_to_angle(0, 0, 0.001)
        self.shift_out(0b00000000)
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 SIMPLE TURRET CONTROL")
    print("="*70)
    print("Key features:")
    print("• No recursion - safe execution")
    print("• Your calibrated motor values")
    print("• Your measured altitude limits")
    print("• Simple manual control")
    print("• Basic competition calculations")
    print("="*70)
    
    turret = SimpleTurret()
    
    try:
        while True:
            print("\n" + "="*60)
            print("MAIN MENU")
            print("="*60)
            print("1. Manual control (test motors)")
            print("2. Calibrate (set home position)")
            print("3. Test limits (verify working range)")
            print("4. Auto competition mode")
            print("5. Cleanup & exit")
            
            choice = input("\nChoice (1-5): ").strip()
            
            if choice == "1":
                turret.manual_control()
            elif choice == "2":
                turret.calibrate()
            elif choice == "3":
                turret.test_limits()
            elif choice == "4":
                turret.auto_competition()
            elif choice == "5":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        turret.cleanup()
        print("\nProgram ended")

if __name__ == "__main__":
    main()
