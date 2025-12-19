#!/usr/bin/env python3
"""
ENME441 COMPETITION TURRET - CLEAN WORKING VERSION
All motors work, no recursion, competition ready
"""

import RPi.GPIO as GPIO
import time
import json
import math
import requests
import sys
import atexit

class CompetitionTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 COMPETITION TURRET")
        print("="*70)
        
        # ========== MOTOR CALIBRATION ==========
        # YOUR WORKING VALUES (from testing)
        self.AZIMUTH_STEPS_PER_REV = 2200      # Works perfectly
        self.ALTITUDE_STEPS_PER_REV = 450      # Half of 900 for double movement
        
        # YOUR MEASURED LIMITS
        self.MAX_ALT_UP = -60.0      # Max UP (negative)
        self.MAX_ALT_DOWN = 45.0     # Max DOWN (positive)
        
        # Current angles
        self.azimuth_angle = 0.0     # 0° = pointing forward
        self.altitude_angle = 0.0    # 0° = horizontal
        
        # Home position
        self.home_azimuth = 0.0
        self.home_altitude = 0.0
        
        # ========== COMPETITION SETTINGS ==========
        self.FIRING_DURATION = 3.0   # 3 seconds per hit
        self.TEAM_NUMBER = None
        self.SERVER_URL = "http://192.168.1.254:8000/positions.json"
        
        # ========== GPIO SETUP ==========
        self.SHIFT_CLK = 11  # GPIO11 -> Pin 11
        self.LATCH_CLK = 10  # GPIO10 -> Pin 12
        self.DATA_PIN = 9    # GPIO9  -> Pin 14
        self.LASER_PIN = 26  # GPIO26
        
        # PROVEN WORKING STEP SEQUENCE
        self.STEP_SEQUENCE = [
            0b00000001,  # Azimuth coil A
            0b00000010,  # Azimuth coil B
            0b00000100,  # Azimuth coil C
            0b00001000,  # Azimuth coil D
        ]
        
        self.ALTITUDE_SEQUENCE = [
            0b00010000,  # Altitude coil A
            0b00100000,  # Altitude coil B
            0b01000000,  # Altitude coil C
            0b10000000,  # Altitude coil D
        ]
        
        self.azimuth_step = 0
        self.altitude_step = 0
        
        # Laser state
        self.laser_on_flag = False
        
        # Initialize
        self.setup_gpio()
        atexit.register(self.cleanup)
        
        print(f"✓ Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"✓ Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        print(f"✓ Altitude limits: {self.MAX_ALT_UP}° to {self.MAX_ALT_DOWN}°")
        print("="*70)
    
    def setup_gpio(self):
        """Initialize GPIO - proven working"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Shift register pins
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Laser pin
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        # Initialize pins
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Start with motors off
        self.shift_out(0b00000000)
    
    def shift_out(self, data):
        """Send data to shift register"""
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
        """Update both motors"""
        az_pattern = self.STEP_SEQUENCE[self.azimuth_step]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_step]
        combined = az_pattern | alt_pattern
        self.shift_out(combined)
    
    # ========== FEATURE 1: MANUAL LASER CONTROL ==========
    def laser_on(self):
        """Turn laser ON"""
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_on_flag = True
        print("🔴 LASER ON")
    
    def laser_off(self):
        """Turn laser OFF"""
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_on_flag = False
        print("⚫ LASER OFF")
    
    def toggle_laser(self):
        """Toggle laser on/off"""
        if self.laser_on_flag:
            self.laser_off()
        else:
            self.laser_on()
    
    # ========== FEATURE 2: MANUAL MOTOR CONTROL ==========
    def step_azimuth(self, direction):
        """Step azimuth motor"""
        self.azimuth_step = (self.azimuth_step + direction) % 4
        self.azimuth_angle += direction * 360.0 / self.AZIMUTH_STEPS_PER_REV
        self.update_motors()
    
    def step_altitude(self, direction):
        """Step altitude motor"""
        self.altitude_step = (self.altitude_step + direction) % 4
        self.altitude_angle += direction * 360.0 / self.ALTITUDE_STEPS_PER_REV
        self.update_motors()
    
    def move_azimuth_degrees(self, degrees, delay=0.001):
        """Move azimuth by degrees"""
        steps = int(degrees * self.AZIMUTH_STEPS_PER_REV / 360.0)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"Azimuth: Moving {degrees:+.1f}° ({steps} steps)")
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(delay)
        
        print(f"✓ Azimuth: {self.azimuth_angle:+.1f}°")
    
    def move_altitude_degrees(self, degrees, delay=0.002):
        """Move altitude by degrees with limit checking"""
        steps = int(degrees * self.ALTITUDE_STEPS_PER_REV / 360.0)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"Altitude: Moving {degrees:+.1f}° ({steps} steps)")
        
        successful = 0
        for i in range(steps):
            # Calculate new angle
            new_angle = self.altitude_angle + (direction * 360.0 / self.ALTITUDE_STEPS_PER_REV)
            
            # Check limits
            if new_angle < self.MAX_ALT_UP or new_angle > self.MAX_ALT_DOWN:
                print(f"⚠ Altitude limit reached: {new_angle:.1f}°")
                break
            
            self.step_altitude(direction)
            successful += 1
            time.sleep(delay)
        
        actual = (successful / self.ALTITUDE_STEPS_PER_REV) * 360.0 * direction
        print(f"✓ Altitude: {self.altitude_angle:+.1f}° (moved {actual:+.1f}°)")
        return actual
    
    def move_to_angle(self, az_target, alt_target):
        """Move to specific angles"""
        print(f"\nMoving to: Az={az_target:+.1f}°, Alt={alt_target:+.1f}°")
        
        # Move azimuth
        az_move = az_target - self.azimuth_angle
        if abs(az_move) > 0.1:
            self.move_azimuth_degrees(az_move)
        
        # Move altitude
        alt_move = alt_target - self.altitude_angle
        if abs(alt_move) > 0.1:
            self.move_altitude_degrees(alt_move)
        
        print(f"✓ Position: Az={self.azimuth_angle:+.1f}°, Alt={self.altitude_angle:+.1f}°")
    
    def manual_adjust(self):
        """Manual motor adjustment interface"""
        print("\n" + "="*60)
        print("MANUAL MOTOR ADJUSTMENT")
        print("="*60)
        
        while True:
            print(f"\nCurrent: Az={self.azimuth_angle:+.1f}°, Alt={self.altitude_angle:+.1f}°")
            print("Options: 1=Set angles, 2=Move relative, 3=Test, 4=Back")
            
            choice = input("Choice: ").strip()
            
            if choice == "1":
                # Set exact angles
                try:
                    az = float(input("Azimuth angle (degrees): "))
                    alt = float(input("Altitude angle (degrees): "))
                    self.move_to_angle(az, alt)
                except:
                    print("Invalid input")
            
            elif choice == "2":
                # Move relative
                try:
                    az_move = float(input("Azimuth change (degrees): "))
                    alt_move = float(input("Altitude change (degrees): "))
                    
                    if az_move != 0:
                        self.move_azimuth_degrees(az_move)
                    if alt_move != 0:
                        self.move_altitude_degrees(alt_move)
                except:
                    print("Invalid input")
            
            elif choice == "3":
                # Test sequence
                print("\nTesting motors...")
                self.move_azimuth_degrees(45)
                time.sleep(1)
                self.move_azimuth_degrees(-45)
                self.move_altitude_degrees(30)
                time.sleep(1)
                self.move_altitude_degrees(-30)
                print("✓ Test complete")
            
            elif choice == "4":
                break
            else:
                print("Invalid choice")
    
    # ========== FEATURE 3: CALIBRATION ==========
    def calibrate(self):
        """Set current position as origin (0°, 0°)"""
        print("\n" + "="*60)
        print("CALIBRATION")
        print("="*60)
        print("Point laser to desired forward direction")
        print("Current position will be set as (0°, 0°)")
        
        confirm = input("\nCalibrate at current position? (y/n): ").lower()
        if confirm == 'y':
            self.home_azimuth = self.azimuth_angle
            self.home_altitude = self.altitude_angle
            
            # Reset relative angles to 0
            self.azimuth_angle = 0.0
            self.altitude_angle = 0.0
            
            print("✓ Calibrated! Current position is now (0°, 0°)")
        else:
            print("Calibration cancelled")
    
    # ========== FEATURE 4: COMPETITION AUTO-FIRE ==========
    def fetch_competition_data(self):
        """Fetch competition JSON data"""
        print("\n" + "="*60)
        print("FETCH COMPETITION DATA")
        print("="*60)
        
        if not self.TEAM_NUMBER:
            self.TEAM_NUMBER = input("Enter your team number: ").strip()
        
        print(f"\nFetching data for Team {self.TEAM_NUMBER}...")
        
        try:
            response = requests.get(self.SERVER_URL, timeout=10)
            if response.status_code == 200:
                data = response.json()
                
                # Find our position
                if self.TEAM_NUMBER in data.get("turrets", {}):
                    our_data = data["turrets"][self.TEAM_NUMBER]
                    r = our_data['r']
                    theta = our_data['theta']
                    
                    print(f"✓ Found Team {self.TEAM_NUMBER}:")
                    print(f"  r = {r} cm")
                    print(f"  θ = {theta:.3f} rad ({math.degrees(theta):.1f}°)")
                    
                    return data, r, theta
                else:
                    print(f"✗ Team {self.TEAM_NUMBER} not found")
                    return None, None, None
            
            else:
                print(f"✗ Server error: {response.status_code}")
                return None, None, None
                
        except Exception as e:
            print(f"✗ Error: {e}")
            return None, None, None
    
    def calculate_target_angles(self, target_r, target_theta, target_z, our_r, our_theta):
        """
        Calculate angles to hit a target
        Returns: (azimuth, altitude) in degrees
        """
        # Convert to cartesian
        our_x = our_r * math.cos(our_theta)
        our_y = our_r * math.sin(our_theta)
        
        target_x = target_r * math.cos(target_theta)
        target_y = target_r * math.sin(target_theta)
        
        # Relative position
        dx = target_x - our_x
        dy = target_y - our_y
        
        # Azimuth: angle from our forward direction
        target_angle = math.atan2(dy, dx)
        azimuth = math.degrees(target_angle - our_theta)
        
        # Altitude: based on height and distance
        distance = math.sqrt(dx*dx + dy*dy)
        if distance > 0:
            altitude = math.degrees(math.atan2(target_z, distance))
        else:
            altitude = 90.0 if target_z > 0 else -90.0
        
        return azimuth, altitude
    
    def automated_firing(self):
        """Automatic firing at all targets"""
        print("\n" + "="*60)
        print("AUTOMATED FIRING SEQUENCE")
        print("="*60)
        
        # Get competition data
        data, our_r, our_theta = self.fetch_competition_data()
        if not data:
            print("Cannot proceed without competition data")
            return
        
        # Collect all targets
        targets = []
        
        # Other turrets
        for team_id, pos in data.get("turrets", {}).items():
            if team_id != self.TEAM_NUMBER:
                targets.append((
                    f"Turret {team_id}",
                    pos['r'],
                    pos['theta'],
                    0  # Turrets are on ground
                ))
        
        # Globes
        if "globes" in data:
            for i, globe in enumerate(data["globes"]):
                targets.append((
                    f"Globe {i+1}",
                    globe['r'],
                    globe['theta'],
                    globe['z']
                ))
        
        print(f"\nFound {len(targets)} targets")
        print("Starting automated firing sequence...")
        
        hits = 0
        
        for name, r, theta, z in targets:
            print(f"\nTargeting: {name}")
            print(f"  Position: r={r} cm, θ={theta:.3f} rad, z={z} cm")
            
            # Calculate angles
            azimuth, altitude = self.calculate_target_angles(r, theta, z, our_r, our_theta)
            print(f"  Required: Az={azimuth:+.1f}°, Alt={altitude:+.1f}°")
            
            # Check if reachable
            if altitude < self.MAX_ALT_UP or altitude > self.MAX_ALT_DOWN:
                print(f"  ⚠ Out of range (altitude {altitude:.1f}°)")
                continue
            
            # Move to target
            self.move_to_angle(azimuth, altitude)
            
            # Fire for 3 seconds
            print(f"  🔴 FIRING for {self.FIRING_DURATION} seconds...")
            self.laser_on()
            time.sleep(self.FIRING_DURATION)
            self.laser_off()
            
            hits += 1
            print(f"  ✓ Hit {name}")
            
            # Brief pause
            time.sleep(0.5)
        
        # Return to home
        self.move_to_angle(0, 0)
        
        print(f"\n" + "="*60)
        print(f"AUTOMATED FIRING COMPLETE")
        print("="*60)
        print(f"Targets hit: {hits}/{len(targets)}")
        print(f"Estimated score: +{hits * 2} points")
    
    # ========== FEATURE 5: MOCK TEST ==========
    def mock_test(self):
        """Test with manually entered target"""
        print("\n" + "="*60)
        print("MOCK TARGET TEST")
        print("="*60)
        
        print("Enter target information:")
        
        try:
            target_r = float(input("Radius r (cm, usually 300): ").strip() or "300")
            target_theta_deg = float(input("Angle θ (degrees from center): ").strip() or "45")
            target_z = float(input("Height z (cm, 0 for turrets): ").strip() or "0")
            
            target_theta = math.radians(target_theta_deg)
            
            # For mock test, assume we're at (300, 0)
            our_r = 300.0
            our_theta = 0.0
            
            # Calculate angles
            azimuth, altitude = self.calculate_target_angles(
                target_r, target_theta, target_z, our_r, our_theta
            )
            
            print(f"\nCalculated angles:")
            print(f"  Azimuth: {azimuth:+.1f}°")
            print(f"  Altitude: {altitude:+.1f}°")
            
            # Check limits
            if altitude < self.MAX_ALT_UP or altitude > self.MAX_ALT_DOWN:
                print(f"⚠ Target at altitude {altitude:.1f}° is OUT OF RANGE")
                print(f"  Your range: {self.MAX_ALT_UP}° to {self.MAX_ALT_DOWN}°")
                return
            
            # Move and fire
            move = input("\nMove to target? (y/n): ").lower()
            if move == 'y':
                self.move_to_angle(azimuth, altitude)
                
                fire = input("Fire laser for 3 seconds? (y/n): ").lower()
                if fire == 'y':
                    print("🔴 FIRING for 3 seconds...")
                    self.laser_on()
                    time.sleep(self.FIRING_DURATION)
                    self.laser_off()
                    print("✓ Target hit!")
                
                # Return to center
                self.move_to_angle(0, 0)
        
        except ValueError:
            print("Invalid input")
    
    # ========== UTILITY FUNCTIONS ==========
    def cleanup(self):
        """Clean shutdown"""
        print("\nCleaning up...")
        self.laser_off()
        self.shift_out(0b00000000)
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")
    
    def show_status(self):
        """Show current status"""
        print("\n" + "="*60)
        print("CURRENT STATUS")
        print("="*60)
        print(f"Position: Az={self.azimuth_angle:+.1f}°, Alt={self.altitude_angle:+.1f}°")
        print(f"Laser: {'ON 🔴' if self.laser_on_flag else 'OFF ⚫'}")
        print(f"Team: {self.TEAM_NUMBER or 'Not set'}")
        print(f"Altitude limits: {self.MAX_ALT_UP}° to {self.MAX_ALT_DOWN}°")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 COMPETITION TURRET")
    print("="*70)
    print("ALL 5 REQUIRED FEATURES:")
    print("1. Manual laser control")
    print("2. Manual motor adjustment")
    print("3. Calibration (set origin)")
    print("4. Automated competition firing")
    print("5. Mock target test")
    print("="*70)
    
    # Check for requests package
    try:
        import requests
    except ImportError:
        print("Installing requests package...")
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "requests"])
        import requests
    
    turret = CompetitionTurret()
    
    try:
        while True:
            print("\n" + "="*70)
            print("MAIN MENU")
            print("="*70)
            print("1. 🔴 Manual laser control")
            print("2. ⚙️  Manual motor adjustment")
            print("3. 🎯 Calibrate (set current as origin)")
            print("4. 🌐 Fetch competition data")
            print("5. 🚀 Automated firing sequence")
            print("6. 🎪 Mock target test")
            print("7. 📊 Show status")
            print("8. 🚪 Exit")
            
            choice = input("\nEnter choice (1-8): ").strip()
            
            if choice == "1":
                print("\nLaser control: 1=ON, 2=OFF, 3=Toggle, 4=Fire 3s")
                laser_choice = input("Choice: ").strip()
                if laser_choice == "1":
                    turret.laser_on()
                elif laser_choice == "2":
                    turret.laser_off()
                elif laser_choice == "3":
                    turret.toggle_laser()
                elif laser_choice == "4":
                    turret.laser_on()
                    time.sleep(3)
                    turret.laser_off()
                else:
                    print("Invalid choice")
            
            elif choice == "2":
                turret.manual_adjust()
            
            elif choice == "3":
                turret.calibrate()
            
            elif choice == "4":
                data, r, theta = turret.fetch_competition_data()
                if data:
                    print("✓ Competition data loaded")
            
            elif choice == "5":
                turret.automated_firing()
            
            elif choice == "6":
                turret.mock_test()
            
            elif choice == "7":
                turret.show_status()
            
            elif choice == "8":
                print("\nExiting...")
                break
            
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\nProgram interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        turret.cleanup()
        print("\nProgram ended")

if __name__ == "__main__":
    main()
