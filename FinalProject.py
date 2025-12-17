#!/usr/bin/env python3
"""
ENME441 Laser Turret - COMPETITION READY
Features:
1. Single IP configuration
2. Home position calibration
3. JSON reading with team selection
4. Motor limits (±120° azimuth)
5. Targeting closest target with laser firing
6. Starting orientation: laser points to ring center
7. Auto-return to home on exit
8. Motor test limited to ±90°
"""

import RPi.GPIO as GPIO
import time
import json
import requests
import math
import sys
import atexit

class CompetitionTurret:
    def __init__(self):
        # ========== CONFIGURATION ==========
        # SET THIS TO YOUR SERVER IP (from ipconfig/ifconfig)
        self.SERVER_IP = "172.20.10.8"  # CHANGE THIS to your laptop's IP
        self.SERVER_URL = f"http://{self.SERVER_IP}:8000/positions.json"
        
        # Team number (will be asked later)
        self.team_number = None
        
        # Motor calibration
        self.AZIMUTH_STEPS_PER_REV = 1024    # Your fast motor
        self.ALTITUDE_STEPS_PER_REV = 4096   # Standard motor
        self.ALTITUDE_SPEED_FACTOR = 4       # Moves 4× faster to match azimuth
        
        # Motor limits (±120° from home)
        self.MAX_AZIMUTH_LEFT = -120    # degrees (negative = left)
        self.MAX_AZIMUTH_RIGHT = 120    # degrees (positive = right)
        
        # Starting orientation: laser points to CENTER of ring
        # This means when turret is at home (0,0), laser points inward
        # We'll handle this in targeting calculations
        
        # Position tracking
        self.azimuth_angle = 0.0    # Current angle in degrees (0 = home)
        self.altitude_angle = 0.0   # Current angle in degrees (0 = horizontal)
        self.azimuth_position = 0   # Steps from home
        self.altitude_position = 0  # Steps from home
        self.azimuth_phase = 0
        self.altitude_phase = 0
        
        # Store original/home position for auto-return
        self.home_azimuth_angle = 0.0
        self.home_altitude_angle = 0.0
        self.home_azimuth_position = 0
        self.home_altitude_position = 0
        self.home_azimuth_phase = 0
        self.home_altitude_phase = 0
        
        # Target tracking
        self.competition_data = None
        self.my_position = None
        self.targets_hit = set()  # Tracks which targets have been hit
        
        # GPIO pins
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP
        self.DATA_PIN = 9    # GPIO9  -> DS
        self.LASER_PIN = 26  # GPIO26 (HIGH = ON, LOW = OFF)
        
        # Stepper sequences (half-step)
        self.AZIMUTH_SEQ = [
            0b00000001, 0b00000011, 0b00000010, 0b00000110,
            0b00000100, 0b00001100, 0b00001000, 0b00001001
        ]
        
        self.ALTITUDE_SEQ = [
            0b00010000, 0b00110000, 0b00100000, 0b01100000,
            0b01000000, 0b11000000, 0b10000000, 0b10010000
        ]
        
        # State
        self.laser_state = False
        self.running = True
        
        self.setup_gpio()
        
        # Register auto-return function to run on exit
        atexit.register(self.auto_return_to_home)
    
    def setup_gpio(self):
        """Initialize all GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        
        # Shift register pins
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Laser pin
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        GPIO.output(self.LASER_PIN, GPIO.LOW)  # Start with laser OFF
        
        # Initialize
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        print("GPIO setup complete - Laser starts OFF (safe)")
    
    # ========== MOTOR CONTROL ==========
    
    def shift_out(self, data_byte):
        """Send 8 bits to shift register"""
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
        """Update both motors with current phases"""
        combined = (self.AZIMUTH_SEQ[self.azimuth_phase] | 
                   self.ALTITUDE_SEQ[self.altitude_phase])
        self.shift_out(combined)
    
    def step_azimuth(self, direction):
        """Take one azimuth step, check limits"""
        # Calculate new angle
        new_angle = self.azimuth_angle + (1.0 * direction * 360 / self.AZIMUTH_STEPS_PER_REV)
        
        # Check limits
        if new_angle < self.MAX_AZIMUTH_LEFT:
            print(f"⚠ Azimuth limit reached: {new_angle:.1f}° < {self.MAX_AZIMUTH_LEFT}° (left limit)")
            return False
        if new_angle > self.MAX_AZIMUTH_RIGHT:
            print(f"⚠ Azimuth limit reached: {new_angle:.1f}° > {self.MAX_AZIMUTH_RIGHT}° (right limit)")
            return False
        
        # Update position
        if direction == 1:  # Clockwise (right)
            self.azimuth_phase = (self.azimuth_phase + 1) % 8
            self.azimuth_position += 1
        else:  # Counterclockwise (left)
            self.azimuth_phase = (self.azimuth_phase - 1) % 8
            self.azimuth_position -= 1
        
        self.azimuth_angle = new_angle
        self.update_motors()
        return True
    
    def step_altitude_fast(self, direction):
        """Take altitude steps (4× faster to match azimuth)"""
        steps = self.ALTITUDE_SPEED_FACTOR
        for _ in range(steps):
            if direction == 1:  # Up
                self.altitude_phase = (self.altitude_phase + 1) % 8
                self.altitude_position += 1
            else:  # Down
                self.altitude_phase = (self.altitude_phase - 1) % 8
                self.altitude_position -= 1
        
        # Update altitude angle
        self.altitude_angle = self.altitude_position * 360 / self.ALTITUDE_STEPS_PER_REV
        self.update_motors()
    
    def move_motors_sync(self, az_steps, alt_steps, delay=0.001):
        """
        Move both motors with synchronized speed
        Returns True if successful, False if hit limits
        """
        if az_steps == 0 and alt_steps == 0:
            return True
        
        az_dir = 1 if az_steps >= 0 else -1
        alt_dir = 1 if alt_steps >= 0 else -1
        
        az_steps_abs = abs(az_steps)
        alt_steps_abs = abs(alt_steps)
        
        # Calculate timing
        if az_steps_abs > 0:
            az_delay = delay
            total_time = az_steps_abs * az_delay
            
            if alt_steps_abs > 0:
                alt_delay = total_time / alt_steps_abs
            else:
                alt_delay = delay
        else:
            az_delay = delay
            alt_delay = delay
        
        # Move motors with proper timing
        az_counter = 0
        alt_counter = 0
        last_az_time = time.time()
        last_alt_time = time.time()
        
        success = True
        
        while az_counter < az_steps_abs or alt_counter < alt_steps_abs:
            current_time = time.time()
            
            # Move azimuth if ready
            if az_counter < az_steps_abs and (current_time - last_az_time) >= az_delay:
                if not self.step_azimuth(az_dir):
                    success = False
                    break
                last_az_time = current_time
                az_counter += 1
            
            # Move altitude if ready (faster timing)
            if alt_counter < alt_steps_abs and (current_time - last_alt_time) >= alt_delay:
                self.step_altitude_fast(alt_dir)
                last_alt_time = current_time
                alt_counter += 1
            
            # Small sleep to prevent CPU hogging
            time.sleep(min(az_delay, alt_delay) / 10)
        
        return success
    
    def move_motors_degrees_sync(self, az_degrees, alt_degrees, delay=0.001):
        """Move by degrees with synchronized timing"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        return self.move_motors_sync(az_steps, alt_steps, delay)
    
    # ========== HOME POSITION CALIBRATION ==========
    
    def calibrate_home(self):
        """
        Calibrate home position
        User manually aligns laser to point to CENTER of ring
        """
        print("\n" + "="*60)
        print("HOME POSITION CALIBRATION")
        print("="*60)
        print("IMPORTANT: Align turret so that:")
        print("1. Laser points to CENTER of competition ring")
        print("2. This is your 'forward' direction (0°)")
        print("3. Flat sides parallel to motor protrusions")
        print("\nPress Enter when aligned...")
        input()
        
        # Reset all position tracking
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        self.azimuth_position = 0
        self.altitude_position = 0
        self.azimuth_phase = 0
        self.altitude_phase = 0
        
        # Store as home position for auto-return
        self.home_azimuth_angle = self.azimuth_angle
        self.home_altitude_angle = self.altitude_angle
        self.home_azimuth_position = self.azimuth_position
        self.home_altitude_position = self.altitude_position
        self.home_azimuth_phase = self.azimuth_phase
        self.home_altitude_phase = self.altitude_phase
        
        self.update_motors()
        
        print("✓ Home position calibrated!")
        print(f"  Azimuth: 0°, Altitude: 0°")
        print(f"  Limits: {self.MAX_AZIMUTH_LEFT}° (left) to {self.MAX_AZIMUTH_RIGHT}° (right)")
        print(f"  Auto-return to this position on exit")
    
    def go_to_home(self):
        """Return to home position"""
        print("Returning to home position...")
        az_steps_needed = self.home_azimuth_position - self.azimuth_position
        alt_steps_needed = self.home_altitude_position - self.altitude_position
        
        success = self.move_motors_sync(az_steps_needed, alt_steps_needed, 0.001)
        
        if success:
            self.azimuth_angle = self.home_azimuth_angle
            self.altitude_angle = self.home_altitude_angle
            self.azimuth_position = self.home_azimuth_position
            self.altitude_position = self.home_altitude_position
            self.azimuth_phase = self.home_azimuth_phase
            self.altitude_phase = self.home_altitude_phase
            print("✓ At home position (pointing to ring center)")
        else:
            print("⚠ Could not return to home (motor limits?)")
    
    def auto_return_to_home(self):
        """Automatically return to home position (called on exit)"""
        print("\nAuto-returning to home position...")
        
        # Calculate steps needed to return to home
        az_steps_needed = self.home_azimuth_position - self.azimuth_position
        alt_steps_needed = self.home_altitude_position - self.altitude_position
        
        # Only move if we're not already at home
        if az_steps_needed != 0 or alt_steps_needed != 0:
            print(f"Moving: Az={az_steps_needed} steps, Alt={alt_steps_needed} steps")
            
            # Move back slowly for safety
            self.move_motors_sync(az_steps_needed, alt_steps_needed, 0.002)
            
            # Update position to home
            self.azimuth_angle = self.home_azimuth_angle
            self.altitude_angle = self.home_altitude_angle
            self.azimuth_position = self.home_azimuth_position
            self.altitude_position = self.home_altitude_position
            self.azimuth_phase = self.home_azimuth_phase
            self.altitude_phase = self.home_altitude_phase
            
            print("✓ Returned to home position")
        else:
            print("✓ Already at home position")
        
        # Turn off motors and laser
        self.shift_out(0b00000000)
        self.laser_off()
    
    # ========== JSON READING & TARGETING ==========
    
    def fetch_competition_data(self, team_number):
        """Fetch JSON data and set team number"""
        self.team_number = team_number
        print(f"\nFetching competition data for Team {team_number}...")
        print(f"Server: {self.SERVER_URL}")
        
        try:
            response = requests.get(self.SERVER_URL, timeout=5)
            if response.status_code == 200:
                self.competition_data = response.json()
                
                # Find our position
                if self.team_number in self.competition_data["turrets"]:
                    self.my_position = self.competition_data["turrets"][self.team_number]
                    print(f"✓ Success! Team {self.team_number} position:")
                    print(f"  r = {self.my_position['r']} cm")
                    print(f"  θ = {self.my_position['theta']} rad ({math.degrees(self.my_position['theta']):.1f}°)")
                    
                    return True
                else:
                    print(f"✗ Team {self.team_number} not found in data")
                    available = list(self.competition_data["turrets"].keys())
                    print(f"Available teams: {available}")
                    return False
            else:
                print(f"✗ Server error: HTTP {response.status_code}")
                return False
                
        except requests.exceptions.ConnectionError:
            print(f"✗ Cannot connect to server at {self.SERVER_IP}")
            print("Check: 1. Server running? 2. Correct IP? 3. Pi connected to same network?")
            return False
        except Exception as e:
            print(f"✗ Error: {e}")
            return False
    
    def calculate_target_angles(self, target_r, target_theta, target_z=0):
        """
        Calculate aiming angles for a target
        IMPORTANT: Home position (0°) points to RING CENTER
        Returns: (azimuth_angle, altitude_angle) in degrees
        """
        if not self.my_position:
            print("Error: Our position not known")
            return (0, 0)
        
        # Our position in polar coordinates
        our_r = self.my_position['r']
        our_theta = self.my_position['theta']
        
        # Since our laser points to CENTER at home (0°),
        # we need to calculate the angle from our position to target
        # relative to the direction to center
        
        # Convert everything to Cartesian for easier calculation
        our_x = our_r * math.cos(our_theta)
        our_y = our_r * math.sin(our_theta)
        
        target_x = target_r * math.cos(target_theta)
        target_y = target_r * math.sin(target_theta)
        target_z = target_z
        
        # Calculate vector from us to target
        dx = target_x - our_x
        dy = target_y - our_y
        dz = target_z  # We're at z=0
        
        # Calculate vector from us to CENTER (0,0)
        center_dx = -our_x  # From us to (0,0)
        center_dy = -our_y
        
        # Calculate angle between "to-center" vector and "to-target" vector
        # This gives us azimuth relative to center-pointing direction
        dot_product = center_dx * dx + center_dy * dy
        cross_product = center_dx * dy - center_dy * dx
        
        distance_to_center = math.sqrt(center_dx**2 + center_dy**2)
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        if distance_to_center > 0 and distance_to_target > 0:
            # Cosine of angle between vectors
            cos_angle = dot_product / (distance_to_center * distance_to_target)
            # Clamp to avoid numerical errors
            cos_angle = max(-1.0, min(1.0, cos_angle))
            
            # Get angle in radians
            angle_rad = math.acos(cos_angle)
            
            # Determine sign using cross product (for left/right)
            if cross_product < 0:
                angle_rad = -angle_rad
            
            azimuth_deg = math.degrees(angle_rad)
        else:
            azimuth_deg = 0
        
        # Calculate altitude angle
        distance_2d = math.sqrt(dx*dx + dy*dy)
        if distance_2d > 0:
            altitude_rad = math.atan2(dz, distance_2d)
            altitude_deg = math.degrees(altitude_rad)
        else:
            altitude_deg = 90 if dz > 0 else -90
        
        return (azimuth_deg, altitude_deg)
    
    def find_closest_target(self):
        """Find closest target that hasn't been hit yet"""
        if not self.competition_data:
            print("No competition data loaded.")
            return None
        
        closest_target = None
        min_angle = float('inf')
        
        # Check other turrets
        for team, pos in self.competition_data["turrets"].items():
            if team != self.team_number:
                target_id = f"turret_{team}"
                if target_id not in self.targets_hit:
                    az, alt = self.calculate_target_angles(pos['r'], pos['theta'])
                    angle_from_current = abs(az - self.azimuth_angle)
                    
                    # Check if within motor limits
                    new_azimuth = self.azimuth_angle + (az - self.azimuth_angle)
                    if (self.MAX_AZIMUTH_LEFT <= new_azimuth <= self.MAX_AZIMUTH_RIGHT):
                        if angle_from_current < min_angle:
                            min_angle = angle_from_current
                            closest_target = {
                                'type': 'turret',
                                'id': team,
                                'azimuth': az,
                                'altitude': alt,
                                'distance_angle': angle_from_current
                            }
        
        # Check globes
        for i, globe in enumerate(self.competition_data["globes"]):
            target_id = f"globe_{i}"
            if target_id not in self.targets_hit:
                az, alt = self.calculate_target_angles(globe['r'], globe['theta'], globe['z'])
                angle_from_current = abs(az - self.azimuth_angle)
                
                # Check if within motor limits
                new_azimuth = self.azimuth_angle + (az - self.azimuth_angle)
                if (self.MAX_AZIMUTH_LEFT <= new_azimuth <= self.MAX_AZIMUTH_RIGHT):
                    if angle_from_current < min_angle:
                        min_angle = angle_from_current
                        closest_target = {
                            'type': 'globe',
                            'id': i,
                            'azimuth': az,
                            'altitude': alt,
                            'distance_angle': angle_from_current
                        }
        
        return closest_target
    
    def fire_at_closest_target(self):
        """Find, aim at, and fire at closest target"""
        print("\n" + "="*60)
        print("FINDING CLOSEST TARGET")
        print("="*60)
        
        target = self.find_closest_target()
        if not target:
            print("No valid targets found (all hit or out of range)")
            return False
        
        print(f"Target found: {target['type'].upper()} {target['id']}")
        print(f"Current position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
        print(f"Target position: Az={target['azimuth']:.1f}°, Alt={target['altitude']:.1f}°")
        print(f"Movement needed: ΔAz={target['azimuth']-self.azimuth_angle:.1f}°, ΔAlt={target['altitude']-self.altitude_angle:.1f}°")
        
        # Move to target
        print(f"\nMoving to target...")
        success = self.move_motors_degrees_sync(
            target['azimuth'] - self.azimuth_angle,
            target['altitude'] - self.altitude_angle,
            0.001
        )
        
        if not success:
            print("⚠ Could not move to target (motor limits?)")
            return False
        
        print(f"✓ Aimed at target")
        print(f"Firing laser for 1 second...")
        
        # Fire laser
        self.laser_on()
        time.sleep(1.0)
        self.laser_off()
        
        # Mark target as hit
        if target['type'] == 'turret':
            target_id = f"turret_{target['id']}"
        else:
            target_id = f"globe_{target['id']}"
        
        self.targets_hit.add(target_id)
        print(f"✓ Target hit! Marked as eliminated.")
        print(f"Targets hit so far: {len(self.targets_hit)}")
        
        return True
    
    # ========== MOTOR TEST (90° VERSION) ==========
    
    def motor_test_90(self):
        """Test motors with 90° rotations (changed from 180°)"""
        print("\n" + "="*60)
        print("MOTOR TEST: 90° rotations")
        print("="*60)
        print("Starting from current position...")
        print(f"Current: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
        
        test_cycles = 0
        max_cycles = 3  # Limit number of test cycles
        
        try:
            while self.running and test_cycles < max_cycles:
                test_cycles += 1
                print(f"\n--- Test Cycle {test_cycles} ---")
                
                # Check if we can move 90° to the right
                if self.azimuth_angle + 90 <= self.MAX_AZIMUTH_RIGHT:
                    print("1. 90° to the right")
                    success = self.move_motors_degrees_sync(90, 0, 0.0005)
                    if not success:
                        print("⚠ Hit motor limit")
                        break
                    print("Pausing 2 seconds...")
                    time.sleep(2)
                else:
                    print(f"⚠ Cannot move 90° right (would exceed {self.MAX_AZIMUTH_RIGHT}° limit)")
                    break
                
                # Check if we can move 90° to the left (back to start)
                if self.azimuth_angle - 90 >= self.MAX_AZIMUTH_LEFT:
                    print("2. 90° to the left (back to start)")
                    success = self.move_motors_degrees_sync(-90, 0, 0.0005)
                    if not success:
                        print("⚠ Hit motor limit")
                        break
                    print("Pausing 2 seconds...")
                    time.sleep(2)
                else:
                    print(f"⚠ Cannot move 90° left (would exceed {self.MAX_AZIMUTH_LEFT}° limit)")
                    break
                
                # Optional: Add some altitude movement
                print("3. Small altitude test")
                success = self.move_motors_degrees_sync(0, 30, 0.0005)
                if success:
                    time.sleep(1)
                    self.move_motors_degrees_sync(0, -30, 0.0005)
                    time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        finally:
            self.running = False
            print(f"\nMotor test completed ({test_cycles} cycle(s))")
    
    # ========== LASER CONTROL ==========
    
    def laser_on(self):
        """Turn laser ON"""
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_state = True
    
    def laser_off(self):
        """Turn laser OFF"""
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_state = False
    
    def fire_laser_test(self, duration=1.0):
        """Test fire laser"""
        print(f"\nFiring laser for {duration} second(s)...")
        self.laser_on()
        time.sleep(duration)
        self.laser_off()
        print("Laser fired.")
    
    # ========== UTILITY FUNCTIONS ==========
    
    def print_status(self):
        """Print current status"""
        print("\n" + "="*60)
        print("CURRENT STATUS")
        print("="*60)
        print(f"Team: {self.team_number if self.team_number else 'Not set'}")
        print(f"Position: Azimuth={self.azimuth_angle:.1f}°, Altitude={self.altitude_angle:.1f}°")
        print(f"Home position: Azimuth={self.home_azimuth_angle:.1f}°, Altitude={self.home_altitude_angle:.1f}°")
        print(f"Motor limits: {self.MAX_AZIMUTH_LEFT}° (left) to {self.MAX_AZIMUTH_RIGHT}° (right)")
        print(f"Targets hit: {len(self.targets_hit)}")
        
        if self.competition_data:
            total_targets = (len(self.competition_data['turrets']) - 1 + 
                           len(self.competition_data['globes']))
            print(f"Remaining targets: {total_targets - len(self.targets_hit)}")
    
    def cleanup(self):
        """Clean shutdown - will auto-return to home via atexit"""
        self.running = False
        print("\nInitiating cleanup...")
        # Note: auto_return_to_home() will be called automatically by atexit
        # This ensures it runs even if program crashes or is force-quit
    
    def force_cleanup(self):
        """Force cleanup without atexit (for manual calls)"""
        print("\nForce cleanup initiated...")
        self.auto_return_to_home()
        GPIO.cleanup()
        print("GPIO cleanup complete.")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 LASER TURRET - COMPETITION SYSTEM")
    print("="*70)
    print("Features:")
    print("  • Laser points to RING CENTER at home position")
    print("  • Motor limits: ±120° from home")
    print("  • Auto-return to home on exit")
    print("  • Motor test: ±90° rotations")
    print("="*70)
    
    turret = None
    try:
        turret = CompetitionTurret()
        
        while True:
            print("\n" + "="*70)
            print("MAIN MENU")
            print("="*70)
            print("1. Calibrate Home Position (Laser points to CENTER)")
            print("2. Set Team Number & Fetch Competition Data")
            print("3. Motor Test (90° rotations)")  # Changed from 180° to 90°
            print("4. Find & Fire at Closest Target")
            print("5. Test Fire Laser (1 second)")
            print("6. Return to Home Position")
            print("7. Show Current Status")
            print("8. Force Cleanup & Exit")
            print("9. Exit (Auto-return to home)")
            
            choice = input("\nEnter choice (1-9): ").strip()
            
            if choice == "1":
                turret.calibrate_home()
            elif choice == "2":
                if turret.team_number:
                    print(f"Current team: {turret.team_number}")
                    change = input("Change team? (y/n): ").strip().lower()
                    if change != 'y':
                        continue
                
                team = input("Enter your team number: ").strip()
                if team:
                    if turret.fetch_competition_data(team):
                        print("\n✓ Competition data loaded successfully!")
                        turret.print_status()
            elif choice == "3":
                turret.running = True
                turret.motor_test_90()  # Changed to 90° test
            elif choice == "4":
                if not turret.team_number or not turret.competition_data:
                    print("Please set team number and fetch competition data first (Option 2)")
                else:
                    turret.fire_at_closest_target()
            elif choice == "5":
                turret.fire_laser_test(1.0)
            elif choice == "6":
                turret.go_to_home()
            elif choice == "7":
                turret.print_status()
            elif choice == "8":
                print("Force cleanup...")
                turret.force_cleanup()
                print("Exiting...")
                break
            elif choice == "9":
                print("Exiting with auto-return to home...")
                break
            else:
                print("Invalid choice")
            
            turret.running = False
            
    except KeyboardInterrupt:
        print("\nProgram interrupted - Auto-returning to home...")
    except Exception as e:
        print(f"\nError: {e}")
        print("Auto-returning to home...")
        import traceback
        traceback.print_exc()
    finally:
        # The atexit handler will automatically call auto_return_to_home()
        # Even if we get here due to an exception
        if turret:
            # Ensure GPIO cleanup happens
            try:
                turret.shift_out(0b00000000)
                turret.laser_off()
                GPIO.cleanup()
            except:
                pass
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
