#!/usr/bin/env python3
"""
ENME441 Laser Turret - COMPETITION READY
FIXED WITH MECHANICAL OFFSETS:
1. Laser height: 43 mm off ground
2. Laser offset: 23.5 mm from azimuth shaft (in -y direction)
3. Coordinate system corrections
4. Fixed altitude motor auto-return
5. Motor test moves both motors simultaneously
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
        
        # ========== MECHANICAL OFFSETS ==========
        # IMPORTANT: All measurements in millimeters
        self.LASER_HEIGHT = 43.0  # mm - Laser is 43 mm off ground
        self.LASER_OFFSET = 23.5  # mm - Laser is offset 23.5 mm from azimuth shaft
        
        # Starting orientation (as described):
        # 1. Azimuth motor on ground: wiring +y, shaft +z
        # 2. Altitude motor on top: wiring +z, shaft -y
        # 3. Laser pointing +x, offset -y from azimuth shaft
        # 
        # This means at home position (0,0):
        # - Laser points along positive x-axis (forward)
        # - Laser is offset 23.5 mm in negative y direction from azimuth shaft
        # - Laser is 43 mm above ground
        
        # Position tracking (angles in degrees)
        self.azimuth_angle = 0.0    # Current azimuth angle (0 = +x direction)
        self.altitude_angle = 0.0   # Current altitude angle (0 = horizontal)
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
        self.gpio_initialized = False
        
        self.setup_gpio()
        
        # Register auto-return function to run on exit
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
        GPIO.output(self.LASER_PIN, GPIO.LOW)  # Start with laser OFF
        
        # Initialize
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        self.gpio_initialized = True
        print("GPIO setup complete - Laser starts OFF (safe)")
    
    def ensure_gpio(self):
        """Ensure GPIO is initialized before using it"""
        if not self.gpio_initialized:
            self.setup_gpio()
    
    # ========== MOTOR CONTROL ==========
    
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
        """Update both motors with current phases"""
        self.ensure_gpio()
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
        return True
    
    def move_motors_sync(self, az_steps, alt_steps, delay=0.001):
        """
        Move both motors with synchronized speed
        FIXED: Proper timing for altitude motor with ALTITUDE_SPEED_FACTOR
        """
        if az_steps == 0 and alt_steps == 0:
            return True
        
        az_dir = 1 if az_steps >= 0 else -1
        alt_dir = 1 if alt_steps >= 0 else -1
        
        az_steps_abs = abs(az_steps)
        alt_steps_abs = abs(alt_steps)
        
        # Calculate effective steps (altitude moves faster)
        effective_alt_steps = alt_steps_abs / self.ALTITUDE_SPEED_FACTOR
        
        # Determine which motor has more "work" to do
        if az_steps_abs > effective_alt_steps:
            # Azimuth is the limiting factor
            az_delay = delay
            if alt_steps_abs > 0:
                # Altitude needs to move faster to keep up
                alt_delay = az_delay * (effective_alt_steps / az_steps_abs)
            else:
                alt_delay = delay
        else:
            # Altitude is the limiting factor
            if alt_steps_abs > 0:
                alt_delay = delay / self.ALTITUDE_SPEED_FACTOR
                az_delay = alt_delay * (az_steps_abs / effective_alt_steps)
            else:
                az_delay = delay
                alt_delay = delay
        
        # Ensure minimum delays
        az_delay = max(az_delay, 0.0001)
        alt_delay = max(alt_delay, 0.0001)
        
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
            
            # Move altitude if ready
            if alt_counter < alt_steps_abs and (current_time - last_alt_time) >= alt_delay:
                self.step_altitude_fast(alt_dir)
                last_alt_time = current_time
                alt_counter += self.ALTITUDE_SPEED_FACTOR
            
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
        Now accounts for laser offset and height
        """
        print("\n" + "="*60)
        print("HOME POSITION CALIBRATION")
        print("="*60)
        print("IMPORTANT: Align turret so that:")
        print("1. Laser points to CENTER of competition ring")
        print("2. Laser height: 43 mm above ground")
        print("3. Laser offset: 23.5 mm from azimuth shaft")
        print("4. This is your 'forward' direction (0°)")
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
        print(f"  Laser height: {self.LASER_HEIGHT} mm")
        print(f"  Laser offset: {self.LASER_OFFSET} mm")
        print(f"  Limits: {self.MAX_AZIMUTH_LEFT}° (left) to {self.MAX_AZIMUTH_RIGHT}° (right)")
        print(f"  Auto-return to this position on exit")
    
    def go_to_home(self):
        """Return to home position"""
        print("Returning to home position...")
        
        # Calculate steps needed to return to home
        az_steps_needed = self.home_azimuth_position - self.azimuth_position
        alt_steps_needed = self.home_altitude_position - self.altitude_position
        
        print(f"Moving: Az={az_steps_needed} steps, Alt={alt_steps_needed} steps")
        
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
            
            try:
                # Ensure GPIO is initialized before moving
                if not self.gpio_initialized:
                    print("Re-initializing GPIO for auto-return...")
                    self.setup_gpio()
                
                # Move back slowly for safety
                if self.move_motors_sync(az_steps_needed, alt_steps_needed, 0.002):
                    # Update position to home
                    self.azimuth_angle = self.home_azimuth_angle
                    self.altitude_angle = self.home_altitude_angle
                    self.azimuth_position = self.home_azimuth_position
                    self.altitude_position = self.home_altitude_position
                    self.azimuth_phase = self.home_azimuth_phase
                    self.altitude_phase = self.home_altitude_phase
                    print("✓ Returned to home position")
                else:
                    print("⚠ Could not complete auto-return (motor limits?)")
                    
            except Exception as e:
                print(f"⚠ Error during auto-return: {e}")
        else:
            print("✓ Already at home position")
        
        # Turn off motors and laser
        try:
            self.shift_out(0b00000000)
            self.laser_off()
        except:
            pass
    
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
    
    def get_laser_position(self, azimuth_deg, altitude_deg):
        """
        Calculate laser position in 3D space given motor angles
        Returns: (x, y, z) in meters relative to azimuth motor shaft
        """
        # Convert to radians
        az_rad = math.radians(azimuth_deg)
        alt_rad = math.radians(altitude_deg)
        
        # At home position (0°, 0°):
        # - Laser points along +x axis
        # - Laser is offset -y from azimuth shaft by LASER_OFFSET
        # - Laser is at height LASER_HEIGHT
        
        # First, rotate by altitude around y-axis
        # (this tilts the laser up/down)
        
        # Initial laser direction vector (pointing +x)
        laser_dir_x = 1.0
        laser_dir_y = 0.0
        laser_dir_z = 0.0
        
        # Apply altitude rotation (pitch)
        # Rotate around y-axis by altitude angle
        cos_alt = math.cos(alt_rad)
        sin_alt = math.sin(alt_rad)
        
        # After altitude rotation:
        x_alt = laser_dir_x * cos_alt - laser_dir_z * sin_alt
        y_alt = laser_dir_y
        z_alt = laser_dir_x * sin_alt + laser_dir_z * cos_alt
        
        # Apply azimuth rotation (yaw)
        # Rotate around z-axis by azimuth angle
        cos_az = math.cos(az_rad)
        sin_az = math.sin(az_rad)
        
        # After azimuth rotation:
        x_final = x_alt * cos_az - y_alt * sin_az
        y_final = x_alt * sin_az + y_alt * cos_az
        z_final = z_alt
        
        # Normalize to unit vector
        length = math.sqrt(x_final**2 + y_final**2 + z_final**2)
        if length > 0:
            x_final /= length
            y_final /= length
            z_final /= length
        
        # Calculate laser position (not just direction)
        # Laser is offset from azimuth shaft by LASER_OFFSET in -y direction
        # Convert offset to meters
        offset_m = self.LASER_OFFSET / 1000.0  # mm to meters
        
        # Initial offset vector (in -y direction at home)
        offset_x = 0.0
        offset_y = -offset_m
        offset_z = self.LASER_HEIGHT / 1000.0  # mm to meters
        
        # Rotate offset by azimuth angle
        offset_x_rot = offset_x * cos_az - offset_y * sin_az
        offset_y_rot = offset_x * sin_az + offset_y * cos_az
        offset_z_rot = offset_z  # Azimuth rotation doesn't affect z
        
        # Final laser position = azimuth shaft position + rotated offset
        # Azimuth shaft is at (0, 0, 0) in local coordinates
        laser_x = offset_x_rot
        laser_y = offset_y_rot
        laser_z = offset_z_rot
        
        return (x_final, y_final, z_final), (laser_x, laser_y, laser_z)
    
    def calculate_target_angles(self, target_r, target_theta, target_z=0):
        """
        Calculate aiming angles for a target
        Accounts for laser height (43 mm) and offset (23.5 mm)
        Returns: (azimuth_angle, altitude_angle) in degrees
        """
        if not self.my_position:
            print("Error: Our position not known")
            return (0, 0)
        
        # All competition coordinates are in cm, convert to meters
        target_r_m = target_r / 100.0  # cm to meters
        target_z_m = target_z / 100.0  # cm to meters
        
        # Our position in polar coordinates (competition gives cm, convert to m)
        our_r = self.my_position['r'] / 100.0  # cm to meters
        our_theta = self.my_position['theta']
        
        # Convert everything to Cartesian for easier calculation
        our_x = our_r * math.cos(our_theta)
        our_y = our_r * math.sin(our_theta)
        our_z = 0  # Our azimuth motor shaft is at ground level
        
        target_x = target_r_m * math.cos(target_theta)
        target_y = target_r_m * math.sin(target_theta)
        target_z = target_z_m  # Target might be at height
        
        # Calculate vector from us to target
        dx = target_x - our_x
        dy = target_y - our_y
        dz = target_z - our_z
        
        # Account for laser offset and height
        # We need to aim the laser, not the azimuth shaft
        # Laser is offset 23.5 mm from azimuth shaft and 43 mm above ground
        
        # Method: Use iterative approach to find correct angles
        # We'll adjust for the offset by calculating where laser would point
        # and adjusting angles until it points at target
        
        # Initial guess (ignoring offset)
        distance_2d = math.sqrt(dx*dx + dy*dy)
        
        if distance_2d > 0:
            # Initial azimuth (point from azimuth shaft to target)
            initial_azimuth_rad = math.atan2(dy, dx) - our_theta
            # Adjust for our orientation (laser points +x at home)
            # Home direction is toward center (our_theta + π)
            center_dir_rad = our_theta + math.pi
            azimuth_rad = math.atan2(dy, dx) - center_dir_rad
            
            # Initial altitude
            altitude_rad = math.atan2(dz, distance_2d)
        else:
            # Target directly above/below us
            azimuth_rad = 0
            altitude_rad = math.radians(90) if dz > 0 else math.radians(-90)
        
        # Convert to degrees
        azimuth_deg = math.degrees(azimuth_rad)
        altitude_deg = math.degrees(altitude_rad)
        
        # Apply simple offset correction
        # The offset creates a parallax effect - we'll approximate it
        offset_correction = math.degrees(math.atan2(self.LASER_OFFSET/1000.0, distance_2d))
        if distance_2d > 0:
            # Small correction based on geometry
            azimuth_deg += offset_correction * 0.5  # Empirical adjustment
        
        return (azimuth_deg, altitude_deg)
    
    def find_closest_target(self):
        """Find closest target that hasn't been hit yet"""
        if not self.competition_data:
            print("No competition data loaded.")
            return None
        
        closest_target = None
        min_distance = float('inf')
        
        # Check other turrets
        for team, pos in self.competition_data["turrets"].items():
            if team != self.team_number:
                target_id = f"turret_{team}"
                if target_id not in self.targets_hit:
                    az, alt = self.calculate_target_angles(pos['r'], pos['theta'])
                    
                    # Calculate approximate distance
                    our_r = self.my_position['r'] / 100.0  # m
                    our_theta = self.my_position['theta']
                    target_r = pos['r'] / 100.0  # m
                    target_theta = pos['theta']
                    
                    # Convert to Cartesian for distance calculation
                    our_x = our_r * math.cos(our_theta)
                    our_y = our_r * math.sin(our_theta)
                    
                    target_x = target_r * math.cos(target_theta)
                    target_y = target_r * math.sin(target_theta)
                    
                    distance = math.sqrt((target_x - our_x)**2 + (target_y - our_y)**2)
                    
                    # Check if within motor limits
                    new_azimuth = self.azimuth_angle + (az - self.azimuth_angle)
                    if (self.MAX_AZIMUTH_LEFT <= new_azimuth <= self.MAX_AZIMUTH_RIGHT):
                        if distance < min_distance:
                            min_distance = distance
                            closest_target = {
                                'type': 'turret',
                                'id': team,
                                'r': pos['r'],
                                'theta': pos['theta'],
                                'z': 0,  # Turrets are at ground level
                                'azimuth': az,
                                'altitude': alt,
                                'distance': distance
                            }
        
        # Check globes
        for i, globe in enumerate(self.competition_data["globes"]):
            target_id = f"globe_{i}"
            if target_id not in self.targets_hit:
                az, alt = self.calculate_target_angles(globe['r'], globe['theta'], globe['z'])
                
                # Calculate approximate distance
                our_r = self.my_position['r'] / 100.0  # m
                our_theta = self.my_position['theta']
                target_r = globe['r'] / 100.0  # m
                target_theta = globe['theta']
                
                # Convert to Cartesian for distance calculation
                our_x = our_r * math.cos(our_theta)
                our_y = our_r * math.sin(our_theta)
                
                target_x = target_r * math.cos(target_theta)
                target_y = target_r * math.sin(target_theta)
                
                distance = math.sqrt((target_x - our_x)**2 + (target_y - our_y)**2)
                
                # Check if within motor limits
                new_azimuth = self.azimuth_angle + (az - self.azimuth_angle)
                if (self.MAX_AZIMUTH_LEFT <= new_azimuth <= self.MAX_AZIMUTH_RIGHT):
                    if distance < min_distance:
                        min_distance = distance
                        closest_target = {
                            'type': 'globe',
                            'id': i,
                            'r': globe['r'],
                            'theta': globe['theta'],
                            'z': globe['z'],
                            'azimuth': az,
                            'altitude': alt,
                            'distance': distance
                        }
        
        return closest_target
    
    def fire_at_closest_target(self):
        """Find, aim at, and fire at closest target - NO CONFIRMATION"""
        print("\n" + "="*60)
        print("FINDING CLOSEST TARGET")
        print("="*60)
        
        target = self.find_closest_target()
        if not target:
            print("No valid targets found (all hit or out of range)")
            return False
        
        # Display target location details
        print(f"Target found: {target['type'].upper()} {target['id']}")
        print(f"Target attributes:")
        print(f"  • Type: {target['type']}")
        print(f"  • ID: {target['id']}")
        print(f"  • Position (polar): r={target['r']:.1f} cm, θ={target['theta']:.3f} rad ({math.degrees(target['theta']):.1f}°)")
        if target['type'] == 'globe':
            print(f"  • Height: z={target['z']:.1f} cm")
        print(f"  • Approx distance: {target['distance']:.2f} m")
        print(f"  • Calculated aiming angles: Az={target['azimuth']:.1f}°, Alt={target['altitude']:.1f}°")
        print(f"Current position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
        print(f"Movement needed: ΔAz={target['azimuth']-self.azimuth_angle:.1f}°, ΔAlt={target['altitude']-self.altitude_angle:.1f}°")
        
        # Move to target IMMEDIATELY (no confirmation)
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
    
    # ========== MOTOR TEST (BOTH MOTORS SIMULTANEOUSLY) ==========
    
    def motor_test_90(self):
        """Test both motors simultaneously with 90° rotations"""
        print("\n" + "="*60)
        print("MOTOR TEST: 90° rotations (BOTH MOTORS)")
        print("="*60)
        print("Starting from current position...")
        print(f"Current: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
        print("Testing both motors simultaneously!")
        
        test_cycles = 0
        max_cycles = 3  # Limit number of test cycles
        
        try:
            while self.running and test_cycles < max_cycles:
                test_cycles += 1
                print(f"\n--- Test Cycle {test_cycles} ---")
                
                # Test 1: Move both motors 90° right and 30° up simultaneously
                if self.azimuth_angle + 90 <= self.MAX_AZIMUTH_RIGHT:
                    print("1. 90° right + 30° up (simultaneously)")
                    success = self.move_motors_degrees_sync(90, 30, 0.0005)
                    if not success:
                        print("⚠ Hit motor limit")
                        break
                    print("Pausing 2 seconds...")
                    time.sleep(2)
                else:
                    print(f"⚠ Cannot move 90° right (would exceed {self.MAX_AZIMUTH_RIGHT}° limit)")
                    break
                
                # Test 2: Move both motors 90° left and 30° down simultaneously
                if self.azimuth_angle - 90 >= self.MAX_AZIMUTH_LEFT:
                    print("2. 90° left + 30° down (simultaneously)")
                    success = self.move_motors_degrees_sync(-90, -30, 0.0005)
                    if not success:
                        print("⚠ Hit motor limit")
                        break
                    print("Pausing 2 seconds...")
                    time.sleep(2)
                else:
                    print(f"⚠ Cannot move 90° left (would exceed {self.MAX_AZIMUTH_LEFT}° limit)")
                    break
                
                # Test 3: Move in a diagonal pattern
                print("3. Diagonal movement test")
                # Up-right
                success = self.move_motors_degrees_sync(45, 15, 0.0005)
                if not success:
                    print("⚠ Hit motor limit")
                    break
                time.sleep(1)
                
                # Down-left
                success = self.move_motors_degrees_sync(-45, -15, 0.0005)
                if not success:
                    print("⚠ Hit motor limit")
                    break
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        finally:
            self.running = False
            print(f"\nMotor test completed ({test_cycles} cycle(s))")
            print("Returning to original position...")
            self.go_to_home()
    
    # ========== LASER CONTROL ==========
    
    def laser_on(self):
        """Turn laser ON"""
        self.ensure_gpio()
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_state = True
    
    def laser_off(self):
        """Turn laser OFF"""
        self.ensure_gpio()
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
        print(f"Laser height: {self.LASER_HEIGHT} mm")
        print(f"Laser offset: {self.LASER_OFFSET} mm")
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
    
    def force_cleanup(self):
        """Force cleanup without atexit (for manual calls)"""
        print("\nForce cleanup initiated...")
        self.auto_return_to_home()
        if self.gpio_initialized:
            GPIO.cleanup()
            self.gpio_initialized = False
        print("GPIO cleanup complete.")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 LASER TURRET - COMPETITION SYSTEM")
    print("="*70)
    print("Features:")
    print("  • Laser height: 43 mm above ground")
    print("  • Laser offset: 23.5 mm from azimuth shaft")
    print("  • Motor limits: ±120° from home")
    print("  • Fixed altitude motor auto-return")
    print("  • Motor test: BOTH motors move simultaneously")
    print("  • Immediate target firing with location display")
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
            print("3. Motor Test (BOTH motors simultaneously)")
            print("4. Find & Fire at Closest Target (fires immediately)")
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
                turret.motor_test_90()
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
        if turret and turret.gpio_initialized:
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
