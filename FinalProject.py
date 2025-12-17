#!/usr/bin/env python3
"""
ENME441 Laser Turret - FINAL COMPETITION VERSION
Ready for ENME441 WiFi router with 3-second firing duration
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
        # ========== COMPETITION CONFIGURATION ==========
        # IMPORTANT: For competition, use the ENME441 WiFi router
        # The server URL will be: http://192.168.1.254:8000/positions.json
        self.SERVER_IP = "192.168.1.254"  # ENME441 WiFi router IP
        self.SERVER_URL = f"http://{self.SERVER_IP}:8000/positions.json"
        
        # Competition laser firing duration: 3 seconds (per rules)
        self.FIRING_DURATION = 3.0
        
        # Team number (will be asked later)
        self.team_number = None
        
        # Motor calibration
        self.AZIMUTH_STEPS_PER_REV = 1024    # Your fast motor
        self.ALTITUDE_STEPS_PER_REV = 4096   # Standard motor
        self.ALTITUDE_SPEED_FACTOR = 4       # Moves 4× faster to match azimuth
        
        # IMPORTANT: Azimuth orientation
        # At home position (0°), laser points to CENTER of arena (positive x direction)
        # Can rotate ±120° from this center-pointing direction
        self.MAX_AZIMUTH_LEFT = -120    # degrees (negative = left from center)
        self.MAX_AZIMUTH_RIGHT = 120    # degrees (positive = right from center)
        # NO LIMITS ON ALTITUDE MOTOR
        
        # Mechanical offsets (mm)
        self.LASER_HEIGHT = 43.0
        self.LASER_OFFSET = 23.5
        
        # Position tracking
        self.azimuth_angle = 0.0    # Current azimuth angle (0 = pointing to center)
        self.altitude_angle = 0.0   # Current altitude angle (0 = horizontal)
        self.azimuth_position = 0   # Steps from home
        self.altitude_position = 0  # Steps from home
        self.azimuth_phase = 0
        self.altitude_phase = 0
        
        # Store original/home position
        self.home_azimuth_angle = 0.0
        self.home_altitude_angle = 0.0
        self.home_azimuth_position = 0
        self.home_altitude_position = 0
        self.home_azimuth_phase = 0
        self.home_altitude_phase = 0
        
        # Target tracking
        self.competition_data = None
        self.my_position = None
        self.targets_hit = set()
        
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
        print("✓ GPIO initialized - Laser OFF")
    
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
        new_angle = self.azimuth_angle + (1.0 * direction * 360 / self.AZIMUTH_STEPS_PER_REV)
        
        # Check azimuth limits (±120° from center-pointing direction)
        if new_angle < self.MAX_AZIMUTH_LEFT:
            print(f"⚠ Azimuth limit reached: {new_angle:.1f}° < {self.MAX_AZIMUTH_LEFT}° (left limit)")
            return False
        if new_angle > self.MAX_AZIMUTH_RIGHT:
            print(f"⚠ Azimuth limit reached: {new_angle:.1f}° > {self.MAX_AZIMUTH_RIGHT}° (right limit)")
            return False
        
        if direction == 1:
            self.azimuth_phase = (self.azimuth_phase + 1) % 8
            self.azimuth_position += 1
        else:
            self.azimuth_phase = (self.azimuth_phase - 1) % 8
            self.azimuth_position -= 1
        
        self.azimuth_angle = new_angle
        self.update_motors()
        return True
    
    def step_altitude_fast(self, direction):
        """Take altitude steps (4× faster to match azimuth) - NO LIMITS"""
        steps = self.ALTITUDE_SPEED_FACTOR
        for _ in range(steps):
            if direction == 1:
                self.altitude_phase = (self.altitude_phase + 1) % 8
                self.altitude_position += 1
            else:
                self.altitude_phase = (self.altitude_phase - 1) % 8
                self.altitude_position -= 1
        
        self.altitude_angle = self.altitude_position * 360 / self.ALTITUDE_STEPS_PER_REV
        self.update_motors()
        return True
    
    def move_motors_sync(self, az_steps, alt_steps, delay=0.001):
        """Move both motors with synchronized speed"""
        if az_steps == 0 and alt_steps == 0:
            return True
        
        az_dir = 1 if az_steps >= 0 else -1
        alt_dir = 1 if alt_steps >= 0 else -1
        
        az_steps_abs = abs(az_steps)
        alt_steps_abs = abs(alt_steps)
        
        effective_alt_steps = alt_steps_abs / self.ALTITUDE_SPEED_FACTOR
        
        if az_steps_abs > effective_alt_steps:
            az_delay = delay
            if alt_steps_abs > 0:
                alt_delay = az_delay * (effective_alt_steps / az_steps_abs)
            else:
                alt_delay = delay
        else:
            if alt_steps_abs > 0:
                alt_delay = delay / self.ALTITUDE_SPEED_FACTOR
                az_delay = alt_delay * (az_steps_abs / effective_alt_steps)
            else:
                az_delay = delay
                alt_delay = delay
        
        az_delay = max(az_delay, 0.0001)
        alt_delay = max(alt_delay, 0.0001)
        
        az_counter = 0
        alt_counter = 0
        last_az_time = time.time()
        last_alt_time = time.time()
        
        success = True
        
        while az_counter < az_steps_abs or alt_counter < alt_steps_abs:
            current_time = time.time()
            
            if az_counter < az_steps_abs and (current_time - last_az_time) >= az_delay:
                if not self.step_azimuth(az_dir):
                    success = False
                    break
                last_az_time = current_time
                az_counter += 1
            
            if alt_counter < alt_steps_abs and (current_time - last_alt_time) >= alt_delay:
                self.step_altitude_fast(alt_dir)  # NO ALTITUDE LIMITS
                last_alt_time = current_time
                alt_counter += self.ALTITUDE_SPEED_FACTOR
            
            time.sleep(min(az_delay, alt_delay) / 10)
        
        return success
    
    def move_motors_degrees_sync(self, az_degrees, alt_degrees, delay=0.001):
        """Move by degrees with synchronized timing"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        return self.move_motors_sync(az_steps, alt_steps, delay)
    
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
        print("3. Test fire (3 seconds - competition duration)")
        print("4. Back to main menu")
        
        choice = input("\nEnter choice (1-4): ").strip()
        
        if choice == "1":
            self.laser_on()
            print("Laser turned ON")
        elif choice == "2":
            self.laser_off()
            print("Laser turned OFF")
        elif choice == "3":
            print(f"Firing laser for {self.FIRING_DURATION} seconds (competition duration)...")
            self.laser_on()
            time.sleep(self.FIRING_DURATION)
            self.laser_off()
            print("Laser fired.")
    
    def manual_adjust_motors(self):
        """Manually set motor angles - NO ALTITUDE LIMITS"""
        print("\n" + "="*60)
        print("MANUAL MOTOR ADJUSTMENT")
        print("="*60)
        
        print(f"Current position: Azimuth={self.azimuth_angle:.1f}°, Altitude={self.altitude_angle:.1f}°")
        print(f"Azimuth limits: {self.MAX_AZIMUTH_LEFT}° to {self.MAX_AZIMUTH_RIGHT}° (center-pointing = 0°)")
        print("Altitude: NO LIMITS")
        
        try:
            # Get azimuth input
            az_input = input(f"\nEnter azimuth angle ({self.MAX_AZIMUTH_LEFT} to {self.MAX_AZIMUTH_RIGHT}, current={self.azimuth_angle:.1f}): ").strip()
            if az_input:
                new_az = float(az_input)
                if new_az < self.MAX_AZIMUTH_LEFT or new_az > self.MAX_AZIMUTH_RIGHT:
                    print(f"Azimuth angle must be between {self.MAX_AZIMUTH_LEFT}° and {self.MAX_AZIMUTH_RIGHT}°")
                    return
            else:
                new_az = self.azimuth_angle
            
            # Get altitude input - NO LIMITS
            alt_input = input(f"Enter altitude angle (current={self.altitude_angle:.1f}): ").strip()
            if alt_input:
                new_alt = float(alt_input)
            else:
                new_alt = self.altitude_angle
            
            print(f"\nMoving to: Azimuth={new_az:.1f}°, Altitude={new_alt:.1f}°")
            
            # Calculate movement needed
            az_move = new_az - self.azimuth_angle
            alt_move = new_alt - self.altitude_angle
            
            print(f"Movement needed: ΔAz={az_move:.1f}°, ΔAlt={alt_move:.1f}°")
            
            success = self.move_motors_degrees_sync(az_move, alt_move, 0.001)
            
            if success:
                print(f"✓ Motors moved to: Azimuth={self.azimuth_angle:.1f}°, Altitude={self.altitude_angle:.1f}°")
            else:
                print("⚠ Could not move to position (azimuth limits?)")
                
        except ValueError:
            print("Invalid input. Please enter numeric values.")
    
    def motor_calibration(self):
        """Manually set motors to starting point"""
        print("\n" + "="*60)
        print("MOTOR CALIBRATION")
        print("="*60)
        
        print("IMPORTANT: Align turret so that:")
        print("1. Laser points to CENTER of competition ring (positive x direction)")
        print("2. This is your 'forward' direction (0°)")
        print("3. Laser height is 43 mm above ground")
        print("4. Laser offset is 23.5 mm from azimuth shaft")
        print("\nPress Enter when aligned...")
        input()
        
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        self.azimuth_position = 0
        self.altitude_position = 0
        self.azimuth_phase = 0
        self.altitude_phase = 0
        
        self.home_azimuth_angle = self.azimuth_angle
        self.home_altitude_angle = self.altitude_angle
        self.home_azimuth_position = self.azimuth_position
        self.home_altitude_position = self.altitude_position
        self.home_azimuth_phase = self.azimuth_phase
        self.home_altitude_phase = self.altitude_phase
        
        self.update_motors()
        
        print("✓ Calibration complete!")
        print(f"Home position set to: Azimuth=0° (pointing to center), Altitude=0° (horizontal)")
        print(f"Azimuth range: {self.MAX_AZIMUTH_LEFT}° (left) to {self.MAX_AZIMUTH_RIGHT}° (right) from center")
    
    def json_file_reading(self):
        """Read JSON file and set team number - COMPETITION VERSION"""
        print("\n" + "="*60)
        print("JSON FILE READING - COMPETITION MODE")
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
        print(f"Server URL: {self.SERVER_URL}")
        print("Note: This should connect to ENME441 WiFi router at 192.168.1.254")
        
        try:
            response = requests.get(self.SERVER_URL, timeout=10)
            if response.status_code == 200:
                self.competition_data = response.json()
                
                if self.team_number in self.competition_data["turrets"]:
                    self.my_position = self.competition_data["turrets"][self.team_number]
                    print(f"✓ SUCCESS! Connected to competition server.")
                    print(f"✓ Team {self.team_number} position:")
                    print(f"  r = {self.my_position['r']} cm")
                    print(f"  θ = {self.my_position['theta']} rad ({math.degrees(self.my_position['theta']):.1f}°)")
                    
                    # Count targets
                    turret_count = len(self.competition_data["turrets"]) - 1
                    globe_count = len(self.competition_data["globes"])
                    print(f"\nCompetition targets:")
                    print(f"  • Other turrets: {turret_count}")
                    print(f"  • Globes: {globe_count}")
                    print(f"  • Total targets: {turret_count + globe_count}")
                    
                else:
                    print(f"✗ Team {self.team_number} not found in competition data")
                    available = list(self.competition_data["turrets"].keys())
                    print(f"Available teams: {available}")
                    self.competition_data = None
                    self.my_position = None
            else:
                print(f"✗ Server error: HTTP {response.status_code}")
                print("Check: 1. Connected to ENME441 WiFi? 2. Server running?")
                
        except requests.exceptions.ConnectionError:
            print(f"✗ CANNOT CONNECT TO SERVER at {self.SERVER_IP}")
            print("\nTROUBLESHOOTING:")
            print("1. Make sure Pi is connected to 'ENME441' WiFi network")
            print("2. Check WiFi password if required")
            print("3. Verify server is running at 192.168.1.254:8000")
            print("4. Try: ping 192.168.1.254")
            print("5. If using a different network, update SERVER_IP in code")
        except Exception as e:
            print(f"✗ Error: {e}")
    
    # ========== AUTOMATED FIRING SEQUENCE ==========
    
    def calculate_target_angles(self, target_r, target_theta, target_z=0):
        """
        Calculate aiming angles for a target
        Home position (0°,0°): Laser points to CENTER of arena (positive x direction)
        Returns: (azimuth_angle, altitude_angle) in degrees
        """
        if not self.my_position:
            return (0, 0)
        
        our_r = self.my_position['r'] / 100.0
        our_theta = self.my_position['theta']
        
        target_r_m = target_r / 100.0
        target_z_m = target_z / 100.0
        
        # Convert to Cartesian
        our_x = our_r * math.cos(our_theta)
        our_y = our_r * math.sin(our_theta)
        
        target_x = target_r_m * math.cos(target_theta)
        target_y = target_r_m * math.sin(target_theta)
        
        # Vector from us to target
        dx = target_x - our_x
        dy = target_y - our_y
        dz = target_z_m
        
        # Vector from us to CENTER (0,0) - this is our "forward" direction
        center_dx = -our_x  # From us to (0,0)
        center_dy = -our_y
        
        # Calculate angle between "to-center" vector and "to-target" vector
        # This gives azimuth relative to center-pointing direction
        dot_product = center_dx * dx + center_dy * dy
        cross_product = center_dx * dy - center_dy * dx
        
        distance_to_center = math.sqrt(center_dx**2 + center_dy**2)
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        if distance_to_center > 0 and distance_to_target > 0:
            cos_angle = dot_product / (distance_to_center * distance_to_target)
            cos_angle = max(-1.0, min(1.0, cos_angle))
            
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
        
        # Simple offset correction
        if distance_2d > 0:
            offset_correction = math.degrees(math.atan2(self.LASER_OFFSET/1000.0, distance_2d))
            azimuth_deg += offset_correction * 0.5
        
        return (azimuth_deg, altitude_deg)
    
    def find_next_target(self):
        """Find next target that hasn't been hit yet"""
        if not self.competition_data:
            print("No competition data loaded.")
            return None
        
        # Store all valid targets
        valid_targets = []
        
        # Check other turrets first
        for team, pos in self.competition_data["turrets"].items():
            if team != self.team_number:
                target_id = f"turret_{team}"
                if target_id not in self.targets_hit:
                    az, alt = self.calculate_target_angles(pos['r'], pos['theta'])
                    
                    # Check if within azimuth limits (±120° from center)
                    if self.MAX_AZIMUTH_LEFT <= az <= self.MAX_AZIMUTH_RIGHT:
                        valid_targets.append({
                            'type': 'turret',
                            'id': team,
                            'r': pos['r'],
                            'theta': pos['theta'],
                            'z': 0,
                            'azimuth': az,
                            'altitude': alt,
                            'priority': 1  # Turrets first
                        })
        
        # Check globes
        for i, globe in enumerate(self.competition_data["globes"]):
            target_id = f"globe_{i}"
            if target_id not in self.targets_hit:
                az, alt = self.calculate_target_angles(globe['r'], globe['theta'], globe['z'])
                
                # Check if within azimuth limits (±120° from center)
                if self.MAX_AZIMUTH_LEFT <= az <= self.MAX_AZIMUTH_RIGHT:
                    valid_targets.append({
                        'type': 'globe',
                        'id': i,
                        'r': globe['r'],
                        'theta': globe['theta'],
                        'z': globe['z'],
                        'azimuth': az,
                        'altitude': alt,
                        'priority': 2  # Globes second
                    })
        
        if not valid_targets:
            # DEBUG: Show why no targets found
            print(f"\nDEBUG: Current azimuth: {self.azimuth_angle:.1f}°")
            print(f"DEBUG: Azimuth limits: {self.MAX_AZIMUTH_LEFT}° to {self.MAX_AZIMUTH_RIGHT}°")
            
            # List all targets and their calculated angles
            print("\nDEBUG: All targets and their calculated azimuth angles:")
            for team, pos in self.competition_data["turrets"].items():
                if team != self.team_number:
                    az, alt = self.calculate_target_angles(pos['r'], pos['theta'])
                    print(f"  Turret {team}: az={az:.1f}°, alt={alt:.1f}°")
            
            for i, globe in enumerate(self.competition_data["globes"]):
                az, alt = self.calculate_target_angles(globe['r'], globe['theta'], globe['z'])
                print(f"  Globe {i}: az={az:.1f}°, alt={alt:.1f}°")
            
            return None
        
        # Sort by priority (turrets first) and then by angular distance from current position
        valid_targets.sort(key=lambda x: (x['priority'], abs(x['azimuth'] - self.azimuth_angle)))
        
        return valid_targets[0]
    
    def initiate_firing_sequence(self):
        """Fire at ALL targets in sequence - COMPETITION VERSION (3 seconds firing)"""
        print("\n" + "="*60)
        print("INITIATING FIRING SEQUENCE - COMPETITION MODE")
        print("="*60)
        print(f"Laser will fire for {self.FIRING_DURATION} seconds per target (competition rule)")
        
        if not self.team_number or not self.competition_data:
            print("Please set team number and fetch competition data first (Option 4)")
            return
        
        # Count total targets
        total_turrets = len(self.competition_data["turrets"]) - 1
        total_globes = len(self.competition_data["globes"])
        total_targets = total_turrets + total_globes
        
        print(f"Total targets: {total_targets}")
        print(f"Targets already hit: {len(self.targets_hit)}")
        print(f"Remaining targets: {total_targets - len(self.targets_hit)}")
        
        if total_targets - len(self.targets_hit) == 0:
            print("All targets have already been hit!")
            reset = input("Reset targets and start over? (y/n): ").strip().lower()
            if reset == 'y':
                self.targets_hit.clear()
                print("Targets reset. Starting fresh...")
            else:
                return
        
        input("\nPress Enter to begin firing sequence...")
        
        targets_fired = 0
        original_position = (self.azimuth_angle, self.altitude_angle)
        
        try:
            # Continue firing until no more targets
            while True:
                target = self.find_next_target()
                if not target:
                    print("\nNo more targets found within azimuth limits (±120° from center)")
                    break
                
                print(f"\n--- Target {targets_fired + 1} ---")
                print(f"Target: {target['type'].upper()} {target['id']}")
                print(f"Position: r={target['r']:.1f} cm, θ={target['theta']:.3f} rad", end="")
                if target['type'] == 'globe':
                    print(f", z={target['z']:.1f} cm")
                else:
                    print()
                print(f"Aiming angles: Az={target['azimuth']:.1f}°, Alt={target['altitude']:.1f}°")
                print(f"Current position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
                
                # Move to target
                print("Moving to target...")
                success = self.move_motors_degrees_sync(
                    target['azimuth'] - self.azimuth_angle,
                    target['altitude'] - self.altitude_angle,
                    0.001
                )
                
                if not success:
                    print("⚠ Could not move to target (azimuth limits?)")
                    # Mark as hit anyway to avoid getting stuck
                    target_id = f"{target['type']}_{target['id']}"
                    self.targets_hit.add(target_id)
                    continue
                
                print("✓ Aimed at target")
                print(f"Firing laser for {self.FIRING_DURATION} seconds (competition duration)...")
                
                # Fire laser for 3 seconds (competition rule)
                self.laser_on()
                time.sleep(self.FIRING_DURATION)
                self.laser_off()
                
                print(f"Laser fired for {self.FIRING_DURATION}s. Waiting 1 second before moving...")
                time.sleep(1.0)  # WAIT 1 SECOND BEFORE MOVING
                
                # Mark target as hit
                target_id = f"{target['type']}_{target['id']}"
                self.targets_hit.add(target_id)
                targets_fired += 1
                print(f"✓ Target hit! ({targets_fired}/{total_targets})")
            
            print(f"\n✓ Firing sequence complete!")
            print(f"Total targets hit in this session: {targets_fired}")
            print(f"Total targets hit overall: {len(self.targets_hit)}/{total_targets}")
            
            # Return to original position
            print("\nReturning to original position...")
            self.move_motors_degrees_sync(
                original_position[0] - self.azimuth_angle,
                original_position[1] - self.altitude_angle,
                0.001
            )
            print("✓ Returned to original position")
            
        except KeyboardInterrupt:
            print("\nFiring sequence interrupted by user")
            print("Returning to original position...")
            self.move_motors_degrees_sync(
                original_position[0] - self.azimuth_angle,
                original_position[1] - self.altitude_angle,
                0.001
            )
    
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
        az_steps_needed = self.home_azimuth_position - self.azimuth_position
        alt_steps_needed = self.home_altitude_position - self.altitude_position
        
        self.move_motors_sync(az_steps_needed, alt_steps_needed, 0.001)
        
        self.azimuth_angle = self.home_azimuth_angle
        self.altitude_angle = self.home_altitude_angle
        self.azimuth_position = self.home_azimuth_position
        self.altitude_position = self.home_altitude_position
        self.azimuth_phase = self.home_azimuth_phase
        self.altitude_phase = self.home_altitude_phase
        print("✓ At home position (pointing to center)")
    
    def auto_return_to_home(self):
        """Automatically return to home position (called on exit)"""
        print("\nAuto-returning to home position...")
        
        az_steps_needed = self.home_azimuth_position - self.azimuth_position
        alt_steps_needed = self.home_altitude_position - self.altitude_position
        
        if az_steps_needed != 0 or alt_steps_needed != 0:
            print(f"Moving: Az={az_steps_needed} steps, Alt={alt_steps_needed} steps")
            
            try:
                if not self.gpio_initialized:
                    self.setup_gpio()
                
                self.move_motors_sync(az_steps_needed, alt_steps_needed, 0.002)
                
                self.azimuth_angle = self.home_azimuth_angle
                self.altitude_angle = self.home_altitude_angle
                self.azimuth_position = self.home_azimuth_position
                self.altitude_position = self.home_altitude_position
                self.azimuth_phase = self.home_azimuth_phase
                self.altitude_phase = self.home_altitude_phase
                print("✓ Returned to home position")
                    
            except Exception as e:
                print(f"⚠ Error during auto-return: {e}")
        else:
            print("✓ Already at home position")
        
        try:
            self.shift_out(0b00000000)
            self.laser_off()
        except:
            pass
    
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
    print("ENME441 LASER TURRET - COMPETITION READY")
    print("="*70)
    print("Configuration:")
    print(f"  • Server IP: 192.168.1.254 (ENME441 WiFi router)")
    print(f"  • Firing duration: 3.0 seconds per target")
    print(f"  • Azimuth range: ±120° from center")
    print(f"  • Altitude: No limits")
    print("="*70)
    
    turret = None
    try:
        turret = CompetitionTurret()
        
        while True:
            print("\n" + "="*70)
            print("MAIN MENU")
            print("="*70)
            print("1. Manual toggle the laser")
            print("2. Manual adjust motors")
            print("3. Motor calibration")
            print("4. JSON file reading (COMPETITION)")
            print("5. Initiate firing sequence (3s firing)")
            print("6. Force cleanup & exit")
            print("7. Exit (Auto-return to home)")
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == "1":
                turret.manual_toggle_laser()
            elif choice == "2":
                turret.manual_adjust_motors()
            elif choice == "3":
                turret.motor_calibration()
            elif choice == "4":
                turret.json_file_reading()
            elif choice == "5":
                turret.initiate_firing_sequence()
            elif choice == "6":
                print("Force cleanup...")
                turret.force_cleanup()
                print("Exiting...")
                break
            elif choice == "7":
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
