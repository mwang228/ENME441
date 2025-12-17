#!/usr/bin/env python3
"""
ENME441 Laser Turret - FIXED MOTOR CONTROL VERSION
Fixed altitude motor and firing sequence issues
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
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP
        self.DATA_PIN = 9    # GPIO9  -> DS
        self.LASER_PIN = 26  # GPIO26 (HIGH = ON, LOW = OFF)
        
        # Stepper sequences (SIMPLIFIED - full-step for reliability)
        self.STEP_SEQUENCE = [
            0b00010001,  # Both motors phase 1
            0b00100010,  # Both motors phase 2  
            0b01000100,  # Both motors phase 3
            0b10001000   # Both motors phase 4
        ]
        
        self.AZIMUTH_BITS = 0b00001111  # Lower 4 bits for azimuth
        self.ALTITUDE_BITS = 0b11110000  # Upper 4 bits for altitude
        
        # Current motor states
        self.azimuth_step = 0
        self.altitude_step = 0
        
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
        """Update both motors with current steps - SIMPLIFIED"""
        self.ensure_gpio()
        
        # Get azimuth and altitude phase values
        az_phase = self.STEP_SEQUENCE[self.azimuth_step % 4] & self.AZIMUTH_BITS
        alt_phase = self.STEP_SEQUENCE[self.altitude_step % 4] & self.ALTITUDE_BITS
        
        # Combine both motor phases
        combined = az_phase | alt_phase
        
        # Send to shift register
        self.shift_out(combined)
    
    def step_azimuth(self, direction):
        """Take one azimuth step, check limits"""
        new_angle = self.azimuth_angle + (direction * 360.0 / self.AZIMUTH_STEPS_PER_REV)
        
        # Check azimuth limits (±120° from center-pointing direction)
        if new_angle < self.MAX_AZIMUTH_LEFT:
            print(f"⚠ Azimuth limit reached: {new_angle:.1f}° < {self.MAX_AZIMUTH_LEFT}° (left limit)")
            return False
        if new_angle > self.MAX_AZIMUTH_RIGHT:
            print(f"⚠ Azimuth limit reached: {new_angle:.1f}° > {self.MAX_AZIMUTH_RIGHT}° (right limit)")
            return False
        
        # Update azimuth step and angle
        if direction > 0:
            self.azimuth_step = (self.azimuth_step + 1) % 4
            self.azimuth_position += 1
        else:
            self.azimuth_step = (self.azimuth_step - 1) % 4
            self.azimuth_position -= 1
        
        self.azimuth_angle = new_angle
        self.update_motors()
        time.sleep(0.001)  # Small delay for motor movement
        return True
    
    def step_altitude(self, direction):
        """Take one altitude step - NO LIMITS"""
        # Update altitude step and angle
        if direction > 0:
            self.altitude_step = (self.altitude_step + 1) % 4
            self.altitude_position += 1
        else:
            self.altitude_step = (self.altitude_step - 1) % 4
            self.altitude_position -= 1
        
        # Calculate new altitude angle
        self.altitude_angle = self.altitude_position * 360.0 / self.ALTITUDE_STEPS_PER_REV
        self.update_motors()
        time.sleep(0.001)  # Small delay for motor movement
        return True
    
    def move_motors_simple(self, az_steps, alt_steps, step_delay=0.002):
        """
        Simple motor movement - move both motors independently
        This is more reliable than complex synchronization
        """
        print(f"Moving motors: Az={az_steps} steps, Alt={alt_steps} steps")
        
        # Move azimuth motor
        if az_steps != 0:
            az_direction = 1 if az_steps > 0 else -1
            az_steps_abs = abs(az_steps)
            
            for _ in range(az_steps_abs):
                if not self.step_azimuth(az_direction):
                    print("⚠ Azimuth movement stopped due to limit")
                    break
                time.sleep(step_delay)
        
        # Move altitude motor (with faster speed factor)
        if alt_steps != 0:
            alt_direction = 1 if alt_steps > 0 else -1
            alt_steps_abs = abs(alt_steps)
            
            # Altitude motor moves faster
            for _ in range(alt_steps_abs // self.ALTITUDE_SPEED_FACTOR):
                self.step_altitude(alt_direction)
                time.sleep(step_delay / self.ALTITUDE_SPEED_FACTOR)
        
        return True
    
    def move_motors_degrees(self, az_degrees, alt_degrees, step_delay=0.002):
        """Move by degrees using simple movement"""
        print(f"Moving: Az={az_degrees:.1f}°, Alt={alt_degrees:.1f}°")
        
        # Calculate steps needed
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        
        return self.move_motors_simple(az_steps, alt_steps, step_delay)
    
    def move_to_angles(self, target_az, target_alt, step_delay=0.002):
        """Move to specific angles"""
        print(f"Moving to: Az={target_az:.1f}°, Alt={target_alt:.1f}°")
        print(f"From current: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
        
        az_move = target_az - self.azimuth_angle
        alt_move = target_alt - self.altitude_angle
        
        print(f"Movement needed: ΔAz={az_move:.1f}°, ΔAlt={alt_move:.1f}°")
        
        success = self.move_motors_degrees(az_move, alt_move, step_delay)
        
        if success:
            print(f"✓ Motors moved to: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
        else:
            print("⚠ Could not complete movement")
        
        return success
    
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
        print("\nYou can also use incremental movements:")
        print("  a/d - Azimuth left/right by 10°")
        print("  w/s - Altitude up/down by 10°")
        print("  q   - Quit to menu")
        
        while True:
            print(f"\nCurrent: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
            cmd = input("Enter command (a/d/w/s/q or 'set' for exact angles): ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'a':  # Azimuth left
                success = self.move_motors_degrees(-10, 0, 0.001)
            elif cmd == 'd':  # Azimuth right
                success = self.move_motors_degrees(10, 0, 0.001)
            elif cmd == 'w':  # Altitude up
                success = self.move_motors_degrees(0, 10, 0.001)
            elif cmd == 's':  # Altitude down
                success = self.move_motors_degrees(0, -10, 0.001)
            elif cmd == 'set':
                try:
                    az_input = input(f"Enter azimuth angle ({self.MAX_AZIMUTH_LEFT} to {self.MAX_AZIMUTH_RIGHT}): ").strip()
                    alt_input = input(f"Enter altitude angle: ").strip()
                    
                    if az_input:
                        new_az = float(az_input)
                        if new_az < self.MAX_AZIMUTH_LEFT or new_az > self.MAX_AZIMUTH_RIGHT:
                            print(f"Azimuth must be between {self.MAX_AZIMUTH_LEFT}° and {self.MAX_AZIMUTH_RIGHT}°")
                            continue
                    else:
                        new_az = self.azimuth_angle
                    
                    if alt_input:
                        new_alt = float(alt_input)
                    else:
                        new_alt = self.altitude_angle
                    
                    success = self.move_to_angles(new_az, new_alt, 0.001)
                    
                except ValueError:
                    print("Invalid input. Please enter numeric values.")
            else:
                print("Unknown command. Use a/d/w/s/q or 'set'")
    
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
        self.azimuth_step = 0
        self.altitude_step = 0
        
        self.home_azimuth_angle = self.azimuth_angle
        self.home_altitude_angle = self.altitude_angle
        self.home_azimuth_position = self.azimuth_position
        self.home_altitude_position = self.altitude_position
        
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
                
                # Move to target using SIMPLE movement
                print("Moving to target...")
                success = self.move_to_angles(target['azimuth'], target['altitude'], 0.001)
                
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
            self.move_to_angles(original_position[0], original_position[1], 0.001)
            print("✓ Returned to original position")
            
        except KeyboardInterrupt:
            print("\nFiring sequence interrupted by user")
            print("Returning to original position...")
            self.move_to_angles(original_position[0], original_position[1], 0.001)
    
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
        self.move_to_angles(self.home_azimuth_angle, self.home_altitude_angle, 0.001)
        print("✓ At home position (pointing to center)")
    
    def auto_return_to_home(self):
        """Automatically return to home position (called on exit)"""
        print("\nAuto-returning to home position...")
        
        try:
            if not self.gpio_initialized:
                self.setup_gpio()
            
            self.move_to_angles(self.home_azimuth_angle, self.home_altitude_angle, 0.002)
            print("✓ Returned to home position")
                
        except Exception as e:
            print(f"⚠ Error during auto-return: {e}")
        
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
    print("ENME441 LASER TURRET - FIXED MOTOR CONTROL")
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
            print("2. Manual adjust motors (NEW: a/d/w/s controls)")
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
