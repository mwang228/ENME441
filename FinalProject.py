#!/usr/bin/env python3
"""
ENME441 LASER TURRET - FINAL COMPETITION VERSION (FIXED)
Non-recursive version to avoid recursion depth error
"""

import RPi.GPIO as GPIO
import time
import json
import requests
import math
import sys
import atexit
from typing import Dict, List, Tuple, Optional

class CompetitionTurret:
    def __init__(self):
        # ========== COMPETITION CONFIGURATION ==========
        self.SERVER_IP = "192.168.1.254"  # ENME441 WiFi router IP
        self.SERVER_URL = f"http://{self.SERVER_IP}:8000/positions.json"
        
        # Competition laser firing duration: 3 seconds (per rules)
        self.FIRING_DURATION = 3.0
        
        # Team number (will be asked later)
        self.team_number = None
        
        # ========== YOUR WORKING MOTOR CALIBRATIONS ==========
        # Based on your testing:
        self.AZIMUTH_STEPS_PER_REV = 2200      # Azimuth motor (works well!)
        self.ALTITUDE_STEPS_PER_REV = 450      # Altitude motor (double movement fixed)
        
        # ========== YOUR MEASURED PHYSICAL LIMITS ==========
        # Azimuth: Full range (works perfectly)
        self.MAX_AZIMUTH_LEFT = -180      # degrees (negative = left)
        self.MAX_AZIMUTH_RIGHT = 180      # degrees (positive = right)
        
        # Altitude: YOUR MEASURED LIMITS (-60° to +45°)
        # IMPORTANT: Negative angles = laser UP, Positive angles = laser DOWN
        self.MAX_ALTITUDE_UP = -60.0      # Maximum UP angle (negative)
        self.MAX_ALTITUDE_DOWN = 45.0     # Maximum DOWN angle (positive)
        
        # Competition field parameters
        self.FIELD_RADIUS = 300.0         # cm (3 meters)
        
        # ========== POSITION TRACKING ==========
        self.azimuth_angle = 0.0          # Current azimuth (0 = pointing forward)
        self.altitude_angle = 0.0         # Current altitude (0 = horizontal)
        self.azimuth_position = 0         # Steps from home
        self.altitude_position = 0        # Steps from home
        
        # Home position (set during calibration)
        self.home_azimuth_angle = 0.0
        self.home_altitude_angle = 0.0
        self.home_azimuth_position = 0
        self.home_altitude_position = 0
        
        # ========== COMPETITION DATA ==========
        self.competition_data = None
        self.my_position = None           # (r, theta) in cm and radians
        self.my_cartesian = (0, 0, 0)     # (x, y, z) coordinates
        self.targets_hit = set()
        
        # ========== GPIO CONFIGURATION ==========
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS (Pin 14)
        self.LASER_PIN = 26  # GPIO26 (HIGH = ON, LOW = OFF)
        
        # Motor bit assignments (confirmed working)
        self.AZIMUTH_BITS = 0b00001111    # Bits 0-3: Azimuth motor
        self.ALTITUDE_BITS = 0b11110000   # Bits 4-7: Altitude motor
        
        # ========== OPTIMIZED STEP SEQUENCE ==========
        # Using 2-phase excitation for more torque
        self.STEP_SEQUENCE = [
            0b00010001,  # Az: coil A, Alt: coil A
            0b00100010,  # Az: coil B, Alt: coil B  
            0b01000100,  # Az: coil C, Alt: coil C
            0b10001000,  # Az: coil D, Alt: coil D
        ]
        
        # Current step indices
        self.azimuth_step_idx = 0
        self.altitude_step_idx = 0
        
        # State tracking
        self.laser_state = False
        self.gpio_initialized = False
        
        # Initialize
        self.setup_gpio()
        
        # Register auto-return function
        atexit.register(self.auto_return_to_home)
        
        print("="*70)
        print("ENME441 COMPETITION TURRET - READY")
        print("="*70)
        print(f"Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev (Full range)")
        print(f"Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        print(f"Altitude limits: UP={self.MAX_ALTITUDE_UP}°, DOWN={self.MAX_ALTITUDE_DOWN}°")
        print("="*70)
    
    def setup_gpio(self):
        """Initialize all GPIO pins"""
        if self.gpio_initialized:
            return
            
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)  # Reduce warning noise
        
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
        
        # Turn off all motors initially
        self.shift_out(0b00000000)
        
        self.gpio_initialized = True
        print("✓ GPIO initialized - Motors OFF, Laser OFF")
    
    def ensure_gpio(self):
        """Ensure GPIO is initialized before using it"""
        if not self.gpio_initialized:
            self.setup_gpio()
    
    # ========== MOTOR CONTROL FUNCTIONS ==========
    
    def shift_out(self, data_byte):
        """Send 8 bits to shift register with optimized timing"""
        self.ensure_gpio()
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        for i in range(7, -1, -1):
            bit = (data_byte >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            # Minimal delay for reliability
            time.sleep(0.000001)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.000001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors with current step indices"""
        combined = (self.STEP_SEQUENCE[self.azimuth_step_idx] & self.AZIMUTH_BITS) | \
                   (self.STEP_SEQUENCE[self.altitude_step_idx] & self.ALTITUDE_BITS)
        self.shift_out(combined)
    
    def step_azimuth(self, direction):
        """Take one azimuth step with boundary checking"""
        self.azimuth_step_idx = (self.azimuth_step_idx + direction) % 4
        self.azimuth_position += direction
        self.azimuth_angle = self.azimuth_position * 360.0 / self.AZIMUTH_STEPS_PER_REV
        self.update_motors()
    
    def step_altitude(self, direction):
        """Take one altitude step with boundary checking"""
        self.altitude_step_idx = (self.altitude_step_idx + direction) % 4
        self.altitude_position += direction
        self.altitude_angle = self.altitude_position * 360.0 / self.ALTITUDE_STEPS_PER_REV
        self.update_motors()
    
    def move_motors_direct(self, az_steps: int, alt_steps: int, step_delay: float = 0.001) -> bool:
        """
        Direct motor movement with boundary checking
        Returns True if successful, False if limits reached
        """
        if az_steps == 0 and alt_steps == 0:
            return True
        
        az_direction = 1 if az_steps > 0 else -1
        alt_direction = 1 if alt_steps > 0 else -1
        
        az_steps_abs = abs(az_steps)
        alt_steps_abs = abs(alt_steps)
        
        az_completed = 0
        alt_completed = 0
        
        while az_completed < az_steps_abs or alt_completed < alt_steps_abs:
            # Move azimuth if needed
            if az_completed < az_steps_abs:
                # Calculate new angle after step
                new_az_angle = self.azimuth_angle + (az_direction * 360.0 / self.AZIMUTH_STEPS_PER_REV)
                
                # Check azimuth limits
                if new_az_angle < self.MAX_AZIMUTH_LEFT or new_az_angle > self.MAX_AZIMUTH_RIGHT:
                    print(f"⚠ Azimuth limit reached: {new_az_angle:.1f}°")
                    return False
                
                self.step_azimuth(az_direction)
                az_completed += 1
            
            # Move altitude if needed
            if alt_completed < alt_steps_abs:
                # Calculate new angle after step
                new_alt_angle = self.altitude_angle + (alt_direction * 360.0 / self.ALTITUDE_STEPS_PER_REV)
                
                # Check altitude limits (YOUR MEASURED LIMITS)
                if new_alt_angle < self.MAX_ALTITUDE_UP or new_alt_angle > self.MAX_ALTITUDE_DOWN:
                    print(f"⚠ Altitude limit reached: {new_alt_angle:.1f}°")
                    return False
                
                self.step_altitude(alt_direction)
                alt_completed += 1
            
            time.sleep(step_delay)
        
        return True
    
    def move_motors_degrees(self, az_degrees: float, alt_degrees: float, step_delay: float = 0.001) -> bool:
        """Move by degrees with boundary checking"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360.0)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360.0)
        
        print(f"Moving: Az={az_degrees:+.1f}° ({az_steps:+d} steps), Alt={alt_degrees:+.1f}° ({alt_steps:+d} steps)")
        
        if az_degrees != 0 or alt_degrees != 0:
            success = self.move_motors_direct(az_steps, alt_steps, step_delay)
            if success:
                print(f"✓ New position: Az={self.azimuth_angle:+.1f}°, Alt={self.altitude_angle:+.1f}°")
            else:
                print(f"✗ Movement failed (limits reached)")
            return success
        return True
    
    def move_to_angle(self, target_az: float, target_alt: float, step_delay: float = 0.001) -> bool:
        """Move to specific angles - NON-RECURSIVE VERSION"""
        az_move = target_az - self.azimuth_angle
        alt_move = target_alt - self.altitude_angle
        
        print(f"Moving to: Az={target_az:.1f}°, Alt={target_alt:.1f}°")
        print(f"Movement needed: ΔAz={az_move:.1f}°, ΔAlt={alt_move:.1f}°")
        
        return self.move_motors_degrees(az_move, alt_move, step_delay)
    
    # ========== MANUAL CONTROLS ==========
    
    def manual_toggle_laser(self):
        """Manually toggle laser on/off"""
        print("\n" + "="*60)
        print("MANUAL LASER CONTROL")
        print("="*60)
        
        while True:
            print(f"\nCurrent laser: {'ON 🔴' if self.laser_state else 'OFF ⚫'}")
            print(f"Position: Az={self.azimuth_angle:+.1f}°, Alt={self.altitude_angle:+.1f}°")
            
            print("\nOptions:")
            print("1. Turn laser ON")
            print("2. Turn laser OFF")
            print("3. Test fire (3 seconds - competition duration)")
            print("4. Return to menu")
            
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                self.laser_on()
            elif choice == "2":
                self.laser_off()
            elif choice == "3":
                print(f"\nFIRING LASER for {self.FIRING_DURATION} seconds...")
                self.laser_on()
                time.sleep(self.FIRING_DURATION)
                self.laser_off()
                print("✓ Laser fired")
            elif choice == "4":
                break
            else:
                print("Invalid choice")
    
    def manual_adjust_motors(self):
        """Manual motor adjustment with safety limits"""
        print("\n" + "="*60)
        print("MANUAL MOTOR ADJUSTMENT")
        print("="*60)
        
        while True:
            print(f"\nCurrent position:")
            print(f"  Azimuth: {self.azimuth_angle:+.1f}° (limits: {self.MAX_AZIMUTH_LEFT}° to {self.MAX_AZIMUTH_RIGHT}°)")
            print(f"  Altitude: {self.altitude_angle:+.1f}° (limits: {self.MAX_ALTITUDE_UP}° UP to {self.MAX_ALTITUDE_DOWN}° DOWN)")
            
            print("\nOptions:")
            print("1. Set exact angles")
            print("2. Move relative amounts")
            print("3. Test movement within limits")
            print("4. Return to menu")
            
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                # Set exact angles
                try:
                    az_input = input(f"Enter azimuth angle [{self.MAX_AZIMUTH_LEFT} to {self.MAX_AZIMUTH_RIGHT}]: ").strip()
                    alt_input = input(f"Enter altitude angle [{self.MAX_ALTITUDE_UP} to {self.MAX_ALTITUDE_DOWN}]: ").strip()
                    
                    if not az_input or not alt_input:
                        print("Both angles required")
                        continue
                    
                    target_az = float(az_input)
                    target_alt = float(alt_input)
                    
                    # Validate inputs
                    if target_az < self.MAX_AZIMUTH_LEFT or target_az > self.MAX_AZIMUTH_RIGHT:
                        print(f"Azimuth must be between {self.MAX_AZIMUTH_LEFT}° and {self.MAX_AZIMUTH_RIGHT}°")
                        continue
                    
                    if target_alt < self.MAX_ALTITUDE_UP or target_alt > self.MAX_ALTITUDE_DOWN:
                        print(f"Altitude must be between {self.MAX_ALTITUDE_UP}° and {self.MAX_ALTITUDE_DOWN}°")
                        continue
                    
                    self.move_to_angle(target_az, target_alt, 0.001)
                    
                except ValueError:
                    print("Invalid input. Please enter numbers.")
            
            elif choice == "2":
                # Move relative amounts
                try:
                    az_move = float(input("Azimuth change (degrees): ").strip())
                    alt_move = float(input("Altitude change (degrees): ").strip())
                    
                    # Check if movement would exceed limits
                    new_az = self.azimuth_angle + az_move
                    new_alt = self.altitude_angle + alt_move
                    
                    if new_az < self.MAX_AZIMUTH_LEFT or new_az > self.MAX_AZIMUTH_RIGHT:
                        print(f"Azimuth would exceed limits: {new_az:.1f}°")
                        continue
                    
                    if new_alt < self.MAX_ALTITUDE_UP or new_alt > self.MAX_ALTITUDE_DOWN:
                        print(f"Altitude would exceed limits: {new_alt:.1f}°")
                        continue
                    
                    self.move_motors_degrees(az_move, alt_move, 0.001)
                    
                except ValueError:
                    print("Invalid input. Please enter numbers.")
            
            elif choice == "3":
                # Test movement within limits
                print("\nTesting movement within your limits:")
                print("1. Move to maximum UP position")
                print("2. Move to maximum DOWN position")
                print("3. Move to center")
                print("4. Test diagonal movement")
                
                test_choice = input("Enter choice (1-4): ").strip()
                
                if test_choice == "1":
                    print(f"Moving to maximum UP ({self.MAX_ALTITUDE_UP}°)...")
                    self.move_to_angle(0, self.MAX_ALTITUDE_UP, 0.001)
                elif test_choice == "2":
                    print(f"Moving to maximum DOWN ({self.MAX_ALTITUDE_DOWN}°)...")
                    self.move_to_angle(0, self.MAX_ALTITUDE_DOWN, 0.001)
                elif test_choice == "3":
                    print("Moving to center (0°, 0°)...")
                    self.move_to_angle(0, 0, 0.001)
                elif test_choice == "4":
                    print("Testing diagonal (45°, -30°)...")
                    self.move_to_angle(45, -30, 0.001)
                else:
                    print("Invalid choice")
            
            elif choice == "4":
                break
            else:
                print("Invalid choice")
    
    def motor_calibration(self):
        """Set home position for competition"""
        print("\n" + "="*60)
        print("COMPETITION CALIBRATION")
        print("="*60)
        
        print("IMPORTANT: For competition, align your turret so that:")
        print("1. Laser points to the FIELD CENTER (marked origin)")
        print("2. This is your 'forward' direction (0°, 0°)")
        print("3. Make sure turret is at its assigned (r, θ) position")
        print("\nPress Enter when aligned...")
        input()
        
        # Reset all positions
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        self.azimuth_position = 0
        self.altitude_position = 0
        self.azimuth_step_idx = 0
        self.altitude_step_idx = 0
        
        # Set as home position
        self.home_azimuth_angle = self.azimuth_angle
        self.home_altitude_angle = self.altitude_angle
        self.home_azimuth_position = self.azimuth_position
        self.home_altitude_position = self.altitude_position
        
        # Update motors to new position
        self.update_motors()
        
        print("✓ Calibration complete!")
        print(f"Home position set: Azimuth=0°, Altitude=0°")
        print("Your turret is now ready for competition")
    
    def fetch_competition_data(self):
        """Fetch competition data from server"""
        print("\n" + "="*60)
        print("FETCH COMPETITION DATA")
        print("="*60)
        
        if self.team_number:
            print(f"Current team: {self.team_number}")
            change = input("Change team number? (y/n): ").strip().lower()
            if change == 'y':
                self.team_number = None
        
        if not self.team_number:
            team = input("Enter your team number (from competition sheet): ").strip()
            if not team:
                print("Team number required")
                return
            self.team_number = team
        
        print(f"\nFetching competition data for Team {self.team_number}...")
        print(f"Server: {self.SERVER_URL}")
        
        try:
            response = requests.get(self.SERVER_URL, timeout=10)
            if response.status_code == 200:
                self.competition_data = response.json()
                
                # Find our position
                if self.team_number in self.competition_data.get("turrets", {}):
                    self.my_position = self.competition_data["turrets"][self.team_number]
                    
                    # Convert to cartesian for calculations
                    r = self.my_position['r']  # cm
                    theta = self.my_position['theta']  # radians
                    
                    self.my_cartesian = (
                        r * math.cos(theta),  # x
                        r * math.sin(theta),  # y
                        0  # z (on ground)
                    )
                    
                    print(f"✓ Success! Team {self.team_number} position:")
                    print(f"  Polar: r={r} cm, θ={theta:.3f} rad ({math.degrees(theta):.1f}°)")
                    print(f"  Cartesian: x={self.my_cartesian[0]:.1f} cm, y={self.my_cartesian[1]:.1f} cm")
                    
                    # Count targets
                    turret_count = len(self.competition_data.get("turrets", {}))
                    globe_count = len(self.competition_data.get("globes", []))
                    print(f"\nCompetition info:")
                    print(f"  Turrets: {turret_count} teams")
                    print(f"  Globes: {globe_count} passive targets")
                    
                else:
                    print(f"✗ Team {self.team_number} not found in competition data")
                    self.competition_data = None
                    self.my_position = None
            else:
                print(f"✗ Server error: HTTP {response.status_code}")
                
        except requests.exceptions.ConnectionError:
            print(f"✗ Cannot connect to server. Check WiFi connection.")
            print(f"  Make sure you're connected to ENME441 WiFi")
        except Exception as e:
            print(f"✗ Error: {e}")
    
    def calculate_target_angles(self, target_x: float, target_y: float, target_z: float) -> Tuple[float, float]:
        """
        Calculate azimuth and altitude angles to hit a target
        Returns: (azimuth_angle, altitude_angle) in degrees
        """
        # Our position
        our_x, our_y, our_z = self.my_cartesian
        
        # Calculate relative position
        dx = target_x - our_x
        dy = target_y - our_y
        dz = target_z - our_z
        
        # Calculate horizontal distance
        horizontal_dist = math.sqrt(dx*dx + dy*dy)
        
        # Calculate azimuth (0° = pointing to field center)
        azimuth_rad = math.atan2(dy, dx)
        
        # Adjust azimuth based on our position
        if hasattr(self, 'my_position'):
            our_theta = self.my_position['theta']
            azimuth_rad = azimuth_rad - our_theta
        
        azimuth_deg = math.degrees(azimuth_rad)
        
        # Calculate altitude (0° = horizontal)
        # Negative altitude = aim UP, Positive altitude = aim DOWN
        if horizontal_dist > 0:
            altitude_rad = math.atan2(dz, horizontal_dist)
            altitude_deg = math.degrees(altitude_rad)
        else:
            altitude_deg = 90.0 if dz > 0 else -90.0
        
        return azimuth_deg, altitude_deg
    
    def automated_firing_sequence(self):
        """Automated firing at all targets"""
        print("\n" + "="*60)
        print("AUTOMATED FIRING SEQUENCE")
        print("="*60)
        
        if not self.team_number or not self.competition_data:
            print("Please fetch competition data first (Option 4)")
            return
        
        print(f"Team {self.team_number} - Starting automated firing...")
        print(f"Your position: r={self.my_position['r']} cm, θ={math.degrees(self.my_position['theta']):.1f}°")
        
        # Get all targets
        all_targets = []
        
        # Other turrets (opponents)
        for team_id, pos in self.competition_data.get("turrets", {}).items():
            if team_id != self.team_number:  # Don't shoot ourselves!
                r = pos['r']
                theta = pos['theta']
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                z = 0  # Turrets are on ground
                all_targets.append((f"Turret {team_id}", x, y, z))
        
        # Globes (passive targets)
        if "globes" in self.competition_data:
            for i, globe in enumerate(self.competition_data["globes"]):
                r = globe['r']
                theta = globe['theta']
                z = globe['z']
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                all_targets.append((f"Globe {i+1}", x, y, z))
        
        print(f"Found {len(all_targets)} targets")
        
        # Ask for confirmation
        print("\nThis will:")
        print("1. Move to each target")
        print(f"2. Fire laser for {self.FIRING_DURATION} seconds")
        print("3. Keep track of hits")
        
        confirm = input("\nStart automated firing? (y/n): ").strip().lower()
        if confirm != 'y':
            print("Cancelled")
            return
        
        print("\n" + "="*60)
        print("STARTING AUTOMATED FIRING")
        print("="*60)
        
        self.targets_hit.clear()
        total_targets = len(all_targets)
        
        for i, (target_name, x, y, z) in enumerate(all_targets):
            print(f"\n[{i+1}/{total_targets}] Targeting: {target_name}")
            print(f"  Position: x={x:.1f} cm, y={y:.1f} cm, z={z:.1f} cm")
            
            # Calculate angles
            azimuth_deg, altitude_deg = self.calculate_target_angles(x, y, z)
            
            print(f"  Required angles: Az={azimuth_deg:+.1f}°, Alt={altitude_deg:+.1f}°")
            
            # Check if target is within our limits
            if altitude_deg < self.MAX_ALTITUDE_UP or altitude_deg > self.MAX_ALTITUDE_DOWN:
                print(f"  ⚠ Target outside altitude range ({altitude_deg:.1f}°)")
                print(f"  Skipping {target_name}")
                continue
            
            # Move to target
            print(f"  Moving to target...")
            success = self.move_to_angle(azimuth_deg, altitude_deg, 0.001)
            
            if not success:
                print(f"  ⚠ Could not reach target (motor limits)")
                continue
            
            # Fire laser
            print(f"  FIRING LASER for {self.FIRING_DURATION} seconds...")
            self.laser_on()
            time.sleep(self.FIRING_DURATION)
            self.laser_off()
            
            # Record hit
            self.targets_hit.add(target_name)
            print(f"  ✓ Hit {target_name}")
            
            # Brief pause between targets
            if i < total_targets - 1:
                time.sleep(1)
        
        # Return to home
        print(f"\nReturning to home position...")
        self.move_to_angle(0, 0, 0.001)
        
        print("\n" + "="*60)
        print("FIRING SEQUENCE COMPLETE")
        print("="*60)
        print(f"Targets hit: {len(self.targets_hit)}/{total_targets}")
        if self.targets_hit:
            print("Hit list:")
            for target in sorted(self.targets_hit):
                print(f"  • {target}")
        
        # Calculate estimated score
        turret_hits = len([t for t in self.targets_hit if t.startswith("Turret")])
        globe_hits = len([t for t in self.targets_hit if t.startswith("Globe")])
        
        print(f"\nEstimated score: +{2 * len(self.targets_hit)} points")
        print(f"  Turrets hit: {turret_hits} (+{2 * turret_hits} points)")
        print(f"  Globes hit: {globe_hits} (+{2 * globe_hits} points)")
    
    def test_within_limits(self):
        """Test movement within your measured limits"""
        print("\n" + "="*60)
        print("TEST WITHIN YOUR LIMITS")
        print("="*60)
        
        print("Testing your measured working range:")
        print(f"Altitude: {self.MAX_ALTITUDE_UP}° UP to {self.MAX_ALTITUDE_DOWN}° DOWN")
        print(f"Azimuth: Full range available")
        
        test_points = [
            ("Center", 0, 0),
            ("Max UP", 0, self.MAX_ALTITUDE_UP),
            ("Max DOWN", 0, self.MAX_ALTITUDE_DOWN),
            ("Right 45°, Mid altitude", 45, (self.MAX_ALTITUDE_UP + self.MAX_ALTITUDE_DOWN)/2),
            ("Left 45°, Mid altitude", -45, (self.MAX_ALTITUDE_UP + self.MAX_ALTITUDE_DOWN)/2),
        ]
        
        for name, az, alt in test_points:
            print(f"\nTesting: {name} (Az={az:+.1f}°, Alt={alt:+.1f}°)")
            
            success = self.move_to_angle(az, alt, 0.001)
            if success:
                print(f"  ✓ Reached target")
            else:
                print(f"  ✗ Could not reach")
            
            time.sleep(1)
        
        # Return to center
        self.move_to_angle(0, 0, 0.001)
        print("\n✓ Limit test complete")
    
    # ========== UTILITY FUNCTIONS ==========
    
    def laser_on(self):
        """Turn laser ON"""
        self.ensure_gpio()
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_state = True
        print("LASER: ON 🔴")
    
    def laser_off(self):
        """Turn laser OFF"""
        self.ensure_gpio()
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_state = False
        print("LASER: OFF ⚫")
    
    def go_to_home(self):
        """Return to home position"""
        print("Returning to home position (0°, 0°)...")
        self.move_to_angle(0, 0, 0.001)
        print("✓ At home position")
    
    def auto_return_to_home(self):
        """Automatically return to home position (called on exit)"""
        print("\nAuto-returning to home position...")
        
        try:
            if not self.gpio_initialized:
                self.setup_gpio()
            
            # Turn off laser
            self.laser_off()
            
            # Return to home
            self.go_to_home()
            
            # Turn off motors
            self.shift_out(0b00000000)
            
            print("✓ Returned to home position")
                
        except Exception as e:
            print(f"⚠ Error during auto-return: {e}")
        
        try:
            if self.gpio_initialized:
                GPIO.cleanup()
                self.gpio_initialized = False
        except:
            pass
    
    def cleanup(self):
        """Force cleanup"""
        print("\nCleanup initiated...")
        self.auto_return_to_home()
        print("✓ Cleanup complete")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 COMPETITION LASER TURRET")
    print("="*70)
    print("FINAL VERSION - READY FOR COMPETITION")
    print("")
    print("YOUR CONFIGURATION:")
    print("• Azimuth: 2200 steps/rev (Full range)")
    print("• Altitude: 450 steps/rev")
    print(f"• Altitude limits: -60° UP to +45° DOWN")
    print("="*70)
    
    # Check for required packages
    try:
        import requests
    except ImportError:
        print("Installing 'requests' package...")
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "requests"])
        import requests
    
    turret = None
    try:
        turret = CompetitionTurret()
        
        while True:
            print("\n" + "="*70)
            print("COMPETITION CONTROL PANEL")
            print("="*70)
            print("PRE-COMPETITION:")
            print("1. Manual laser control")
            print("2. Manual motor adjustment (with limits)")
            print("3. Competition calibration (SET HOME POSITION)")
            print("4. Fetch competition data (REQUIRED)")
            print("5. Test within your limits")
            print("")
            print("COMPETITION:")
            print("6. Automated firing sequence (MAIN EVENT)")
            print("7. Return to home position")
            print("")
            print("SYSTEM:")
            print("8. Cleanup & exit")
            print("="*70)
            
            choice = input("\nEnter choice (1-8): ").strip()
            
            if choice == "1":
                turret.manual_toggle_laser()
            elif choice == "2":
                turret.manual_adjust_motors()
            elif choice == "3":
                turret.motor_calibration()
            elif choice == "4":
                turret.fetch_competition_data()
            elif choice == "5":
                turret.test_within_limits()
            elif choice == "6":
                turret.automated_firing_sequence()
            elif choice == "7":
                turret.go_to_home()
            elif choice == "8":
                print("\nCleaning up...")
                turret.cleanup()
                print("Exiting. Good luck in the competition!")
                break
            else:
                print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
        if turret:
            turret.cleanup()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        if turret:
            turret.cleanup()
    
    print("\nProgram ended.")

if __name__ == "__main__":
    main()
