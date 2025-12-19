#!/usr/bin/env python3
"""
ENME441 - FINAL WORKING VERSION
With altitude motor workaround and competition features
"""

import RPi.GPIO as GPIO
import time
import json
import os
import math
import urllib.request
import threading

class WorkingTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 - FINAL WORKING VERSION")
        print("="*70)
        print("SOLUTIONS:")
        print("• Azimuth: 2200 steps/rev")
        print("• Altitude: Power/torque issue → Software workaround")
        print("• Laser: GPIO26 control with offsets")
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        self.LASER_PIN = 26  # Updated laser control pin
        
        # Configuration
        self.CONFIG_FILE = "working_config.json"
        
        # FINAL VALUES
        self.AZIMUTH_STEPS_PER_REV = 2200  # Between 2000 and 2400
        self.ALTITUDE_STEPS_PER_REV = 450  # Keep as is
        
        self.load_config()
        
        print(f"FINAL configuration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        self.laser_on = False
        
        # Origin offset tracking
        self.azimuth_offset = 0
        self.altitude_offset = 0
        
        # Sequences - using 2-PHASE for more torque
        self.AZIMUTH_SEQUENCE = [
            0b00000011,  # A+B
            0b00000110,  # B+C
            0b00001100,  # C+D
            0b00001001,  # D+A
        ]
        
        # **ALTITUDE FIX: 2-phase with slower timing**
        self.ALTITUDE_SEQUENCE = [
            0b00010000 | 0b00100000,  # A+B
            0b00100000 | 0b01000000,  # B+C
            0b01000000 | 0b10000000,  # C+D
            0b10000000 | 0b00010000,  # D+A
        ]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # **POWER MANAGEMENT**
        self.AZIMUTH_DELAY = 0.015  # Normal speed
        self.ALTITUDE_DELAY = 0.040  # Slower for altitude (more torque)
        
        # Altitude workaround settings
        self.altitude_max_up = 45.0   # Maximum UP angle we can achieve
        self.altitude_max_down = -45.0 # Maximum DOWN angle
        
        # Turret physical properties (in cm)
        self.turret_radius = 300.0  # cm - distance from center
        self.laser_height = 4.3     # cm - height of laser above ground (43 mm)
        self.laser_offset_x = 2.7   # cm - horizontal offset from azimuth shaft to laser (27 mm)
        
        # Initialize
        self.setup_gpio()
        
        print(f"\n✓ FINAL system ready")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print(f"Laser: OFF (GPIO{self.LASER_PIN})")
        print(f"Laser offset: {self.laser_height} cm height, {self.laser_offset_x} cm right")
        print("="*70)
    
    def load_config(self):
        """Load configuration"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 2200)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 450)
                print("✓ Configuration loaded")
        except:
            print("Using final values")
    
    def save_config(self):
        """Save configuration"""
        try:
            config = {
                'azimuth_steps_per_rev': self.AZIMUTH_STEPS_PER_REV,
                'altitude_steps_per_rev': self.ALTITUDE_STEPS_PER_REV
            }
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
        except:
            pass
    
    def setup_gpio(self):
        """Initialize GPIO with power stabilization"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        
        # Initialize all pins to LOW
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        # Test laser briefly to ensure it works
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        # Start with motors OFF to save power
        self.send_to_shift_register(0b00000000)
        time.sleep(0.1)
    
    def send_to_shift_register(self, data):
        """Send with power stabilization"""
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
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth(self, direction):
        """Step azimuth motor"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude_with_power_boost(self, direction):
        """
        Step altitude with power boost
        Returns True if successful, False if failed
        """
        # Try stepping with retry logic
        max_retries = 2
        
        for attempt in range(max_retries):
            try:
                # Store current position
                old_pos = self.altitude_seq_pos
                old_steps = self.altitude_steps
                
                # Take the step
                self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
                self.altitude_steps += direction
                self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
                
                # Update with extra power delay
                self.update_motors()
                time.sleep(0.005)  # Extra hold time
                
                return True
                
            except Exception as e:
                # Revert on failure
                self.altitude_seq_pos = old_pos
                self.altitude_steps = old_steps
                self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
                
                if attempt < max_retries - 1:
                    # Retry with longer delay
                    time.sleep(self.ALTITUDE_DELAY * 2)
                    continue
        
        return False
    
    def step_both_motors_simultaneously(self, az_direction, alt_direction):
        """
        Step both motors simultaneously
        Returns (az_success, alt_success)
        """
        # Store current positions
        old_az_pos = self.azimuth_seq_pos
        old_az_steps = self.azimuth_steps
        old_alt_pos = self.altitude_seq_pos
        old_alt_steps = self.altitude_steps
        
        try:
            # Update azimuth
            self.azimuth_seq_pos = (self.azimuth_seq_pos + az_direction) % 4
            self.azimuth_steps += az_direction
            
            # Update altitude
            self.altitude_seq_pos = (self.altitude_seq_pos + alt_direction) % 4
            self.altitude_steps += alt_direction
            
            # Update angles
            self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
            self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
            
            # Update motors
            self.update_motors()
            time.sleep(0.005)  # Extra hold time
            
            return True, True
            
        except Exception as e:
            # Revert on failure
            self.azimuth_seq_pos = old_az_pos
            self.azimuth_steps = old_az_steps
            self.altitude_seq_pos = old_alt_pos
            self.altitude_steps = old_alt_steps
            self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
            self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
            return False, False
    
    def move_azimuth_safe(self, degrees):
        """Safe azimuth movement"""
        steps = int((degrees / 360) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(self.AZIMUTH_DELAY)
    
    def move_altitude_safe(self, degrees):
        """
        Safe altitude movement within limits
        Returns actual movement achieved
        """
        # Check limits
        target_angle = self.altitude_angle + degrees
        
        if target_angle > self.altitude_max_up:
            print(f"⚠️ Can't move to {target_angle:.1f}° (max UP: {self.altitude_max_up:.1f}°)")
            degrees = self.altitude_max_up - self.altitude_angle
        
        if target_angle < self.altitude_max_down:
            print(f"⚠️ Can't move to {target_angle:.1f}° (max DOWN: {self.altitude_max_down:.1f}°)")
            degrees = self.altitude_max_down - self.altitude_angle
        
        steps = int((degrees / 360) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        successful_steps = 0
        
        for i in range(steps):
            if self.step_altitude_with_power_boost(direction):
                successful_steps += 1
                time.sleep(self.ALTITUDE_DELAY)
            else:
                print(f"  Stopped at step {i+1}/{steps}")
                break
        
        actual_degrees = (successful_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0 * direction
        return actual_degrees
    
    def move_both_simultaneously(self, az_degrees, alt_degrees):
        """
        Move both motors simultaneously
        Returns actual movements achieved
        """
        # Calculate steps for each motor
        az_steps = int((az_degrees / 360) * self.AZIMUTH_STEPS_PER_REV)
        alt_steps = int((alt_degrees / 360) * self.ALTITUDE_STEPS_PER_REV)
        
        az_direction = 1 if az_steps > 0 else -1
        alt_direction = 1 if alt_steps > 0 else -1
        
        az_steps = abs(az_steps)
        alt_steps = abs(alt_steps)
        
        # Determine maximum steps and ratio
        max_steps = max(az_steps, alt_steps)
        az_done = 0
        alt_done = 0
        
        print(f"Moving simultaneously: Az={az_degrees:+.1f}° ({az_steps} steps), Alt={alt_degrees:+.1f}° ({alt_steps} steps)")
        
        for i in range(max_steps):
            az_step_now = (i < az_steps)
            alt_step_now = (i < alt_steps)
            
            if az_step_now and alt_step_now:
                # Try simultaneous step
                az_success, alt_success = self.step_both_motors_simultaneously(
                    az_direction if az_step_now else 0,
                    alt_direction if alt_step_now else 0
                )
                
                if az_success:
                    az_done += 1
                if alt_success:
                    alt_done += 1
                    
                # Use the longer delay for power stability
                time.sleep(max(self.AZIMUTH_DELAY, self.ALTITUDE_DELAY))
                
            elif az_step_now:
                # Only azimuth
                self.step_azimuth(az_direction)
                az_done += 1
                time.sleep(self.AZIMUTH_DELAY)
                
            elif alt_step_now:
                # Only altitude
                if self.step_altitude_with_power_boost(alt_direction):
                    alt_done += 1
                time.sleep(self.ALTITUDE_DELAY)
        
        # Calculate actual movements
        az_actual = (az_done / self.AZIMUTH_STEPS_PER_REV) * 360.0 * az_direction
        alt_actual = (alt_done / self.ALTITUDE_STEPS_PER_REV) * 360.0 * alt_direction
        
        print(f"Simultaneous movement complete:")
        print(f"  Azimuth: {az_done}/{az_steps} steps ({az_actual:+.1f}° of {az_degrees:+.1f}°)")
        print(f"  Altitude: {alt_done}/{alt_steps} steps ({alt_actual:+.1f}° of {alt_degrees:+.1f}°)")
        
        return az_actual, alt_actual
    
    def set_calibration_origin(self):
        """Set current position as origin (0,0)"""
        print("\n" + "="*60)
        print("CALIBRATION - SET ORIGIN")
        print("="*60)
        
        print(f"Current position:")
        print(f"  Azimuth: {self.azimuth_angle:.1f}° ({self.azimuth_steps} steps)")
        print(f"  Altitude: {self.altitude_angle:.1f}° ({self.altitude_steps} steps)")
        
        confirm = input("\nSet this position as origin (0,0)? (yes/no): ").lower()
        
        if confirm == 'yes':
            # Store current steps as offset
            self.azimuth_offset = self.azimuth_steps
            self.altitude_offset = self.altitude_steps
            
            # Reset step counters (but keep physical position)
            self.azimuth_steps = 0
            self.altitude_steps = 0
            self.azimuth_angle = 0.0
            self.altitude_angle = 0.0
            
            print("✓ Origin set at current position")
            print(f"  Azimuth offset: {self.azimuth_offset} steps")
            print(f"  Altitude offset: {self.altitude_offset} steps")
        else:
            print("Calibration cancelled")
    
    def convert_to_absolute_steps(self, rel_steps, axis='azimuth'):
        """Convert relative steps to absolute motor steps"""
        if axis == 'azimuth':
            return rel_steps + self.azimuth_offset
        else:  # altitude
            return rel_steps + self.altitude_offset
    
    def manual_laser_control(self):
        """Manual laser control interface"""
        print("\n" + "="*60)
        print("MANUAL LASER CONTROL")
        print("="*60)
        
        while True:
            print(f"\nCurrent laser state: {'ON' if self.laser_on else 'OFF'}")
            print(f"Position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
            
            print("\nCommands:")
            print("1 - Turn laser ON")
            print("2 - Turn laser OFF")
            print("3 - Fire laser for 3 seconds")
            print("t - Test laser briefly")
            print("q - Return to main menu")
            
            cmd = input("\nEnter command: ").lower()
            
            if cmd == '1':
                self.laser_on = True
                GPIO.output(self.LASER_PIN, GPIO.HIGH)
                print("✓ Laser turned ON")
            elif cmd == '2':
                self.laser_on = False
                GPIO.output(self.LASER_PIN, GPIO.LOW)
                print("✓ Laser turned OFF")
            elif cmd == '3':
                print("Firing laser for 3 seconds...")
                GPIO.output(self.LASER_PIN, GPIO.HIGH)
                self.laser_on = True
                
                # Countdown
                for i in range(3, 0, -1):
                    print(f"  {i}...")
                    time.sleep(1.0)
                
                GPIO.output(self.LASER_PIN, GPIO.LOW)
                self.laser_on = False
                print("✓ Laser fired for 3 seconds")
            elif cmd == 't':
                print("Testing laser briefly...")
                GPIO.output(self.LASER_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(self.LASER_PIN, GPIO.LOW)
                print("✓ Laser test complete")
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def manual_motor_control(self):
        """Manual motor control interface"""
        print("\n" + "="*60)
        print("MANUAL MOTOR CONTROL")
        print("="*60)
        
        while True:
            print(f"\nCurrent position:")
            print(f"  Azimuth: {self.azimuth_angle:.1f}°")
            print(f"  Altitude: {self.altitude_angle:.1f}°")
            print(f"  Altitude limits: +{self.altitude_max_up:.1f}° to {self.altitude_max_down:.1f}°")
            print(f"  Origin offsets: Az={self.azimuth_offset}, Alt={self.altitude_offset}")
            
            print("\nCommands:")
            print("1 - Set specific angles")
            print("2 - Small movements (Azimuth ±10°, Altitude ±5°)")
            print("3 - Zero azimuth")
            print("4 - Move both motors simultaneously")
            print("q - Return to main menu")
            
            cmd = input("\nEnter command: ").lower()
            
            if cmd == '1':
                # Set specific angles
                try:
                    az_angle = float(input("Enter azimuth angle (degrees): "))
                    alt_angle = float(input("Enter altitude angle (degrees): "))
                    
                    # Move azimuth
                    az_move = az_angle - self.azimuth_angle
                    if abs(az_move) > 0.1:
                        print(f"Moving azimuth by {az_move:+.1f}°...")
                        self.move_azimuth_safe(az_move)
                    
                    # Move altitude
                    alt_move = alt_angle - self.altitude_angle
                    if abs(alt_move) > 0.1:
                        print(f"Moving altitude by {alt_move:+.1f}°...")
                        actual = self.move_altitude_safe(alt_move)
                        print(f"Altitude actually moved: {actual:+.1f}°")
                    
                    print(f"✓ Position set to: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
                    
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                    
            elif cmd == '2':
                # Small movements
                print("\nSmall movements:")
                print("a/d - Azimuth left/right 10°")
                print("w/s - Altitude up/down 5°")
                
                subcmd = input("Enter movement command: ").lower()
                
                if subcmd == 'a':
                    self.move_azimuth_safe(-10)
                elif subcmd == 'd':
                    self.move_azimuth_safe(10)
                elif subcmd == 'w':
                    self.move_altitude_safe(5)
                elif subcmd == 's':
                    self.move_altitude_safe(-5)
                else:
                    print("Invalid command")
                    
            elif cmd == '3':
                # Zero azimuth
                self.move_azimuth_safe(-self.azimuth_angle)
                print("✓ Azimuth zeroed")
                
            elif cmd == '4':
                # Simultaneous movement
                try:
                    az_move = float(input("Enter azimuth movement (degrees): "))
                    alt_move = float(input("Enter altitude movement (degrees): "))
                    
                    print("\nMoving both motors simultaneously...")
                    az_actual, alt_actual = self.move_both_simultaneously(az_move, alt_move)
                    
                    print(f"✓ Simultaneous movement complete")
                    print(f"  Final: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
                    
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def calculate_target_angles(self, target_r, target_theta, target_z, team_r, team_theta):
        """
        Calculate required azimuth and altitude angles to hit a target
        Accounting for laser offset (43mm height, 27mm right from azimuth shaft)
        """
        # Convert polar coordinates to Cartesian (centered at arena center)
        target_x = target_r * math.cos(target_theta)
        target_y = target_r * math.sin(target_theta)
        
        team_x = team_r * math.cos(team_theta)
        team_y = team_r * math.sin(team_theta)
        
        # Calculate relative position (target relative to team)
        dx = target_x - team_x
        dy = target_y - team_y
        
        # Account for laser offset (27mm to the right from azimuth shaft)
        # When turret rotates, the offset rotates with it
        # The offset creates a lever arm effect
        offset_angle = team_theta + math.pi/2  # Offset is 90° from facing direction (to the right)
        dx_offset = self.laser_offset_x * math.cos(offset_angle)
        dy_offset = self.laser_offset_x * math.sin(offset_angle)
        
        # Adjust relative position for laser offset
        dx_adjusted = dx - dx_offset
        dy_adjusted = dy - dy_offset
        
        # Calculate azimuth angle (relative to turret's forward direction)
        # Turret initially faces center (negative radial direction from team position)
        # Forward direction is from team position toward center
        center_x = 0
        center_y = 0
        forward_x = center_x - team_x
        forward_y = center_y - team_y
        
        # Calculate angle between forward direction and target direction
        forward_angle = math.atan2(forward_y, forward_x)
        target_angle = math.atan2(dy_adjusted, dx_adjusted)
        
        azimuth_angle = target_angle - forward_angle
        
        # Normalize azimuth angle to [-π, π]
        if azimuth_angle > math.pi:
            azimuth_angle -= 2 * math.pi
        elif azimuth_angle < -math.pi:
            azimuth_angle += 2 * math.pi
        
        # Calculate distance to target (horizontal)
        horizontal_distance = math.sqrt(dx_adjusted*dx_adjusted + dy_adjusted*dy_adjusted)
        
        # Calculate altitude angle
        height_diff = target_z - self.laser_height  # Laser is 43mm above ground
        altitude_angle = math.atan2(height_diff, horizontal_distance)
        
        # Convert to degrees
        azimuth_deg = math.degrees(azimuth_angle)
        altitude_deg = math.degrees(altitude_angle)
        
        return azimuth_deg, altitude_deg, horizontal_distance
    
    def aim_and_fire(self, target_name, azimuth_deg, altitude_deg):
        """Aim at target and fire laser for 3 seconds"""
        print(f"\nTargeting {target_name}...")
        print(f"Required angles: Az={azimuth_deg:.1f}°, Alt={altitude_deg:.1f}°")
        
        # Move to target position
        current_az = self.azimuth_angle
        current_alt = self.altitude_angle
        
        az_move = azimuth_deg - current_az
        alt_move = altitude_deg - current_alt
        
        print(f"Moving azimuth by {az_move:+.1f}°...")
        self.move_azimuth_safe(az_move)
        
        print(f"Moving altitude by {alt_move:+.1f}°...")
        actual_alt = self.move_altitude_safe(alt_move)
        
        print(f"Actual altitude movement: {actual_alt:+.1f}°")
        print(f"Final position: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
        
        # Fire laser for 3 seconds
        print("Firing laser for 3 seconds...")
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_on = True
        
        # Countdown
        for i in range(3, 0, -1):
            print(f"  {i}...")
            time.sleep(1.0)
        
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_on = False
        print("✓ Laser fired for 3 seconds")
    
    def competition_mode(self):
        """Competition mode - read JSON from network and fire at targets"""
        print("\n" + "="*60)
        print("COMPETITION MODE")
        print("="*60)
        
        # JSON URL from competition rules
        json_url = "http://192.168.1.254:8000/positions.json"
        
        try:
            print(f"Fetching JSON data from {json_url}...")
            with urllib.request.urlopen(json_url, timeout=5) as response:
                data = json.loads(response.read().decode())
                print("✓ JSON data loaded successfully")
                
        except Exception as e:
            print(f"Error loading JSON data: {e}")
            print("Using sample data instead...")
            # Fallback to sample data
            data = {
                "turrets": {
                    "1": {"r": 300.0, "theta": 2.580},
                    "2": {"r": 300.0, "theta": 0.661},
                    "3": {"r": 300.0, "theta": 5.152}
                },
                "globes": [
                    {"r": 300.0, "theta": 1.015, "z": 20.4},
                    {"r": 300.0, "theta": 4.512, "z": 32.0},
                    {"r": 300.0, "theta": 3.979, "z": 10.8}
                ]
            }
        
        # Display available teams
        print("\nAvailable teams:")
        for team_id in data.get("turrets", {}).keys():
            print(f"  Team {team_id}")
        
        # Get team number
        while True:
            team_num = input("\nEnter YOUR team number: ").strip()
            if team_num in data.get("turrets", {}):
                team_data = data["turrets"][team_num]
                print(f"✓ Team {team_num} found at position: r={team_data['r']} cm, θ={team_data['theta']:.3f} rad")
                break
            else:
                print(f"Team {team_num} not found in data. Please try again.")
        
        # Get team position
        team_r = team_data['r']
        team_theta = team_data['theta']
        
        # Identify targets (all other turrets and globes)
        targets = []
        
        # Add other turrets as targets
        for target_id, target_data in data.get("turrets", {}).items():
            if target_id != team_num:
                targets.append({
                    "name": f"Turret {target_id}",
                    "r": target_data['r'],
                    "theta": target_data['theta'],
                    "z": self.laser_height  # Ground level target (same height as our laser)
                })
        
        # Add globes as targets
        for i, globe in enumerate(data.get("globes", []), 1):
            targets.append({
                "name": f"Globe {i}",
                "r": globe['r'],
                "theta": globe['theta'],
                "z": globe['z']
            })
        
        print(f"\nFound {len(targets)} targets:")
        for target in targets:
            print(f"  {target['name']}: r={target['r']} cm, θ={target['theta']:.3f} rad, z={target['z']} cm")
        
        # Confirm before starting
        print("\n" + "="*40)
        print("WARNING: Laser will fire automatically!")
        print("Ensure area is clear of eyes and reflective surfaces.")
        print("="*40)
        
        confirm = input("\nStart automated targeting? (yes/no): ").lower()
        if confirm != 'yes':
            print("Aborting competition mode.")
            return
        
        # Begin targeting sequence
        print("\n" + "="*40)
        print("BEGINNING TARGETING SEQUENCE")
        print("="*40)
        
        successful_targets = 0
        failed_targets = 0
        
        for target in targets:
            # Calculate angles
            az_deg, alt_deg, distance = self.calculate_target_angles(
                target['r'], target['theta'], target['z'],
                team_r, team_theta
            )
            
            print(f"\n--- Targeting {target['name']} ---")
            print(f"Distance: {distance:.1f} cm")
            print(f"Required angles: Az={az_deg:.1f}°, Alt={alt_deg:.1f}°")
            
            # Check if altitude is within limits
            if alt_deg > self.altitude_max_up:
                print(f"⚠️ Cannot target: Altitude {alt_deg:.1f}° exceeds maximum {self.altitude_max_up:.1f}°")
                failed_targets += 1
                continue
            elif alt_deg < self.altitude_max_down:
                print(f"⚠️ Cannot target: Altitude {alt_deg:.1f}° exceeds minimum {self.altitude_max_down:.1f}°")
                failed_targets += 1
                continue
            
            # Aim and fire
            try:
                self.aim_and_fire(target['name'], az_deg, alt_deg)
                successful_targets += 1
            except Exception as e:
                print(f"Error targeting {target['name']}: {e}")
                failed_targets += 1
            
            # Small pause between targets
            time.sleep(1.0)
        
        print("\n" + "="*40)
        print("TARGETING SEQUENCE COMPLETE")
        print("="*40)
        print(f"Successful targets: {successful_targets}")
        print(f"Failed targets: {failed_targets}")
        print(f"Total targets: {len(targets)}")
        
        # Return to zero position
        print("\nReturning to home position...")
        self.move_azimuth_safe(-self.azimuth_angle)
        self.move_altitude_safe(-self.altitude_angle)
    
    def mock_competition_mode(self):
        """Mock competition mode - manually input target coordinates"""
        print("\n" + "="*60)
        print("MOCK COMPETITION MODE")
        print("="*60)
        
        print("Enter target coordinates:")
        
        try:
            target_r = float(input("Target radius (cm): "))
            target_theta = float(input("Target angle (radians): "))
            target_z = float(input("Target height (cm): "))
            
            # Get team position (assuming at standard position)
            print("\nAssuming turret is at standard position:")
            print(f"  Radius: {self.turret_radius} cm")
            print(f"  Angle: 0 rad (facing center)")
            print(f"  Laser height: {self.laser_height} cm")
            
            team_r = self.turret_radius
            team_theta = 0.0  # Facing center
            
            # Calculate angles
            az_deg, alt_deg, distance = self.calculate_target_angles(
                target_r, target_theta, target_z,
                team_r, team_theta
            )
            
            print(f"\nCalculated angles:")
            print(f"  Azimuth: {az_deg:.1f}°")
            print(f"  Altitude: {alt_deg:.1f}°")
            print(f"  Distance: {distance:.1f} cm")
            
            # Check altitude limits
            if alt_deg > self.altitude_max_up:
                print(f"\n⚠️ Cannot target:")
                print(f"  Required altitude {alt_deg:.1f}° exceeds maximum {self.altitude_max_up:.1f}°")
                return
            elif alt_deg < self.altitude_max_down:
                print(f"\n⚠️ Cannot target:")
                print(f"  Required altitude {alt_deg:.1f}° exceeds minimum {self.altitude_max_down:.1f}°")
                return
            
            # Confirm before firing
            confirm = input("\nAim and fire at target? (yes/no): ").lower()
            if confirm == 'yes':
                self.aim_and_fire("Mock Target", az_deg, alt_deg)
            else:
                print("Aborted.")
                
        except ValueError:
            print("Invalid input. Please enter numbers only.")
    
    def simple_interface(self):
        """Simple control interface"""
        print("\n" + "="*60)
        print("SIMPLE CONTROL INTERFACE")
        print("="*60)
        
        while True:
            print(f"\nPosition: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
            print(f"Altitude limits: +{self.altitude_max_up:.1f}° to {self.altitude_max_down:.1f}°")
            
            print("\nCommands:")
            print("a/d - Azimuth left/right 10°")
            print("w/s - Altitude up/down 5° (within limits)")
            print("1/2 - Azimuth 45°/90°")
            print("z   - Zero azimuth")
            print("q   - Quit")
            
            cmd = input("Command: ").lower()
            
            if cmd == 'a':
                self.move_azimuth_safe(-10)
            elif cmd == 'd':
                self.move_azimuth_safe(10)
            elif cmd == 'w':
                self.move_altitude_safe(5)
            elif cmd == 's':
                self.move_altitude_safe(-5)
            elif cmd == '1':
                self.move_azimuth_safe(45)
            elif cmd == '2':
                self.move_azimuth_safe(90)
            elif cmd == 'z':
                self.move_azimuth_safe(-self.azimuth_angle)
                print(f"Azimuth zeroed")
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def cleanup(self):
        """Clean shutdown"""
        print("\nCleaning up...")
        self.send_to_shift_register(0b00000000)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 - FINAL WORKING VERSION")
    print("="*70)
    print("Key features:")
    print("• Laser control via GPIO26")
    print("• Calibration to set origin")
    print("• Manual laser control (on/off/3-second fire)")
    print("• Manual motor control with simultaneous movement")
    print("• Competition mode with JSON data")
    print("• Mock competition mode for testing")
    print("="*70)
    
    turret = None
    try:
        turret = WorkingTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU")
            print("="*60)
            print("1. Manual Laser Control")
            print("2. Manual Motor Control")
            print("3. Calibration - Set Origin")
            print("4. Competition Mode")
            print("5. Mock Competition Mode")
            print("6. Simple Control Interface")
            print("7. Exit and cleanup")
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == "1":
                turret.manual_laser_control()
            elif choice == "2":
                turret.manual_motor_control()
            elif choice == "3":
                turret.set_calibration_origin()
            elif choice == "4":
                turret.competition_mode()
            elif choice == "5":
                turret.mock_competition_mode()
            elif choice == "6":
                turret.simple_interface()
            elif choice == "7":
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
