# ENME441 - MIKE WANG

import RPi.GPIO as GPIO
import time
import json
import os
import math
import urllib.request

class WorkingTurret:
    def __init__(self):
        
        # Pinout
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        self.LASER_PIN = 26  # Laser
        
        self.CONFIG_FILE = "working_config.json"
        
        # I dont know why the motors are wonky, it took forever to test for these values, I hate it
        # Also they're still not the most accurate
        self.AZIMUTH_STEPS_PER_REV = 2200
        self.ALTITUDE_STEPS_PER_REV = 450
        
        self.load_config()
        
        # print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        # print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0  # degrees
        self.altitude_angle = 0.0  # degrees
        self.laser_on = False
        
        # offset tracking
        self.azimuth_offset = 0
        self.altitude_offset = 0
        
        # Azimuth Squences
        self.AZIMUTH_SEQUENCE = [
            0b00000011,  # A+B
            0b00000110,  # B+C
            0b00001100,  # C+D
            0b00001001,  # D+A
        ]
        
        # Altitude Squences
        self.ALTITUDE_SEQUENCE = [
            0b00010000 | 0b00100000,  # A+B
            0b00100000 | 0b01000000,  # B+C
            0b01000000 | 0b10000000,  # C+D
            0b10000000 | 0b00010000,  # D+A
        ]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Speeds
        self.AZIMUTH_DELAY = 0.015
        self.ALTITUDE_DELAY = 0.040  # Slower for torque (something is wrong with this motor)
        
        # Maximum angles for altitude motor (again, something is wrong with it)
        self.altitude_max_up = 45.0
        self.altitude_max_down = -45.0
        
        # Turret locations
        self.turret_radius = 3.0      # meters - distance from center
        self.laser_height = 0.043     # meters - height of laser above ground
        self.laser_offset_x = 0.027   # meters - horizontal offset from azimuth shaft to laser
        
        self.setup_gpio()
        
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print(f"Laser: OFF")
        print(f"Laser offset: {self.laser_height*100:.1f} cm height, {self.laser_offset_x*100:.1f} cm right")
        print("="*70)
    
    def load_config(self):
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 2200)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 450)
                print("Configuration loaded")
        except:
            print("Using final values")
    
    def save_config(self):
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
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        
        # All pins to LOW
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        # GPIO.output(self.LASER_PIN, GPIO.HIGH)
        # time.sleep(0.1)
        # GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        # Motors off
        self.send_to_shift_register(0b00000000)
        time.sleep(0.1)
    
    def send_to_shift_register(self, data):
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
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth(self, direction):
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude(self, direction):
        # Something is wrong with the altitude motor, and this is my attempt to make it work somehow.
        # I had to go online and figure this out. I don't think it's perfect, but the result is good enough.
        max_retries = 2
        
        for attempt in range(max_retries):
            try:
                # Store current position
                old_pos = self.altitude_seq_pos
                old_steps = self.altitude_steps
                
                # Take step
                self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
                self.altitude_steps += direction
                self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
                
                self.update_motors()
                time.sleep(0.005)
                
                return True
                
            except Exception as e:
                self.altitude_seq_pos = old_pos
                self.altitude_steps = old_steps
                self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
                
                if attempt < max_retries - 1:
                    # Retry with longer delay
                    time.sleep(self.ALTITUDE_DELAY * 2)
                    continue
        
        return False
    
    def move_azimuth_safe(self, degrees):
        steps = int((degrees / 360) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(self.AZIMUTH_DELAY)
    
    def move_altitude_safe(self, degrees):
        # Check limits
        target_angle = self.altitude_angle + degrees
        
        if target_angle > self.altitude_max_up:
            print(f"Can't move to {target_angle:.1f}° (max UP: {self.altitude_max_up:.1f}°)")
            degrees = self.altitude_max_up - self.altitude_angle
        
        if target_angle < self.altitude_max_down:
            print(f"Can't move to {target_angle:.1f}° (max DOWN: {self.altitude_max_down:.1f}°)")
            degrees = self.altitude_max_down - self.altitude_angle
        
        steps = int((degrees / 360) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        successful_steps = 0
        
        for i in range(steps):
            if self.step_altitude(direction):
                successful_steps += 1
                time.sleep(self.ALTITUDE_DELAY)
            else:
                print(f"  Stopped at step {i+1}/{steps}")
                break
        
        actual_degrees = (successful_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0 * direction
        return actual_degrees
    
    def set_calibration_origin(self):
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
            
            # Reset step counters
            self.azimuth_steps = 0
            self.altitude_steps = 0
            self.azimuth_angle = 0.0
            self.altitude_angle = 0.0
            
            print(" Origin set at current position")
            print(f"  Azimuth offset: {self.azimuth_offset} steps")
            print(f"  Altitude offset: {self.altitude_offset} steps")
        else:
            print("Calibration cancelled")
    
    def convert_to_absolute_steps(self, rel_steps, axis='azimuth'):
        if axis == 'azimuth':
            return rel_steps + self.azimuth_offset
        else:  # altitude
            return rel_steps + self.altitude_offset
    
    def manual_laser_control(self):
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
                print(" Laser turned ON")
            elif cmd == '2':
                self.laser_on = False
                GPIO.output(self.LASER_PIN, GPIO.LOW)
                print(" Laser turned OFF")
            elif cmd == '3':
                GPIO.output(self.LASER_PIN, GPIO.HIGH)
                self.laser_on = True
                
                GPIO.output(self.LASER_PIN, GPIO.LOW)
                self.laser_on = False
                print(" Laser fired for 3 seconds")
            elif cmd == 't':
                print("Testing laser briefly...")
                GPIO.output(self.LASER_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(self.LASER_PIN, GPIO.LOW)
                print(" Laser test complete")
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def manual_motor_control(self):
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
                    
                    print(f" Position set to: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
                    
                except ValueError:
                    print("Invalid input.")
                    
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
                print(" Azimuth zeroed")
                
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def calculate_target_angles(self, target_r, target_theta_deg, target_z, team_r, team_theta_deg):
        target_theta = math.radians(target_theta_deg)
        team_theta = math.radians(team_theta_deg)

        target_x = target_r * math.cos(target_theta)
        target_y = target_r * math.sin(target_theta)
        
        team_x = team_r * math.cos(team_theta)
        team_y = team_r * math.sin(team_theta)

        dx = target_x - team_x
        dy = target_y - team_y

        # Account for offsets
        offset_angle = team_theta + math.pi/2
        dx_offset = self.laser_offset_x * math.cos(offset_angle)
        dy_offset = self.laser_offset_x * math.sin(offset_angle)

        dx_adjusted = dx - dx_offset
        dy_adjusted = dy - dy_offset
        
        center_x = 0
        center_y = 0
        forward_x = center_x - team_x
        forward_y = center_y - team_y

        forward_angle = math.atan2(forward_y, forward_x)
        target_angle = math.atan2(dy_adjusted, dx_adjusted)
        
        azimuth_rad = target_angle - forward_angle

        if azimuth_rad > math.pi:
            azimuth_rad -= 2 * math.pi
        elif azimuth_rad < -math.pi:
            azimuth_rad += 2 * math.pi

        # Calculate from turret's POV
        horizontal_distance = math.sqrt(dx_adjusted*dx_adjusted + dy_adjusted*dy_adjusted)

        height_diff = target_z - self.laser_height  # 43mm above ground
        altitude_rad = math.atan2(height_diff, horizontal_distance)

        azimuth_deg = math.degrees(azimuth_rad)
        altitude_deg = math.degrees(altitude_rad)
        
        return azimuth_deg, altitude_deg, horizontal_distance
    
    def aim_and_fire(self, target_name, azimuth_deg, altitude_deg):
        print(f"\nTargeting {target_name}...")
        print(f"Required angles: Az={azimuth_deg:.1f}°, Alt={altitude_deg:.1f}°")

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
        
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_on = True
        
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_on = False
        print(" Laser fired for 3 seconds")
    
    def competition_mode(self):
        print("\n" + "="*60)
        print("COMPETITION MODE")
        print("="*60)
        
        # JSON URL
        json_url = "http://192.168.1.254:8000/positions.json"
        
        try:
            print(f"Fetching JSON data from {json_url}...")
            with urllib.request.urlopen(json_url, timeout=5) as response:
                data = json.loads(response.read().decode())
                print(" JSON data loaded successfully")
                
                # Convert JSON data from cm/rad to m/deg
                data = self.convert_json_units(data)
                
        except Exception as e:
            print(f"Error loading JSON data: {e}")
            print("Using backup data instead...")
            # Use the provided data before
            data = {
                "turrets": {
                    "1": {"r": 3.0, "theta": 147.8},    # 2.580 rad = 147.8°
                    "2": {"r": 3.0, "theta": 37.9},     # 0.661 rad = 37.9°
                    "3": {"r": 3.0, "theta": 295.2},    # 5.152 rad = 295.2°
                },
                "globes": [
                    {"r": 3.0, "theta": 58.2, "z": 0.204},   # 1.015 rad = 58.2°, 20.4 cm = 0.204 m
                    {"r": 3.0, "theta": 258.5, "z": 0.320},  # 4.512 rad = 258.5°, 32.0 cm = 0.320 m
                    {"r": 3.0, "theta": 228.0, "z": 0.108},  # 3.979 rad = 228.0°, 10.8 cm = 0.108 m
                ]
            }
        
        print("\nAvailable teams:")
        for team_id in data.get("turrets", {}).keys():
            print(f"  Team {team_id}")
        
        while True:
            team_num = input("\nEnter YOUR team number: ").strip()
            if team_num in data.get("turrets", {}):
                team_data = data["turrets"][team_num]
                print(f" Team {team_num} found at position: r={team_data['r']} m, θ={team_data['theta']:.1f}°")
                break
            else:
                print(f"Team {team_num} not found in data. Please try again.")
        
        # Team position
        team_r = team_data['r']
        team_theta = team_data['theta']
        
        # Identify targets
        targets = []
        
        for target_id, target_data in data.get("turrets", {}).items():
            if target_id != team_num:
                targets.append({
                    "name": f"Turret {target_id}",
                    "r": target_data['r'],
                    "theta": target_data['theta'],  # degrees
                    "z": self.laser_height  # Assume turrets are same height
                })
        
        for i, globe in enumerate(data.get("globes", []), 1):
            targets.append({
                "name": f"Globe {i}",
                "r": globe['r'],
                "theta": globe['theta'],  # degrees
                "z": globe['z']  # meters
            })
        
        print(f"\nFound {len(targets)} targets:")
        for target in targets:
            print(f"  {target['name']}: r={target['r']} m, θ={target['theta']:.1f}°, z={target['z']*100:.1f} cm")
        
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
            print(f"Distance: {distance:.2f} m")
            print(f"Required angles: Az={az_deg:.1f}°, Alt={alt_deg:.1f}°")
            
            # Check if altitude is within limits
            if alt_deg > self.altitude_max_up:
                print(f"Cannot target: Altitude {alt_deg:.1f}° exceeds maximum {self.altitude_max_up:.1f}°")
                failed_targets += 1
                continue
            elif alt_deg < self.altitude_max_down:
                print(f"Cannot target: Altitude {alt_deg:.1f}° exceeds minimum {self.altitude_max_down:.1f}°")
                failed_targets += 1
                continue

            try:
                self.aim_and_fire(target['name'], az_deg, alt_deg)
                successful_targets += 1
            except Exception as e:
                print(f"Error targeting {target['name']}: {e}")
                failed_targets += 1

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
    
    def convert_json_units(self, data):
        converted_data = {"turrets": {}, "globes": []}
        
        if "turrets" in data:
            for team_id, turret_data in data["turrets"].items():
                converted_data["turrets"][team_id] = {
                    "r": turret_data["r"] / 100.0,  # cm to m
                    "theta": math.degrees(turret_data["theta"])  # rad to deg
                }

        if "globes" in data:
            for globe in data["globes"]:
                converted_data["globes"].append({
                    "r": globe["r"] / 100.0,  # cm to m
                    "theta": math.degrees(globe["theta"]),  # rad to deg
                    "z": globe["z"] / 100.0  # cm to m
                })
        
        return converted_data
    
    def mock_competition_mode(self):
        print("\n" + "="*60)
        print("MOCK COMPETITION MODE")
        print("="*60)
        
        print("Enter target coordinates:")
        
        try:
            target_r = float(input("Target radius (meters): "))
            target_theta = float(input("Target angle (degrees): "))
            target_z = float(input("Target height (meters): "))
            
            # Team position
            print("\nAssuming turret is at standard position:")
            print(f"  Radius: {self.turret_radius} m")
            print(f"  Angle: 0° (facing center)")
            print(f"  Laser height: {self.laser_height*100:.1f} cm")
            
            team_r = self.turret_radius
            team_theta = 0.0  # Facing center, degrees
            
            # Calculate angles
            az_deg, alt_deg, distance = self.calculate_target_angles(
                target_r, target_theta, target_z,
                team_r, team_theta
            )
            
            print(f"\nCalculated angles:")
            print(f"  Azimuth: {az_deg:.1f}°")
            print(f"  Altitude: {alt_deg:.1f}°")
            print(f"  Distance: {distance:.2f} m")
            
            # Check altitude limits
            if alt_deg > self.altitude_max_up:
                print(f"\nCannot target:")
                print(f"  Required altitude {alt_deg:.1f}° exceeds maximum {self.altitude_max_up:.1f}°")
                return
            elif alt_deg < self.altitude_max_down:
                print(f"\nCannot target:")
                print(f"  Required altitude {alt_deg:.1f}° exceeds minimum {self.altitude_max_down:.1f}°")
                return
            
            # Confirm before firing
            confirm = input("\nAim and fire at target? (yes/no): ").lower()
            if confirm == 'yes':
                self.aim_and_fire("Mock Target", az_deg, alt_deg)
            else:
                print("Aborted.")
                
        except ValueError:
            print("Invalid input.")
    
    def simple_interface(self):
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

def main():
    print("="*70)
    print("ENME441 COURSE PROJECT - MIKE WANG")
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
