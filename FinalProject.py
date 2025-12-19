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
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        self.LASER_PIN = 17  # Added laser control pin
        
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
        
        # Turret physical properties
        self.turret_radius = 300.0  # cm - distance from center
        self.laser_height = 10.0    # cm - height of laser above ground
        self.laser_offset = 5.0     # cm - distance from turret center to laser
        
        # Initialize
        self.setup_gpio()
        
        print(f"\n✓ FINAL system ready")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print("Laser: OFF")
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
        
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
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
                time.sleep(3.0)
                GPIO.output(self.LASER_PIN, GPIO.LOW)
                self.laser_on = False
                print("✓ Laser fired for 3 seconds")
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
                
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def calculate_target_angles(self, target_r, target_theta, target_z, team_r, team_theta):
        """
        Calculate required azimuth and altitude angles to hit a target
        Considering turret is facing center of arena initially
        """
        # Convert polar coordinates to Cartesian
        target_x = target_r * math.cos(target_theta)
        target_y = target_r * math.sin(target_theta)
        
        team_x = team_r * math.cos(team_theta)
        team_y = team_r * math.sin(team_theta)
        
        # Calculate relative position
        dx = target_x - team_x
        dy = target_y - team_y
        
        # Calculate azimuth angle (relative to turret's forward direction)
        # Turret initially faces center (negative x direction)
        azimuth_angle = math.atan2(dy, dx) - (team_theta + math.pi)
        
        # Normalize azimuth angle to [-π, π]
        if azimuth_angle > math.pi:
            azimuth_angle -= 2 * math.pi
        elif azimuth_angle < -math.pi:
            azimuth_angle += 2 * math.pi
        
        # Calculate distance to target
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Adjust for laser offset
        if distance > self.laser_offset:
            distance -= self.laser_offset
        
        # Calculate altitude angle
        height_diff = target_z - self.laser_height
        altitude_angle = math.atan2(height_diff, distance)
        
        # Convert to degrees
        azimuth_deg = math.degrees(azimuth_angle)
        altitude_deg = math.degrees(altitude_angle)
        
        return azimuth_deg, altitude_deg
    
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
        time.sleep(3.0)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_on = False
        print("✓ Laser fired")
    
    def competition_mode(self):
        """Competition mode - read JSON from network and fire at targets"""
        print("\n" + "="*60)
        print("COMPETITION MODE")
        print("="*60)
        
        # JSON URL from competition rules
        json_url = "http://192.168.1.254:8000/positions.json"
        
        try:
            print(f"Fetching JSON data from {json_url}...")
            with urllib.request.urlopen(json_url) as response:
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
                    "z": self.laser_height  # Ground level target
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
        
        for target in targets:
            # Calculate angles
            az_deg, alt_deg = self.calculate_target_angles(
                target['r'], target['theta'], target['z'],
                team_r, team_theta
            )
            
            # Check if altitude is within limits
            if alt_deg > self.altitude_max_up:
                print(f"\n⚠️ Cannot target {target['name']}:")
                print(f"  Required altitude {alt_deg:.1f}° exceeds maximum {self.altitude_max_up:.1f}°")
                continue
            elif alt_deg < self.altitude_max_down:
                print(f"\n⚠️ Cannot target {target['name']}:")
                print(f"  Required altitude {alt_deg:.1f}° exceeds minimum {self.altitude_max_down:.1f}°")
                continue
            
            # Aim and fire
            self.aim_and_fire(target['name'], az_deg, alt_deg)
            
            # Small pause between targets
            time.sleep(1.0)
        
        print("\n" + "="*40)
        print("TARGETING SEQUENCE COMPLETE")
        print("="*40)
        print(f"Engaged {len(targets)} targets")
        print("Returning to home position...")
        
        # Return to zero position
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
            az_deg, alt_deg = self.calculate_target_angles(
                target_r, target_theta, target_z,
                team_r, team_theta
            )
            
            print(f"\nCalculated angles:")
            print(f"  Azimuth: {az_deg:.1f}°")
            print(f"  Altitude: {alt_deg:.1f}°")
            
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
    print("• Manual laser control (on/off/3-second fire)")
    print("• Manual motor control with angle setting")
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
            print("3. Competition Mode")
            print("4. Mock Competition Mode")
            print("5. Simple Control Interface")
            print("6. Exit and cleanup")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                turret.manual_laser_control()
            elif choice == "2":
                turret.manual_motor_control()
            elif choice == "3":
                turret.competition_mode()
            elif choice == "4":
                turret.mock_competition_mode()
            elif choice == "5":
                turret.simple_interface()
            elif choice == "6":
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
