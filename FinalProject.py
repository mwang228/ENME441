#!/usr/bin/env python3
"""
ENME441 Laser Turret - COMPLETE SYSTEM
Combines: Motor control + JSON fetching + Targeting
"""

import RPi.GPIO as GPIO
import time
import json
import requests
import math
import sys

class CompleteTurret:
    def __init__(self, team_number="1"):
        self.team_number = team_number
        
        # Motor calibration
        self.AZIMUTH_STEPS_PER_REV = 1024    # Your fast motor
        self.ALTITUDE_STEPS_PER_REV = 4096   # Standard motor
        self.ALTITUDE_SPEED_FACTOR = 4       # Moves 4× faster to match azimuth
        
        # Server IPs to try (add your laptop's IP here!)
        self.SERVER_IPS = [
            "192.168.1.254",      # Competition server
            "192.168.137.1",      # Windows hotspot default
            "192.168.43.1",       # Common phone hotspot gateway
            "192.168.1.100",      # Common laptop IP
            "10.42.0.1",          # Ubuntu hotspot
            "172.20.10.1",        # iPhone hotspot
        ]
        
        # ADD YOUR LAPTOP'S ACTUAL IP HERE:
        # Run 'ipconfig' on Windows or 'ifconfig' on Mac/Linux
        # Look for IP when connected to same network as Pi
        # Example: self.SERVER_IPS.append("192.168.43.50")
        
        # Position tracking
        self.azimuth_position = 0    # Steps from home
        self.altitude_position = 0   # Steps from home
        self.azimuth_phase = 0
        self.altitude_phase = 0
        
        # Competition data
        self.competition_data = None
        self.my_position = None
        
        # GPIO pins for shift register
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS    (Pin 14)
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
    
    # ========== SHIFT REGISTER CONTROL ==========
    
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
        """Take one azimuth step"""
        if direction == 1:  # Clockwise
            self.azimuth_phase = (self.azimuth_phase + 1) % 8
            self.azimuth_position += 1
        else:  # Counterclockwise
            self.azimuth_phase = (self.azimuth_phase - 1) % 8
            self.azimuth_position -= 1
        
        self.update_motors()
    
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
        
        self.update_motors()
    
    def move_motors_sync(self, az_steps, alt_steps, delay=0.001):
        """
        Move both motors with synchronized speed
        Altitude moves 4× faster to match azimuth
        """
        if az_steps == 0 and alt_steps == 0:
            return
        
        az_dir = 1 if az_steps >= 0 else -1
        alt_dir = 1 if alt_steps >= 0 else -1
        
        az_steps_abs = abs(az_steps)
        alt_steps_abs = abs(alt_steps)
        
        print(f"Moving: Azimuth={az_steps} steps, Altitude={alt_steps} steps")
        print(f"Altitude moving {self.ALTITUDE_SPEED_FACTOR}× faster")
        
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
        
        while az_counter < az_steps_abs or alt_counter < alt_steps_abs:
            current_time = time.time()
            
            # Move azimuth if ready
            if az_counter < az_steps_abs and (current_time - last_az_time) >= az_delay:
                self.step_azimuth(az_dir)
                last_az_time = current_time
                az_counter += 1
            
            # Move altitude if ready (faster timing)
            if alt_counter < alt_steps_abs and (current_time - last_alt_time) >= alt_delay:
                self.step_altitude_fast(alt_dir)
                last_alt_time = current_time
                alt_counter += 1
            
            # Small sleep to prevent CPU hogging
            time.sleep(min(az_delay, alt_delay) / 10)
        
        print(f"Completed: Az={az_counter}/{az_steps_abs}, Alt={alt_counter}/{alt_steps_abs}")
    
    def move_motors_degrees_sync(self, az_degrees, alt_degrees, delay=0.001):
        """Move by degrees with synchronized timing"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        self.move_motors_sync(az_steps, alt_steps, delay)
    
    # ========== JSON FETCHING ==========
    
    def fetch_competition_data(self):
        """
        Fetch JSON from server - tries multiple IPs
        Returns True if successful, False otherwise
        """
        print("\n" + "="*60)
        print("FETCHING COMPETITION COORDINATES")
        print("="*60)
        
        for ip in self.SERVER_IPS:
            url = f"http://{ip}:8000/positions.json"
            print(f"\nTrying: {url}")
            
            try:
                response = requests.get(url, timeout=3)
                if response.status_code == 200:
                    data = response.json()
                    if "turrets" in data and "globes" in data:
                        self.competition_data = data
                        
                        # Find our position
                        if self.team_number in data["turrets"]:
                            self.my_position = data["turrets"][self.team_number]
                            print(f"✓ SUCCESS! Connected to {ip}")
                            print(f"  Team {self.team_number} position:")
                            print(f"    r = {self.my_position['r']} cm")
                            print(f"    θ = {self.my_position['theta']} rad")
                            print(f"    ({math.degrees(self.my_position['theta']):.1f}°)")
                            print(f"  Targets: {len(data['turrets'])-1} turrets + {len(data['globes'])} globes")
                            return True
                        else:
                            print(f"✗ Team {self.team_number} not found in data")
                    else:
                        print(f"✗ Invalid data structure from {ip}")
                else:
                    print(f"✗ HTTP {response.status_code} from {ip}")
            except requests.exceptions.ConnectionError:
                print(f"✗ Cannot connect to {ip}")
            except requests.exceptions.Timeout:
                print(f"✗ Timeout from {ip}")
            except Exception as e:
                print(f"✗ Error: {e}")
        
        print("\n" + "="*60)
        print("ALL CONNECTION ATTEMPTS FAILED")
        print("="*60)
        print("Trying manual IP input...")
        
        # Manual IP input
        manual_ip = input("Enter server IP address (e.g., 192.168.43.50): ").strip()
        if manual_ip:
            url = f"http://{manual_ip}:8000/positions.json"
            print(f"Trying manual: {url}")
            try:
                response = requests.get(url, timeout=5)
                if response.status_code == 200:
                    self.competition_data = response.json()
                    if self.team_number in self.competition_data["turrets"]:
                        self.my_position = self.competition_data["turrets"][self.team_number]
                        print(f"✓ Manual connection successful!")
                        return True
            except Exception as e:
                print(f"✗ Manual connection failed: {e}")
        
        return False
    
    def print_all_positions(self):
        """Print all competition positions"""
        if not self.competition_data:
            print("No competition data loaded. Fetch data first.")
            return
        
        print("\n" + "="*60)
        print("COMPETITION POSITIONS")
        print("="*60)
        
        # Our position
        if self.my_position:
            print(f"\nOUR TEAM ({self.team_number}):")
            print(f"  r = {self.my_position['r']} cm")
            print(f"  θ = {self.my_position['theta']} rad")
            print(f"     ({math.degrees(self.my_position['theta']):.1f}°)")
        
        # Other turrets
        print("\nOTHER TURRETS:")
        for team, pos in self.competition_data["turrets"].items():
            if team != self.team_number:
                deg = math.degrees(pos['theta'])
                print(f"  Team {team}: r={pos['r']}cm, θ={pos['theta']}rad ({deg:.1f}°)")
        
        # Globes
        print("\nGLOBES (Passive Targets):")
        for i, globe in enumerate(self.competition_data["globes"]):
            deg = math.degrees(globe['theta'])
            print(f"  Globe {i+1}: r={globe['r']}cm, θ={globe['theta']}rad ({deg:.1f}°), z={globe['z']}cm")
    
    # ========== TARGETING CALCULATIONS ==========
    
    def calculate_target_angles(self, target_r, target_theta, target_z=0):
        """
        Calculate aiming angles for a target
        Returns: (azimuth_angle, altitude_angle) in degrees
        """
        if not self.my_position:
            print("Error: Our position not known")
            return (0, 0)
        
        # Convert to Cartesian (x, y, z)
        our_x = self.my_position['r'] * math.cos(self.my_position['theta'])
        our_y = self.my_position['r'] * math.sin(self.my_position['theta'])
        our_z = 0  # We're on ground
        
        target_x = target_r * math.cos(target_theta)
        target_y = target_r * math.sin(target_theta)
        target_z = target_z
        
        # Calculate relative position
        dx = target_x - our_x
        dy = target_y - our_y
        dz = target_z - our_z
        
        # Calculate azimuth (0° = forward, positive = CCW)
        azimuth_rad = math.atan2(dy, dx)
        azimuth_deg = math.degrees(azimuth_rad)
        
        # Calculate altitude (0° = horizontal)
        distance_2d = math.sqrt(dx*dx + dy*dy)
        if distance_2d > 0:
            altitude_rad = math.atan2(dz, distance_2d)
            altitude_deg = math.degrees(altitude_rad)
        else:
            altitude_deg = 90 if dz > 0 else -90
        
        return (azimuth_deg, altitude_deg)
    
    def calculate_all_targets(self):
        """Calculate aiming angles for ALL targets"""
        if not self.competition_data:
            print("No data loaded. Fetch data first.")
            return
        
        print("\n" + "="*60)
        print("TARGETING CALCULATIONS")
        print("="*60)
        
        # Calculate for other turrets
        print("\nOTHER TURRETS:")
        for team, pos in self.competition_data["turrets"].items():
            if team != self.team_number:
                az, alt = self.calculate_target_angles(pos['r'], pos['theta'])
                print(f"  Team {team}: Aim: Az={az:.1f}°, Alt={alt:.1f}°")
        
        # Calculate for globes
        print("\nGLOBES:")
        for i, globe in enumerate(self.competition_data["globes"]):
            az, alt = self.calculate_target_angles(globe['r'], globe['theta'], globe['z'])
            print(f"  Globe {i+1}: Aim: Az={az:.1f}°, Alt={alt:.1f}°")
    
    # ========== LASER CONTROL ==========
    
    def laser_on(self):
        """Turn laser ON (GPIO HIGH)"""
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_state = True
        print("Laser: ON")
    
    def laser_off(self):
        """Turn laser OFF (GPIO LOW)"""
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_state = False
        print("Laser: OFF")
    
    def fire_laser(self, duration=3.0):
        """Fire laser for specified seconds"""
        print(f"Firing laser for {duration} seconds...")
        self.laser_on()
        time.sleep(duration)
        self.laser_off()
    
    # ========== AUTO-HOME ==========
    
    def auto_home(self):
        """Set current position as home (horizontal)"""
        print("\n" + "="*60)
        print("AUTO-HOME SETUP")
        print("="*60)
        print("Manually position turret:")
        print("1. Laser pointing forward (0°)")
        print("2. Flat sides parallel to motor protrusions")
        print("3. Press Enter when aligned...")
        input()
        
        self.azimuth_position = 0
        self.altitude_position = 0
        self.azimuth_phase = 0
        self.altitude_phase = 0
        self.update_motors()
        
        print("✓ Home position set to (0,0)")
    
    def go_to_home(self):
        """Return to home position"""
        print("Returning to home position...")
        self.move_motors_sync(-self.azimuth_position, -self.altitude_position, 0.001)
        self.azimuth_position = 0
        self.altitude_position = 0
        self.azimuth_phase = 0
        self.altitude_phase = 0
        print("✓ At home position")
    
    # ========== DEMO FUNCTIONS ==========
    
    def motor_demo(self):
        """Demo: 180° rotations with matched speeds"""
        print("\n" + "="*60)
        print("MOTOR DEMO: 180° rotations")
        print("="*60)
        
        cycle = 0
        try:
            while self.running:
                cycle += 1
                print(f"\n--- Cycle {cycle} ---")
                
                print("180° Clockwise/Up")
                self.move_motors_degrees_sync(180, 180, 0.0005)
                time.sleep(2)
                
                print("180° Counterclockwise/Down")
                self.move_motors_degrees_sync(-180, -180, 0.0005)
                time.sleep(2)
                
        except KeyboardInterrupt:
            self.running = False
            print("\nDemo stopped")
    
    def laser_demo(self):
        """Demo: Laser 2s ON/OFF"""
        print("\n" + "="*60)
        print("LASER DEMO: 2 seconds ON/OFF")
        print("="*60)
        
        cycle = 0
        try:
            while self.running:
                cycle += 1
                print(f"\nCycle {cycle}")
                
                self.laser_on()
                time.sleep(2)
                self.laser_off()
                time.sleep(2)
                
        except KeyboardInterrupt:
            self.running = False
    
    def test_targeting(self):
        """Test targeting on first available target"""
        if not self.competition_data:
            print("No data loaded. Fetch data first.")
            return
        
        # Find first other turret
        other_teams = [t for t in self.competition_data["turrets"].keys() if t != self.team_number]
        if other_teams:
            target_team = other_teams[0]
            target = self.competition_data["turrets"][target_team]
            az, alt = self.calculate_target_angles(target['r'], target['theta'])
            
            print(f"\nTargeting Team {target_team}:")
            print(f"  Position: r={target['r']}cm, θ={target['theta']}rad")
            print(f"  Aim angles: Azimuth={az:.1f}°, Altitude={alt:.1f}°")
            
            response = input("Move turret to aim? (y/n): ").strip().lower()
            if response == 'y':
                print(f"Moving to: Az={az:.1f}°, Alt={alt:.1f}°")
                self.move_motors_degrees_sync(az, alt, 0.001)
                
                response = input("Fire laser? (y/n): ").strip().lower()
                if response == 'y':
                    self.fire_laser(3)
        
        # Also test a globe
        if self.competition_data["globes"]:
            globe = self.competition_data["globes"][0]
            az, alt = self.calculate_target_angles(globe['r'], globe['theta'], globe['z'])
            
            print(f"\nTargeting Globe 1:")
            print(f"  Position: r={globe['r']}cm, θ={globe['theta']}rad, z={globe['z']}cm")
            print(f"  Aim angles: Azimuth={az:.1f}°, Altitude={alt:.1f}°")
    
    # ========== CLEANUP ==========
    
    def cleanup(self):
        """Clean shutdown"""
        self.running = False
        print("\nCleaning up...")
        self.go_to_home()
        self.shift_out(0b00000000)  # Turn off motors
        self.laser_off()
        GPIO.cleanup()
        print("Cleanup complete")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 LASER TURRET - COMPLETE SYSTEM")
    print("="*70)
    print("Features:")
    print("  1. Motor control with speed matching")
    print("  2. JSON coordinate fetching")
    print("  3. Targeting calculations")
    print("  4. Laser control")
    print("  5. Auto-home positioning")
    print("="*70)
    
    # Get team number
    team = input("Enter your team number (default: 1): ").strip()
    if not team:
        team = "1"
    
    turret = None
    try:
        turret = CompleteTurret(team_number=team)
        
        while True:
            print("\n" + "="*70)
            print(f"TEAM {team} - MAIN MENU")
            print(f"Position: Azi={turret.azimuth_position} steps, Alt={turret.altitude_position} steps")
            print("="*70)
            print("1. Fetch competition coordinates")
            print("2. Print all positions")
            print("3. Calculate all target angles")
            print("4. Test targeting (aim at first target)")
            print("5. Motor demo (180° rotations)")
            print("6. Laser demo (2s ON/OFF)")
            print("7. Test fire laser (3 seconds)")
            print("8. Auto-home setup")
            print("9. Return to home")
            print("10. Exit")
            
            choice = input("\nEnter choice (1-10): ").strip()
            
            if choice == "1":
                if turret.fetch_competition_data():
                    print("\n✓ Ready for competition!")
                else:
                    print("\n✗ Failed to fetch data")
            elif choice == "2":
                turret.print_all_positions()
            elif choice == "3":
                turret.calculate_all_targets()
            elif choice == "4":
                turret.test_targeting()
            elif choice == "5":
                turret.running = True
                turret.motor_demo()
            elif choice == "6":
                turret.running = True
                turret.laser_demo()
            elif choice == "7":
                turret.fire_laser(3)
            elif choice == "8":
                turret.auto_home()
            elif choice == "9":
                turret.go_to_home()
            elif choice == "10":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
            
            turret.running = False
            
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if turret:
            turret.cleanup()
        print("\nProgram ended")

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
