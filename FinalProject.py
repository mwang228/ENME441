#!/usr/bin/env python3
"""
ENME441 - WORKING TURRET CODE
Based on code that actually made your motors work
"""

import RPi.GPIO as GPIO
import time
import math

class WorkingTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 WORKING TURRET")
        print("="*70)
        
        # GPIO Pins (confirmed working)
        self.SHIFT_CLK = 11  # GPIO11 -> Pin 11
        self.LATCH_CLK = 10  # GPIO10 -> Pin 12
        self.DATA_PIN = 9    # GPIO9  -> Pin 14
        self.LASER_PIN = 26
        
        # YOUR WORKING CALIBRATIONS
        self.AZIMUTH_STEPS_PER_REV = 2200    # Works well
        self.ALTITUDE_STEPS_PER_REV = 450    # Half of 900 for double movement
        
        # YOUR MEASURED LIMITS
        self.MAX_ALTITUDE_UP = -60.0    # Max UP (negative)
        self.MAX_ALTITUDE_DOWN = 45.0   # Max DOWN (positive)
        
        # Current position
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        self.azimuth_steps = 0
        self.altitude_steps = 0
        
        # **CRITICAL: Use the SEQUENCE THAT WORKED**
        # Based on your testing, this sequence made altitude work
        self.AZIMUTH_SEQUENCE = [
            0b00000001,  # Coil A (Pin 15) - Azimuth
            0b00000010,  # Coil B (Pin 1)
            0b00000100,  # Coil C (Pin 2)
            0b00001000,  # Coil D (Pin 3)
        ]
        
        self.ALTITUDE_SEQUENCE = [
            0b00010000,  # Coil A (Pin 4) - Altitude
            0b00100000,  # Coil B (Pin 5)
            0b01000000,  # Coil C (Pin 6)
            0b10000000,  # Coil D (Pin 7)
        ]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Initialize
        self.setup_gpio()
        
        print(f"✓ Using YOUR working calibrations:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        print(f"  Altitude limits: {self.MAX_ALTITUDE_UP}° to {self.MAX_ALTITUDE_DOWN}°")
        print("="*70)
    
    def setup_gpio(self):
        """Initialize GPIO - same as working test code"""
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
        
        # Start with motors OFF
        self.send_to_shift_register(0b00000000)
    
    def send_to_shift_register(self, data):
        """Send data to shift register - proven working"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.000001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors(self):
        """Update both motors - SIMPLE and CORRECT"""
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        
        # Combine patterns
        combined = az_pattern | alt_pattern
        self.send_to_shift_register(combined)
    
    def step_azimuth(self, direction):
        """Step azimuth motor"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude(self, direction):
        """Step altitude motor - THIS WORKED IN TESTING"""
        self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
        self.altitude_steps += direction
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def move_azimuth_degrees(self, degrees, step_delay=0.001):
        """Move azimuth by degrees"""
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"Azimuth: Moving {degrees:+.1f}° ({steps} steps)")
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(step_delay)
            
            if steps > 20 and (i + 1) % (steps // 5) == 0:
                print(f"  {((i+1)/steps*100):.0f}% - Angle: {self.azimuth_angle:+.1f}°")
        
        print(f"✓ Azimuth: {self.azimuth_angle:+.1f}°")
    
    def move_altitude_degrees(self, degrees, step_delay=0.002):
        """Move altitude by degrees - SLOWER for reliability"""
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"Altitude: Moving {degrees:+.1f}° ({steps} steps)")
        
        successful_steps = 0
        
        for i in range(steps):
            # Calculate new angle
            new_angle = self.altitude_angle + (direction * 360.0 / self.ALTITUDE_STEPS_PER_REV)
            
            # Check limits
            if new_angle < self.MAX_ALTITUDE_UP or new_angle > self.MAX_ALTITUDE_DOWN:
                print(f"  ⚠ Altitude limit: {new_angle:.1f}°")
                break
            
            # Take step
            self.step_altitude(direction)
            successful_steps += 1
            time.sleep(step_delay)
            
            if steps > 20 and (i + 1) % (steps // 5) == 0:
                print(f"  {((i+1)/steps*100):.0f}% - Angle: {self.altitude_angle:+.1f}°")
        
        actual_degrees = (successful_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0 * direction
        print(f"✓ Altitude: {self.altitude_angle:+.1f}° (moved {actual_degrees:+.1f}°)")
        
        return actual_degrees
    
    def move_to_angle(self, target_az, target_alt):
        """Move to specific angle"""
        print(f"\nMoving to: Az={target_az:+.1f}°, Alt={target_alt:+.1f}°")
        
        # Move azimuth first
        az_move = target_az - self.azimuth_angle
        if abs(az_move) > 0.1:
            self.move_azimuth_degrees(az_move)
        
        # Then altitude
        alt_move = target_alt - self.altitude_angle
        if abs(alt_move) > 0.1:
            self.move_altitude_degrees(alt_move)
        
        print(f"✓ At: Az={self.azimuth_angle:+.1f}°, Alt={self.altitude_angle:+.1f}°")
    
    def test_both_motors(self):
        """Test both motors - proven working"""
        print("\n" + "="*60)
        print("TESTING BOTH MOTORS")
        print("="*60)
        
        print("1. Testing azimuth motor...")
        print("   Moving +45° (right)...")
        self.move_azimuth_degrees(45, 0.001)
        time.sleep(1)
        print("   Moving -45° (back to center)...")
        self.move_azimuth_degrees(-45, 0.001)
        
        print("\n2. Testing altitude motor...")
        print("   Moving +30° (DOWN)...")
        moved_down = self.move_altitude_degrees(30, 0.002)
        time.sleep(1)
        print("   Moving -30° (UP)...")
        moved_up = self.move_altitude_degrees(-30, 0.002)
        
        print(f"\n✓ Test complete!")
        print(f"  Azimuth: {self.azimuth_angle:+.1f}°")
        print(f"  Altitude: {self.altitude_angle:+.1f}°")
        
        # Check if altitude worked
        if abs(moved_down - 30) < 5 and abs(moved_up + 30) < 5:
            print("✓ Both motors working correctly!")
        else:
            print("⚠ Altitude motor may need adjustment")
    
    def test_altitude_limits(self):
        """Test your measured altitude limits"""
        print("\n" + "="*60)
        print("TESTING YOUR ALTITUDE LIMITS")
        print("="*60)
        print(f"Your limits: UP={self.MAX_ALTITUDE_UP}°, DOWN={self.MAX_ALTITUDE_DOWN}°")
        
        # Test going DOWN (positive angles)
        print("\n1. Testing DOWN movement...")
        test_down = min(45, self.MAX_ALTITUDE_DOWN)
        print(f"   Trying to move {test_down:+.1f}° DOWN...")
        moved_down = self.move_altitude_degrees(test_down, 0.002)
        
        # Return to center
        self.move_altitude_degrees(-moved_down, 0.002)
        
        # Test going UP (negative angles)
        print("\n2. Testing UP movement...")
        test_up = max(-60, self.MAX_ALTITUDE_UP)
        print(f"   Trying to move {test_up:+.1f}° UP...")
        moved_up = self.move_altitude_degrees(test_up, 0.002)
        
        # Return to center
        self.move_altitude_degrees(-moved_up, 0.002)
        
        print(f"\n✓ Limit test complete:")
        print(f"  Can move DOWN: {moved_down:+.1f}° of {test_down:+.1f}°")
        print(f"  Can move UP: {moved_up:+.1f}° of {test_up:+.1f}°")
        print(f"  Working range: {abs(moved_up) + moved_down:.1f}° total")
    
    def manual_control(self):
        """Simple manual control"""
        print("\n" + "="*60)
        print("MANUAL CONTROL")
        print("="*60)
        print("Commands: a=left, d=right, w=up, s=down, f=fire, z=zero, q=quit")
        
        while True:
            print(f"\nPosition: Az={self.azimuth_angle:+.1f}°, Alt={self.altitude_angle:+.1f}°")
            cmd = input("Command: ").lower()
            
            if cmd == 'a':
                self.move_azimuth_degrees(-10, 0.001)
            elif cmd == 'd':
                self.move_azimuth_degrees(10, 0.001)
            elif cmd == 'w':
                self.move_altitude_degrees(-10, 0.002)  # UP = negative
            elif cmd == 's':
                self.move_altitude_degrees(10, 0.002)   # DOWN = positive
            elif cmd == 'f':
                print("Firing laser for 3s...")
                GPIO.output(self.LASER_PIN, GPIO.HIGH)
                time.sleep(3)
                GPIO.output(self.LASER_PIN, GPIO.LOW)
                print("Laser off")
            elif cmd == 'z':
                self.move_to_angle(0, 0)
                print("At center (0°, 0°)")
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def competition_calculation(self):
        """Simple competition targeting"""
        print("\n" + "="*60)
        print("COMPETITION TARGETING")
        print("="*60)
        
        # Get your position
        print("Enter your competition position:")
        r = float(input("Radius r (cm, usually 300): ").strip() or "300")
        theta_deg = float(input("Angle θ (degrees from center): ").strip() or "0")
        theta = math.radians(theta_deg)
        
        # Example target
        print("\nEnter target information:")
        target_r = float(input("Target radius r (cm): ").strip() or "300")
        target_theta_deg = float(input("Target angle θ (degrees): ").strip() or "45")
        target_theta = math.radians(target_theta_deg)
        target_z = float(input("Target height z (cm, 0 for turrets): ").strip() or "0")
        
        # Convert to cartesian
        our_x = r * math.cos(theta)
        our_y = r * math.sin(theta)
        
        target_x = target_r * math.cos(target_theta)
        target_y = target_r * math.sin(target_theta)
        
        # Calculate angles
        dx = target_x - our_x
        dy = target_y - our_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        azimuth_rad = math.atan2(dy, dx)
        azimuth_deg = math.degrees(azimuth_rad)
        
        if distance > 0:
            altitude_rad = math.atan2(target_z, distance)
            altitude_deg = math.degrees(altitude_rad)
        else:
            altitude_deg = 90 if target_z > 0 else -90
        
        print(f"\nCalculated angles:")
        print(f"  Azimuth: {azimuth_deg:+.1f}°")
        print(f"  Altitude: {altitude_deg:+.1f}°")
        
        # Check if reachable
        if altitude_deg < self.MAX_ALTITUDE_UP or altitude_deg > self.MAX_ALTITUDE_DOWN:
            print(f"⚠ Target at altitude {altitude_deg:.1f}° is OUTSIDE your range")
            print(f"  Your range: {self.MAX_ALTITUDE_UP}° to {self.MAX_ALTITUDE_DOWN}°")
        else:
            print(f"✓ Target is WITHIN your range")
            
            move = input("Move to target? (y/n): ").lower()
            if move == 'y':
                self.move_to_angle(azimuth_deg, altitude_deg)
                
                fire = input("Fire laser? (y/n): ").lower()
                if fire == 'y':
                    print("Firing for 3 seconds...")
                    GPIO.output(self.LASER_PIN, GPIO.HIGH)
                    time.sleep(3)
                    GPIO.output(self.LASER_PIN, GPIO.LOW)
                    print("Target hit!")
    
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
    print("ENME441 WORKING TURRET CONTROL")
    print("="*70)
    print("This uses the SEQUENCE THAT WORKED in testing")
    print("No recursion, just proven motor control")
    print("="*70)
    
    turret = WorkingTurret()
    
    try:
        while True:
            print("\n" + "="*60)
            print("MAIN MENU")
            print("="*60)
            print("1. Test both motors (START HERE)")
            print("2. Test altitude limits (verify -60° to +45°)")
            print("3. Manual control (a/d/w/s keys)")
            print("4. Competition targeting calculator")
            print("5. Move to specific angle")
            print("6. Cleanup and exit")
            
            choice = input("\nChoice (1-6): ").strip()
            
            if choice == "1":
                turret.test_both_motors()
            elif choice == "2":
                turret.test_altitude_limits()
            elif choice == "3":
                turret.manual_control()
            elif choice == "4":
                turret.competition_calculation()
            elif choice == "5":
                try:
                    az = float(input("Target azimuth (degrees): "))
                    alt = float(input("Target altitude (degrees): "))
                    turret.move_to_angle(az, alt)
                except:
                    print("Invalid input")
            elif choice == "6":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        turret.cleanup()
        print("\nProgram ended")

if __name__ == "__main__":
    main()
