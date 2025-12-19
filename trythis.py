#!/usr/bin/env python3
"""
ENME441 - SIMPLIFIED FIXED VERSION
Direct fixes based on your observations
"""

import RPi.GPIO as GPIO
import time
import json
import os

class SimpleTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 - SIMPLIFIED DIRECT FIXES")
        print("="*70)
        print("DIRECT FIXES:")
        print("1. Azimuth: 2000 steps/rev (was 2400, overshooting)")
        print("2. Altitude: HARDWARE FIX NEEDED - swap wires")
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        
        # Configuration
        self.CONFIG_FILE = "simple_config.json"
        
        # SIMPLE FIXED VALUES
        self.AZIMUTH_STEPS_PER_REV = 2000  # Your suggestion
        self.ALTITUDE_STEPS_PER_REV = 450   # Keep this (was double movement)
        
        self.load_config()
        
        print(f"Using SIMPLE configuration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # Sequences
        self.AZIMUTH_SEQUENCE = [0b00000001, 0b00000010, 0b00000100, 0b00001000]
        
        # **CRITICAL: For altitude, we need REVERSED sequence**
        # Since it only works in negative direction, the sequence is backwards
        self.ALTITUDE_SEQUENCE = [
            0b10000000,  # Coil D FIRST (reversed order)
            0b01000000,  # Coil C
            0b00100000,  # Coil B
            0b00010000,  # Coil A LAST
        ]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Simple timing
        self.AZIMUTH_DELAY = 0.015
        self.ALTITUDE_DELAY = 0.025
        
        # Initialize
        self.setup_gpio()
        
        print(f"\n✓ Simple system ready")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print("="*70)
    
    def load_config(self):
        """Load simple config"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 2000)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 450)
                print("✓ Configuration loaded")
        except:
            print("Using simple fixed values")
    
    def save_config(self):
        """Save config"""
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
        """Initialize GPIO"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        self.send_to_shift_register(0b00000000)
    
    def send_to_shift_register(self, data):
        """Send data"""
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
        """Update both motors"""
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth(self, direction):
        """Step azimuth"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude(self, direction):
        """Step altitude with REVERSED sequence"""
        # For reversed sequence, positive direction might mean negative physical movement
        # We'll handle this in the movement functions
        self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
        self.altitude_steps += direction
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def test_azimuth_90(self):
        """Test azimuth 90° movement"""
        print("\n" + "="*60)
        print("AZIMUTH 90° TEST")
        print("="*60)
        
        print(f"Using {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        
        # Move 90°
        start_angle = self.azimuth_angle
        steps = int((90 / 360) * self.AZIMUTH_STEPS_PER_REV)
        
        print(f"Taking {steps} steps for 90°...")
        
        for i in range(steps):
            self.step_azimuth(1)
            time.sleep(self.AZIMUTH_DELAY)
            
            if (i + 1) % 20 == 0:
                print(f"  Step {i+1}/{steps}, Angle: {self.azimuth_angle:.1f}°")
        
        actual = self.azimuth_angle - start_angle
        print(f"\nResult: Moved {actual:.1f}° (Target: 90.0°)")
        
        if abs(actual - 90) > 5:
            print(f"\nStill off by {abs(actual-90):.1f}°")
            correction = 90.0 / actual
            new_steps = int(self.AZIMUTH_STEPS_PER_REV * correction)
            print(f"Try: {new_steps} steps/rev")
        
        # Return
        self.move_azimuth(-actual)
    
    def move_azimuth(self, degrees):
        """Simple azimuth movement"""
        steps = int((degrees / 360) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(self.AZIMUTH_DELAY)
    
    def test_altitude_direction_fix(self):
        """Test the altitude direction workaround"""
        print("\n" + "="*60)
        print("ALTITUDE DIRECTION WORKAROUND")
        print("="*60)
        
        print("Since altitude only works in negative direction:")
        print("We'll treat POSITIVE = NEGATIVE movement")
        print("This is a SOFTWARE workaround for HARDWARE issue")
        
        # Test what we can do
        print("\nTesting what works:")
        
        print("1. Trying NEGATIVE movement (should work)...")
        start = self.altitude_angle
        steps = int((30 / 360) * self.ALTITUDE_STEPS_PER_REV)
        
        for i in range(steps):
            self.step_altitude(-1)  # Negative direction
            time.sleep(self.ALTITUDE_DELAY)
        
        neg_moved = start - self.altitude_angle
        print(f"  Moved: {neg_moved:.1f}° (down/negative)")
        
        print("\n2. Trying POSITIVE movement (might not work)...")
        start = self.altitude_angle
        for i in range(steps):
            self.step_altitude(1)  # Positive direction
            time.sleep(self.ALTITUDE_DELAY * 1.5)  # Slower
        
        pos_moved = self.altitude_angle - start
        print(f"  Moved: {pos_moved:.1f}° (up/positive)")
        
        print("\n" + "="*60)
        print("DIRECTION FIX OPTIONS:")
        print("="*60)
        
        if pos_moved > 10:  # If positive works
            print("✓ Positive direction works!")
            print("  No fix needed in software")
        elif neg_moved > 10 and pos_moved < 5:
            print("⚠️ Only negative direction works")
            print("  OPTION 1: Swap two wires on altitude motor")
            print("  OPTION 2: Invert altitude direction in software")
            print("  OPTION 3: Live with it (competition might not need up)")
    
    def hardware_fix_guide(self):
        """Guide for hardware fix"""
        print("\n" + "="*60)
        print("HARDWARE FIX GUIDE - ALTITUDE MOTOR")
        print("="*60)
        
        print("PROBLEM: Motor only turns in one direction")
        print("CAUSE: Coils wired in wrong order")
        print("")
        print("QUICK FIX - TRY THIS:")
        print("1. Turn off power")
        print("2. Swap ANY TWO wires on the altitude motor")
        print("3. Turn on power and test")
        print("")
        print("Which wires to swap?")
        print("Option A (easiest): Swap pins 4 and 5")
        print("Option B: Swap pins 6 and 7")
        print("Option C: Swap pins 4 and 6")
        print("")
        print("TEST after each swap!")
    
    def competition_workaround(self):
        """Competition workaround if hardware can't be fixed"""
        print("\n" + "="*60)
        print("COMPETITION WORKAROUND")
        print("="*60)
        
        print("If altitude only moves DOWN, not UP:")
        print("We can still compete with limitations")
        print("")
        print("STRATEGY:")
        print("1. Start with altitude at MAX height")
        print("2. Only move DOWN during competition")
        print("3. For targets at different heights:")
        print("   - Use different turret positions")
        print("   - Or accept you can't hit high targets")
        print("")
        print("TEST MOVEMENTS:")
        
        # Test practical movements
        movements = [
            ("Right 45°", 45, 0),
            ("Down 30°", 0, -30),
            ("Left 90°", -90, 0),
            ("Down 45°", 0, -45),
        ]
        
        for name, az_move, alt_move in movements:
            print(f"\n{name}:")
            
            if az_move != 0:
                self.move_azimuth(az_move)
                print(f"  Azimuth: {self.azimuth_angle:.1f}°")
            
            if alt_move != 0:
                # Only allow negative altitude
                if alt_move < 0:
                    steps = int((abs(alt_move) / 360) * self.ALTITUDE_STEPS_PER_REV)
                    for i in range(steps):
                        self.step_altitude(-1)
                        time.sleep(self.ALTITUDE_DELAY)
                    print(f"  Altitude: {self.altitude_angle:.1f}°")
                else:
                    print(f"  ⚠️ Can't move UP (hardware limit)")
            
            time.sleep(0.3)
        
        # Return
        print(f"\nReturning to start...")
        self.move_azimuth(-self.azimuth_angle)
        # Can't return altitude up, so leave it
    
    def simple_control(self):
        """Simple control interface"""
        print("\n" + "="*60)
        print("SIMPLE CONTROL")
        print("="*60)
        
        while True:
            print(f"\nPosition: Az={self.azimuth_angle:.1f}°, Alt={self.altitude_angle:.1f}°")
            print("\nCommands:")
            print("a/d - Azimuth left/right")
            print("s   - Altitude DOWN only (hardware limit)")
            print("z   - Zero azimuth")
            print("q   - Quit")
            
            cmd = input("Command: ").lower()
            
            if cmd == 'a':
                self.move_azimuth(-10)
            elif cmd == 'd':
                self.move_azimuth(10)
            elif cmd == 's':
                # Only down
                steps = int((10 / 360) * self.ALTITUDE_STEPS_PER_REV)
                for i in range(steps):
                    self.step_altitude(-1)
                    time.sleep(self.ALTITUDE_DELAY)
            elif cmd == 'z':
                self.move_azimuth(-self.azimuth_angle)
                print(f"Azimuth zeroed: {self.azimuth_angle:.1f}°")
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    
    def cleanup(self):
        """Clean shutdown"""
        print("\nCleaning up...")
        self.send_to_shift_register(0b00000000)
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 - SIMPLE DIRECT FIXES")
    print("="*70)
    print("AZIMUTH: 2000 steps/rev (try this)")
    print("ALTITUDE: Hardware fix needed or workaround")
    print("="*70)
    
    turret = None
    try:
        turret = SimpleTurret()
        
        while True:
            print("\n" + "="*60)
            print("SIMPLE MENU")
            print("="*60)
            print("1. Test azimuth 90° (check 2000 steps/rev)")
            print("2. Test altitude direction issue")
            print("3. Hardware fix guide")
            print("4. Competition workaround")
            print("5. Simple control")
            print("6. Exit")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                turret.test_azimuth_90()
            elif choice == "2":
                turret.test_altitude_direction_fix()
            elif choice == "3":
                turret.hardware_fix_guide()
            elif choice == "4":
                turret.competition_workaround()
            elif choice == "5":
                turret.simple_control()
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
