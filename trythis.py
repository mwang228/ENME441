#!/usr/bin/env python3
"""
ENME441 - FINAL WORKING VERSION
With altitude motor workaround
"""

import RPi.GPIO as GPIO
import time
import json
import os

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
        
        # Initialize
        self.setup_gpio()
        
        print(f"\n✓ FINAL system ready")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
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
        
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
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
    
    def find_altitude_limits(self):
        """Find what altitude movements actually work"""
        print("\n" + "="*60)
        print("FIND ALTITUDE WORKING LIMITS")
        print("="*60)
        
        print("Testing what altitude movements actually work...")
        
        # Test DOWN first (usually works better)
        print("\n1. Testing DOWN movement...")
        down_limit = 0
        
        for attempt in [10, 20, 30, 45, 60]:
            print(f"  Trying -{attempt}°...")
            start_angle = self.altitude_angle
            
            steps = int((attempt / 360) * self.ALTITUDE_STEPS_PER_REV)
            success_count = 0
            
            for i in range(steps):
                if self.step_altitude_with_power_boost(-1):
                    success_count += 1
                    time.sleep(self.ALTITUDE_DELAY)
                else:
                    print(f"    Failed at step {i+1}/{steps}")
                    break
            
            moved = start_angle - self.altitude_angle
            print(f"    Moved: {moved:.1f}° of {attempt}°")
            
            if moved > attempt * 0.8:  # 80% success
                down_limit = -attempt
                print(f"    ✓ Can move down to -{attempt}°")
            else:
                print(f"    ✗ Limited at -{attempt}°")
                break
        
        # Test UP
        print("\n2. Testing UP movement...")
        up_limit = 0
        
        for attempt in [10, 20, 30, 45]:
            print(f"  Trying +{attempt}°...")
            start_angle = self.altitude_angle
            
            steps = int((attempt / 360) * self.ALTITUDE_STEPS_PER_REV)
            success_count = 0
            
            for i in range(steps):
                if self.step_altitude_with_power_boost(1):
                    success_count += 1
                    time.sleep(self.ALTITUDE_DELAY)
                else:
                    print(f"    Failed at step {i+1}/{steps}")
                    break
            
            moved = self.altitude_angle - start_angle
            print(f"    Moved: {moved:.1f}° of {attempt}°")
            
            if moved > attempt * 0.8:  # 80% success
                up_limit = attempt
                print(f"    ✓ Can move up to +{attempt}°")
            else:
                print(f"    ✗ Limited at +{attempt}°")
                break
        
        self.altitude_max_up = up_limit
        self.altitude_max_down = down_limit
        
        print("\n" + "="*60)
        print("ALTITUDE LIMITS FOUND:")
        print("="*60)
        print(f"Maximum UP: +{up_limit:.1f}°")
        print(f"Maximum DOWN: {down_limit:.1f}°")
        print(f"Working range: {up_limit + abs(down_limit):.1f}° total")
        
        # Return to middle of range
        middle = (up_limit + down_limit) / 2
        self.move_altitude_safe(middle - self.altitude_angle)
    
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
    
    def set_altitude_angle_test(self):
        """Test to set the altitude motor to a specific angle"""
        print("\n" + "="*60)
        print("SET ALTITUDE ANGLE TEST")
        print("="*60)
        
        print(f"Current altitude angle: {self.altitude_angle:.1f}°")
        print(f"Altitude limits: +{self.altitude_max_up:.1f}° to {self.altitude_max_down:.1f}°")
        
        try:
            target_angle = float(input("\nEnter target angle (degrees): "))
        except ValueError:
            print("Invalid input. Please enter a number.")
            return
        
        # Check if target is within limits
        if target_angle > self.altitude_max_up:
            print(f"⚠️ Target angle {target_angle:.1f}° exceeds maximum UP limit of {self.altitude_max_up:.1f}°")
            choice = input(f"Move to maximum UP limit ({self.altitude_max_up:.1f}°) instead? (y/n): ")
            if choice.lower() == 'y':
                target_angle = self.altitude_max_up
            else:
                return
        
        if target_angle < self.altitude_max_down:
            print(f"⚠️ Target angle {target_angle:.1f}° exceeds maximum DOWN limit of {self.altitude_max_down:.1f}°")
            choice = input(f"Move to maximum DOWN limit ({self.altitude_max_down:.1f}°) instead? (y/n): ")
            if choice.lower() == 'y':
                target_angle = self.altitude_max_down
            else:
                return
        
        # Calculate movement needed
        degrees_to_move = target_angle - self.altitude_angle
        print(f"\nMoving altitude from {self.altitude_angle:.1f}° to {target_angle:.1f}°")
        print(f"Movement required: {degrees_to_move:+.1f}°")
        
        # Perform the movement
        actual_movement = self.move_altitude_safe(degrees_to_move)
        
        print(f"\n✓ Movement complete")
        print(f"Target angle: {target_angle:.1f}°")
        print(f"Actual angle reached: {self.altitude_angle:.1f}°")
        print(f"Movement achieved: {actual_movement:+.1f}°")
        print(f"Accuracy error: {abs(target_angle - self.altitude_angle):.2f}°")
    
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
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 - FINAL WORKING VERSION")
    print("="*70)
    print("Key features:")
    print("• Azimuth: 2200 steps/rev")
    print("• Altitude: Power-aware movement with limits")
    print("• Set angle test for altitude motor")
    print("="*70)
    
    turret = None
    try:
        turret = WorkingTurret()
        
        while True:
            print("\n" + "="*60)
            print("FINAL WORKING MENU")
            print("="*60)
            print("1. Find altitude working limits")
            print("2. Set altitude angle test")
            print("3. Simple control interface")
            print("4. Show current status")
            print("5. Exit and cleanup")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == "1":
                turret.find_altitude_limits()
            elif choice == "2":
                turret.set_altitude_angle_test()
            elif choice == "3":
                turret.simple_interface()
            elif choice == "4":
                print(f"\nCurrent status:")
                print(f"  Azimuth: {turret.azimuth_steps} steps, {turret.azimuth_angle:.1f}°")
                print(f"  Altitude: {turret.altitude_steps} steps, {turret.altitude_angle:.1f}°")
                print(f"  Azimuth steps/rev: {turret.AZIMUTH_STEPS_PER_REV}")
                print(f"  Altitude steps/rev: {turret.ALTITUDE_STEPS_PER_REV}")
                print(f"  Altitude limits: +{turret.altitude_max_up:.1f}° to {turret.altitude_max_down:.1f}°")
            elif choice == "5":
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
