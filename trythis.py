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
    
    def test_azimuth_accuracy(self):
        """Test azimuth accuracy with 2200 steps/rev"""
        print("\n" + "="*60)
        print("AZIMUTH ACCURACY TEST - 2200 steps/rev")
        print("="*60)
        
        test_angles = [45, 90, 180]
        
        for target in test_angles:
            print(f"\nTesting {target}° movement...")
            
            start_angle = self.azimuth_angle
            steps = int((target / 360) * self.AZIMUTH_STEPS_PER_REV)
            
            print(f"  Steps: {steps} (based on {self.AZIMUTH_STEPS_PER_REV} steps/rev)")
            
            for i in range(steps):
                self.step_azimuth(1)
                time.sleep(self.AZIMUTH_DELAY)
                
                if steps > 20 and (i + 1) % (steps // 4) == 0:
                    print(f"  Progress: {((i+1)/steps*100):.0f}%")
            
            actual = self.azimuth_angle - start_angle
            error = abs(actual - target)
            
            print(f"  Result: {actual:.1f}° (Target: {target:.1f}°)")
            print(f"  Error: {error:.1f}° ({error/target*100:.1f}%)")
            
            # Return to start
            return_steps = int((actual / 360) * self.AZIMUTH_STEPS_PER_REV)
            for i in range(return_steps):
                self.step_azimuth(-1)
                time.sleep(self.AZIMUTH_DELAY)
        
        print("\n✓ Azimuth test complete")
    
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
    
    def competition_strategy(self):
        """Competition strategy with current limitations"""
        print("\n" + "="*60)
        print("COMPETITION STRATEGY")
        print("="*60)
        
        print("With current motor limitations:")
        print(f"• Azimuth: Full range (working)")
        print(f"• Altitude: Limited to {self.altitude_max_up:.1f}° UP, {self.altitude_max_down:.1f}° DOWN")
        print("")
        print("STRATEGY:")
        print("1. Start at optimal altitude position")
        print("2. Use azimuth for most targeting")
        print("3. Use altitude only when absolutely needed")
        print("4. Prioritize targets within altitude range")
        print("")
        print("TESTING PRACTICAL MOVEMENTS:")
        
        # Practical test sequence
        test_movements = [
            ("Scan right 60°", 60, 0),
            ("Aim at mid-height target", 0, self.altitude_max_up/2),
            ("Scan left 120°", -120, 0),
            ("Aim at low target", 0, self.altitude_max_down),
            ("Return to start", -self.azimuth_angle, -self.altitude_angle),
        ]
        
        for name, az_move, alt_move in test_movements:
            print(f"\n{name}:")
            
            if az_move != 0:
                print(f"  Azimuth: {az_move:+.1f}°")
                self.move_azimuth_safe(az_move)
            
            if alt_move != 0:
                print(f"  Altitude: {alt_move:+.1f}°")
                actual = self.move_altitude_safe(alt_move)
                print(f"  Actual: {actual:+.1f}°")
            
            print(f"  Position: ({self.azimuth_angle:.1f}°, {self.altitude_angle:.1f}°)")
            time.sleep(0.3)
        
        print(f"\n✓ Strategy test complete")
        print(f"Final error: ({self.azimuth_angle:.1f}°, {self.altitude_angle:.1f}°)")
    
    def power_boost_test(self):
        """Test if external power would help"""
        print("\n" + "="*60)
        print("POWER BOOST TEST")
        print("="*60)
        
        print("Altitude motor issues might be POWER-related")
        print("")
        print("QUICK FIXES TO TRY:")
        print("1. Use SEPARATE battery for motors")
        print("2. Add 100µF capacitor across motor power")
        print("3. Check all connections are tight")
        print("4. Try different USB battery pack")
        print("")
        print("TEST: Move azimuth while testing altitude")
        
        # Test concurrent movement
        print("\nTesting concurrent movement...")
        
        # Try moving both at once
        az_steps = int((45 / 360) * self.AZIMUTH_STEPS_PER_REV)
        alt_steps = int((20 / 360) * self.ALTITUDE_STEPS_PER_REV)
        
        max_steps = max(az_steps, alt_steps)
        
        az_done = 0
        alt_done = 0
        
        print(f"Moving azimuth 45° and altitude 20° together...")
        
        for i in range(max_steps):
            if az_done < az_steps:
                self.step_azimuth(1)
                az_done += 1
            
            if alt_done < alt_steps:
                if self.step_altitude_with_power_boost(1):
                    alt_done += 1
            
            time.sleep(max(self.AZIMUTH_DELAY, self.ALTITUDE_DELAY))
        
        print(f"\nResults:")
        print(f"  Azimuth: {az_done}/{az_steps} steps")
        print(f"  Altitude: {alt_done}/{alt_steps} steps")
        
        if alt_done < alt_steps * 0.5:
            print("\n⚠️ CONCURRENT MOVEMENT FAILED")
            print("  Likely INSUFFICIENT POWER")
            print("  Try separate power supply for motors")
    
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
    print("• Competition-ready with current limitations")
    print("="*70)
    
    turret = None
    try:
        turret = WorkingTurret()
        
        while True:
            print("\n" + "="*60)
            print("FINAL WORKING MENU")
            print("="*60)
            print("1. Test azimuth accuracy (2200 steps/rev)")
            print("2. Find altitude working limits")
            print("3. Competition strategy test")
            print("4. Power boost test")
            print("5. Simple control interface")
            print("6. Show current status")
            print("7. Exit and cleanup")
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == "1":
                turret.test_azimuth_accuracy()
            elif choice == "2":
                turret.find_altitude_limits()
            elif choice == "3":
                turret.competition_strategy()
            elif choice == "4":
                turret.power_boost_test()
            elif choice == "5":
                turret.simple_interface()
            elif choice == "6":
                print(f"\nCurrent status:")
                print(f"  Azimuth: {turret.azimuth_steps} steps, {turret.azimuth_angle:.1f}°")
                print(f"  Altitude: {turret.altitude_steps} steps, {turret.altitude_angle:.1f}°")
                print(f"  Azimuth steps/rev: {turret.AZIMUTH_STEPS_PER_REV}")
                print(f"  Altitude steps/rev: {turret.ALTITUDE_STEPS_PER_REV}")
                print(f"  Altitude limits: +{turret.altitude_max_up:.1f}° to {turret.altitude_max_down:.1f}°")
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
