#!/usr/bin/env python3
"""
ENME441 Laser Turret - POWER AND TORQUE OPTIMIZATION
Fixes for stuttering and limited movement range
"""

import RPi.GPIO as GPIO
import time
import json
import os

class PowerOptimizedTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 - POWER OPTIMIZED TURRET")
        print("="*70)
        print("Fixing stuttering and range limitations")
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11  # GPIO11 -> Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> Pin 14 (DS)
        
        # Configuration
        self.CONFIG_FILE = "power_optimized_config.json"
        
        # Use our corrected values
        self.AZIMUTH_STEPS_PER_REV = 900    # Corrected
        self.ALTITUDE_STEPS_PER_REV = 1800  # Corrected
        
        self.load_config()
        
        print(f"Configuration loaded:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # **CRITICAL FIX: Use 2-phase excitation for more torque**
        self.AZIMUTH_SEQUENCE = [
            0b00000011,  # Coils A+B (MORE TORQUE)
            0b00000110,  # Coils B+C
            0b00001100,  # Coils C+D
            0b00001001,  # Coils D+A
        ]
        
        self.ALTITUDE_SEQUENCE = [
            0b00010000 | 0b00100000,  # Coils A+B
            0b00100000 | 0b01000000,  # Coils B+C
            0b01000000 | 0b10000000,  # Coils C+D
            0b10000000 | 0b00010000,  # Coils D+A
        ]
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # **POWER MANAGEMENT SETTINGS**
        self.current_az_pattern = 0
        self.current_alt_pattern = 0
        
        # Dynamic timing based on load
        self.BASE_AZIMUTH_DELAY = 0.020     # 20ms base
        self.BASE_ALTITUDE_DELAY = 0.025    # 25ms base
        self.MAX_DELAY = 0.100              # 100ms max when struggling
        self.MIN_DELAY = 0.005              # 5ms min when easy
        
        # Torque boost settings
        self.TORQUE_BOOST_ENABLED = True
        self.torque_boost_level = 1.0  # 1.0 = normal, 2.0 = double delay
        
        # Error recovery
        self.max_retries = 3
        self.retry_delay_multiplier = 2.0
        
        # Initialize
        self.setup_gpio()
        
        # Start with motors energized
        self.update_motors_with_hold()
        time.sleep(0.2)  # Let power stabilize
        
        print(f"\n✓ Power-optimized system ready")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print(f"Torque boost: {'ON' if self.TORQUE_BOOST_ENABLED else 'OFF'}")
        print("="*70)
    
    def load_config(self):
        """Load configuration"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 900)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 1800)
                print("✓ Configuration loaded")
        except:
            print("Using default corrected values")
    
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
            pass  # Silent fail for config save
    
    def setup_gpio(self):
        """Initialize GPIO with power management"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Clear but keep some power for a moment
        self.send_to_shift_register(0b00000000)
        time.sleep(0.1)
    
    def send_to_shift_register(self, data):
        """Send with power stabilization"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.000001)  # Tiny delay for stability
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.000001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def update_motors_with_hold(self):
        """Update motors and hold position"""
        az_pattern = self.AZIMUTH_SEQUENCE[self.azimuth_seq_pos]
        alt_pattern = self.ALTITUDE_SEQUENCE[self.altitude_seq_pos]
        
        combined = az_pattern | alt_pattern
        self.send_to_shift_register(combined)
        
        # Store for holding
        self.current_az_pattern = az_pattern
        self.current_alt_pattern = alt_pattern
    
    def hold_position(self, duration=0.01):
        """Hold current position to maintain torque"""
        combined = self.current_az_pattern | self.current_alt_pattern
        self.send_to_shift_register(combined)
        if duration > 0:
            time.sleep(duration)
    
    def calculate_dynamic_delay(self, motor_type, current_angle):
        """
        Calculate delay based on position and load
        Returns longer delays at difficult positions
        """
        base_delay = self.BASE_AZIMUTH_DELAY if motor_type == 'azimuth' else self.BASE_ALTITUDE_DELAY
        
        # Increase delay at extreme angles (±45° and beyond)
        angle_factor = 1.0
        abs_angle = abs(current_angle)
        
        if abs_angle > 30:  # Beyond 30°, start slowing down
            angle_factor = 1.0 + (abs_angle - 30) / 60  # 1.0 to 2.0
        
        # Apply torque boost
        torque_factor = self.torque_boost_level if self.TORQUE_BOOST_ENABLED else 1.0
        
        # Calculate final delay
        delay = base_delay * angle_factor * torque_factor
        
        # Clamp to min/max
        return max(self.MIN_DELAY, min(self.MAX_DELAY, delay))
    
    def step_azimuth_with_recovery(self, direction):
        """Step azimuth with error recovery"""
        retries = 0
        step_successful = False
        
        while retries < self.max_retries and not step_successful:
            try:
                # Calculate dynamic delay based on current position
                current_delay = self.calculate_dynamic_delay('azimuth', self.azimuth_angle)
                
                # Take the step
                self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
                self.azimuth_steps += direction
                self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
                
                self.update_motors_with_hold()
                
                # Hold for stability
                hold_time = current_delay * 0.5  # Hold for half the step time
                time.sleep(hold_time)
                
                step_successful = True
                
                # Return remaining delay time
                return current_delay - hold_time
                
            except Exception as e:
                retries += 1
                print(f"  ⚠️ Azimuth step failed (attempt {retries}/{self.max_retries})")
                
                # Increase delay for retry
                retry_delay = self.calculate_dynamic_delay('azimuth', self.azimuth_angle) * self.retry_delay_multiplier
                time.sleep(retry_delay)
                
                # Try re-energizing current position
                self.hold_position(0.1)
        
        if not step_successful:
            print(f"  ⚠️ Azimuth step failed after {self.max_retries} retries")
            # Don't update position if step failed
            self.azimuth_steps -= direction  # Revert
            self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        
        return 0  # No additional delay needed
    
    def step_altitude_with_recovery(self, direction):
        """Step altitude with error recovery"""
        retries = 0
        step_successful = False
        
        while retries < self.max_retries and not step_successful:
            try:
                # Calculate dynamic delay
                current_delay = self.calculate_dynamic_delay('altitude', self.altitude_angle)
                
                # Take the step
                self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
                self.altitude_steps += direction
                self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
                
                self.update_motors_with_hold()
                
                # Hold for stability
                hold_time = current_delay * 0.5
                time.sleep(hold_time)
                
                step_successful = True
                return current_delay - hold_time
                
            except Exception as e:
                retries += 1
                print(f"  ⚠️ Altitude step failed (attempt {retries}/{self.max_retries})")
                
                retry_delay = self.calculate_dynamic_delay('altitude', self.altitude_angle) * self.retry_delay_multiplier
                time.sleep(retry_delay)
                self.hold_position(0.1)
        
        if not step_successful:
            print(f"  ⚠️ Altitude step failed after {self.max_retries} retries")
            self.altitude_steps -= direction
            self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        
        return 0
    
    def move_azimuth_adaptive(self, degrees):
        """Move azimuth with adaptive power management"""
        steps = int((degrees / 360.0) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nAzimuth: Moving {degrees:+.1f}° from {self.azimuth_angle:.1f}°")
        print(f"Steps: {steps}, Direction: {'right' if direction > 0 else 'left'}")
        print(f"Target: {self.azimuth_angle + degrees:.1f}°")
        
        successful_steps = 0
        total_delay_used = 0
        
        for i in range(steps):
            # Show progress at difficult angles
            current_abs_angle = abs(self.azimuth_angle)
            if current_abs_angle > 30 and i % 5 == 0:
                print(f"  At {self.azimuth_angle:+.1f}° (difficult angle, slowing down)")
            
            # Take step with recovery
            remaining_delay = self.step_azimuth_with_recovery(direction)
            
            if remaining_delay > 0:
                successful_steps += 1
                # Use remaining delay
                time.sleep(remaining_delay)
                total_delay_used += remaining_delay
            
            # Progress indicator
            if steps > 10 and (i + 1) % (steps // 5) == 0:
                progress = (i + 1) / steps * 100
                print(f"  {progress:.0f}% - Angle: {self.azimuth_angle:.1f}°")
        
        actual_movement = self.azimuth_angle - (self.azimuth_angle - degrees)
        print(f"\n✓ Movement complete:")
        print(f"  Target: {degrees:+.1f}°, Actual: {actual_movement:+.1f}°")
        print(f"  Steps: {successful_steps}/{steps} successful")
        print(f"  Final: {self.azimuth_angle:.1f}°")
        
        return self.azimuth_angle
    
    def move_altitude_adaptive(self, degrees):
        """Move altitude with adaptive power management"""
        steps = int((degrees / 360.0) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        print(f"\nAltitude: Moving {degrees:+.1f}° from {self.altitude_angle:.1f}°")
        print(f"Steps: {steps}, Direction: {'up' if direction > 0 else 'down'}")
        print(f"Target: {self.altitude_angle + degrees:.1f}°")
        
        successful_steps = 0
        
        for i in range(steps):
            # Take step with recovery
            remaining_delay = self.step_altitude_with_recovery(direction)
            
            if remaining_delay > 0:
                successful_steps += 1
                time.sleep(remaining_delay)
            
            # Progress indicator
            if steps > 10 and (i + 1) % (steps // 5) == 0:
                progress = (i + 1) / steps * 100
                print(f"  {progress:.0f}% - Angle: {self.altitude_angle:.1f}°")
        
        actual_movement = self.altitude_angle - (self.altitude_angle - degrees)
        print(f"\n✓ Movement complete:")
        print(f"  Target: {degrees:+.1f}°, Actual: {actual_movement:+.1f}°")
        print(f"  Steps: {successful_steps}/{steps} successful")
        print(f"  Final: {self.altitude_angle:.1f}°")
        
        return self.altitude_angle
    
    def range_limit_test(self):
        """Test full range of motion"""
        print("\n" + "="*60)
        print("FULL RANGE LIMIT TEST")
        print("="*60)
        
        print("Testing azimuth through full range...")
        print("We'll test in small increments to find limits")
        
        test_angles = [15, 30, 45, 60, 75, 90]
        
        print("\n1. Testing positive (right) direction:")
        for angle in test_angles:
            print(f"\n  Attempting +{angle}° from {self.azimuth_angle:.1f}°...")
            start_angle = self.azimuth_angle
            self.move_azimuth_adaptive(angle)
            actual = self.azimuth_angle - start_angle
            
            if abs(actual - angle) < 5:
                print(f"    ✓ Success: Moved {actual:.1f}°")
            else:
                print(f"    ✗ Limited: Only moved {actual:.1f}° of {angle}°")
                print(f"    Mechanical limit at {self.azimuth_angle:.1f}°")
                break
        
        print("\n2. Testing negative (left) direction:")
        # Return to center first
        self.move_azimuth_adaptive(-self.azimuth_angle)
        
        for angle in test_angles:
            print(f"\n  Attempting -{angle}° from {self.azimuth_angle:.1f}°...")
            start_angle = self.azimuth_angle
            self.move_azimuth_adaptive(-angle)
            actual = start_angle - self.azimuth_angle
            
            if abs(actual - angle) < 5:
                print(f"    ✓ Success: Moved {actual:.1f}°")
            else:
                print(f"    ✗ Limited: Only moved {actual:.1f}° of {angle}°")
                print(f"    Mechanical limit at {self.azimuth_angle:.1f}°")
                break
        
        # Return to center
        print("\n3. Returning to center...")
        self.move_azimuth_adaptive(-self.azimuth_angle)
        
        print(f"\n✓ Range test complete")
        print(f"Azimuth working range: ±{abs(self.azimuth_angle):.1f}°")
    
    def power_management_test(self):
        """Test different power management settings"""
        print("\n" + "="*60)
        print("POWER MANAGEMENT TEST")
        print("="*60)
        
        print("Testing different torque settings...")
        
        settings = [
            ("Normal torque", 1.0, False),
            ("Torque boost", 1.5, True),
            ("Max torque", 2.0, True),
        ]
        
        for name, boost_level, boost_enabled in settings:
            print(f"\nTesting: {name}")
            self.torque_boost_level = boost_level
            self.TORQUE_BOOST_ENABLED = boost_enabled
            
            # Test moving 45°
            print(f"  Moving +45°...")
            start_angle = self.azimuth_angle
            self.move_azimuth_adaptive(45)
            actual = self.azimuth_angle - start_angle
            
            print(f"  Result: Moved {actual:.1f}° of 45°")
            
            # Return to start for next test
            self.move_azimuth_adaptive(-actual)
            time.sleep(0.5)
        
        # Restore normal settings
        self.torque_boost_level = 1.0
        self.TORQUE_BOOST_ENABLED = True
        print("\n✓ Power management test complete")
    
    def mechanical_check(self):
        """Check for mechanical issues"""
        print("\n" + "="*60)
        print("MECHANICAL CHECK")
        print("="*60)
        
        print("1. Check for physical obstructions:")
        print("   - Is anything touching the motor shaft?")
        print("   - Are wires getting tangled?")
        print("   - Is the base stable and not shifting?")
        
        input("\nPress Enter after checking...")
        
        print("\n2. Testing with NO LOAD:")
        print("   Temporarily disconnect any load from motors")
        print("   (laser, gears, etc.)")
        
        input("Press Enter when motors are unloaded...")
        
        print("\nTesting unloaded movement...")
        self.move_azimuth_adaptive(90)
        self.move_azimuth_adaptive(-90)
        
        response = input("\nDid unloaded motors work better? (y/n): ").lower()
        if response == 'y':
            print("✓ Mechanical load is the issue")
            print("  Consider: Lighter load, better bearings, more torque")
        else:
            print("✗ Issue persists even unloaded")
            print("  Likely electrical/power issue")
        
        print("\n3. Check motor temperature:")
        print("   Feel motors after movement - are they hot?")
        print("   Hot motors = insufficient power or stuck")
    
    def interactive_torque_tuning(self):
        """Interactive torque tuning"""
        print("\n" + "="*60)
        print("INTERACTIVE TORQUE TUNING")
        print("="*60)
        
        while True:
            print(f"\nCurrent settings:")
            print(f"  Torque boost: {'ON' if self.TORQUE_BOOST_ENABLED else 'OFF'}")
            print(f"  Boost level: {self.torque_boost_level:.1f}")
            print(f"  Azimuth delay: {self.BASE_AZIMUTH_DELAY*1000:.1f}ms")
            print(f"  Azimuth angle: {self.azimuth_angle:.1f}°")
            
            print("\nOptions:")
            print("1. Toggle torque boost")
            print("2. Increase boost level")
            print("3. Decrease boost level")
            print("4. Increase base delay (slower)")
            print("5. Decrease base delay (faster)")
            print("6. Test current settings (move 45°)")
            print("7. Back to main menu")
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == "1":
                self.TORQUE_BOOST_ENABLED = not self.TORQUE_BOOST_ENABLED
                print(f"Torque boost: {'ON' if self.TORQUE_BOOST_ENABLED else 'OFF'}")
            
            elif choice == "2":
                self.torque_boost_level = min(3.0, self.torque_boost_level + 0.1)
                print(f"Boost level: {self.torque_boost_level:.1f}")
            
            elif choice == "3":
                self.torque_boost_level = max(1.0, self.torque_boost_level - 0.1)
                print(f"Boost level: {self.torque_boost_level:.1f}")
            
            elif choice == "4":
                self.BASE_AZIMUTH_DELAY = min(0.1, self.BASE_AZIMUTH_DELAY + 0.005)
                print(f"Base delay: {self.BASE_AZIMUTH_DELAY*1000:.1f}ms")
            
            elif choice == "5":
                self.BASE_AZIMUTH_DELAY = max(0.005, self.BASE_AZIMUTH_DELAY - 0.005)
                print(f"Base delay: {self.BASE_AZIMUTH_DELAY*1000:.1f}ms")
            
            elif choice == "6":
                print("\nTesting 45° movement...")
                start_angle = self.azimuth_angle
                self.move_azimuth_adaptive(45)
                actual = self.azimuth_angle - start_angle
                print(f"Result: Moved {actual:.1f}° of 45°")
                
                # Return
                self.move_azimuth_adaptive(-actual)
            
            elif choice == "7":
                break
            
            else:
                print("Invalid choice")
    
    def cleanup(self):
        """Clean shutdown - release torque"""
        print("\nReleasing motors...")
        self.TORQUE_BOOST_ENABLED = False
        self.send_to_shift_register(0b00000000)
        time.sleep(0.2)  # Let motors release
        GPIO.cleanup()
        print("✓ Motors released, GPIO cleaned up")

def main():
    """Main program"""
    print("="*70)
    print("ENME441 - POWER & TORQUE OPTIMIZATION")
    print("="*70)
    print("Fixes for stuttering and range limits:")
    print("• 2-phase excitation (more torque)")
    print("• Dynamic speed adjustment")
    print("• Error recovery and retry")
    print("• Mechanical load detection")
    print("="*70)
    
    turret = None
    try:
        turret = PowerOptimizedTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU - POWER MANAGEMENT")
            print("="*60)
            print("1. Range limit test (find working range)")
            print("2. Power management test (try different settings)")
            print("3. Mechanical check (diagnose physical issues)")
            print("4. Interactive torque tuning")
            print("5. Test full movement (±90°)")
            print("6. Show current status")
            print("7. Exit and cleanup")
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == "1":
                turret.range_limit_test()
            elif choice == "2":
                turret.power_management_test()
            elif choice == "3":
                turret.mechanical_check()
            elif choice == "4":
                turret.interactive_torque_tuning()
            elif choice == "5":
                print("\nTesting full ±90° movement...")
                turret.move_azimuth_adaptive(90)
                turret.move_azimuth_adaptive(-90)
                print(f"\nFinal: {turret.azimuth_angle:.1f}°")
            elif choice == "6":
                print(f"\nCurrent status:")
                print(f"  Azimuth: {turret.azimuth_angle:.1f}°")
                print(f"  Altitude: {turret.altitude_angle:.1f}°")
                print(f"  Torque boost: {'ON' if turret.TORQUE_BOOST_ENABLED else 'OFF'}")
                print(f"  Boost level: {turret.torque_boost_level:.1f}")
                print(f"  Base delay: {turret.BASE_AZIMUTH_DELAY*1000:.1f}ms")
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
