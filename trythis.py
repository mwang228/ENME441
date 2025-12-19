#!/usr/bin/env python3
"""
ENME441 - FINAL CALIBRATION & ALTITUDE FIX
"""

import RPi.GPIO as GPIO
import time
import json
import os

class FinalTurret:
    def __init__(self):
        print("="*70)
        print("ENME441 - FINAL CALIBRATION & FIXES")
        print("="*70)
        print("STATUS:")
        print("• Azimuth: ~2400 steps/rev (needs fine-tuning)")
        print("• Altitude: Direction issues (needs wiring/sequence fix)")
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        
        # Configuration
        self.CONFIG_FILE = "final_config.json"
        
        # Start with known values
        self.AZIMUTH_STEPS_PER_REV = 2400
        self.ALTITUDE_STEPS_PER_REV = 450  # Half of 900 for double movement
        
        self.load_config()
        
        print(f"Loaded configuration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/rev")
        
        # Current state
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # **AZIMUTH: Standard sequence (works)**
        self.AZIMUTH_SEQUENCE = [0b00000001, 0b00000010, 0b00000100, 0b00001000]
        
        # **ALTITUDE: Multiple sequence options to test**
        self.ALT_SEQUENCES = {
            "Standard": [0b00010000, 0b00100000, 0b01000000, 0b10000000],
            "Reversed": [0b10000000, 0b01000000, 0b00100000, 0b00010000],
            "SwapAB":   [0b00100000, 0b00010000, 0b01000000, 0b10000000],  # A↔B swapped
            "SwapCD":   [0b00010000, 0b00100000, 0b10000000, 0b01000000],  # C↔D swapped
            "Cross1":   [0b00010000, 0b01000000, 0b00100000, 0b10000000],  # A→C→B→D
            "Cross2":   [0b00100000, 0b10000000, 0b00010000, 0b01000000],  # B→D→A→C
        }
        
        # Start with standard
        self.current_alt_sequence = self.ALT_SEQUENCES["Standard"]
        self.alt_sequence_name = "Standard"
        
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Timing
        self.AZIMUTH_STEP_DELAY = 0.015
        self.ALTITUDE_STEP_DELAY = 0.025
        
        # Initialize
        self.setup_gpio()
        
        print(f"\n✓ System initialized")
        print(f"Azimuth: {self.azimuth_angle:.1f}°, Altitude: {self.altitude_angle:.1f}°")
        print("="*70)
    
    def load_config(self):
        """Load configuration"""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    self.AZIMUTH_STEPS_PER_REV = config.get('azimuth_steps_per_rev', 2400)
                    self.ALTITUDE_STEPS_PER_REV = config.get('altitude_steps_per_rev', 450)
                print("✓ Configuration loaded")
        except:
            print("Using default values")
    
    def save_config(self):
        """Save configuration"""
        try:
            config = {
                'azimuth_steps_per_rev': self.AZIMUTH_STEPS_PER_REV,
                'altitude_steps_per_rev': self.ALTITUDE_STEPS_PER_REV
            }
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f, indent=2)
            print("✓ Configuration saved")
        except:
            print("Warning: Could not save")
    
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
        """Send data to shift register"""
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
        alt_pattern = self.current_alt_sequence[self.altitude_seq_pos]
        self.send_to_shift_register(az_pattern | alt_pattern)
    
    def step_azimuth(self, direction):
        """Step azimuth motor"""
        self.azimuth_seq_pos = (self.azimuth_seq_pos + direction) % 4
        self.azimuth_steps += direction
        self.azimuth_angle = (self.azimuth_steps / self.AZIMUTH_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def step_altitude(self, direction):
        """Step altitude motor with current sequence"""
        self.altitude_seq_pos = (self.altitude_seq_pos + direction) % 4
        self.altitude_steps += direction
        self.altitude_angle = (self.altitude_steps / self.ALTITUDE_STEPS_PER_REV) * 360.0
        self.update_motors()
    
    def fine_calibrate_azimuth(self):
        """Fine-tune azimuth calibration"""
        print("\n" + "="*60)
        print("FINE-CALIBRATE AZIMUTH")
        print("="*60)
        
        print("Azimuth overshot 90° with 2400 steps/rev")
        print("We need to find the exact value.")
        
        # Test 90° movement
        print("\nTesting 90° movement...")
        start_angle = self.azimuth_angle
        steps = int((90 / 360) * self.AZIMUTH_STEPS_PER_REV)
        
        print(f"Taking {steps} steps (based on {self.AZIMUTH_STEPS_PER_REV} steps/rev)")
        
        for i in range(steps):
            self.step_azimuth(1)
            time.sleep(self.AZIMUTH_STEP_DELAY)
        
        actual_angle = self.azimuth_angle - start_angle
        print(f"\nResult: Moved {actual_angle:.1f}° (Target: 90.0°)")
        
        if actual_angle > 0:
            # Calculate correction
            correction = 90.0 / actual_angle
            new_steps = int(self.AZIMUTH_STEPS_PER_REV * correction)
            
            print(f"\nCalibration calculation:")
            print(f"  Target: 90.0°")
            print(f"  Actual: {actual_angle:.1f}°")
            print(f"  Correction factor: {correction:.3f}")
            print(f"  Old: {self.AZIMUTH_STEPS_PER_REV} steps/rev")
            print(f"  New: {new_steps} steps/rev")
            
            # Update
            self.AZIMUTH_STEPS_PER_REV = new_steps
            self.save_config()
            
            # Return and test
            print(f"\nTesting new calibration...")
            self.move_azimuth(-actual_angle)
            self.fine_calibrate_azimuth()  # Test again
        
        else:
            print("Error: No movement detected")
    
    def move_azimuth(self, degrees):
        """Move azimuth"""
        steps = int((degrees / 360) * self.AZIMUTH_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        for i in range(steps):
            self.step_azimuth(direction)
            time.sleep(self.AZIMUTH_STEP_DELAY)
    
    def test_altitude_sequences(self):
        """Test all possible altitude sequences"""
        print("\n" + "="*60)
        print("TEST ALL ALTITUDE SEQUENCES")
        print("="*60)
        
        print("Testing each sequence with small movement...")
        
        working_sequences = []
        
        for name, sequence in self.ALT_SEQUENCES.items():
            print(f"\nTesting: {name}")
            self.current_alt_sequence = sequence
            self.alt_sequence_name = name
            
            # Reset to known state
            self.altitude_seq_pos = 0
            self.update_motors()
            time.sleep(0.5)
            
            # Test small movement in both directions
            try:
                print("  Testing +15° (UP)...")
                start_up = self.altitude_angle
                
                # Calculate steps for 15°
                steps = int((15 / 360) * self.ALTITUDE_STEPS_PER_REV)
                for i in range(steps):
                    self.step_altitude(1)
                    time.sleep(self.ALTITUDE_STEP_DELAY)
                
                up_moved = self.altitude_angle - start_up
                print(f"    Moved: {up_moved:.1f}°")
                
                print("  Testing -15° (DOWN)...")
                start_down = self.altitude_angle
                for i in range(steps):
                    self.step_altitude(-1)
                    time.sleep(self.ALTITUDE_STEP_DELAY)
                
                down_moved = start_down - self.altitude_angle
                print(f"    Moved: {down_moved:.1f}°")
                
                # Check if both directions worked
                up_ok = abs(up_moved - 15) < 5
                down_ok = abs(down_moved - 15) < 5
                
                if up_ok and down_ok:
                    print(f"  ✓ Sequence works in BOTH directions!")
                    working_sequences.append((name, sequence))
                elif up_ok:
                    print(f"  ⚠️ Works only UP")
                elif down_ok:
                    print(f"  ⚠️ Works only DOWN")
                else:
                    print(f"  ✗ Doesn't work")
            
            except Exception as e:
                print(f"  ✗ Error: {e}")
        
        print("\n" + "="*60)
        print("RESULTS:")
        print("="*60)
        
        if working_sequences:
            print("Working sequences found:")
            for name, seq in working_sequences:
                print(f"  • {name}")
            
            # Use the first working sequence
            best_name, best_seq = working_sequences[0]
            self.current_alt_sequence = best_seq
            self.alt_sequence_name = best_name
            print(f"\n✓ Using sequence: {best_name}")
        else:
            print("✗ No sequence worked in both directions")
            print("Try swapping physical wires on the altitude motor")
    
    def diagnose_altitude_wiring(self):
        """Diagnose altitude motor wiring"""
        print("\n" + "="*60)
        print("ALTITUDE WIRING DIAGNOSIS")
        print("="*60)
        
        print("Testing each coil individually...")
        print("(You should feel each coil pull)")
        
        # Test pins 4,5,6,7 individually
        test_patterns = [
            ("Pin 4 (Coil A)", 0b00010000),
            ("Pin 5 (Coil B)", 0b00100000),
            ("Pin 6 (Coil C)", 0b01000000),
            ("Pin 7 (Coil D)", 0b10000000),
        ]
        
        for name, pattern in test_patterns:
            input(f"\nPress Enter to test {name}...")
            print(f"Energizing {name}...")
            self.send_to_shift_register(pattern)
            time.sleep(2)
            self.send_to_shift_register(0b00000000)
            time.sleep(0.5)
        
        print("\nWIRING FIX OPTIONS:")
        print("1. If coils energize but motor doesn't turn: Wrong sequence order")
        print("2. If some coils don't energize: Bad connection")
        print("3. Try swapping adjacent coil pairs:")
        print("   Option A: Swap pins 4↔5 (A↔B)")
        print("   Option B: Swap pins 6↔7 (C↔D)")
        print("   Option C: Swap pins 4↔6 and 5↔7 (A↔C, B↔D)")
    
    def quick_fix_altitude(self):
        """Quick fix attempt for altitude"""
        print("\n" + "="*60)
        print("QUICK ALTITUDE FIX ATTEMPT")
        print("="*60)
        
        print("Trying common fixes:")
        
        # Try 2-phase excitation (more torque)
        print("\n1. Trying 2-phase excitation...")
        two_phase_seq = [
            0b00010000 | 0b00100000,  # A+B
            0b00100000 | 0b01000000,  # B+C
            0b01000000 | 0b10000000,  # C+D
            0b10000000 | 0b00010000,  # D+A
        ]
        
        self.current_alt_sequence = two_phase_seq
        self.alt_sequence_name = "2-Phase"
        
        print("Testing +20° with 2-phase...")
        start = self.altitude_angle
        steps = int((20 / 360) * self.ALTITUDE_STEPS_PER_REV)
        
        for i in range(steps):
            self.step_altitude(1)
            time.sleep(self.ALTITUDE_STEP_DELAY * 1.5)  # Slower for more torque
        
        moved = self.altitude_angle - start
        print(f"Result: {moved:.1f}° of 20°")
        
        if moved < 10:
            print("2-phase didn't help")
            # Return to original sequence
            self.current_alt_sequence = self.ALT_SEQUENCES["Standard"]
            self.alt_sequence_name = "Standard"
        
        # Return to start
        self.move_altitude(-moved)
    
    def move_altitude(self, degrees):
        """Move altitude"""
        steps = int((degrees / 360) * self.ALTITUDE_STEPS_PER_REV)
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        for i in range(steps):
            self.step_altitude(direction)
            time.sleep(self.ALTITUDE_STEP_DELAY)
    
    def competition_test(self):
        """Final competition-style test"""
        print("\n" + "="*60)
        print("FINAL COMPETITION TEST")
        print("="*60)
        
        print("Testing movements needed for competition...")
        
        test_sequence = [
            ("Aim at target 45° right, 30° up", 45, 30),
            ("Aim at target 90° left, 45° up", -90, 45),
            ("Return to center", -self.azimuth_angle, -self.altitude_angle),
            ("Test precision: 10° increments", 10, 10),
            ("Return to center", -self.azimuth_angle, -self.altitude_angle),
        ]
        
        for name, az_move, alt_move in test_sequence:
            print(f"\n{name}:")
            
            if az_move != 0:
                print(f"  Azimuth: {az_move:+.1f}°")
                self.move_azimuth(az_move)
            
            if alt_move != 0:
                print(f"  Altitude: {alt_move:+.1f}°")
                self.move_altitude(alt_move)
            
            time.sleep(0.5)
            print(f"  Current: ({self.azimuth_angle:.1f}°, {self.altitude_angle:.1f}°)")
        
        print(f"\n✓ Test complete!")
        print(f"Final error from center: ({self.azimuth_angle:.1f}°, {self.altitude_angle:.1f}°)")
    
    def interactive_debug(self):
        """Interactive debugging"""
        print("\n" + "="*60)
        print("INTERACTIVE DEBUG")
        print("="*60)
        
        while True:
            print(f"\nCurrent:")
            print(f"  Azimuth: {self.azimuth_angle:.1f}°")
            print(f"  Altitude: {self.altitude_angle:.1f}°")
            print(f"  Altitude sequence: {self.alt_sequence_name}")
            
            print("\nDebug options:")
            print("1. Test azimuth small movement")
            print("2. Test altitude small movement")
            print("3. Change altitude sequence")
            print("4. Zero both motors")
            print("5. Back to main menu")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == "1":
                try:
                    deg = float(input("Azimuth degrees: "))
                    self.move_azimuth(deg)
                except:
                    print("Invalid input")
            
            elif choice == "2":
                try:
                    deg = float(input("Altitude degrees: "))
                    self.move_altitude(deg)
                except:
                    print("Invalid input")
            
            elif choice == "3":
                print("\nAvailable sequences:")
                for name in self.ALT_SEQUENCES.keys():
                    print(f"  {name}" + (" *" if name == self.alt_sequence_name else ""))
                
                new_name = input("\nEnter sequence name: ").strip()
                if new_name in self.ALT_SEQUENCES:
                    self.current_alt_sequence = self.ALT_SEQUENCES[new_name]
                    self.alt_sequence_name = new_name
                    print(f"✓ Changed to {new_name}")
                else:
                    print("Invalid sequence name")
            
            elif choice == "4":
                print("\nZeroing...")
                self.move_azimuth(-self.azimuth_angle)
                self.move_altitude(-self.altitude_angle)
                print(f"✓ At (0°, 0°)")
            
            elif choice == "5":
                break
            
            else:
                print("Invalid choice")
    
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
    print("ENME441 - FINAL CALIBRATION & FIXES")
    print("="*70)
    print("TASKS:")
    print("1. Fine-tune azimuth steps/rev (overshot 90°)")
    print("2. Fix altitude direction issues")
    print("="*70)
    
    turret = None
    try:
        turret = FinalTurret()
        
        while True:
            print("\n" + "="*60)
            print("MAIN MENU - FINAL FIXES")
            print("="*60)
            print("1. Fine-calibrate azimuth (overshot 90°)")
            print("2. Test all altitude sequences")
            print("3. Diagnose altitude wiring")
            print("4. Quick altitude fix attempt")
            print("5. Competition test")
            print("6. Interactive debug")
            print("7. Show current status")
            print("8. Exit and cleanup")
            
            choice = input("\nEnter choice (1-8): ").strip()
            
            if choice == "1":
                turret.fine_calibrate_azimuth()
            elif choice == "2":
                turret.test_altitude_sequences()
            elif choice == "3":
                turret.diagnose_altitude_wiring()
            elif choice == "4":
                turret.quick_fix_altitude()
            elif choice == "5":
                turret.competition_test()
            elif choice == "6":
                turret.interactive_debug()
            elif choice == "7":
                print(f"\nCurrent status:")
                print(f"  Azimuth steps/rev: {turret.AZIMUTH_STEPS_PER_REV}")
                print(f"  Altitude steps/rev: {turret.ALTITUDE_STEPS_PER_REV}")
                print(f"  Altitude sequence: {turret.alt_sequence_name}")
                print(f"  Azimuth angle: {turret.azimuth_angle:.1f}°")
                print(f"  Altitude angle: {turret.altitude_angle:.1f}°")
            elif choice == "8":
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
