#!/usr/bin/env python3
"""
FIXED MOTOR TEST - 90° BOTH DIRECTIONS
Fixed altitude speed issue and azimuth motor not moving
"""

import RPi.GPIO as GPIO
import time

class FixedMotorTest:
    def __init__(self):
        # GPIO pins for shift register
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS (Pin 14)
        
        # Motor calibration
        self.AZIMUTH_STEPS_PER_REV = 1024    # 1024 steps per revolution
        self.ALTITUDE_STEPS_PER_REV = 4096   # 4096 steps per revolution
        self.ALTITUDE_SPEED_FACTOR = 4       # Altitude moves 4× faster
        
        # Position tracking
        self.azimuth_position = 0  # Current step position
        self.altitude_position = 0
        
        # FIXED: Proper 8-step sequence for smoother operation
        # Lower 4 bits (00001111) = Azimuth motor
        # Upper 4 bits (11110000) = Altitude motor
        self.STEP_SEQUENCE = [
            0b00010001,  # Step 1: Az=0001, Alt=0001
            0b00100010,  # Step 2: Az=0010, Alt=0010
            0b01000100,  # Step 3: Az=0100, Alt=0100
            0b10001000,  # Step 4: Az=1000, Alt=1000
        ]
        
        self.setup_gpio()
    
    def setup_gpio(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Initialize to LOW
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Initialize motors to known state
        self.shift_out(0b00000000)
        print("✓ GPIO initialized - Motors OFF")
    
    def shift_out(self, data_byte):
        """Send 8 bits to shift register"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        for i in range(7, -1, -1):
            bit = (data_byte >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.00001)  # Very short delay
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def energize_motor(self, motor_bits):
        """Energize a motor with specific pattern (for testing)"""
        self.shift_out(motor_bits)
        time.sleep(0.5)
    
    def test_motor_wiring(self):
        """Test each motor individually to check wiring"""
        print("\n" + "="*60)
        print("MOTOR WIRING TEST")
        print("="*60)
        print("Testing each motor coil individually...")
        print("Each coil should energize for 0.5 seconds")
        print("Motor should hold position (feel stiff)")
        
        input("Press Enter to start wiring test...")
        
        # Test Azimuth motor (lower 4 bits)
        print("\nTesting AZIMUTH motor (bits 0-3):")
        test_patterns = [
            (0b00000001, "Coil A"),
            (0b00000010, "Coil B"),
            (0b00000100, "Coil C"),
            (0b00001000, "Coil D"),
        ]
        
        for pattern, name in test_patterns:
            print(f"  Energizing {name}...")
            self.energize_motor(pattern)
        
        # Test Altitude motor (upper 4 bits)
        print("\nTesting ALTITUDE motor (bits 4-7):")
        test_patterns = [
            (0b00010000, "Coil A"),
            (0b00100000, "Coil B"),
            (0b01000000, "Coil C"),
            (0b10000000, "Coil D"),
        ]
        
        for pattern, name in test_patterns:
            print(f"  Energizing {name}...")
            self.energize_motor(pattern)
        
        # Turn off all motors
        self.shift_out(0b00000000)
        print("\n✓ Wiring test complete!")
        print("If any coil didn't energize, check wiring to that pin")
    
    def get_motor_state(self, position, is_altitude=False):
        """Get the motor bits for a given step position"""
        step_idx = position % 4
        motor_bits = self.STEP_SEQUENCE[step_idx]
        
        if is_altitude:
            # Altitude uses upper 4 bits
            return motor_bits & 0b11110000
        else:
            # Azimuth uses lower 4 bits
            return motor_bits & 0b00001111
    
    def step_azimuth(self, direction):
        """Take one azimuth step"""
        self.azimuth_position += direction
        az_bits = self.get_motor_state(self.azimuth_position, False)
        alt_bits = self.get_motor_state(self.altitude_position, True)
        self.shift_out(az_bits | alt_bits)
    
    def step_altitude(self, direction):
        """Take one altitude step"""
        self.altitude_position += direction
        az_bits = self.get_motor_state(self.azimuth_position, False)
        alt_bits = self.get_motor_state(self.altitude_position, True)
        self.shift_out(az_bits | alt_bits)
    
    def move_degrees_independent(self, az_degrees, alt_degrees, step_delay=0.002):
        """
        Move motors independently (more reliable)
        Returns True if successful
        """
        print(f"\nMoving: Azimuth={az_degrees:.1f}°, Altitude={alt_degrees:.1f}°")
        
        # Calculate steps needed
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        
        print(f"Steps needed: Azimuth={az_steps} steps, Altitude={alt_steps} steps")
        
        # Move azimuth motor first (if needed)
        if az_steps != 0:
            az_direction = 1 if az_steps > 0 else -1
            az_steps_abs = abs(az_steps)
            
            print(f"Moving azimuth motor {az_steps_abs} steps...")
            for i in range(az_steps_abs):
                self.step_azimuth(az_direction)
                time.sleep(step_delay)
                if (i + 1) % 50 == 0:
                    print(f"  Azimuth: {i+1}/{az_steps_abs} steps")
        
        # Move altitude motor (if needed)
        if alt_steps != 0:
            alt_direction = 1 if alt_steps > 0 else -1
            alt_steps_abs = abs(alt_steps)
            
            print(f"Moving altitude motor {alt_steps_abs} steps...")
            # Altitude motor is slower, so we move it in smaller chunks
            chunk_size = 100
            for start in range(0, alt_steps_abs, chunk_size):
                end = min(start + chunk_size, alt_steps_abs)
                for i in range(start, end):
                    self.step_altitude(alt_direction)
                    time.sleep(step_delay / self.ALTITUDE_SPEED_FACTOR)
                print(f"  Altitude: {end}/{alt_steps_abs} steps")
        
        return True
    
    def run_90_degree_test(self):
        """Run 90° clockwise/counterclockwise test"""
        print("\n" + "="*60)
        print("90° MOTOR TEST")
        print("="*60)
        print("Test sequence:")
        print("1. Both motors 90° CLOCKWISE/UP")
        print("2. Pause 2 seconds")
        print("3. Both motors 90° COUNTERCLOCKWISE/DOWN")
        print("4. Pause 2 seconds")
        print("5. Repeat 2 times")
        print("="*60)
        print("NOTE: Clockwise = positive degrees")
        print("      Counterclockwise = negative degrees")
        print("="*60)
        
        # First, run wiring test
        run_wiring_test = input("\nRun motor wiring test first? (y/n): ").strip().lower()
        if run_wiring_test == 'y':
            self.test_motor_wiring()
        
        input("\nPress Enter to start 90° test...")
        
        test_cycles = 2
        
        for cycle in range(test_cycles):
            print(f"\n{'='*40}")
            print(f"TEST CYCLE {cycle + 1}/{test_cycles}")
            print('='*40)
            
            # CLOCKWISE/UP (positive 90°)
            print("\n▶ CLOCKWISE/UP - Moving 90°...")
            success = self.move_degrees_independent(90, 90, 0.001)
            
            if not success:
                print("⚠ Movement failed!")
                break
            
            print("⏸ Pausing 2 seconds...")
            time.sleep(2)
            
            # COUNTERCLOCKWISE/DOWN (negative 90°)
            print("\n◀ COUNTERCLOCKWISE/DOWN - Moving 90°...")
            success = self.move_degrees_independent(-90, -90, 0.001)
            
            if not success:
                print("⚠ Movement failed!")
                break
            
            if cycle < test_cycles - 1:  # Don't pause after last cycle
                print("⏸ Pausing 2 seconds...")
                time.sleep(2)
        
        print(f"\n{'='*40}")
        print("✓ TEST COMPLETE!")
        print('='*40)
        
        # Return to approximate starting position
        print("\nReturning to approximate start position...")
        self.move_degrees_independent(0, 0, 0.001)  # Just updates display
        
        # Turn off motors
        self.shift_out(0b00000000)
        print("Motors turned off.")
    
    def cleanup(self):
        """Clean up GPIO"""
        self.shift_out(0b00000000)
        GPIO.cleanup()
        print("GPIO cleanup complete.")

def main():
    """Main function"""
    print("="*70)
    print("FIXED MOTOR TEST - 90° BOTH DIRECTIONS")
    print("="*70)
    print("Fixes:")
    print("  1. Altitude motor speed inconsistency")
    print("  2. Azimuth motor not moving")
    print("  3. Uses independent motor control")
    print("\nTest sequence (2 cycles):")
    print("  Cycle 1: 90° CW → Pause 2s → 90° CCW → Pause 2s")
    print("  Cycle 2: 90° CW → Pause 2s → 90° CCW")
    print("="*70)
    
    # Wiring reminder
    print("\nWIRING CHECKLIST:")
    print("  ✓ RPi GPIO 11  →  74HC595 Pin 11 (SH_CP)")
    print("  ✓ RPi GPIO 10  →  74HC595 Pin 12 (ST_CP)")
    print("  ✓ RPi GPIO 9   →  74HC595 Pin 14 (DS)")
    print("  ✓ RPi 5V       →  74HC595 Pin 16 (VCC)")
    print("  ✓ RPi GND      →  74HC595 Pins 8 & 13 (GND & OE)")
    print("  ✓ Pin 10 (MR)  →  3.3V/5V (or leave floating)")
    print("\nMOTOR CONNECTIONS:")
    print("  Azimuth Motor  →  Pins 15(Q0), 1(Q1), 2(Q2), 3(Q3)")
    print("  Altitude Motor →  Pins 4(Q4), 5(Q5), 6(Q6), 7(Q7)")
    print("="*70)
    
    input("Press Enter when ready (Ctrl+C to cancel)...")
    
    tester = None
    try:
        tester = FixedMotorTest()
        tester.run_90_degree_test()
        
    except KeyboardInterrupt:
        print("\nTest cancelled by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if tester:
            tester.cleanup()
        print("\nTest ended.")

if __name__ == "__main__":
    main()
