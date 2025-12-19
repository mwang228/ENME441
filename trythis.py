#!/usr/bin/env python3
"""
ENME441 Laser Turret - Azimuth Motor Debug
Focus on getting the azimuth motor working
"""

import RPi.GPIO as GPIO
import time

class AzimuthDebug:
    def __init__(self):
        print("="*70)
        print("AZIMUTH MOTOR DEBUG TOOL")
        print("="*70)
        
        # GPIO Pins for Shift Register
        self.SHIFT_CLK = 11  # GPIO11 -> 74HC595 Pin 11 (SH_CP)
        self.LATCH_CLK = 10  # GPIO10 -> 74HC595 Pin 12 (ST_CP)
        self.DATA_PIN = 9    # GPIO9  -> 74HC595 Pin 14 (DS)
        
        # Steps per revolution (use your calibrated value for altitude)
        self.ALTITUDE_STEPS_PER_REV = 2400  # From your calibration
        
        # Current positions
        self.azimuth_steps = 0
        self.altitude_steps = 0
        self.azimuth_angle = 0.0
        self.altitude_angle = 0.0
        
        # TEST DIFFERENT STEP SEQUENCES:
        # Try each of these sequences for the azimuth motor
        
        # Sequence 1: Standard 4-step (wave drive)
        self.SEQ_4_STEP = [
            0b00000001,  # Step 1: Coil A only (Q0)
            0b00000010,  # Step 2: Coil B only (Q1)
            0b00000100,  # Step 3: Coil C only (Q2)
            0b00001000,  # Step 4: Coil D only (Q3)
        ]
        
        # Sequence 2: 8-step half-step (more torque, smoother)
        self.SEQ_8_STEP = [
            0b00000001,  # Step 1: Coil A
            0b00000011,  # Step 2: Coils A+B
            0b00000010,  # Step 3: Coil B
            0b00000110,  # Step 4: Coils B+C
            0b00000100,  # Step 5: Coil C
            0b00001100,  # Step 6: Coils C+D
            0b00001000,  # Step 7: Coil D
            0b00001001,  # Step 8: Coils D+A
        ]
        
        # Sequence 3: 2-phase excitation (more torque)
        self.SEQ_2_PHASE = [
            0b00000011,  # Coils A+B
            0b00000110,  # Coils B+C
            0b00001100,  # Coils C+D
            0b00001001,  # Coils D+A
        ]
        
        # Start with 4-step sequence
        self.azimuth_sequence = self.SEQ_4_STEP
        self.azimuth_seq_pos = 0
        self.altitude_seq_pos = 0
        
        # Initialize GPIO
        self.setup_gpio()
        
        print("✓ System initialized")
        print("="*70)
    
    def setup_gpio(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        
        # Setup shift register pins
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Initialize pins to LOW
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Turn off all motors initially
        self.send_to_shift_register(0b00000000)
        
        print("✓ GPIO pins initialized")
        print("✓ Shift register ready")
    
    def send_to_shift_register(self, data):
        """Send 8 bits to shift register"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def test_individual_coils(self):
        """Test each coil of azimuth motor individually"""
        print("\n" + "="*60)
        print("INDIVIDUAL COIL TEST")
        print("="*60)
        print("Testing each coil (pins 15, 1, 2, 3) one at a time.")
        print("You should feel/hear each coil energize.")
        print("="*60)
        
        coils = [
            ("Coil A (Pin 15)", 0b00000001),
            ("Coil B (Pin 1)",  0b00000010),
            ("Coil C (Pin 2)",  0b00000100),
            ("Coil D (Pin 3)",  0b00001000),
        ]
        
        for name, pattern in coils:
            input(f"\nPress Enter to test {name}...")
            print(f"Energizing {name}...")
            
            # Turn on just this coil (keep altitude motor off)
            self.send_to_shift_register(pattern)
            time.sleep(2)
            
            # Turn off
            self.send_to_shift_register(0b00000000)
            time.sleep(0.5)
        
        print("\n✓ Individual coil test complete")
        print("Did you feel/hear each coil engage?")
    
    def test_wiring_swap(self):
        """Test if coils are wired in wrong order"""
        print("\n" + "="*60)
        print("WIRING ORDER TEST")
        print("="*60)
        print("If motor vibrates but doesn't turn, the coil order might be wrong.")
        print("Let's test different coil sequences.")
        print("="*60)
        
        # Different possible coil orders
        sequences = {
            "Standard (A,B,C,D)": [0b00000001, 0b00000010, 0b00000100, 0b00001000],
            "Reversed (D,C,B,A)": [0b00001000, 0b00000100, 0b00000010, 0b00000001],
            "Alternating (A,C,B,D)": [0b00000001, 0b00000100, 0b00000010, 0b00001000],
            "Pair (A+B, B+C, C+D, D+A)": [0b00000011, 0b00000110, 0b00001100, 0b00001001],
        }
        
        for name, seq in sequences.items():
            input(f"\nPress Enter to test sequence: {name}...")
            print(f"Testing {name}...")
            
            # Run through 8 steps (2 full cycles)
            for step in range(8):
                pattern = seq[step % 4]
                self.send_to_shift_register(pattern)
                print(f"  Step {step+1}: {pattern:08b}")
                time.sleep(0.5)
            
            # Turn off
            self.send_to_shift_register(0b00000000)
            time.sleep(1)
            
            response = input("Did the motor turn? (y/n): ").lower()
            if response == 'y':
                print(f"✓ Found working sequence: {name}")
                return seq
        
        print("\n⚠️  No sequence worked. Check power and connections.")
        return None
    
    def test_with_altitude_disabled(self):
        """Test azimuth with altitude motor completely disconnected"""
        print("\n" + "="*60)
        print("TEST WITH ALTITUDE DISCONNECTED")
        print("="*60)
        print("Temporarily disconnect the altitude motor wires")
        print("from the shift register (pins 4,5,6,7).")
        print("This eliminates any interference.")
        print("="*60)
        
        input("Press Enter when altitude motor is disconnected...")
        
        # Test a simple sequence
        sequence = [0b00000001, 0b00000010, 0b00000100, 0b00001000]
        
        print("\nTesting basic 4-step sequence...")
        for i in range(12):  # 3 full cycles
            pattern = sequence[i % 4]
            self.send_to_shift_register(pattern)
            print(f"Step {i+1}: {pattern:08b}")
            time.sleep(0.3)
        
        self.send_to_shift_register(0b00000000)
        
        response = input("\nDid the azimuth motor turn? (y/n): ").lower()
        if response == 'y':
            print("✓ Azimuth motor works when altitude is disconnected")
            print("⚠️  Possible issue: Both motors interfering with each other")
            return True
        else:
            print("✗ Azimuth still not turning")
            return False
    
    def check_power_supply(self):
        """Check if power supply is adequate for both motors"""
        print("\n" + "="*60)
        print("POWER SUPPLY CHECK")
        print("="*60)
        print("Two stepper motors can draw significant current.")
        print("Make sure:")
        print("1. Battery is fully charged")
        print("2. Shift register has proper VCC (5V recommended)")
        print("3. Motors are getting enough current")
        print("="*60)
        
        input("Press Enter to test with slower stepping...")
        
        # Test with very slow stepping and reduced voltage
        sequence = [0b00000001, 0b00000010, 0b00000100, 0b00001000]
        
        print("\nTesting with 1 second between steps...")
        for i in range(8):
            pattern = sequence[i % 4]
            self.send_to_shift_register(pattern)
            print(f"Step {i+1}: {pattern:08b}")
            time.sleep(1.0)  # Very slow
        
        self.send_to_shift_register(0b00000000)
        
        response = input("\nDid the motor turn slowly? (y/n): ").lower()
        if response == 'y':
            print("✓ Motor works with slow stepping")
            print("⚠️  Possible power issue - try a beefier power supply")
            return True
        else:
            print("✗ Motor still not moving")
            return False
    
    def test_minimal_setup(self):
        """Test with minimal setup - just one motor"""
        print("\n" + "="*60)
        print("MINIMAL SETUP TEST")
        print("="*60)
        print("Connect ONLY the azimuth motor:")
        print("  Azimuth coil A -> Pin 15")
        print("  Azimuth coil B -> Pin 1")
        print("Leave coils C and D disconnected for now.")
        print("="*60)
        
        input("Press Enter when only coils A and B are connected...")
        
        # Test with just 2 coils (A and B)
        sequence = [0b00000001, 0b00000010]  # Just coil A, then just coil B
        
        print("\nTesting with 2 coils (A and B)...")
        for i in range(8):
            pattern = sequence[i % 2]
            self.send_to_shift_register(pattern)
            print(f"Step {i+1}: {pattern:08b}")
            time.sleep(0.5)
        
        response = input("\nDid the motor move between two positions? (y/n): ").lower()
        if response == 'y':
            print("✓ 2-coil operation works")
            print("Now connect coils C and D and test again")
            return True
        else:
            print("✗ Even 2-coil operation fails")
            print("Check: Power, wiring, motor itself")
            return False
    
    def run_comprehensive_debug(self):
        """Run all debug tests"""
        print("\nStarting comprehensive azimuth motor debugging...")
        
        # 1. Test individual coils
        self.test_individual_coils()
        
        # 2. Test wiring order
        working_seq = self.test_wiring_swap()
        if working_seq:
            print(f"\nUsing sequence: {working_seq}")
            self.azimuth_sequence = working_seq
        
        # 3. Test with altitude disabled
        self.test_with_altitude_disabled()
        
        # 4. Check power supply
        self.check_power_supply()
        
        # 5. Test minimal setup
        self.test_minimal_setup()
        
        print("\n" + "="*70)
        print("DEBUGGING SUMMARY")
        print("="*70)
        print("Based on the tests above:")
        print("1. If individual coils work but motor doesn't turn → Wrong sequence")
        print("2. If motor works with altitude disconnected → Interference issue")
        print("3. If motor works slowly but not fast → Power issue")
        print("4. If nothing works → Check wiring/power/motor")
        print("="*70)
    
    def cleanup(self):
        """Clean up GPIO"""
        print("\nCleaning up...")
        self.send_to_shift_register(0b00000000)
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")

def main():
    """Main debug program"""
    print("="*70)
    print("AZIMUTH MOTOR TROUBLESHOOTING")
    print("="*70)
    print("Altitude motor: WORKING (2400 steps/rev)")
    print("Azimuth motor: NOT TURNING")
    print("="*70)
    print("\nWe'll systematically debug the azimuth motor.")
    print("Follow the prompts and report what happens.")
    
    debug = None
    try:
        debug = AzimuthDebug()
        
        print("\nSelect debug option:")
        print("1. Run comprehensive debug (recommended)")
        print("2. Test individual coils only")
        print("3. Test wiring sequences only")
        print("4. Test with altitude motor disconnected")
        print("5. Exit")
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            debug.run_comprehensive_debug()
        elif choice == "2":
            debug.test_individual_coils()
        elif choice == "3":
            debug.test_wiring_swap()
        elif choice == "4":
            debug.test_with_altitude_disabled()
        elif choice == "5":
            print("Exiting...")
        else:
            print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if debug:
            debug.cleanup()
        print("\nDebug session ended.")

if __name__ == "__main__":
    main()
