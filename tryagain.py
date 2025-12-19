#!/usr/bin/env python3
"""
ENME441 - DIAGNOSTIC ALTITUDE MOTOR TEST
Figure out why altitude motor isn't working
"""

import RPi.GPIO as GPIO
import time

class AltitudeDiagnostic:
    def __init__(self):
        print("="*70)
        print("ALTITUDE MOTOR DIAGNOSTIC")
        print("="*70)
        print("Testing ONLY the altitude motor")
        print("Wiring: Pins 4,5,6,7 -> Altitude motor")
        print("="*70)
        
        # GPIO Pins
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        
        # Altitude sequence ONLY
        self.ALTITUDE_SEQUENCE = [
            0b00010000,  # Coil A (Pin 4)
            0b00100000,  # Coil B (Pin 5)
            0b01000000,  # Coil C (Pin 6)
            0b10000000,  # Coil D (Pin 7)
        ]
        
        self.current_step = 0
        
        # Initialize
        self.setup_gpio()
        
        print("✓ Diagnostic ready")
        print("="*70)
    
    def setup_gpio(self):
        """Simple GPIO setup"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        # Start with motor OFF
        self.send_to_shift_register(0b00000000)
    
    def send_to_shift_register(self, data):
        """Send data to shift register"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.000001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
    
    def energize_coil(self, coil_name, pattern):
        """Energize a single coil for testing"""
        print(f"\nEnergizing {coil_name}...")
        print(f"Pattern: {pattern:08b}")
        self.send_to_shift_register(pattern)
        
        input("Press Enter when ready to feel motor...")
        
        # Hold for 3 seconds
        print("Motor should be energized NOW")
        print("Feel if the motor is holding position")
        time.sleep(3)
        
        # Turn off
        self.send_to_shift_register(0b00000000)
        print("Motor off")
        time.sleep(1)
    
    def test_individual_coils(self):
        """Test each coil individually"""
        print("\n" + "="*60)
        print("TEST INDIVIDUAL COILS")
        print("="*60)
        print("Testing each coil (pins 4,5,6,7) one at a time")
        print("You should feel each coil PULL when energized")
        print("="*60)
        
        coils = [
            ("Coil A (Pin 4)", 0b00010000),
            ("Coil B (Pin 5)", 0b00100000),
            ("Coil C (Pin 6)", 0b01000000),
            ("Coil D (Pin 7)", 0b10000000),
        ]
        
        for name, pattern in coils:
            self.energize_coil(name, pattern)
            
            response = input(f"\nDid you feel {name} pull? (y/n): ").lower()
            if response == 'y':
                print(f"✓ {name} works")
            else:
                print(f"✗ {name} not working")
    
    def test_sequence_slow(self):
        """Test sequence VERY SLOWLY"""
        print("\n" + "="*60)
        print("TEST SEQUENCE SLOWLY")
        print("="*60)
        print("Running through sequence with 2 second delays")
        print("Motor should step through 4 positions")
        print("="*60)
        
        input("Press Enter to start slow sequence...")
        
        for i in range(8):  # Two full cycles
            pattern = self.ALTITUDE_SEQUENCE[i % 4]
            self.current_step = i % 4
            
            print(f"\nStep {i+1}: Pattern {pattern:08b}")
            print(f"  Coil {self.current_step} energized")
            
            self.send_to_shift_register(pattern)
            time.sleep(2.0)  # Very slow
        
        # Turn off
        self.send_to_shift_register(0b00000000)
        print("\n✓ Sequence test complete")
    
    def test_with_different_voltage(self):
        """Test if voltage is the issue"""
        print("\n" + "="*60)
        print("TEST VOLTAGE ISSUE")
        print("="*60)
        print("If motor doesn't move, try:")
        print("1. Use SEPARATE 5V power for motors")
        print("2. Add 100µF capacitor to motor power")
        print("3. Check all connections are tight")
        print("="*60)
        
        print("\nTesting with 2-phase excitation (more torque):")
        two_phase_seq = [
            0b00010000 | 0b00100000,  # A + B
            0b00100000 | 0b01000000,  # B + C
            0b01000000 | 0b10000000,  # C + D
            0b10000000 | 0b00010000,  # D + A
        ]
        
        input("Press Enter to test 2-phase sequence...")
        
        for i in range(8):
            pattern = two_phase_seq[i % 4]
            print(f"Step {i+1}: {pattern:08b} (2 coils energized)")
            self.send_to_shift_register(pattern)
            time.sleep(1.0)
        
        self.send_to_shift_register(0b00000000)
    
    def quick_hardware_check(self):
        """Quick hardware troubleshooting"""
        print("\n" + "="*60)
        print("HARDWARE CHECKLIST")
        print("="*60)
        print("1. POWER:")
        print("   - Shift register Pin 16 (VCC) connected to 5V")
        print("   - Shift register Pin 8 (GND) connected to GND")
        print("   - Shift register Pin 13 (OE) connected to GND")
        print("")
        print("2. MOTOR CONNECTIONS:")
        print("   - Altitude motor connected to pins 4,5,6,7")
        print("   - No loose wires")
        print("   - Motor coils are paired correctly")
        print("")
        print("3. QUICK TEST:")
        print("   - Connect ONE motor coil directly to 5V and GND")
        print("   - Motor should hold firmly in place")
        print("")
        
        input("Press Enter after checking...")
    
    def test_simple_movement(self):
        """Very simple movement test"""
        print("\n" + "="*60)
        print("SIMPLE MOVEMENT TEST")
        print("="*60)
        
        print("Trying to move 4 steps...")
        
        for i in range(4):
            print(f"\nStep {i+1}:")
            pattern = self.ALTITUDE_SEQUENCE[i]
            print(f"  Pattern: {pattern:08b}")
            
            self.send_to_shift_register(pattern)
            
            # Ask user
            moved = input("  Did motor move/click? (y/n): ").lower()
            if moved == 'y':
                print("  ✓ Motor moved")
            else:
                print("  ✗ No movement")
            
            time.sleep(0.5)
        
        self.send_to_shift_register(0b00000000)
    
    def cleanup(self):
        """Cleanup"""
        print("\nCleaning up...")
        self.send_to_shift_register(0b00000000)
        time.sleep(0.1)
        GPIO.cleanup()
        print("✓ Cleanup complete")

def main():
    """Main diagnostic program"""
    print("="*70)
    print("ALTITUDE MOTOR DIAGNOSTIC")
    print("="*70)
    print("This will help find WHY altitude motor isn't working")
    print("="*70)
    
    diag = AltitudeDiagnostic()
    
    try:
        while True:
            print("\n" + "="*60)
            print("DIAGNOSTIC MENU")
            print("="*60)
            print("1. Test individual coils (most important)")
            print("2. Test sequence slowly")
            print("3. Test with 2-phase excitation")
            print("4. Simple movement test")
            print("5. Hardware checklist")
            print("6. Exit")
            
            choice = input("\nChoice (1-6): ").strip()
            
            if choice == "1":
                diag.test_individual_coils()
            elif choice == "2":
                diag.test_sequence_slow()
            elif choice == "3":
                diag.test_with_different_voltage()
            elif choice == "4":
                diag.test_simple_movement()
            elif choice == "5":
                diag.quick_hardware_check()
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
        diag.cleanup()
        print("\nDiagnostic complete")

if __name__ == "__main__":
    main()
