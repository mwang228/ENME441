#!/usr/bin/env python3
"""
ULTRA SIMPLE MOTOR TEST - Direct GPIO control
"""

import RPi.GPIO as GPIO
import time

# GPIO pins
SHIFT_CLK = 11  # GPIO11 -> 74HC595 Pin 11 (SH_CP)
LATCH_CLK = 10  # GPIO10 -> 74HC595 Pin 12 (ST_CP)
DATA_PIN = 9    # GPIO9  -> 74HC595 Pin 14 (DS)

def setup_gpio():
    """Initialize GPIO pins"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SHIFT_CLK, GPIO.OUT)
    GPIO.setup(LATCH_CLK, GPIO.OUT)
    GPIO.setup(DATA_PIN, GPIO.OUT)
    
    GPIO.output(SHIFT_CLK, GPIO.LOW)
    GPIO.output(LATCH_CLK, GPIO.LOW)
    GPIO.output(DATA_PIN, GPIO.LOW)
    
    print("✓ GPIO pins initialized")

def shift_out_simple(data_byte):
    """Send 8 bits to shift register - SIMPLIFIED"""
    # Latch low to start
    GPIO.output(LATCH_CLK, GPIO.LOW)
    
    # Send bits
    for i in range(8):
        # Get bit (starting with MSB)
        bit = (data_byte >> (7 - i)) & 0x01
        
        # Set data pin
        GPIO.output(DATA_PIN, bit)
        
        # Pulse clock
        GPIO.output(SHIFT_CLK, GPIO.HIGH)
        time.sleep(0.0001)  # 100µs
        GPIO.output(SHIFT_CLK, GPIO.LOW)
        time.sleep(0.0001)
    
    # Latch high to update outputs
    GPIO.output(LATCH_CLK, GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(LATCH_CLK, GPIO.LOW)
    
    print(f"Sent to shift register: {data_byte:08b} (0x{data_byte:02X})")

def test_power():
    """Test if shift register is getting power"""
    print("\n" + "="*60)
    print("POWER TEST")
    print("="*60)
    print("Testing shift register connections...")
    print("Make sure:")
    print("  1. Pin 16 (VCC) connected to 3.3V or 5V")
    print("  2. Pin 8 (GND) connected to GND")
    print("  3. Pin 13 (OE) connected to GND")
    print("  4. Pin 10 (MR) NOT connected to GND (should be HIGH)")
    input("\nPress Enter to continue...")

def test_individual_coils():
    """Test each motor coil individually"""
    print("\n" + "="*60)
    print("INDIVIDUAL COIL TEST")
    print("="*60)
    print("Testing each motor coil for 1 second")
    print("Motor should get stiff/hold position when coil is energized")
    
    # Test patterns for each coil
    test_patterns = [
        (0b00000001, "Azimuth Coil A (Pin 15)"),
        (0b00000010, "Azimuth Coil B (Pin 1)"),
        (0b00000100, "Azimuth Coil C (Pin 2)"),
        (0b00001000, "Azimuth Coil D (Pin 3)"),
        (0b00010000, "Altitude Coil A (Pin 4)"),
        (0b00100000, "Altitude Coil B (Pin 5)"),
        (0b01000000, "Altitude Coil C (Pin 6)"),
        (0b10000000, "Altitude Coil D (Pin 7)"),
    ]
    
    for pattern, description in test_patterns:
        print(f"\nTesting: {description}")
        print(f"  Pattern: {pattern:08b}")
        input("  Press Enter to energize coil for 1 second...")
        
        shift_out_simple(pattern)
        time.sleep(1)
        shift_out_simple(0b00000000)  # Turn off
        time.sleep(0.5)
    
    print("\n✓ All coils tested")

def test_simple_sequence():
    """Test a simple stepping sequence"""
    print("\n" + "="*60)
    print("SIMPLE STEPPING SEQUENCE")
    print("="*60)
    
    # Simple 4-step sequence
    sequence = [
        0b00010001,  # Step 1
        0b00100010,  # Step 2
        0b01000100,  # Step 3
        0b10001000,  # Step 4
    ]
    
    steps = 20  # Number of steps to take
    
    print(f"Testing {steps} steps forward, then {steps} steps back")
    print("Both motors should rotate slightly")
    
    direction = input("\nTest which motor? (a=azimuth, b=altitude, b=both): ").strip().lower()
    
    if direction == 'a':
        # Azimuth only
        sequence = [s & 0b00001111 for s in sequence]
        print("Testing AZIMUTH motor only")
    elif direction == 'b':
        # Altitude only
        sequence = [s & 0b11110000 for s in sequence]
        print("Testing ALTITUDE motor only")
    else:
        print("Testing BOTH motors")
    
    input("Press Enter to start forward steps...")
    
    print("\nMoving FORWARD...")
    for step in range(steps):
        step_pattern = sequence[step % 4]
        print(f"  Step {step + 1}/{steps}: {step_pattern:08b}")
        shift_out_simple(step_pattern)
        time.sleep(0.1)  # 100ms between steps
    
    input(f"\n{steps} forward steps complete. Press Enter for backward steps...")
    
    print("\nMoving BACKWARD...")
    for step in range(steps):
        # Go through sequence in reverse
        step_pattern = sequence[(steps - step - 1) % 4]
        print(f"  Step {step + 1}/{steps}: {step_pattern:08b}")
        shift_out_simple(step_pattern)
        time.sleep(0.1)
    
    shift_out_simple(0b00000000)
    print("\n✓ Sequence test complete")

def test_rapid_movement():
    """Test rapid movement to see if motors vibrate"""
    print("\n" + "="*60)
    print("RAPID MOVEMENT TEST")
    print("="*60)
    print("Testing rapid stepping - motors should vibrate/hum")
    
    sequence = [
        0b00010001,  # Step 1
        0b00100010,  # Step 2  
        0b01000100,  # Step 3
        0b10001000,  # Step 4
    ]
    
    input("Press Enter to start rapid stepping (10Hz)...")
    
    print("Rapid stepping for 3 seconds...")
    start_time = time.time()
    step_count = 0
    
    while time.time() - start_time < 3:
        step_pattern = sequence[step_count % 4]
        shift_out_simple(step_pattern)
        step_count += 1
        time.sleep(0.025)  # 40Hz
    
    shift_out_simple(0b00000000)
    print(f"✓ Rapid test complete - {step_count} steps")

def check_wiring():
    """Check wiring connections"""
    print("\n" + "="*60)
    print("WIRING CHECKLIST")
    print("="*60)
    print("CRITICAL CONNECTIONS:")
    print("  ✓ Raspberry Pi GPIO 11  →  74HC595 Pin 11 (SH_CP)")
    print("  ✓ Raspberry Pi GPIO 10  →  74HC595 Pin 12 (ST_CP)")
    print("  ✓ Raspberry Pi GPIO 9   →  74HC595 Pin 14 (DS)")
    print("  ✓ Raspberry Pi 3.3V/5V  →  74HC595 Pin 16 (VCC)")
    print("  ✓ Raspberry Pi GND      →  74HC595 Pin 8 (GND)")
    print("  ✓ Raspberry Pi GND      →  74HC595 Pin 13 (OE)  [MUST BE GROUNDED]")
    print("  ✓ 3.3V/5V              →  74HC595 Pin 10 (MR)  [OR leave floating]")
    print("\nMOTOR CONNECTIONS:")
    print("  Azimuth Motor (4 wires) →  Pins 15, 1, 2, 3")
    print("  Altitude Motor (4 wires) → Pins 4, 5, 6, 7")
    print("\nPOWER:")
    print("  ✓ Motors need separate power (not from Pi)")
    print("  ✓ Motor power supply GND connected to Pi GND")
    print("="*60)
    
    input("\nPress Enter when you've checked all connections...")

def main():
    """Main function"""
    print("="*70)
    print("ULTRA SIMPLE MOTOR TEST - DIRECT DEBUGGING")
    print("="*70)
    print("This test removes all complex calculations")
    print("and tests the most basic motor functions")
    print("="*70)
    
    try:
        setup_gpio()
        
        while True:
            print("\n" + "="*70)
            print("DEBUG MENU")
            print("="*70)
            print("1. Check wiring checklist")
            print("2. Test power/connections")
            print("3. Test individual coils (MOST IMPORTANT)")
            print("4. Test simple stepping sequence")
            print("5. Test rapid movement")
            print("6. Exit and cleanup")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                check_wiring()
            elif choice == "2":
                test_power()
            elif choice == "3":
                test_individual_coils()
            elif choice == "4":
                test_simple_sequence()
            elif choice == "5":
                test_rapid_movement()
            elif choice == "6":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        shift_out_simple(0b00000000)
        GPIO.cleanup()
        print("\n✓ GPIO cleanup complete")
        print("Test ended")

if __name__ == "__main__":
    main()
