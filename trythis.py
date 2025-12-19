#!/usr/bin/env python3
"""
ENME441 - HARDWARE DIAGNOSTIC TOOL
Tests shift register and motor wiring
"""

import RPi.GPIO as GPIO
import time

def hardware_diagnostic():
    print("="*70)
    print("HARDWARE DIAGNOSTIC TOOL")
    print("="*70)
    print("Checking shift register and motor connections...")
    print("="*70)
    
    # GPIO pins
    SHIFT_CLK = 11   # GPIO11 -> Pin 11 (SH_CP)
    LATCH_CLK = 10   # GPIO10 -> Pin 12 (ST_CP)
    DATA_PIN = 9     # GPIO9  -> Pin 14 (DS)
    
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SHIFT_CLK, GPIO.OUT)
    GPIO.setup(LATCH_CLK, GPIO.OUT)
    GPIO.setup(DATA_PIN, GPIO.OUT)
    
    # Start with all LOW
    GPIO.output(SHIFT_CLK, GPIO.LOW)
    GPIO.output(LATCH_CLK, GPIO.LOW)
    GPIO.output(DATA_PIN, GPIO.LOW)
    
    def send_byte(data):
        """Send one byte to shift register"""
        GPIO.output(LATCH_CLK, GPIO.LOW)
        
        for i in range(7, -1, -1):
            bit = (data >> i) & 0x01
            GPIO.output(DATA_PIN, bit)
            
            # Clock pulse
            GPIO.output(SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.0001)
            GPIO.output(SHIFT_CLK, GPIO.LOW)
        
        # Latch to update outputs
        GPIO.output(LATCH_CLK, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(LATCH_CLK, GPIO.LOW)
    
    print("\n1. Testing: Sending ALL ZEROS (0b00000000)")
    print("   All shift register outputs should be OFF")
    print("   All motor lights should be OFF")
    input("   Press Enter to test...")
    
    send_byte(0b00000000)
    time.sleep(2)
    
    print("\n2. Testing: Sending ALL ONES (0b11111111)")
    print("   All shift register outputs should be ON")
    print("   All motor lights should be ON (motors locked)")
    input("   Press Enter to test...")
    
    send_byte(0b11111111)
    time.sleep(2)
    
    print("\n3. Testing: Walk through each output")
    print("   Testing each pin one at a time...")
    
    # Test each output pin individually
    pins = [
        ("Pin 15 (Azimuth Coil A)", 0b00000001),
        ("Pin 1  (Azimuth Coil B)", 0b00000010),
        ("Pin 2  (Azimuth Coil C)", 0b00000100),
        ("Pin 3  (Azimuth Coil D)", 0b00001000),
        ("Pin 4  (Altitude Coil A)", 0b00010000),
        ("Pin 5  (Altitude Coil B)", 0b00100000),
        ("Pin 6  (Altitude Coil C)", 0b01000000),
        ("Pin 7  (Altitude Coil D)", 0b10000000),
    ]
    
    for name, pattern in pins:
        print(f"\n   Testing: {name}")
        print(f"   Pattern: {pattern:08b}")
        send_byte(pattern)
        
        response = input(f"   Is ONLY the correct light ON? (y/n/stop): ").lower()
        if response == 'n':
            print(f"   ⚠️  WIRING ISSUE: Wrong light is on for {name}")
        elif response == 'stop':
            break
        time.sleep(0.5)
    
    print("\n4. Testing: Clear all outputs")
    send_byte(0b00000000)
    
    print("\n5. Testing: Simple stepper sequence")
    print("   Testing azimuth motor with ABCD sequence...")
    
    sequence = [
        0b00000001,  # A
        0b00000010,  # B
        0b00000100,  # C
        0b00001000,  # D
    ]
    
    input("   Press Enter to run sequence (slow)...")
    
    for i in range(8):  # Two full cycles
        pattern = sequence[i % 4]
        print(f"   Step {i+1}: {pattern:08b}")
        send_byte(pattern)
        time.sleep(1.0)  # Very slow
    
    # Turn off
    send_byte(0b00000000)
    
    print("\n" + "="*70)
    print("DIAGNOSTIC RESULTS")
    print("="*70)
    print("If all lights were ON at step 1: Shift register stuck HIGH")
    print("If wrong lights came on: Wiring mismatch")
    print("If correct lights but no motor movement: Motor/power issue")
    print("="*70)
    
    GPIO.cleanup()

if __name__ == "__main__":
    hardware_diagnostic()
