#!/usr/bin/env python3
"""
ENME441 Laser Turret - FIXED VERSION
- Fixed azimuth motor calibration
- Fixed laser polarity (off by default)
"""

import RPi.GPIO as GPIO
import time
import threading
import sys

class LaserTurretController:
    def __init__(self):
        # GPIO pins for shift register
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP
        self.DATA_PIN = 9    # GPIO9  -> DS
        
        # Laser control pin
        self.LASER_PIN = 26  # GPIO26 (HIGH = ON, LOW = OFF)
        
        # CALIBRATED steps per revolution
        # 28BYJ-48 can vary: Common values: 512, 1024, 2048, 4096
        self.AZIMUTH_STEPS_PER_REV = 2048   # ADJUST THIS! Was 4096
        self.ALTITUDE_STEPS_PER_REV = 4096  # Altitude seems correct
        
        # Stepper sequences (half-step)
        self.AZIMUTH_SEQ = [
            0b00000001, 0b00000011, 0b00000010, 0b00000110,
            0b00000100, 0b00001100, 0b00001000, 0b00001001
        ]
        
        self.ALTITUDE_SEQ = [
            0b00010000, 0b00110000, 0b00100000, 0b01100000,
            0b01000000, 0b11000000, 0b10000000, 0b10010000
        ]
        
        # State tracking
        self.azimuth_phase = 0
        self.altitude_phase = 0
        self.laser_state = False
        self.running = True
        
        self.setup_gpio()
        
    def setup_gpio(self):
        """Initialize GPIO - Laser starts OFF"""
        GPIO.setmode(GPIO.BCM)
        
        # Shift register pins
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Laser pin - OUTPUT, start LOW (laser OFF)
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        # Initialize
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        print("GPIO setup: Laser starts OFF (safe)")
        
    def shift_out(self, data_byte):
        """Send 8 bits to shift register"""
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        for i in range(7, -1, -1):
            bit = (data_byte >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
    def step_azimuth(self, direction, delay=0.001):
        """Take one step with azimuth motor"""
        if direction == 1:
            self.azimuth_phase = (self.azimuth_phase + 1) % 8
        else:
            self.azimuth_phase = (self.azimuth_phase - 1) % 8
        
        combined = (self.AZIMUTH_SEQ[self.azimuth_phase] | 
                   self.ALTITUDE_SEQ[self.altitude_phase])
        self.shift_out(combined)
        time.sleep(delay)
        
    def step_altitude(self, direction, delay=0.001):
        """Take one step with altitude motor"""
        if direction == 1:
            self.altitude_phase = (self.altitude_phase + 1) % 8
        else:
            self.altitude_phase = (self.altitude_phase - 1) % 8
        
        combined = (self.AZIMUTH_SEQ[self.azimuth_phase] | 
                   self.ALTITUDE_SEQ[self.altitude_phase])
        self.shift_out(combined)
        time.sleep(delay)
        
    def move_motors(self, azimuth_steps, altitude_steps, delay=0.001):
        """Move both motors with individual calibration"""
        az_dir = 1 if azimuth_steps >= 0 else -1
        alt_dir = 1 if altitude_steps >= 0 else -1
        
        az_steps_abs = abs(azimuth_steps)
        alt_steps_abs = abs(altitude_steps)
        max_steps = max(az_steps_abs, alt_steps_abs)
        
        for i in range(max_steps):
            if i < az_steps_abs:
                self.step_azimuth(az_dir, 0)
            if i < alt_steps_abs:
                self.step_altitude(alt_dir, 0)
            time.sleep(delay)
            
        self.all_off()
        
    def move_motors_degrees(self, az_degrees, alt_degrees, delay=0.001):
        """Move by degrees with individual calibration"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        self.move_motors(az_steps, alt_steps, delay)
        
    def calibrate_azimuth(self):
        """Interactive calibration for azimuth motor"""
        print("\n=== AZIMUTH MOTOR CALIBRATION ===")
        print("We'll find the correct steps per revolution.")
        
        test_angles = [90, 180, 360]  # Test 90°, 180°, and full rotation
        
        for angle in test_angles:
            print(f"\nTesting {angle}° rotation...")
            
            # Try different step values
            for steps_per_rev in [512, 1024, 2048, 4096]:
                steps_needed = int(angle * steps_per_rev / 360)
                
                print(f"  {steps_per_rev} steps/rev = {steps_needed} steps")
                response = input(f"  Try this? (y/n/skip): ").lower()
                
                if response == 'y':
                    self.AZIMUTH_STEPS_PER_REV = steps_per_rev
                    self.move_motors_degrees(angle, 0, delay=0.002)
                    
                    correct = input(f"  Was that close to {angle}°? (y/n): ").lower()
                    if correct == 'y':
                        print(f"  ✓ Calibrated: {steps_per_rev} steps/revolution")
                        return
                elif response == 'skip':
                    break
        
        print("Calibration complete (using default 2048)")
        self.AZIMUTH_STEPS_PER_REV = 2048
        
    def laser_on(self):
        """Turn laser ON (GPIO HIGH)"""
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_state = True
        print("Laser: ON")
        
    def laser_off(self):
        """Turn laser OFF (GPIO LOW) - DEFAULT/SAFE"""
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_state = False
        print("Laser: OFF")
        
    def all_off(self):
        """Turn off all coils and ensure laser is OFF"""
        self.shift_out(0b00000000)
        self.laser_off()
        
    def motor_demo(self):
        """Your requested pattern: 180° CW, pause, 180° CCW, pause"""
        print("\n=== MOTOR DEMO (180° rotations) ===")
        
        cycle = 0
        while self.running:
            cycle += 1
            print(f"\nCycle {cycle}")
            
            # 180° Clockwise
            print("→ 180° Clockwise")
            self.move_motors_degrees(180, 180, delay=0.0005)
            time.sleep(2)
            
            # 180° Counterclockwise
            print("← 180° Counterclockwise")
            self.move_motors_degrees(-180, -180, delay=0.0005)
            time.sleep(2)
            
    def laser_demo(self):
        """Your requested pattern: 2s ON, 2s OFF"""
        print("\n=== LASER DEMO (2s ON/OFF) ===")
        
        cycle = 0
        while self.running:
            cycle += 1
            print(f"\nLaser Cycle {cycle}")
            
            # ON for 2s
            self.laser_on()
            time.sleep(2)
            
            # OFF for 2s
            self.laser_off()
            time.sleep(2)
            
    def cleanup(self):
        """Clean shutdown"""
        self.running = False
        self.all_off()
        GPIO.cleanup()
        print("\nCleanup complete. Laser is OFF.")

def main():
    """Main program with fixes"""
    print("ENME441 Laser Turret - FIXED VERSION")
    print("=" * 40)
    
    controller = None
    try:
        controller = LaserTurretController()
        
        print("\nSelect action:")
        print("1. Calibrate azimuth motor (RECOMMENDED FIRST)")
        print("2. Run motor demo (180° CW/CCW)")
        print("3. Run laser demo (2s ON/OFF)")
        print("4. Run both simultaneously")
        print("5. Exit")
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            controller.calibrate_azimuth()
        elif choice == "2":
            controller.running = True
            controller.motor_demo()
        elif choice == "3":
            controller.running = True
            controller.laser_demo()
        elif choice == "4":
            # Run both in threads
            controller.running = True
            motor_thread = threading.Thread(target=controller.motor_demo)
            laser_thread = threading.Thread(target=controller.laser_demo)
            motor_thread.daemon = True
            laser_thread.daemon = True
            
            motor_thread.start()
            laser_thread.start()
            
            try:
                while controller.running:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                controller.running = False
        elif choice == "5":
            print("Exiting...")
        else:
            print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller:
            controller.cleanup()
        print("Program ended.")

if __name__ == "__main__":
    main()
