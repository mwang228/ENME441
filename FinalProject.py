#!/usr/bin/env python3
"""
ENME441 Laser Turret - CALIBRATED VERSION
Azimuth: 1024 steps/revolution (yours)
Altitude: 4096 steps/revolution (standard)
Laser: OFF by default (safe)
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
        
        # CALIBRATED steps per revolution - YOUR SPECIFIC MOTORS
        self.AZIMUTH_STEPS_PER_REV = 1024   # Your azimuth motor
        self.ALTITUDE_STEPS_PER_REV = 4096  # Standard 28BYJ-48
        
        # Calculate steps for common angles
        self.AZIMUTH_STEPS_90 = int(90 * self.AZIMUTH_STEPS_PER_REV / 360)
        self.AZIMUTH_STEPS_180 = int(180 * self.AZIMUTH_STEPS_PER_REV / 360)
        self.ALTITUDE_STEPS_90 = int(90 * self.ALTITUDE_STEPS_PER_REV / 360)
        self.ALTITUDE_STEPS_180 = int(180 * self.ALTITUDE_STEPS_PER_REV / 360)
        
        print(f"Motor calibration:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_PER_REV} steps/revolution")
        print(f"  Altitude: {self.ALTITUDE_STEPS_PER_REV} steps/revolution")
        
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
        
        print("GPIO setup complete - Laser starts OFF (safe)")
        
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
        """Move both motors with correct calibration"""
        az_dir = 1 if azimuth_steps >= 0 else -1
        alt_dir = 1 if altitude_steps >= 0 else -1
        
        az_steps_abs = abs(azimuth_steps)
        alt_steps_abs = abs(altitude_steps)
        max_steps = max(az_steps_abs, alt_steps_abs)
        
        print(f"Moving: Azimuth={azimuth_steps} steps, Altitude={altitude_steps} steps")
        
        for i in range(max_steps):
            if i < az_steps_abs:
                self.step_azimuth(az_dir, 0)
            if i < alt_steps_abs:
                self.step_altitude(alt_dir, 0)
            time.sleep(delay)
            
        # Turn off coils
        self.shift_out(0b00000000)
        
    def move_motors_degrees(self, az_degrees, alt_degrees, delay=0.001):
        """Move by degrees with calibrated motors"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        self.move_motors(az_steps, alt_steps, delay)
        
    def move_exact_180(self):
        """Exactly 180° rotation for both motors (your request)"""
        print("\nMoving exactly 180°:")
        print(f"  Azimuth: {self.AZIMUTH_STEPS_180} steps")
        print(f"  Altitude: {self.ALTITUDE_STEPS_180} steps")
        self.move_motors(self.AZIMUTH_STEPS_180, self.ALTITUDE_STEPS_180, delay=0.0005)
        
    def move_exact_180_reverse(self):
        """Exactly 180° reverse rotation"""
        print("\nMoving exactly 180° reverse:")
        print(f"  Azimuth: -{self.AZIMUTH_STEPS_180} steps")
        print(f"  Altitude: -{self.ALTITUDE_STEPS_180} steps")
        self.move_motors(-self.AZIMUTH_STEPS_180, -self.ALTITUDE_STEPS_180, delay=0.0005)
        
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
        
    def motor_demo(self):
        """Your requested pattern: 180° CW, pause, 180° CCW, pause"""
        print("\n" + "="*50)
        print("MOTOR DEMO: 180° Clockwise, pause, 180° Counterclockwise")
        print("="*50)
        
        cycle = 0
        try:
            while self.running:
                cycle += 1
                print(f"\n--- CYCLE {cycle} ---")
                
                # 180° Clockwise
                print("1. 180° Clockwise rotation")
                self.move_exact_180()
                print("   Pausing 2 seconds...")
                time.sleep(2)
                
                # 180° Counterclockwise
                print("2. 180° Counterclockwise rotation")
                self.move_exact_180_reverse()
                print("   Pausing 2 seconds...")
                time.sleep(2)
                
        except KeyboardInterrupt:
            self.running = False
            print("\nMotor demo stopped")
            
    def laser_demo(self):
        """Your requested pattern: 2s ON, 2s OFF"""
        print("\n" + "="*50)
        print("LASER DEMO: 2 seconds ON, 2 seconds OFF")
        print("="*50)
        
        cycle = 0
        try:
            while self.running:
                cycle += 1
                print(f"\nLaser Cycle {cycle}")
                
                # ON for 2s
                print("  Laser ON for 2 seconds")
                self.laser_on()
                time.sleep(2)
                
                # OFF for 2s
                print("  Laser OFF for 2 seconds")
                self.laser_off()
                time.sleep(2)
                
        except KeyboardInterrupt:
            self.running = False
            print("\nLaser demo stopped")
            
    def run_simultaneous_demo(self):
        """Run both demos in parallel threads"""
        print("\n" + "="*50)
        print("SIMULTANEOUS DEMO: Motors + Laser")
        print("="*50)
        print("Motors: 180° CW/CCW every 4 seconds")
        print("Laser: 2s ON/OFF cycle")
        print("Press Ctrl+C to stop\n")
        
        self.running = True
        
        # Create threads
        motor_thread = threading.Thread(target=self.motor_demo)
        laser_thread = threading.Thread(target=self.laser_demo)
        
        # Set as daemon threads
        motor_thread.daemon = True
        laser_thread.daemon = True
        
        # Start threads
        motor_thread.start()
        laser_thread.start()
        
        # Keep main thread alive
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.running = False
            print("\nStopping all demos...")
            
        # Wait briefly for threads to finish
        motor_thread.join(timeout=1)
        laser_thread.join(timeout=1)
        
    def test_single_movements(self):
        """Test individual movements"""
        print("\n" + "="*50)
        print("TEST INDIVIDUAL MOVEMENTS")
        print("="*50)
        
        while True:
            print("\nTest options:")
            print("  1. Azimuth 90° clockwise")
            print("  2. Azimuth 90° counterclockwise")
            print("  3. Altitude 90° up")
            print("  4. Altitude 90° down")
            print("  5. Laser ON for 1 second")
            print("  6. Return to main menu")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                print("Azimuth 90° clockwise")
                self.move_motors(self.AZIMUTH_STEPS_90, 0, delay=0.001)
            elif choice == "2":
                print("Azimuth 90° counterclockwise")
                self.move_motors(-self.AZIMUTH_STEPS_90, 0, delay=0.001)
            elif choice == "3":
                print("Altitude 90° up")
                self.move_motors(0, self.ALTITUDE_STEPS_90, delay=0.001)
            elif choice == "4":
                print("Altitude 90° down")
                self.move_motors(0, -self.ALTITUDE_STEPS_90, delay=0.001)
            elif choice == "5":
                print("Laser ON for 1 second")
                self.laser_on()
                time.sleep(1)
                self.laser_off()
            elif choice == "6":
                break
            else:
                print("Invalid choice")
                
    def cleanup(self):
        """Clean shutdown - ensure laser is OFF"""
        self.running = False
        self.shift_out(0b00000000)  # Turn off motors
        self.laser_off()            # Ensure laser is OFF
        GPIO.cleanup()
        print("\nCleanup complete. All motors off, laser OFF.")

def main():
    """Main program"""
    print("ENME441 Laser Turret - CALIBRATED")
    print("="*50)
    print(f"Calibration confirmed:")
    print(f"  Azimuth motor: 1024 steps/revolution")
    print(f"  Altitude motor: 4096 steps/revolution")
    print("="*50)
    
    controller = None
    try:
        controller = LaserTurretController()
        
        while True:
            print("\n" + "="*50)
            print("MAIN MENU")
            print("="*50)
            print("1. Motor Demo (180° CW/CCW)")
            print("2. Laser Demo (2s ON/OFF)")
            print("3. Simultaneous Demo (Both)")
            print("4. Test Individual Movements")
            print("5. Exit")
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == "1":
                controller.running = True
                controller.motor_demo()
            elif choice == "2":
                controller.running = True
                controller.laser_demo()
            elif choice == "3":
                controller.run_simultaneous_demo()
            elif choice == "4":
                controller.test_single_movements()
            elif choice == "5":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
                
            # Reset running flag
            controller.running = False
            time.sleep(0.5)  # Brief pause between operations
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller:
            controller.cleanup()
        print("\nProgram ended.")

if __name__ == "__main__":
    main()
