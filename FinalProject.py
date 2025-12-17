#!/usr/bin/env python3
"""
ENME441 Laser Turret Control
Controls 2 stepper motors via shift register and laser via GPIO
Cycles: Motors 180° CW/CCW, Laser 2s ON/OFF
"""

import RPi.GPIO as GPIO
import time
import threading
import sys

class LaserTurretController:
    def __init__(self):
        # GPIO pins for shift register
        self.SHIFT_CLK = 11  # GPIO11 -> SH_CP (Pin 11)
        self.LATCH_CLK = 10  # GPIO10 -> ST_CP (Pin 12)
        self.DATA_PIN = 9    # GPIO9  -> DS    (Pin 14)
        
        # Laser control pin
        self.LASER_PIN = 26  # GPIO26
        
        # Stepper motor parameters
        self.STEPS_PER_REV = 4096  # 28BYJ-48 in half-step mode
        self.STEPS_180_DEG = self.STEPS_PER_REV // 2  # 2048 steps
        
        # Stepper sequences (half-step for 28BYJ-48)
        # Format: [Azimuth bits, Altitude bits]
        self.AZIMUTH_SEQ = [
            0b00000001,  # Step 0: Azimuth Coil 1 ON (Q0=1)
            0b00000011,  # Step 1: Coils 1+2
            0b00000010,  # Step 2: Coil 2
            0b00000110,  # Step 3: Coils 2+3
            0b00000100,  # Step 4: Coil 3
            0b00001100,  # Step 5: Coils 3+4
            0b00001000,  # Step 6: Coil 4
            0b00001001   # Step 7: Coils 4+1
        ]
        
        self.ALTITUDE_SEQ = [
            0b00010000,  # Step 0: Altitude Coil 1 ON (Q4=1)
            0b00110000,  # Step 1: Coils 1+2
            0b00100000,  # Step 2: Coil 2
            0b01100000,  # Step 3: Coils 2+3
            0b01000000,  # Step 4: Coil 3
            0b11000000,  # Step 5: Coils 3+4
            0b10000000,  # Step 6: Coil 4
            0b10010000   # Step 7: Coils 4+1
        ]
        
        # State tracking
        self.azimuth_phase = 0
        self.altitude_phase = 0
        self.laser_state = False
        self.running = True
        
        # Setup GPIO
        self.setup_gpio()
        
    def setup_gpio(self):
        """Initialize all GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        
        # Shift register pins
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Laser pin
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        
        # Initialize pins
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        print("GPIO setup complete")
        
    def shift_out(self, data_byte):
        """Send 8 bits to shift register"""
        # Bring latch low while shifting
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
        # Shift out 8 bits MSB first
        for i in range(7, -1, -1):
            bit = (data_byte >> i) & 0x01
            GPIO.output(self.DATA_PIN, bit)
            
            # Pulse shift clock
            GPIO.output(self.SHIFT_CLK, GPIO.HIGH)
            time.sleep(0.00001)  # Very short pulse
            GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        
        # Latch the data to outputs
        GPIO.output(self.LATCH_CLK, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        
    def step_azimuth(self, direction, delay=0.001):
        """Take one step with azimuth motor"""
        if direction == 1:  # Clockwise
            self.azimuth_phase = (self.azimuth_phase + 1) % 8
        else:  # Counterclockwise
            self.azimuth_phase = (self.azimuth_phase - 1) % 8
        
        # Get altitude bits (keep current state)
        altitude_bits = self.ALTITUDE_SEQ[self.altitude_phase]
        
        # Get azimuth bits for new phase
        azimuth_bits = self.AZIMUTH_SEQ[self.azimuth_phase]
        
        # Combine bits (azimuth in low nibble, altitude in high nibble)
        combined_bits = azimuth_bits | altitude_bits
        
        self.shift_out(combined_bits)
        time.sleep(delay)
        
    def step_altitude(self, direction, delay=0.001):
        """Take one step with altitude motor"""
        if direction == 1:  # Clockwise/Up
            self.altitude_phase = (self.altitude_phase + 1) % 8
        else:  # Counterclockwise/Down
            self.altitude_phase = (self.altitude_phase - 1) % 8
        
        # Get azimuth bits (keep current state)
        azimuth_bits = self.AZIMUTH_SEQ[self.azimuth_phase]
        
        # Get altitude bits for new phase
        altitude_bits = self.ALTITUDE_SEQ[self.altitude_phase]
        
        # Combine bits
        combined_bits = azimuth_bits | altitude_bits
        
        self.shift_out(combined_bits)
        time.sleep(delay)
        
    def move_motors(self, azimuth_steps, altitude_steps, delay=0.001):
        """
        Move both motors specified number of steps
        Positive steps = clockwise/up
        Negative steps = counterclockwise/down
        """
        az_dir = 1 if azimuth_steps >= 0 else -1
        alt_dir = 1 if altitude_steps >= 0 else -1
        
        # Calculate which motor needs more steps
        az_steps_abs = abs(azimuth_steps)
        alt_steps_abs = abs(altitude_steps)
        max_steps = max(az_steps_abs, alt_steps_abs)
        
        # Move both motors simultaneously
        for i in range(max_steps):
            if i < az_steps_abs:
                self.step_azimuth(az_dir, 0)  # No delay here
            if i < alt_steps_abs:
                self.step_altitude(alt_dir, 0)  # No delay here
            time.sleep(delay)  # Single delay per step pair
            
        # Turn off coils to save power
        self.all_off()
        
    def move_motors_degrees(self, az_degrees, alt_degrees, delay=0.001):
        """Move motors by degrees instead of steps"""
        az_steps = int(az_degrees * self.STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.STEPS_PER_REV / 360)
        self.move_motors(az_steps, alt_steps, delay)
        
    def laser_on(self):
        """Turn laser on"""
        GPIO.output(self.LASER_PIN, GPIO.HIGH)
        self.laser_state = True
        print("Laser: ON")
        
    def laser_off(self):
        """Turn laser off"""
        GPIO.output(self.LASER_PIN, GPIO.LOW)
        self.laser_state = False
        print("Laser: OFF")
        
    def all_off(self):
        """Turn off all coils and laser"""
        self.shift_out(0b00000000)  # Turn off all motor coils
        self.laser_off()
        
    def motor_demo_cycle(self):
        """Run motor demo: 180° CW, pause, 180° CCW, pause"""
        print("\n" + "="*40)
        print("STARTING MOTOR DEMO CYCLE")
        print("="*40)
        
        cycle_count = 0
        while self.running:
            cycle_count += 1
            print(f"\n--- Cycle {cycle_count} ---")
            
            # 180° Clockwise
            print("Motors: 180° Clockwise")
            self.move_motors_degrees(180, 180, delay=0.0005)
            
            # Pause 2 seconds
            print("Pausing 2 seconds...")
            time.sleep(2)
            
            # 180° Counterclockwise
            print("Motors: 180° Counterclockwise")
            self.move_motors_degrees(-180, -180, delay=0.0005)
            
            # Pause 2 seconds
            print("Pausing 2 seconds...")
            time.sleep(2)
            
    def laser_demo_cycle(self):
        """Run laser demo: 2s ON, 2s OFF"""
        print("\n" + "="*40)
        print("STARTING LASER DEMO CYCLE")
        print("="*40)
        
        cycle_count = 0
        while self.running:
            cycle_count += 1
            print(f"\n--- Laser Cycle {cycle_count} ---")
            
            # Laser ON for 2 seconds
            self.laser_on()
            time.sleep(2)
            
            # Laser OFF for 2 seconds
            self.laser_off()
            time.sleep(2)
            
    def run_demos_simultaneously(self):
        """Run both demos in separate threads"""
        print("Starting simultaneous motor and laser demos...")
        print("Press Ctrl+C to stop\n")
        
        # Create threads for each demo
        motor_thread = threading.Thread(target=self.motor_demo_cycle)
        laser_thread = threading.Thread(target=self.laser_demo_cycle)
        
        # Set as daemon threads so they stop when main stops
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
            print("\nStopping demos...")
            
        # Wait for threads to finish
        motor_thread.join(timeout=1)
        laser_thread.join(timeout=1)
        
    def run_demos_alternating(self):
        """Run demos alternating (motors, then laser, repeat)"""
        print("Starting alternating demos...")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                print("\n=== MOTOR CYCLE ===")
                # Motors: 180° CW
                self.move_motors_degrees(180, 180, delay=0.0005)
                time.sleep(2)
                
                # Motors: 180° CCW
                self.move_motors_degrees(-180, -180, delay=0.0005)
                time.sleep(2)
                
                print("\n=== LASER CYCLE ===")
                # Laser: ON 2s
                self.laser_on()
                time.sleep(2)
                
                # Laser: OFF 2s
                self.laser_off()
                time.sleep(2)
                
        except KeyboardInterrupt:
            self.running = False
            print("\nStopping...")
            
    def cleanup(self):
        """Clean up GPIO and stop everything"""
        self.running = False
        self.all_off()
        GPIO.cleanup()
        print("GPIO cleaned up. Goodbye!")

def main():
    """Main program with menu"""
    print("ENME441 Laser Turret Demo")
    print("="*40)
    
    try:
        controller = LaserTurretController()
        
        # Menu
        print("\nSelect demo mode:")
        print("1. Motors and Lasers SIMULTANEOUS (separate threads)")
        print("2. Motors and Lasers ALTERNATING")
        print("3. Motors ONLY (180° CW/CCW)")
        print("4. Laser ONLY (2s ON/OFF)")
        print("5. Test single movements")
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            controller.run_demos_simultaneously()
        elif choice == "2":
            controller.run_demos_alternating()
        elif choice == "3":
            # Motors only
            controller.running = True
            while controller.running:
                try:
                    controller.move_motors_degrees(180, 180, delay=0.0005)
                    time.sleep(2)
                    controller.move_motors_degrees(-180, -180, delay=0.0005)
                    time.sleep(2)
                except KeyboardInterrupt:
                    controller.running = False
        elif choice == "4":
            # Laser only
            controller.running = True
            while controller.running:
                try:
                    controller.laser_on()
                    time.sleep(2)
                    controller.laser_off()
                    time.sleep(2)
                except KeyboardInterrupt:
                    controller.running = False
        elif choice == "5":
            # Test mode
            print("\nTest Mode:")
            print("a. Move azimuth 90° CW")
            print("b. Move altitude 90° UP")
            print("c. Test laser")
            print("d. Exit")
            
            while True:
                cmd = input("\nEnter test command (a/b/c/d): ").strip().lower()
                if cmd == 'a':
                    controller.move_motors_degrees(90, 0, delay=0.001)
                elif cmd == 'b':
                    controller.move_motors_degrees(0, 90, delay=0.001)
                elif cmd == 'c':
                    controller.laser_on()
                    time.sleep(1)
                    controller.laser_off()
                elif cmd == 'd':
                    break
                else:
                    print("Unknown command")
        else:
            print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.cleanup()
        print("\nProgram ended.")

if __name__ == "__main__":
    main()
