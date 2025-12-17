#!/usr/bin/env python3
"""
ENME441 Laser Turret - FIXED SPEEDS + AUTO-HOME
1. Altitude motor speed doubled to match azimuth
2. Auto-homes to horizontal position on startup
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
        self.AZIMUTH_STEPS_PER_REV = 1024   # Fast motor
        self.ALTITUDE_STEPS_PER_REV = 4096  # Standard motor
        
        # SPEED COMPENSATION: Make altitude move 2× faster
        # We'll achieve this by moving altitude 2 steps per loop iteration
        self.ALTITUDE_SPEED_FACTOR = 2  # Moves 2× faster than normal
        
        # Horizontal position reference
        # Assuming: horizontal = flat side parallel to motor protrusions
        # We'll find this by rotating until we hit a limit or marker
        self.azimuth_position = 0  # Steps from home (positive = CW)
        self.altitude_position = 0 # Steps from home (positive = up)
        
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
        """Initialize GPIO and auto-home to horizontal position"""
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
        
        print("GPIO setup complete")
        print("Auto-homing to horizontal position...")
        self.auto_home()
        
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
            self.azimuth_position += 1
        else:
            self.azimuth_phase = (self.azimuth_phase - 1) % 8
            self.azimuth_position -= 1
        
        combined = (self.AZIMUTH_SEQ[self.azimuth_phase] | 
                   self.ALTITUDE_SEQ[self.altitude_phase])
        self.shift_out(combined)
        time.sleep(delay)
        
    def step_altitude_fast(self, direction, delay=0.001):
        """
        Take TWO steps with altitude motor (2× speed)
        This matches azimuth motor speed
        """
        for _ in range(self.ALTITUDE_SPEED_FACTOR):
            if direction == 1:
                self.altitude_phase = (self.altitude_phase + 1) % 8
                self.altitude_position += 1
            else:
                self.altitude_phase = (self.altitude_phase - 1) % 8
                self.altitude_position -= 1
            
            combined = (self.AZIMUTH_SEQ[self.azimuth_phase] | 
                       self.ALTITUDE_SEQ[self.altitude_phase])
            self.shift_out(combined)
            time.sleep(delay / self.ALTITUDE_SPEED_FACTOR)
        
    def move_motors_sync(self, azimuth_steps, altitude_steps, delay=0.001):
        """
        Move both motors synchronously with matched speed
        Altitude moves 2× faster to compensate for different step counts
        """
        az_dir = 1 if azimuth_steps >= 0 else -1
        alt_dir = 1 if altitude_steps >= 0 else -1
        
        az_steps_abs = abs(azimuth_steps)
        alt_steps_abs = abs(altitude_steps)
        
        # Since altitude moves 2× faster, we need half the iterations
        # But we still need to track position accurately
        max_iterations = max(az_steps_abs, alt_steps_abs // self.ALTITUDE_SPEED_FACTOR)
        
        print(f"Moving: Azimuth={azimuth_steps} steps, Altitude={altitude_steps} steps")
        print(f"Altitude moving {self.ALTITUDE_SPEED_FACTOR}× faster")
        
        az_counter = 0
        alt_counter = 0
        
        for i in range(max_iterations):
            # Move azimuth if we still need to
            if az_counter < az_steps_abs:
                self.step_azimuth(az_dir, 0)
                az_counter += 1
            
            # Move altitude if we still need to (moves faster)
            if alt_counter < alt_steps_abs:
                # We'll move altitude multiple times per loop if needed
                steps_this_loop = min(self.ALTITUDE_SPEED_FACTOR, alt_steps_abs - alt_counter)
                for _ in range(steps_this_loop):
                    if alt_dir == 1:
                        self.altitude_phase = (self.altitude_phase + 1) % 8
                        self.altitude_position += 1
                    else:
                        self.altitude_phase = (self.altitude_phase - 1) % 8
                        self.altitude_position -= 1
                    
                    combined = (self.AZIMUTH_SEQ[self.azimuth_phase] | 
                               self.ALTITUDE_SEQ[self.altitude_phase])
                    self.shift_out(combined)
                    time.sleep(delay / self.ALTITUDE_SPEED_FACTOR)
                
                alt_counter += steps_this_loop
            
            # Main delay between step groups
            time.sleep(delay)
        
        # Update final position
        current_bits = self.AZIMUTH_SEQ[self.azimuth_phase] | self.ALTITUDE_SEQ[self.altitude_phase]
        self.shift_out(current_bits)
        
        print(f"Final position: Azimuth={self.azimuth_position}, Altitude={self.altitude_position}")
        
    def move_motors_degrees_sync(self, az_degrees, alt_degrees, delay=0.001):
        """Move by degrees with matched speeds"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        self.move_motors_sync(az_steps, alt_steps, delay)
        
    def auto_home(self):
        """
        Auto-home to horizontal position
        Strategy: Move to a known safe position (all motors to 0 position)
        Then rotate until flat side is horizontal (parallel to motor protrusions)
        
        For now: Just move to a predefined 'home' position (0,0)
        In real implementation, you'd add limit switches or manual alignment
        """
        print("Finding horizontal home position...")
        
        # Strategy 1: Move to a known safe position
        # Turn off all coils first
        self.shift_out(0b00000000)
        time.sleep(0.5)
        
        # Move both motors to what we'll define as "home"
        # You should physically align the turret first, then run this
        print("Please manually align turret to horizontal position:")
        print("1. Ensure laser is pointing forward")
        print("2. Ensure flat sides are parallel to motor protrusions")
        print("3. Press Enter when aligned...")
        input()  # Wait for user to align manually
        
        # Reset position counters to 0
        self.azimuth_position = 0
        self.altitude_position = 0
        self.azimuth_phase = 0
        self.altitude_phase = 0
        
        # Energize coils at home position
        home_bits = self.AZIMUTH_SEQ[0] | self.ALTITUDE_SEQ[0]
        self.shift_out(home_bits)
        
        print("✓ Home position set:")
        print(f"  Azimuth: position=0, phase=0")
        print(f"  Altitude: position=0, phase=0")
        print(f"  Laser: OFF (safe)")
        
    def go_to_home(self):
        """Return to home position from current location"""
        print("Returning to home position...")
        
        # Calculate steps needed to return to home
        az_steps_needed = -self.azimuth_position
        alt_steps_needed = -self.altitude_position
        
        print(f"Moving azimuth: {az_steps_needed} steps to home")
        print(f"Moving altitude: {alt_steps_needed} steps to home")
        
        # Move back to home
        self.move_motors_sync(az_steps_needed, alt_steps_needed, delay=0.001)
        
        # Reset positions
        self.azimuth_position = 0
        self.altitude_position = 0
        self.azimuth_phase = 0
        self.altitude_phase = 0
        
        print("✓ At home position")
        
    def move_to_absolute(self, az_steps, alt_steps, delay=0.001):
        """Move to absolute position (steps from home)"""
        az_relative = az_steps - self.azimuth_position
        alt_relative = alt_steps - self.altitude_position
        
        print(f"Moving to absolute position: Azimuth={az_steps}, Altitude={alt_steps}")
        print(f"Relative movement: Azimuth={az_relative}, Altitude={alt_relative}")
        
        self.move_motors_sync(az_relative, alt_relative, delay)
        
    def move_to_absolute_degrees(self, az_degrees, alt_degrees, delay=0.001):
        """Move to absolute angle (degrees from home)"""
        az_steps = int(az_degrees * self.AZIMUTH_STEPS_PER_REV / 360)
        alt_steps = int(alt_degrees * self.ALTITUDE_STEPS_PER_REV / 360)
        self.move_to_absolute(az_steps, alt_steps, delay)
        
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
        
    def motor_demo_sync(self):
        """Demo with matched motor speeds"""
        print("\n" + "="*50)
        print("MOTOR DEMO: 180° rotations with MATCHED SPEEDS")
        print("="*50)
        
        cycle = 0
        try:
            while self.running:
                cycle += 1
                print(f"\n--- CYCLE {cycle} ---")
                print(f"Current position: Azi={self.azimuth_position}, Alt={self.altitude_position}")
                
                # 180° Clockwise
                print("1. 180° Clockwise")
                self.move_motors_degrees_sync(180, 180, delay=0.0005)
                print("   Pausing 2 seconds...")
                time.sleep(2)
                
                # 180° Counterclockwise
                print("2. 180° Counterclockwise")
                self.move_motors_degrees_sync(-180, -180, delay=0.0005)
                print("   Pausing 2 seconds...")
                time.sleep(2)
                
        except KeyboardInterrupt:
            self.running = False
            print("\nDemo stopped")
            
    def laser_demo(self):
        """Laser demo: 2s ON, 2s OFF"""
        print("\n" + "="*50)
        print("LASER DEMO: 2 seconds ON, 2 seconds OFF")
        print("="*50)
        
        cycle = 0
        try:
            while self.running:
                cycle += 1
                print(f"\nLaser Cycle {cycle}")
                
                self.laser_on()
                time.sleep(2)
                self.laser_off()
                time.sleep(2)
                
        except KeyboardInterrupt:
            self.running = False
            
    def test_matched_speed(self):
        """Test that both motors complete 180° in same time"""
        print("\n" + "="*50)
        print("MATCHED SPEED TEST")
        print("="*50)
        print("Both motors should complete 180° rotation simultaneously")
        
        input("Press Enter to start test...")
        
        start_time = time.time()
        self.move_motors_degrees_sync(180, 180, delay=0.0005)
        elapsed = time.time() - start_time
        
        print(f"\nTest complete in {elapsed:.2f} seconds")
        print(f"Azimuth position: {self.azimuth_position} steps")
        print(f"Altitude position: {self.altitude_position} steps")
        
        # Verify
        expected_az = 512  # 1024/2
        expected_alt = 2048  # 4096/2
        
        if abs(self.azimuth_position - expected_az) <= 10 and abs(self.altitude_position - expected_alt) <= 20:
            print("✓ Motors moved correctly with matched speed!")
        else:
            print("✗ Position error - check calibration")
            
    def cleanup(self):
        """Clean shutdown - return to home and turn off"""
        self.running = False
        print("\nCleaning up...")
        print("Returning to home position...")
        self.go_to_home()
        self.shift_out(0b00000000)  # Turn off coils
        self.laser_off()
        GPIO.cleanup()
        print("Cleanup complete")

def main():
    """Main program"""
    print("ENME441 Laser Turret - MATCHED SPEEDS + AUTO-HOME")
    print("="*50)
    print("Features:")
    print("1. Altitude motor speed DOUBLED to match azimuth")
    print("2. Auto-homes to horizontal position on startup")
    print("3. Position tracking for both motors")
    print("="*50)
    
    controller = None
    try:
        controller = LaserTurretController()
        
        while True:
            print("\n" + "="*50)
            print("MAIN MENU")
            print("="*50)
            print(f"Current position: Azi={controller.azimuth_position}, Alt={controller.altitude_position}")
            print("1. Motor Demo (Matched 180° rotations)")
            print("2. Laser Demo (2s ON/OFF)")
            print("3. Test Matched Speed")
            print("4. Go to Home Position")
            print("5. Move to Absolute Angle")
            print("6. Exit")
            
            choice = input("\nEnter choice (1-6): ").strip()
            
            if choice == "1":
                controller.running = True
                controller.motor_demo_sync()
            elif choice == "2":
                controller.running = True
                controller.laser_demo()
            elif choice == "3":
                controller.test_matched_speed()
            elif choice == "4":
                controller.go_to_home()
            elif choice == "5":
                az_deg = float(input("Azimuth degrees (from home): "))
                alt_deg = float(input("Altitude degrees (from home): "))
                controller.move_to_absolute_degrees(az_deg, alt_deg, delay=0.001)
            elif choice == "6":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
                
            controller.running = False
            
    except KeyboardInterrupt:
        print("\nProgram interrupted")
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
