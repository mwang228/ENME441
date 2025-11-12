# stepper_class_shiftregister_multiprocessing.py

import time
import multiprocessing
import math
from shifter import Shifter

class Stepper:
    """
    Supports simultaneous operation of multiple stepper motors using shift registers.
    """

    # Class attributes:
    num_steppers = 0
    shifter_outputs = 0
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001] # CCW sequence
    delay = 1200          # delay between motor steps [us]
    steps_per_degree = 4096/360

    def __init__(self, shifter, lock):
        self.s = shifter
        # Use multiprocessing.Value for shared angle tracking
        self.angle = multiprocessing.Value('d', 0.0)  # 'd' for double
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers
        self.lock = lock
        Stepper.num_steppers += 1

    def __sgn(self, x):
        if x == 0: 
            return 0
        else: 
            return int(abs(x)/x)

    def __step(self, dir):
        """Modified to allow simultaneous motor operation"""
        self.step_state += dir
        self.step_state %= 8
        
        # Clear only this motor's bits, preserve others
        mask_clear = ~(0b1111 << self.shifter_bit_start)
        mask_set = Stepper.seq[self.step_state] << self.shifter_bit_start
        
        Stepper.shifter_outputs &= mask_clear  # Clear this motor's bits
        Stepper.shifter_outputs |= mask_set    # Set new bits for this motor
        
        self.s.shiftByte(Stepper.shifter_outputs)
        
        # Update shared angle value
        with self.angle.get_lock():
            self.angle.value += dir/Stepper.steps_per_degree
            self.angle.value %= 360

    def __rotate(self, delta):
        self.lock.acquire()
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        dir = self.__sgn(delta)
        for s in range(numSteps):
            self.__step(dir)
            time.sleep(Stepper.delay/1e6)
        self.lock.release()

    def rotate(self, delta):
        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    def goAngle(self, target_angle):
        """Move to absolute angle taking shortest path"""
        # Get current angle from shared memory
        with self.angle.get_lock():
            current_angle = self.angle.value
        
        # Normalize angles to [0, 360)
        current_angle %= 360
        target_angle %= 360
        
        # Calculate the difference and find shortest path
        diff = target_angle - current_angle
        
        # Handle wrap-around for shortest path
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        
        # Start rotation process
        self.rotate(diff)

    def zero(self):
        """Set motor zero point"""
        with self.angle.get_lock():
            self.angle.value = 0

    def getAngle(self):
        """Helper method to get current angle"""
        with self.angle.get_lock():
            return self.angle.value


# Example use with the required demonstration sequence:

if __name__ == '__main__':

    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero the motors
    m1.zero()
    m2.zero()

    # Required demonstration sequence
    print("Starting demonstration sequence...")
    
    # These should run simultaneously
    m1.goAngle(90)
    m1.goAngle(-45)

    m2.goAngle(-90)
    m2.goAngle(45)

    m1.goAngle(-135)
    m1.goAngle(135)
    m1.goAngle(0)

    # Wait for processes to complete
    time.sleep(10)  # Adjust based on your motor speed
    
    print("Demonstration complete")
    print(f"Final angles - m1: {m1.getAngle():.1f}°, m2: {m2.getAngle():.1f}°")
