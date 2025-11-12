# stepper_class_shiftregister_multiprocessing.py

import time
import multiprocessing
import math
from shifter import Shifter

class Stepper:

    # Class attributes:
    num_steppers = 0
    shifter_outputs = 0
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001] # CCW sequence
    delay = 1200          # delay between motor steps [us]
    steps_per_degree = 4096/360

    def __init__(self, shifter, lock):
        self.s = shifter
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
        self.step_state += dir
        self.step_state %= 8
        
        mask_clear = ~(0b1111 << self.shifter_bit_start)
        mask_set = Stepper.seq[self.step_state] << self.shifter_bit_start
        
        Stepper.shifter_outputs &= mask_clear
        Stepper.shifter_outputs |= mask_set
        
        self.s.shiftByte(Stepper.shifter_outputs)
        
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
        with self.angle.get_lock():
            current_angle = self.angle.value
        
        current_angle %= 360
        target_angle %= 360

        diff = target_angle - current_angle
        
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        
        self.rotate(diff)

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0

    def getAngle(self):
        """Helper method to get current angle"""
        with self.angle.get_lock():
            return self.angle.value

if __name__ == '__main__':

    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero the motors
    m1.zero()
    m2.zero()
    
    m1.goAngle(90)
    m1.goAngle(-45)

    m2.goAngle(-90)
    m2.goAngle(45)

    m1.goAngle(-135)
    m1.goAngle(135)
    m1.goAngle(0)

    time.sleep(10)  
    
    print("Done")
