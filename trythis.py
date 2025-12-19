import RPi.GPIO as GPIO
import time

SHIFT_CLK = 11  # GPIO11 -> Pin 11
LATCH_CLK = 10  # GPIO10 -> Pin 12  
DATA_PIN = 9    # GPIO9  -> Pin 14

GPIO.setmode(GPIO.BCM)
GPIO.setup(SHIFT_CLK, GPIO.OUT)
GPIO.setup(LATCH_CLK, GPIO.OUT)
GPIO.setup(DATA_PIN, GPIO.OUT)

def shift_out(data_byte):
    GPIO.output(LATCH_CLK, GPIO.LOW)
    for i in range(7, -1, -1):
        bit = (data_byte >> i) & 0x01
        GPIO.output(DATA_PIN, bit)
        GPIO.output(SHIFT_CLK, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(SHIFT_CLK, GPIO.LOW)
    GPIO.output(LATCH_CLK, GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(LATCH_CLK, GPIO.LOW)

# Test 1: Turn on just Azimuth Motor coil A (should get warm/hold position)
print("Testing Azimuth Motor coil A (Q0)")
shift_out(0b00000001)  # Only Q0 HIGH
time.sleep(2)

# Test 2: Turn on just Altitude Motor coil A (should get warm/hold position)  
print("Testing Altitude Motor coil A (Q4)")
shift_out(0b00010000)  # Only Q4 HIGH
time.sleep(2)

# Test 3: Turn everything OFF
shift_out(0b00000000)
print("Test complete")
GPIO.cleanup()
