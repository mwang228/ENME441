#!/usr/bin/env python3
"""
ENME441 - Simple Turret Controller with JSON Reading
"""

import RPi.GPIO as GPIO
import time
import json
import requests

# Motor calibration
AZIMUTH_STEPS_PER_REV = 1024    # Your fast motor
ALTITUDE_STEPS_PER_REV = 4096   # Standard motor

class SimpleTurret:
    def __init__(self):
        print("Initializing turret...")
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        
        # Shift register pins
        self.SHIFT_CLK = 11
        self.LATCH_CLK = 10
        self.DATA_PIN = 9
        
        GPIO.setup(self.SHIFT_CLK, GPIO.OUT)
        GPIO.setup(self.LATCH_CLK, GPIO.OUT)
        GPIO.setup(self.DATA_PIN, GPIO.OUT)
        
        # Initialize
        GPIO.output(self.SHIFT_CLK, GPIO.LOW)
        GPIO.output(self.LATCH_CLK, GPIO.LOW)
        GPIO.output(self.DATA_PIN, GPIO.LOW)
        
        print("GPIO setup complete")
    
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
    
    def test_motors(self):
        """Simple motor test"""
        print("\nTesting motors...")
        patterns = [0b10101010, 0b01010101, 0b11110000, 0b00001111]
        for pattern in patterns:
            print(f"Sending pattern: {bin(pattern)}")
            self.shift_out(pattern)
            time.sleep(1)
        
        # Turn off
        self.shift_out(0b00000000)
        print("Test complete")
    
    def fetch_json_test(self):
        """Test JSON fetching"""
        print("\nTesting JSON reading...")
        
        # URL from competition instructions
        url = "http://192.168.1.254:8000/positions.json"
        
        try:
            print(f"Trying to fetch: {url}")
            response = requests.get(url, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                print("✓ Successfully fetched JSON!")
                print(f"\nData structure:")
                print(f"  Turrets: {len(data.get('turrets', {}))} teams")
                print(f"  Globes: {len(data.get('globes', []))} targets")
                
                # Show a sample
                if data.get('turrets'):
                    first_team = list(data['turrets'].keys())[0]
                    print(f"\nSample (Team {first_team}):")
                    print(f"  r = {data['turrets'][first_team]['r']} cm")
                    print(f"  θ = {data['turrets'][first_team]['theta']} rad")
                
                return data
            else:
                print(f"✗ HTTP Error: {response.status_code}")
                
        except requests.exceptions.ConnectionError:
            print("✗ Connection failed. Check:")
            print("  1. Is Pi connected to WiFi?")
            print("  2. Is the server running at 192.168.1.254?")
        except requests.exceptions.Timeout:
            print("✗ Timeout. Server not responding.")
        except Exception as e:
            print(f"✗ Error: {e}")
        
        # Try local file as fallback
        print("\nTrying local file 'positions.json'...")
        try:
            with open("positions.json", "r") as f:
                data = json.load(f)
                print("✓ Loaded from local file")
                return data
        except FileNotFoundError:
            print("✗ Local file not found.")
        except json.JSONDecodeError:
            print("✗ Invalid JSON in local file.")
        
        return None
    
    def create_test_json(self):
        """Create a test JSON file if none exists"""
        test_data = {
            "turrets": {
                "1": {"r": 300.0, "theta": 2.580},
                "2": {"r": 300.0, "theta": 0.661},
                "3": {"r": 300.0, "theta": 5.152}
            },
            "globes": [
                {"r": 300.0, "theta": 1.015, "z": 20.4},
                {"r": 300.0, "theta": 4.512, "z": 32.0},
                {"r": 300.0, "theta": 3.979, "z": 10.8}
            ]
        }
        
        with open("positions.json", "w") as f:
            json.dump(test_data, f, indent=2)
        print("Created test positions.json file")
    
    def cleanup(self):
        """Clean up GPIO"""
        self.shift_out(0b00000000)
        GPIO.cleanup()
        print("\nGPIO cleaned up")

def main():
    """Main program"""
    print("=" * 50)
    print("ENME441 Turret - Simple Test Program")
    print("=" * 50)
    
    turret = None
    try:
        turret = SimpleTurret()
        
        while True:
            print("\nMenu:")
            print("1. Test motors")
            print("2. Fetch JSON from network")
            print("3. Create test JSON file")
            print("4. Exit")
            
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                turret.test_motors()
            elif choice == "2":
                data = turret.fetch_json_test()
                if data:
                    print("\nWant to save this data to file? (y/n)")
                    if input().strip().lower() == 'y':
                        with open("positions.json", "w") as f:
                            json.dump(data, f, indent=2)
                        print("Saved to positions.json")
            elif choice == "3":
                turret.create_test_json()
            elif choice == "4":
                print("Exiting...")
                break
            else:
                print("Invalid choice")
                
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if turret:
            turret.cleanup()
        print("Program ended.")

if __name__ == "__main__":
    # Check for required packages
    try:
        import requests
    except ImportError:
        print("Installing 'requests' package...")
        import subprocess
        subprocess.check_call(["pip3", "install", "requests"])
        import requests
    
    main()
