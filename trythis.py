import requests

# Replace with your laptop's actual IP
LAPTOP_IP = "192.168.43.50"  # <-- CHANGE THIS

url = f"http://{LAPTOP_IP}:8000/positions.json"

try:
    response = requests.get(url, timeout=5)
    print(f"Status: {response.status_code}")
    if response.status_code == 200:
        print("SUCCESS! Got JSON:")
        print(response.json())
except Exception as e:
    print(f"Error: {e}")
