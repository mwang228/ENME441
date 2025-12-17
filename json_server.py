# Save this as test_server.py on your LAPTOP
from http.server import HTTPServer, BaseHTTPRequestHandler
import json

# Test data matching competition format
positions = {
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

class JSONHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/positions.json":
            response = json.dumps(positions).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)
            print(f"Served JSON to {self.client_address[0]}")
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not found")
    
    def log_message(self, format, *args):
        pass  # Suppress default logs

def run_server():
    # Use your laptop's IP address
    HOST = "0.0.0.0"  # Listen on all interfaces
    PORT = 8000
    
    print(f"Starting test JSON server on port {PORT}")
    print(f"Your laptop IP: (check with 'ipconfig' or 'ifconfig')")
    print(f"Pi should connect to: http://YOUR_LAPTOP_IP:8000/positions.json")
    
    server = HTTPServer((HOST, PORT), JSONHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped")

if __name__ == "__main__":
    run_server()
