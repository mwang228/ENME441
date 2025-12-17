# Save this as json_server.py on your laptop
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import socket

# Your competition coordinates (edit as needed)
COMPETITION_DATA = {
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
            response = json.dumps(COMPETITION_DATA).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not found")
    
    def log_message(self, format, *args):
        # Suppress default logging
        pass

def get_local_ip():
    """Get laptop's local IP address"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

def run_server():
    HOST = get_local_ip()  # Automatically detect IP
    PORT = 8000
    print(f"Starting JSON server at http://{HOST}:{PORT}/positions.json")
    print(f"Server IP: {HOST}")
    print("Press Ctrl+C to stop\n")
    
    server = HTTPServer((HOST, PORT), JSONHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped")

if __name__ == "__main__":
    run_server()
