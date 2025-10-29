import socket
from gpiozero import PWMOutputDevice

leds = {
    '1': PWMOutputDevice(17),
    '2': PWMOutputDevice(27),
    '3': PWMOutputDevice(22),
}

brightness = {'1': 0, '2': 0, '3': 0}

def parse_post_data(data):
    headers_end = data.find('\r\n\r\n')
    if headers_end == -1:
        return {}
    
    body = data[headers_end + 4:]
    params = body.split('&')
    post_data = {}
    
    for param in params:
        if '=' in param:
            key, value = param.split('=', 1)
            post_data[key] = value
    
    return post_data

def generate_html():
    return f"""HTTP/1.1 200 OK
Content-Type: text/html
Connection: close

<html>
<head>
    <title>LED Brightness Control</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; }}
        .status {{ background: #f0f0f0; padding: 15px; border-radius: 5px; margin-bottom: 20px; }}
        .form-group {{ margin: 15px 0; }}
        input[type="range"] {{ width: 200px; }}
        input[type="submit"] {{ padding: 8px 16px; background: #007cba; color: white; border: none; border-radius: 4px; }}
    </style>
</head>
<body>
    <h1>LED Brightness Control</h1>
    
    <div class="status">
        <h2>Current Brightness Levels:</h2>
        <ul>
            <li>LED 1: {brightness['1']}%</li>
            <li>LED 2: {brightness['2']}%</li>
            <li>LED 3: {brightness['3']}%</li>
        </ul>
    </div>
    
    <form method="POST">
        <div class="form-group">
            <strong>Select LED:</strong><br>
            <input type="radio" name="led" value="1" required> LED 1<br>
            <input type="radio" name="led" value="2"> LED 2<br>
            <input type="radio" name="led" value="3"> LED 3<br>
        </div>
        
        <div class="form-group">
            <strong>Brightness: <span id="brightnessValue">0</span>%</strong><br>
            <input type="range" name="brightness" id="brightnessSlider" 
                   min="0" max="100" value="0" 
                   oninput="document.getElementById('brightnessValue').textContent = this.value">
        </div>
        
        <input type="submit" value="Change Brightness">
    </form>
    
    <script>
        // Update slider value display
        document.getElementById('brightnessSlider').oninput();
    </script>
</body>
</html>"""

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', 8080))
        s.listen(5)
        
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Connection from {addr}")
                
                data = conn.recv(1024).decode('utf-8')
                print(f"Received request: {data.split()[0]}")
                
                if data.startswith('POST'):
                    post_data = parse_post_data(data)
                    print(f"POST data: {post_data}")
                    
                    led = post_data.get('led')
                    brightness_val = post_data.get('brightness')
                    
                    if led and brightness_val:
                        try:
                            brightness[led] = int(brightness_val)
                            leds[led].value = brightness[led] / 100.0
                            print(f"Set LED {led} to {brightness[led]}%")
                        except ValueError:
                            print("Invalid brightness value")
                
                response = generate_html()
                conn.send(response.encode('utf-8'))
                
            print("Connection closed\n")

if __name__ == '__main__':
    try:
        start_server()
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    finally:
        # Clean up GPIO
        for led in leds.values():
            led.close()
