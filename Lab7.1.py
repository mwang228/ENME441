import socket
from gpiozero import PWMOutputDevice

# Setup the LEDs
led1 = PWMOutputDevice(17)
led2 = PWMOutputDevice(27)
led3 = PWMOutputDevice(22)

leds = {
    '1': led1,
    '2': led2, 
    '3': led3
}

# Track brightness for each LED
brightness = {'1': 0, '2': 0, '3': 0}

def get_post_data(request_data):
    # Look for the blank line that separates headers from data
    blank_line = request_data.find('\r\n\r\n')
    if blank_line == -1:
        return {}
    
    # Get everything after the blank line
    post_body = request_data[blank_line + 4:]
    
    # Split into key=value pairs
    pairs = post_body.split('&')
    result = {}
    
    for pair in pairs:
        if '=' in pair:
            key, value = pair.split('=', 1)
            result[key] = value
    
    return result

def create_html_page():
    # Build the HTML page with current brightness values
    html_content = f"""HTTP/1.1 200 OK
Content-Type: text/html
Connection: close

<html>
<head>
    <title>LED Control</title>
    <style>
        body {{ font-family: Arial; margin: 30px; }}
        .box {{ border: 1px solid #ccc; padding: 15px; margin-bottom: 20px; }}
    </style>
</head>
<body>
    <h1>LED Brightness Control</h1>
    
    <div class="box">
        <h3>Current Brightness Levels:</h3>
        <ul>
            <li>LED 1: {brightness['1']}%</li>
            <li>LED 2: {brightness['2']}%</li>
            <li>LED 3: {brightness['3']}%</li>
        </ul>
    </div>
    
    <form method="POST">
        <h3>Select LED:</h3>
        <input type="radio" name="led" value="1" required> LED 1<br>
        <input type="radio" name="led" value="2"> LED 2<br>
        <input type="radio" name="led" value="3"> LED 3<br>
        
        <h3>Brightness Level (0-100):</h3>
        <input type="range" name="brightness" min="0" max="100" value="0">
        
        <br><br>
        <input type="submit" value="Change Brightness">
    </form>
</body>
</html>"""
    
    return html_content

# Main server code
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 8080))
server_socket.listen(5)

print("Server started on port 8080")
print("Go to http://your-pi-address:8080 in your browser")

try:
    while True:
        # Wait for a connection
        client_conn, client_addr = server_socket.accept()
        
        # Get the request from client
        request = client_conn.recv(1024).decode('utf-8')
        
        if request:
            # Check if it's a POST request
            if request.startswith('POST'):
                print(f"POST request from {client_addr}")
                
                # Parse the POST data
                post_data = get_post_data(request)
                
                # Get the LED and brightness values
                led_number = post_data.get('led')
                brightness_value = post_data.get('brightness')
                
                # If we got both values, update the LED
                if led_number and brightness_value:
                    try:
                        # Convert to integer and update
                        bright_level = int(brightness_value)
                        brightness[led_number] = bright_level
                        
                        # Set the actual LED brightness
                        leds[led_number].value = bright_level / 100.0
                        
                        print(f"LED {led_number} set to {bright_level}%")
                        
                    except ValueError:
                        print("Error: Could not convert brightness to number")
                    except KeyError:
                        print("Error: Invalid LED number")
            
            # Send the HTML page back to client
            html_page = create_html_page()
            client_conn.send(html_page.encode('utf-8'))
        
        # Close the connection
        client_conn.close()

except KeyboardInterrupt:
    print("\nShutting down server...")
    server_socket.close()
    
    # Turn off all LEDs
    for led in leds.values():
        led.value = 0
        led.close()
    
    print("Server stopped")
