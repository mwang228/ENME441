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
    # Build the HTML page with JavaScript for real-time control
    html_content = f"""HTTP/1.1 200 OK
Content-Type: text/html
Connection: close

<html>
<head>
    <title>LED Brightness Control</title>
    <style>
        body {{ 
            font-family: Arial, sans-serif; 
            margin: 30px; 
            background-color: #f5f5f5;
        }}
        .container {{
            max-width: 500px;
            margin: 0 auto;
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }}
        h1 {{
            text-align: center;
            color: #333;
        }}
        .led-control {{
            margin: 20px 0;
            padding: 15px;
            border: 1px solid #ddd;
            border-radius: 5px;
            background: #f9f9f9;
        }}
        .led-label {{
            font-weight: bold;
            margin-bottom: 10px;
            display: block;
        }}
        .slider-container {{
            display: flex;
            align-items: center;
            gap: 15px;
        }}
        .slider {{
            flex: 1;
        }}
        .brightness-value {{
            font-weight: bold;
            min-width: 40px;
            text-align: center;
        }}
        .status {{
            margin-top: 20px;
            padding: 10px;
            background: #e8f5e8;
            border-radius: 5px;
            text-align: center;
            display: none;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>LED Brightness Control</h1>
        <p style="text-align: center; color: #666;">Move any slider to instantly change LED brightness</p>
        
        <div class="led-control">
            <span class="led-label">LED 1</span>
            <div class="slider-container">
                <input type="range" class="slider" id="led1" min="0" max="100" value="{brightness['1']}" 
                       oninput="updateLED('1', this.value)">
                <span class="brightness-value" id="value1">{brightness['1']}%</span>
            </div>
        </div>
        
        <div class="led-control">
            <span class="led-label">LED 2</span>
            <div class="slider-container">
                <input type="range" class="slider" id="led2" min="0" max="100" value="{brightness['2']}" 
                       oninput="updateLED('2', this.value)">
                <span class="brightness-value" id="value2">{brightness['2']}%</span>
            </div>
        </div>
        
        <div class="led-control">
            <span class="led-label">LED 3</span>
            <div class="slider-container">
                <input type="range" class="slider" id="led3" min="0" max="100" value="{brightness['3']}" 
                       oninput="updateLED('3', this.value)">
                <span class="brightness-value" id="value3">{brightness['3']}%</span>
            </div>
        </div>
        
        <div class="status" id="statusMessage"></div>
    </div>

    <script>
        function updateLED(ledNumber, brightness) {{
            // Update the display immediately
            document.getElementById('value' + ledNumber).textContent = brightness + '%';
            
            // Show loading status
            var status = document.getElementById('statusMessage');
            status.textContent = 'Updating LED ' + ledNumber + '...';
            status.style.display = 'block';
            status.style.background = '#fff3cd';
            
            // Create form data
            var formData = new FormData();
            formData.append('led', ledNumber);
            formData.append('brightness', brightness);
            
            // Send POST request to server
            fetch('/', {{
                method: 'POST',
                body: formData
            }})
            .then(response => {{
                if (response.ok) {{
                    status.textContent = 'LED ' + ledNumber + ' updated to ' + brightness + '%';
                    status.style.background = '#e8f5e8';
                }} else {{
                    status.textContent = 'Error updating LED ' + ledNumber;
                    status.style.background = '#f8d7da';
                }}
            }})
            .catch(error => {{
                status.textContent = 'Network error: ' + error;
                status.style.background = '#f8d7da';
            }});
            
            // Hide status after 2 seconds
            setTimeout(function() {{
                status.style.display = 'none';
            }}, 2000);
        }}
        
        // Add smooth interaction for sliders
        document.addEventListener('DOMContentLoaded', function() {{
            var sliders = document.querySelectorAll('.slider');
            sliders.forEach(function(slider) {{
                slider.addEventListener('mousedown', function() {{
                    this.style.cursor = 'grabbing';
                }});
                slider.addEventListener('mouseup', function() {{
                    this.style.cursor = 'grab';
                }});
            }});
        }});
    </script>
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
print("Problem 2: JavaScript version - sliders update LEDs without page reload")

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
