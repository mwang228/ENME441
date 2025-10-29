import socket
from gpiozero import PWMOutputDevice

led1 = PWMOutputDevice(17)
led2 = PWMOutputDevice(27)
led3 = PWMOutputDevice(22)

leds = {
    '1': led1,
    '2': led2, 
    '3': led3
}

brightness = {'1': 0, '2': 0, '3': 0}

def get_post_data(request_data):
    blank_line = request_data.find('\r\n\r\n')
    if blank_line == -1:
        return {}
    
    post_body = request_data[blank_line + 4:]
    
    pairs = post_body.split('&')
    result = {}
    
    for pair in pairs:
        if '=' in pair:
            key, value = pair.split('=', 1)
            result[key] = value
    
    return result

def create_html_page():
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

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 8080))
server_socket.listen(5)

print("http://your-pi-address:8080")


while True:
    client_conn, client_addr = server_socket.accept()
    
    request = client_conn.recv(1024).decode('utf-8')
    
    if request:
        if request.startswith('POST'):
            post_data = get_post_data(request)
            
            led_number = post_data.get('led')
            brightness_value = post_data.get('brightness')
            
            if led_number and brightness_value:
                bright_level = int(brightness_value)
                brightness[led_number] = bright_level
                
                leds[led_number].value = bright_level / 100.0
                
                print(f"LED {led_number} set to {bright_level}%")

        html_page = create_html_page()
        client_conn.send(html_page.encode('utf-8'))
    client_conn.close()
