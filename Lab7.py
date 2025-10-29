import socket
from gpiozero import PWMOutputDevice

# Initialize PWM LEDs on GPIO pins 17, 27, 22
leds = {
    '1': PWMOutputDevice(17),
    '2': PWMOutputDevice(27),
    '3': PWMOutputDevice(22),
}

brightness = {'1': 0, '2': 0, '3': 0}

def handle_post(data):
    # Parse form data like 'led=1&brightness=50'
    lines = data.split('\r\n')
    body = lines[-1]
    params = body.split('&')
    led = None
    level = None
    for p in params:
        k, v = p.split('=')
        if k == 'led': led = v
        if k == 'brightness': level = int(v)
    if led and level is not None:
        brightness[led] = level
        leds[led].value = level / 100.0

def generate_html():
    # Returns HTML form and status page
    return f"""
    <html>
    <body>
        <h2>Brightness level:</h2>
        <ul>
            <li>LED 1 ({brightness['1']}%)</li>
            <li>LED 2 ({brightness['2']}%)</li>
            <li>LED 3 ({brightness['3']}%)</li>
        </ul>
        <form method="POST">
            Select LED:<br>
            <input type="radio" name="led" value="1"> LED 1<br>
            <input type="radio" name="led" value="2"> LED 2<br>
            <input type="radio" name="led" value="3"> LED 3<br>
            Brightness: <input type="range" name="brightness" min="0" max="100"><br>
            <input type="submit" value="Change Brightness">
        </form>
    </body>
    </html>
    """

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('0.0.0.0', 8080))
        s.listen()
        while True:
            conn, addr = s.accept()
            with conn:
                data = conn.recv(1024).decode()
                if 'POST' in data:
                    handle_post(data)
                response = generate_html()
                conn.send(b'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n' + response.encode())

if __name__ == '__main__':
    start_server()
