from flask import Flask, render_template
from flask_socketio import SocketIO
from bluepy.btle import Peripheral, DefaultDelegate
import threading

app = Flask(__name__)
app.config['SECRET_KEY'] = b'*z\xf8\x99i\xed\xc6\xb1\xeb\xd6\xc4?\xde\xabp\xac\xa8\xd0j\x81"x3F'

socketio = SocketIO(app, cors_allowed_origins="*")

class MyDelegate(DefaultDelegate):
    def __init__(self, socketio):
        super().__init__()
        self.socketio = socketio

    def handleNotification(self, cHandle, data):
        decoded_data = data.decode()
        if decoded_data.startswith(('a:', 'g:', 'm:', 'r:', 'p:', 'y:')):
            self.socketio.emit('sensor_data', {'data': decoded_data})

def connect_and_listen(device_address, socketio):
    try:
        nano_peripheral = Peripheral(device_address)
        nano_peripheral.setDelegate(MyDelegate(socketio))

        print("Connected to Nano33BLE")
        nano_service = nano_peripheral.getServiceByUUID("12345678-1234-1234-1234-123456789ABC")

        for characteristic in nano_service.getCharacteristics():
            # Add additional UUIDs for roll, pitch, yaw characteristics
            if characteristic.uuid in ["87654321-4321-4321-4321-210987654321", 
                                       "87654321-4321-4321-4321-210987654322", 
                                       "87654321-4321-4321-4321-210987654323", 
                                       "87654321-4321-4321-4321-210987654324", 
                                       "87654321-4321-4321-4321-210987654325", 
                                       "87654321-4321-4321-4321-210987654326"]:
                nano_peripheral.writeCharacteristic(characteristic.valHandle + 1, b"\x01\x00")

        while True:
            if nano_peripheral.waitForNotifications(1.0):
                continue

    finally:
        nano_peripheral.disconnect()
        print("Disconnected")

@app.route('/')
def index():
    return render_template('index.html')

def background_thread():
    device_address = "7a:d8:ee:7d:4d:1e"  # Replace with your device's address
    connect_and_listen(device_address, socketio)

if __name__ == '__main__':
    thread = threading.Thread(target=background_thread)
    thread.daemon = True
    thread.start()
    socketio.run(app, host='0.0.0.0', port=5000)
