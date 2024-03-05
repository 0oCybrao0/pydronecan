import socketserver
import threading
import socket
import serial
from argparse import ArgumentParser

parser = ArgumentParser(description='Serial forwarder')
parser.add_argument("--port", default=2001, type=int, help="Listen port")
parser.add_argument("--serial-port", "-d", default='/dev/ttyUSB0', type=str, help="Serial port for USB device")
parser.add_argument("--baud-rate", default=115200, type=int, help="Baud rate for USB device")
args = parser.parse_args()

ucenter_connected = False  # Flag to track u-center connection status

class UCenterHandler(socketserver.BaseRequestHandler):
    def handle(self):
        global ucenter_connected
        ucenter_connected = True  # Set the flag when a connection is established
        data = self.request.recv(1024)
        print("Received from client:", data.decode())
        # Process the received data here if needed
        # For now, let's just echo it back to the client
        self.request.sendall(data)

def start_ucenter_server():
    with socketserver.TCPServer(('localhost', args.port), UCenterHandler) as server:
        print("u-center server running on port", args.port)
        server.serve_forever()

def is_ucenter_connected():
    return ucenter_connected

def send_to_ucenter(message):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect(('localhost', args.port))
            sock.sendall(message)
    except Exception as e:
        print("Error sending message to u-center:", e)

def read_from_usb_device():
    with serial.Serial(args.serial_port, args.baud_rate) as ser:
        while True:
            if is_ucenter_connected():
                # Read data from USB device
                data = ser.read(ser.in_waiting or 1)
                if data:
                    print("Received from USB device:", data)
                    send_to_ucenter(data)

if __name__ == "__main__":
    # Start the u-center server in a separate thread
    ucenter_thread = threading.Thread(target=start_ucenter_server)
    ucenter_thread.daemon = True
    ucenter_thread.start()

    # Start reading data from USB device
    usb_thread = threading.Thread(target=read_from_usb_device)
    usb_thread.daemon = True
    usb_thread.start()

    # Check if u-center is connected
    while True:
        if is_ucenter_connected():
            print("u-center is connected")
            ucenter_thread.join()
            usb_thread.join()
