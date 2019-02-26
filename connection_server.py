import socket
from Utils.utils import *
import time
import threading

"""This module runs a dummy server to simulate the RPi to facilitate testing."""

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((WIFI_HOST, RPI_PORT))
    s.listen(5)
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)

        def recv():
            while True:
                data = conn.recv(1024)
                data = data.decode()
                if data:
                    print('Received a message from PC: {}'.format(data))

        threading.Thread(target=recv).start()

        while True:
            data = input('TO SEND:')
            if data == 'k':
                s.close()
                break
            data = str.encode(data, "UTF-8")
            conn.sendall(data)
