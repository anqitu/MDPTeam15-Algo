import socket
import argparse

"""This module runs a dummy server to simulate the RPi to facilitate testing."""

parser = argparse.ArgumentParser()
parser.add_argument('port', help='port number to connect to')
args = parser.parse_args()

HOST = '127.0.0.1'
PORT = int(args.port)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(5)
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        data = input('TO SEND:')
        data = str.encode(data, "UTF-8")
        conn.sendall(data)

        data = input('TO SEND:')
        data = str.encode(data, "UTF-8")
        conn.sendall(data)

        while True:
            data = conn.recv(1024)
            data = data.decode()
            print(data)
