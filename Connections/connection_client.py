# Threading
import socket
import threading
from Utils.utils import *


"""This module defines the Message Handler class that handles network communications."""

class Message_Handler:
    """This is the Message_Handler that handles communications over the network."""
    def __init__(self, android_receive_handler):
        """Initialize the sender."""

        # # Socket to listen for android commands
        # self._android_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self._android_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self._android_sock.connect((WIFI_HOST, RPI_PORT))
        #
        # # Socket to listen for arduino responses
        # self._arduino_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self._arduino_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self._arduino_sock.connect((WIFI_HOST, RPI_PORT))

        # Socket to listen for messages from RPi
        self._rpi_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._rpi_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rpi_sock.connect((WIFI_HOST, RPI_PORT))

        self._android_recv_queue = []
        self._arduino_recv_queue = []
        self._rpi_recv_queue = []
        self._android_receive_handler = android_receive_handler

        # # Start listener threads
        # arduino_recv_thread = threading.Thread(target=self._receiver_arduino, args=(self._arduino_sock,))
        # arduino_recv_thread.daemon = True
        # arduino_recv_thread.start()
        #
        # android_recv_thread = threading.Thread(target=self._receiver_android, args=(self._android_sock,))
        # android_recv_thread.daemon = True
        # android_recv_thread.start()

        rpi_recv_thread = threading.Thread(target=self._receiver_rpi, args=(self._rpi_sock,))
        rpi_recv_thread.daemon = True
        rpi_recv_thread.start()

    def _receiver_rpi(self, sock):
        enable_print()
        """
        Listen for messages from the RPi and store the messages into a queue.

        :param sock: The socket to listen on.
        :return: N/A
        """
        while True:
            data = sock.recv(1024)
            data = data.decode().strip()
            print('RECEIVED PRi: {}'.format(data))
            if not data:
                print('RPi disconnected')
                break

            # 'P{"status":"explore done"}P{"status":"explore done"}'
            if data[0] == 'R':
                data = data.split('R')
                data[:] = [x for x in data if x != '']
                print('Data from RPi: {}'.format(data))
                self._rpi_recv_queue.extend(data)
                print('rpi_recv_queue: {}'.format(self._rpi_recv_queue))

            if data[0] == 'T':
                data = data.split('T')
                data[:] = [x for x in data if x != '']
                print('Data from Android: {}'.format(data))
                self._android_recv_queue.extend(data)
                print('android_recv_queue: {}'.format(self._android_recv_queue))

            if data[0] == 'A':
                data = data.split('A')
                data[:] = [x for x in data if x != '']
                print('Data from Arduino: {}'.format(data))
                self._arduino_recv_queue.extend(data)
                print('arduino_recv_queue: {}'.format(self._arduino_recv_queue))

            if self._android_recv_queue:
                next_command = self._android_recv_queue.pop(0)
                print('Pop Andoird Command: {}'.format(next_command))
                print('arduino_recv_queue: {}'.format(self._arduino_recv_queue))
                self._android_receive_handler(next_command)

    # def _receiver_android(self, sock):
    #     enable_print()
    #     """
    #     Listen for messages from the Android and store the messages in a queue.
    #
    #     :param sock: The socket to listen on.
    #     :return: N/A
    #     """
    #     while True:
    #         data = sock.recv(1024)
    #         data = data.decode().strip()
    #         print('RECEIVED ANDROID', data)
    #         if not data:
    #             print('RPi disconnected')
    #             break
    #
    #         if data[0] == 'P':
    #             data = data.split('P')
    #             data[:] = [x for x in data if x != '']
    #             print(data)
    #             self._android_recv_queue.extend(data)
    #
    #         if self._android_recv_queue:
    #             next_command = self._android_recv_queue.pop(0)
    #             self._receive(next_command)

    # def _receiver_arduino(self, sock):
    #     enable_print()
    #     """
    #     Listen for messages from the Arduino and store the messages into a queue.
    #
    #     :param sock: The socket to listen on.
    #     :return: N/A
    #     """
    #     while True:
    #         data = sock.recv(1024)
    #         data = data.decode().strip()
    #         print('RECEIVED ARDUINO', data)
    #         if not data:
    #             print('RPi disconnected')
    #             break
    #
    #         if data[0] == 'P':
    #             data = data.split('P')
    #             data[:] = [x for x in data if x != '']
    #             print(data)
    #             self._arduino_recv_queue.extend(data)


    def send_android(self, msg):
        """Send a message to the Android."""
        to_send = 'T%s' % msg
        _send(self._rpi_sock, to_send)

    def send_arduino(self, msg):
        """Send a message to the Arduino."""
        to_send = 'A%s' % msg
        _send(self._rpi_sock, to_send)

    def send_rpi(self, msg):
        """Send a message to the RPi."""
        to_send = 'R%s' % msg
        _send(self._rpi_sock, to_send)

    def wait_arduino(self, msg_or_pattern, is_regex=False):
        enable_print()
        """
        Wait for a message from the Arduino.

        :param msg_or_pattern: message to wait for, or pattern for message to match.
        :param is_regex: true if waiting for pattern, false if waiting for message.
        :return: returns matched string if waiting for pattern, nothing otherwise.
        """
        print("WAITING", msg_or_pattern)
        while True:
            if self._arduino_recv_queue:
                next_command = self._arduino_recv_queue.pop(0)

                if not is_regex:
                    if next_command == msg_or_pattern:
                        print("RECEIVED ARDUINO", next_command)
                        break
                else:
                    match = msg_or_pattern.fullmatch(next_command)
                    if match:
                        print("RECEIVED ARDUINO", next_command)
                        return next_command

    # def wait_rpi(self, msg):
    #     """
    #     Wait for a message from the RPi.
    #
    #     :param msg: The message to wait for.
    #     :return: N/A
    #     """
    #     while True:
    #         if self._rpi_recv_queue:
    #             next_command = self._rpi_recv_queue.pop(0)
    #
    #             if next_command == msg:
    #                 print("WAITED RPI", next_command)
    #                 break

def _send(sock, msg):
    """Send a message on a socket."""
    print("SENDING", msg)
    sock.sendall(msg.encode())
