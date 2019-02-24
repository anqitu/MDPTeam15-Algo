from Connections.connection_client import Message_Handler
from time import sleep

"""This module drains a fully-charged battery to the level of power at which the robot runs most optimally."""

if __name__ == '__main__':
    to_send = 'n' * 137
    run_times = 12

    sender = Message_Handler(None)
    for i in range(run_times):
        sender.send_arduino(to_send)
        print(i)
        for _ in range(137):
            sender.wait_arduino('M')
        sleep(0.5)
