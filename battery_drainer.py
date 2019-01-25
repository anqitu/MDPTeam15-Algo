from Connections.connection_client import Sender
from time import sleep

"""This module drains a fully-charged battery to the level of power at which the robot runs most optimally."""

__author__ = 'Harold Lim Jie Yu (1621635L)'
__email__ = 'HARO0002@e.ntu.edu.sg'


if __name__ == '__main__':
    to_send = 'n' * 137
    run_times = 12

    sender = Sender(None)
    for i in range(run_times):
        sender.send_arduino(to_send)
        print(i)
        for _ in range(137):
            sender.wait_arduino('M')
        sleep(0.5)
