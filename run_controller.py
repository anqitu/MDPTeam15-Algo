from Controllers.controller import Controller
from Utils.utils import *

"""This module starts an instance of the Algorithm application that controls the robot during a physical run."""

if __name__ == '__main__':

    if not IS_DEBUG_MODE:
        disable_print()

    # Initialize Controller
    controller = Controller()

    # Keep main thread alive to keep daemons alive
    while True:
        pass
