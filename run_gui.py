from Controllers.gui import Window
from tkinter import *
from Utils.utils import *

"""This module starts an instance of the GUI application that simulates a run."""

__author__ = 'MDPTeam15'


if __name__ == '__main__':

    if not IS_DEBUG_MODE:
        disable_print()

    top = Tk()

    app = Window(top)

    top.mainloop()
