from Controllers.controller_gui import Window
from tkinter import *
from Utils.utils import *

"""This module starts an instance of the GUI application that simulates a run."""

__author__ = 'MDPTeam15'


if __name__ == '__main__':

    top = Tk()

    app = Window(top)

    top.mainloop()
