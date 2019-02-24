from tkinter import *

import Controllers.controller_with_gui as gui

"""This module starts an instance of the GUI application that simulates a run."""

__author__ = 'MDPTeam15'


if __name__ == '__main__':

    top = Tk()

    app = gui.Window(top)

    top.mainloop()
