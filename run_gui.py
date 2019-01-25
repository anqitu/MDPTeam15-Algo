from tkinter import *

import GUI.main_window as mw

"""This module starts an instance of the GUI application that simulates a run."""

__author__ = 'Harold Lim Jie Yu (1621635L)'
__email__ = 'HARO0002@e.ntu.edu.sg'


if __name__ == '__main__':

    top = Tk()

    app = mw.Window(top)

    top.mainloop()
