from Utils.constants import *
import os
import sys

""" This module contains miscellaneous functions that are required throughout the program. """

def get_matrix_coords(cell):
    """ Calculate and return the yx coordinates of a given cell index. """
    x = (cell - 1) % NUM_COLS
    y = (cell - 1) // NUM_COLS

    return y, x


def get_grid_index(y, x):
    """ Calculate and return the cell index given its xy coordinates. """
    if y not in range(COL_LENGTH) or x not in range(ROW_LENGTH):
        raise IndexError

    return (y * ROW_LENGTH) + x + 1


def get_robot_cells(cell):
    """ Calculate and return the list of indexes of the cells that the robot currently covered. """
    cells = [cell + ROW_LENGTH - 1, cell + ROW_LENGTH, cell + ROW_LENGTH + 1,
             cell - 1, cell, cell + 1,
             cell - (ROW_LENGTH + 1), cell - ROW_LENGTH, cell - (ROW_LENGTH - 1)]

    return cells


def previous_cell(cell, facing):
    """
    Calculate and return the xy coordinates of the previous cell the robot was standing on

    This function will calculate the xy coordinates given the current location and facing of the robot.

    :param cell: The current center of the robot.
    :param facing: The current facing of the robot.
    :return: The xy coordinates of the previous cell the robot was standing on.
    """
    y, x = get_matrix_coords(cell)
    if facing == 0:
        return y - 1, x
    elif facing == 1:
        return y, x - 1
    elif facing == 2:
        return y + 1, x
    elif facing == 3:
        return y, x + 1
    return None


def get_arduino_cmd(direction):
    """ Return the appropriate command to send to the Arduino for it to turn or move in a certain direction. """
    if direction == FORWARD:
        return 'w'
    if direction == LEFT:
        return 'a'
    if direction == BACKWARD:
        return 'dd'
    if direction == RIGHT:
        return 'd'


def get_fastest_path_move_string(fastest_path, for_exploration=False):
    """ Calculate and return the list of moves the robot has to make given a path. """
    move_str = ''
    for move in fastest_path:
        if move == RIGHT or move == BACKWARD or move == LEFT:
            move_str += '/'
            move_str += get_arduino_cmd(move)
            move_str += '/'

        if for_exploration:
            move_str += get_arduino_cmd(FORWARD)
        else:
            move_str += 'n'

    print(move_str)

    return move_str


def disable_print():
    """ Suppress output from the print() function by piping stdout to /dev/null. """
    if not DEBUG_MODE:
        sys.stdout = open(os.devnull, 'w')


def enable_print():
    """ Allow output from the print() function by piping stdout back to the console. """
    if not DEBUG_MODE:
        sys.stdout = sys.__stdout__
