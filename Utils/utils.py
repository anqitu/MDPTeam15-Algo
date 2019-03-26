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
    # 0, 1, 2
    # 3, r, 4
    # 5, 6, 7
    cells = [cell + ROW_LENGTH - 1, cell + ROW_LENGTH, cell + ROW_LENGTH + 1,
             cell - 1, cell, cell + 1,
             cell - (ROW_LENGTH + 1), cell - ROW_LENGTH, cell - (ROW_LENGTH - 1)]

    return cells

def is_at_border(y, x):
    if y < 1 or y > 18 or x < 1 or x > 13:
        return True
    return False


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
        return ARDUINO_FORWARD
    if direction == LEFT:
        return ARDUINO_TURN_LEFT
    if direction == BACKWARD:
        return ARDUINO_TURN_TO_BACKWARD
    if direction == RIGHT:
        return ARDUINO_TURN_RIGHT

def convert_arduino_cmd_to_direction(cmd):
    """ Return the appropriate command to send to the Arduino for it to turn or move in a certain direction. """
    if cmd == ARDUINO_FORWARD or cmd == ARDUINO_FORWARD[0]:
        return FORWARD
    if cmd == ARDUINO_TURN_LEFT:
        return LEFT
    if cmd == ARDUINO_TURN_TO_BACKWARD:
        return BACKWARD
    if cmd == ARDUINO_TURN_RIGHT:
        return RIGHT

def get_fastest_path_moves(fastest_path):
    """ Calculate and return the list of moves the robot has to make given a path. """

    # fastest_path = [0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 1, 0, 0, 0]
    moves = []
    for move in fastest_path:
        if move != FORWARD:
            moves.append(get_arduino_cmd(move))
            moves.append(get_arduino_cmd(FORWARD))
        else:
            if len(moves) == 0 :
                moves.append(get_arduino_cmd(FORWARD))
            else:
                moves[-1] = moves[-1] + get_arduino_cmd(FORWARD)

    print('Original Commands: {}'.format(moves))

    moves_arduino = []
    for move in moves:
        moves_arduino = moves_arduino + [move[i:i+FAST_PATH_STEP] for i in range(0, len(move), FAST_PATH_STEP)]

    print('Arduino Commands: {}'.format(moves_arduino))

    return moves_arduino

def add_calibration_to_arduino_moves(moves_arduino, robot):
    from Algo.real_robot import Robot
    clone_robot = Robot(exploration_status=robot.exploration_status,
                        facing=robot.facing,
                        discovered_map=robot.discovered_map)
    is_calibration = False
    moves_ardiono_with_calibration = []
    for moves in moves_arduino:
        if moves[0] is not 'W':
            is_calibration = True
        new_move = ''
        for move in moves:
            if move != 'B':
                new_move += move
                print(move)
                clone_robot.move_robot_algo(convert_arduino_cmd_to_direction(move))
                if is_calibration and clone_robot.is_calibrate_side_possible():
                    if new_move[0] == 'W':
                        new_move += 'B'
                    moves_ardiono_with_calibration.append(new_move)
                    moves_ardiono_with_calibration.append('C')
                    new_move = ''
        if new_move != '':
            if new_move[0] == 'W':
                new_move += 'B'
            moves_ardiono_with_calibration.append(new_move)
    print('Arduino Commands with Calidation: {}'.format(moves_ardiono_with_calibration))

    return moves_ardiono_with_calibration


def disable_print():
    """ Suppress output from the print() function by piping stdout to /dev/null. """
    if not IS_DEBUG_MODE:
        sys.stdout = open(os.devnull, 'w')


def enable_print():
    """ Allow output from the print() function by piping stdout back to the console. """
    if not IS_DEBUG_MODE:
        sys.stdout = sys.__stdout__

# explore_string = 'fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff3'
def convert_explore_string_to_map(explore_string):
    # Trim '0b' and padding sequences of ‘11’ at the beginning and end.
    explore_string = bin(int(explore_string, 16))[4:-2]
    # Convert string to list of list
    # Reverse the row direction to from top to bottom
    explore_map = [[int(j) for j in explore_string[i:i+15]] for i in range(0, len(explore_string), 15)][::-1]
    return explore_map

# obstacle_string = '000000000400000001c800000000000700000000800000001f8000070000000002000000000'
def convert_obstacle_string_to_map(obstacle_string, explore_map):
    explored_map_count = sum([sum(row) for row in explore_map])
    # Trim '0b' and padding sequences of ‘1111’ at the beginning.
    # Trim the padding '0' at the end to match to the count of explored cells according to explored_map_count
    obstacle_string = bin(int('f' + obstacle_string, 16))[6:][:explored_map_count]

    # Initialize discovered map as all undiscovered (represented by 2)
    discovered_string = [2 for _ in range(ROW_LENGTH * COL_LENGTH)]

    # Convert explore map from list to string. (Inverse direction to match obstacle string, which is from bottom to up)
    explore_string = [item for sublist in explore_map[::-1] for item in sublist]

    # Fill 0 or 1 for explored cell
    obstacle_string_index = 0
    for i in range(len(explore_string)):
        if explore_string[i] == 1:
            discovered_string[i] = obstacle_string[obstacle_string_index]
        obstacle_string_index += 1

    # Convert string to list of list
    # Reverse the row direction to from top to bottom
    discovered_map = [[int(j) for j in discovered_string[i:i+15]] for i in range(0, len(discovered_string), 15)][::-1]

    return discovered_map

def print_map_info(robot):
    print('-' * 50)
    msgs = []
    msgs.append('"exploreMap":"%s"'%robot.get_explore_string())
    msgs.append('"obstacleMap":"%s"'%robot.get_map_string())
    y, x = get_matrix_coords(robot.center)
    msgs.append('"robotPosition":"%s,%s,%s"' % (str(x), str(y), str(robot.facing)))
    msgs.append('"ARrobotPosition":"%s,%s,%s"' % (str(x), str(19 - y), str(robot.facing)))
    msgs.append('"arrowPosition":"{}"'.format(','.join([str(value) for pos in robot.arrows for value in pos])))
    msgs.append('"ARarrowPosition":"{}"'.format(';'.join(robot.arrows_arduino)))
    print('{' + ','.join(msgs) + '}')

    print('Exploration Status Map:')
    for _ in robot.exploration_status[::-1]:
        print(_)

    print('Discovered Map:')
    for _ in robot.discovered_map[::-1]:
        print(_)

    # if IS_ARROW_SCAN:
    #     print('Arrow Taken Status Map:')
    #     for _ in robot.arrow_taken_status[::-1]:
    #         print(_)

    # if robot.real_map:
    #     print('Real Map:')
    #     for _ in robot.real_map:
    #         print(_)

    print()
