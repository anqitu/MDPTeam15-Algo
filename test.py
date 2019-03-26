from Utils.utils import *

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

FAST_PATH_STEP = 100
get_fastest_path_moves([0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 1, 0, 0, 0])
moves_arduino = get_fastest_path_moves([0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 1, 0, 0, 0])

clone_robot = Robot(exploration_status=self._robot.exploration_status,
                    facing=self._robot.facing,
                    discovered_map=self._robot.discovered_map,
                    real_map=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)])
is_calibration = False
moves_ardiono_with_calibration = []
for moves in moves_arduino:
    if moves[0] is not 'W':
        is_calibration = True
    new_move = ''
    for move in moves:
        if move != 'B':
            new_move += move
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


''.join(['WW', 'A', 'WWWWWWWWWW', 'D', 'C', 'WWWWW', 'A', 'WWWW', 'D', 'W', 'A', 'WWW', 'D', 'C', 'WWWW'])
