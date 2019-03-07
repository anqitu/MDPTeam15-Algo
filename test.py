FORWARD = 0                                             # Move forward 1 square
LEFT = -1                                               # Turn left and move forward 1 square
RIGHT = 1                                               # Turn right and move forward 1 square
BACKWARD = 2

# Messages to Arduino
ARDUINO_SENSOR = 'R'
ARDUINO_FORWARD = 'W'
ARDUINO_TURN_LEFT = 'A'
ARDUINO_TURN_RIGHT = 'D'
ARDUINO_TURN_TO_BACKWARD = 'S'
ARDUIMO_MOVED = 'M'


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


fastest_path = [2, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 1, 0, 0, 0]
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

move_str = ''
for move in fastest_path:
    if move != FORWARD:
        move_str += get_arduino_cmd(move)
    move_str += get_arduino_cmd(FORWARD)
moves = [move_str]
