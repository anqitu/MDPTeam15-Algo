""" This module defines all the constants that are used throughout the program. """

""" Constants to play with (Start)"""
IS_DEBUG_MODE = True # Whether enable console printout at the cost of a slower run
IS_ARROW_SCAN = True # Whether scan for arrows during exploration
CALIBRATION_SIDE_STEPS = 10 # Number of steps per side calibration
CALIBRATION_FRONT_STEPS = 10 # Number of steps per front calibration
TURNING_STEP = 2
STRAIGHT_STEP = 1
COMPLETION_THRESHOLD = 300 # Target threshold
TIME_LIMITE = 330
FAST_PATH_STEP = 6 # Maximum of number of straight moves
FAST_PATH_SLEEP_SEC = 0.2 # Duration of delay per instruction during fast path
BATTERY_DRAINER_STEP_X = 2
BATTERY_DRAINER_STEP_Y = 2
BATTERY_DRAINER_TURN = 'D' #Clockwise
# BATTERY_DRAINER_TURN = 'A' #Anti-Clockwise
""" Constants to play with (End)"""

IS_SLEEP = False
SLEEP_SEC = 2

# Sockets
WIFI_HOST = '192.168.15.15'
# WIFI_HOST = '127.0.0.1'
RPI_PORT = 6688

# Directions
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3
CAMERA_FACING = WEST

DIRECTIONS = {
    0: 'NORTH',
    1: 'EAST',
    2: 'SOUTH',
    3: 'WEST'}

# Robot Direction
FORWARD = 0                                             # Move forward 1 square
LEFT = -1                                               # Turn left and move forward 1 square
RIGHT = 1                                               # Turn right and move forward 1 square
BACKWARD = 2                                            # Turn behind and move forward 1 square

MOVEMENTS = {
    0: 'FORWARD',
    -1: 'LEFT',
    1: 'RIGHT',
    2: 'BACKWARD'}

# Map Constants                                         Order cells from left to right, bottom to top
# 19  [286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300]
# 18  [271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285]
# 17  [256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270]
# 16  [241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255]
# 15  [226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240]
# 14  [211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225]
# 13  [196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210]
# 12  [181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195]
# 11  [166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180]
# 10  [151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165]
# 9   [136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150]
# 8   [121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135]
# 7   [106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120]
# 6   [91,  92,  93,  94,  95,  96,  97,  98,  99,  100, 101, 102, 103, 104, 105]
# 5   [76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90]
# 4   [61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75]
# 3   [46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60]
# 2   [31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45]
# 1   [16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30]
# 0   [1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15]
# y
#   x  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14

NUM_ROWS = COL_LENGTH = 20                              # Number of rows in the grid; length of one col
NUM_COLS = ROW_LENGTH = 15                              # Number of cols in the grid; length of one row
START = 1 + ROW_LENGTH + 1                              # Center of Start area --> 17
GOAL = (COL_LENGTH * ROW_LENGTH) - ROW_LENGTH - 1       # Center of Goal area --> 284

# Communication Constants
MOVE = 0
TURN = 1

MOVE_TURN = {0: 'MOVE', 1: 'TURN'}

# Limit                                                  Outer limits of the grid for the robot center
TOP_LEFT_LIMIT = (COL_LENGTH * ROW_LENGTH) - (2 * ROW_LENGTH) + 2               # 272 (18, 1)
TOP_RIGHT_LIMIT = GOAL                                                          # 284 (18, 13)
BOTTOM_LEFT_LIMIT = START                                                       # 17  (1, 1)
BOTTOM_RIGHT_LIMIT = 2 * ROW_LENGTH - 1                                         # 29  (1, 13)
LIMITS = {NORTH: range(TOP_LEFT_LIMIT, TOP_RIGHT_LIMIT + 1),
          SOUTH: range(BOTTOM_LEFT_LIMIT, BOTTOM_RIGHT_LIMIT + 1),
          EAST: range(BOTTOM_RIGHT_LIMIT, TOP_RIGHT_LIMIT + 1, ROW_LENGTH),
          WEST: range(BOTTOM_LEFT_LIMIT, TOP_LEFT_LIMIT + 1, ROW_LENGTH)}

# Borders                                                 Borders of the grid; robot center cannot reach here
TOP_LEFT_CORNER = (COL_LENGTH * ROW_LENGTH) - ROW_LENGTH + 1                    # 286  (19, 0)
TOP_RIGHT_CORNER = COL_LENGTH * ROW_LENGTH                                      # 300  (19, 14)
BOTTOM_LEFT_CORNER = 1                                                          # 1    (0,  0)
BOTTOM_RIGHT_CORNER = ROW_LENGTH                                                # 15   (0,  14)
BORDERS = {NORTH: range(TOP_LEFT_CORNER, TOP_RIGHT_CORNER + 1),
           SOUTH: range(BOTTOM_LEFT_CORNER, BOTTOM_RIGHT_CORNER + 1),
           EAST: range(BOTTOM_RIGHT_CORNER, TOP_RIGHT_CORNER + 1, ROW_LENGTH),
           WEST: range(BOTTOM_LEFT_CORNER, TOP_LEFT_CORNER + 1, ROW_LENGTH)}


# Efficient movement limits                               Limits of the grid if robot is to move efficiently ?
E_LIMITS = {
    NORTH: range(TOP_LEFT_LIMIT - ROW_LENGTH, TOP_RIGHT_LIMIT - ROW_LENGTH + 1),            # (17, 1) to (17, 13)
    SOUTH: range(BOTTOM_LEFT_LIMIT + ROW_LENGTH, BOTTOM_RIGHT_LIMIT + ROW_LENGTH + 1),      # (2, 1) to (2, 13)
    EAST: range(BOTTOM_RIGHT_LIMIT - 1, TOP_RIGHT_LIMIT - 1 + 1, ROW_LENGTH),               # (1, 12) to (18, 12)
    WEST: range(BOTTOM_LEFT_LIMIT + 1, TOP_LEFT_LIMIT + 1 + 1, ROW_LENGTH)}                 # (1, 2) to (18, 2)

# Sensor Constants
#   N
# 1  0  3
# 6 -1  2
# 7  4  5
NWS = 1
NES = 3
SES = 5
SWS = 7
NS = 0
ES = 2
SS = 4
WS = 6
CS = -1

# Cell types
UNEXPLORED = "#1A1E24"
EXPLORED = "#F3F3F3"
OBSTACLE = "#F44336"
START_AREA = "#30807D"
GOAL_AREA = "#08AE69"
PATH = "#7ACDC8"
WAY_POINT = "#673AB7"

# Messages from Andoird
ANDROID_WAYPOINT = 'waypoint'
ANDROID_CALIBRATE = 'ca'
ANDROID_EXPLORE = 'ex'
ANDROID_MOVE_FAST_PATH = 'fp'
ANDROID_LOAD_EXPLORE_MAP = 'lo'
ANDROID_FORWARD = 'fo'
ANDROID_TURN_LEFT = 'tl'
ANDROID_TURN_RIGHT = 'tr'
ANDROID_TURN_TO_BACKWARD = 'ba'
ANDROID_BATTERY_DRAINER = 'D'

# Messages to Arduino
ARDUINO_SENSOR = 'R'
ARDUINO_FORWARD = 'W'
ARDUINO_TURN_LEFT = 'A'
ARDUINO_TURN_RIGHT = 'D'
ARDUINO_TURN_TO_BACKWARD = 'S'
ARDUIMO_MOVED = 'M'

# RPi
API_TAKEN_PHOTO = 'I'

EXPLORE_STATUS_MAP = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]][::-1]

EXPLORATION_OBSTACLE_MAP_1 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
                              [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]][::-1]

EXPLORATION_OBSTACLE_MAP = EXPLORATION_OBSTACLE_MAP_1
