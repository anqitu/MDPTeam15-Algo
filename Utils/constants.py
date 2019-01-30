""" This module defines all the constants that are used throughout the program. """

# Directions
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

# Movement
FORWARD = 0                                             # Move forward 1 square
LEFT = -1                                               # Turn left and move forward 1 square
RIGHT = 1                                               # Turn right and move forward 1 square
BACKWARD = 2                                            # Turn behind and move forward 1 square

# Map Constants                                         Order cells from left to right, bottom to top
NUM_ROWS = COL_LENGTH = 20                              # Number of rows in the grid; length of one col
NUM_COLS = ROW_LENGTH = 15                              # Number of cols in the grid; length of one row
START = 1 + ROW_LENGTH + 1                              # Center of Start area
GOAL = (COL_LENGTH * ROW_LENGTH) - ROW_LENGTH - 1       # Center of Goal area

# Communication Constants
MOVE = 0
TURN = 1

# Limit                                                  Outer limits of the grid for the robot center
TOP_LEFT_LIMIT = (COL_LENGTH * ROW_LENGTH) - (2 * ROW_LENGTH) + 2
TOP_RIGHT_LIMIT = GOAL
BOTTOM_LEFT_LIMIT = START
BOTTOM_RIGHT_LIMIT = 2 * ROW_LENGTH - 1
LIMITS = {NORTH: range(TOP_LEFT_LIMIT, TOP_RIGHT_LIMIT + 1),
          SOUTH: range(BOTTOM_LEFT_LIMIT, BOTTOM_RIGHT_LIMIT + 1),
          EAST: range(BOTTOM_RIGHT_LIMIT, TOP_RIGHT_LIMIT + 1, ROW_LENGTH),
          WEST: range(BOTTOM_LEFT_LIMIT, TOP_LEFT_LIMIT + 1, ROW_LENGTH)}

# Borders                                                 Borders of the grid; robot center cannot reach here
TOP_LEFT_CORNER = (COL_LENGTH * ROW_LENGTH) - ROW_LENGTH + 1
TOP_RIGHT_CORNER = COL_LENGTH * ROW_LENGTH
BOTTOM_LEFT_CORNER = 1
BOTTOM_RIGHT_CORNER = ROW_LENGTH
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

# Disable printing to console
# DEBUG_MODE = False
DEBUG_MODE = True
ARROW_SCAN = False

# Sockets
HOST = '192.168.14.14'
ARDUINO_PORT = 5560
ANDROID_PORT = 6560
RPI_PORT = 7560
