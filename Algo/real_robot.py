import re

from Utils.utils import *
from time import time, sleep

"""This module defines the Robot class that represents the robot in a physical run."""

__author__ = "Harold Lim Jie Yu (U1621635L)"
__email__ = "HARO0002@e.ntu.edu.sg"


class Robot:
    """
    This class is a representation of the physical robot.
    """
    def __init__(self, exploration_status, facing, discovered_map):
        """
        Initialize the Robot class.

        :param exploration_status: The map that shows whether each cell is explored or unexplored.
        :param facing: The current facing of the robot. (N/S/E/W)
        :param discovered_map: The map built by the robot that shows whether each cell is an obstacle or not.
        """
        self.exploration_status = exploration_status
        self.center = START
        self.facing = facing
        self.discovered_map = discovered_map
        self._probability_map = [[[0.0, 0.0] for _ in range(ROW_LENGTH)] for _ in range(COL_LENGTH)]
        self._arrow_map = [[[0, 0, 0, 0] for _ in range(ROW_LENGTH)] for _ in range(COL_LENGTH)]
        self._sensors = [
            {"mount_loc": SWS, "facing": WEST, "range": 2, "blind_spot": 0},
            {"mount_loc": NWS, "facing": WEST, "range": 2, "blind_spot": 0},
            {"mount_loc": NWS, "facing": NORTH, "range": 3, "blind_spot": 0},
            {"mount_loc": NS, "facing": NORTH, "range": 3, "blind_spot": 0},
            {"mount_loc": NES, "facing": NORTH, "range": 3, "blind_spot": 0},
            {"mount_loc": NES, "facing": EAST, "range": 4, "blind_spot": 0}
        ]

        self.num_sensor_readings = 11
        regex_str = '^(\d,){%s}$' % (len(self._sensors) * self.num_sensor_readings)
        self._readings_regex = re.compile(regex_str)

    def _mark_probability(self, cell, count, total):
        """
        Mark the probability of a cell being an obstacle.

        If the cell has > 50% chance of being an obstacle, mark the cell as
        an obstacle. Ignore cells that have been marked as guaranteed non-obstacles.

        :param cell: The number of the cell being marked.
        :param count: The number of times the cell was flagged as an obstacle when this method was called.
        :param total: The number of times the cell was scanned when this method was called.
        :return: Nothing if the cell is marked 100% non-obstacle. The new value of the cell otherwise.
        """
        y, x = get_matrix_coords(cell)
        print(y, x, count, total)

        if self._probability_map[y][x][0] == 1.0 and self._probability_map[y][x][1] == 0.0:
            print('perm')
            return

        # Update counts
        self._probability_map[y][x][0] += count
        self._probability_map[y][x][1] += total

        prob_obstacle = self._probability_map[y][x][0]
        prob_total = self._probability_map[y][x][1]

        print(y, x, prob_obstacle, prob_total)

        if not self.exploration_status[y][x]:
            self.exploration_status[y][x] = 1

        if prob_obstacle / prob_total > 0.5:
            self.discovered_map[y][x] = 1
            return 1
        else:
            self.discovered_map[y][x] = 0
            return 0

    def _mark_permanent(self, cell):
        """
        Mark a cell as a guaranteed non-obstacle.

        Called when the robot walks over a cell.

        :param cell: The number of the cell being marked.
        :return: True if success. No definition of failure provided, however it is easy to add if required.
        """
        y, x = get_matrix_coords(cell)

        self._probability_map[y][x][0] = 1.0
        self._probability_map[y][x][1] = 0.0

        if not self.exploration_status[y][x]:
            self.exploration_status[y][x] = 1

        self.discovered_map[y][x] = 0

        return True

    def _mark_arrow_taken(self, y, x, facing):
        """
        Mark the face and opposite face of a cell having its picture taken by the arrow recognizer.

        :param y: The y-coordinate of the cell to be marked.
        :param x: The x-coordinate of the cell to be marked.
        :param facing: The facing of the robot when the photo was taken.
        :return: True if success. No definition of failure provided, however it is easy to add if required.
        """
        opposite = (facing + 2) % 4

        self._arrow_map[y][x][facing] = 1
        self._arrow_map[y][x][opposite] = 1

        return True

    def in_efficiency_limit(self):
        """
        Check if the robot is one grid before the maximum limit of the maze.

        The method is called to check if the robot has just moved past a 1-cell-width obstacle along
        the wall of the maze.

        :return: True if robot is in the limit, false otherwise.
        """
        if (self.center in E_LIMITS[NORTH] and self.facing == EAST) \
                or (self.center in E_LIMITS[EAST] and self.facing == SOUTH) \
                or (self.center in E_LIMITS[SOUTH] and self.facing == WEST) \
                or (self.center in E_LIMITS[WEST] and self.facing == NORTH):
            return True

        return False

    def mark_robot_standing(self):
        """
        Mark the area the robot is standing on as explored and guaranteed non-obstacles.

        :return: The cells that were updated.
        """
        robot_cells = get_robot_cells(self.center)
        updated_cells = {}
        mark_permanent = self._mark_permanent
        for cell in robot_cells:
            if mark_permanent(cell):
                updated_cells[cell] = 0

        return updated_cells

    def get_completion_percentage(self):
        """
        Calculate how much of the maze the robot has explored in percentage.

        :return: The percentage of the map the robot has explored.
        """
        count = 0.0
        for row in self.exploration_status:
            for i in row:
                count += i

        return (count / (NUM_COLS * NUM_ROWS)) * 100.0

    def is_complete(self, explore_limit, start_time, time_limit):
        """
        Check if the exploration is complete based on the exploration limit and the time limit.

        :param explore_limit: The percentage of the maze up to which the robot is allowed to explore.
        :param start_time: The start time of the exploration
        :param time_limit: The maximum time that the robot was allowed to explore until.
        :return: True if the exploration should be stopped, false otherwise.
        """
        completion = self.get_completion_percentage
        return completion() >= explore_limit \
            or float(time() - start_time >= time_limit)

    def turn_robot(self, sender, direction):
        """
        Turn the robot in a chosen direction.

        :param sender: The object that communicates with the RPi
        :param direction: The direction to turn (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Nothing. Stops the method if the direction is FORWARD to save time as the robot does not need to turn.
        """

        if direction == FORWARD:
            return

        if direction == BACKWARD:
            sender.send_arduino(get_arduino_cmd(RIGHT))
            self.facing = (self.facing + RIGHT) % 4
            direction = RIGHT
            sender.wait_arduino('M')
            sleep(0.05)

        sender.send_arduino(get_arduino_cmd(direction))
        self.facing = (self.facing + direction) % 4

        sender.wait_arduino('M')

        if ARROW_SCAN:
            self.check_arrow(sender)

    def move_robot(self, sender, direction):
        """
        Move the robot 1 step in a chosen direction.

        Turn the robot towards the chosen direction then move one step forward. Assume step is not obstacle.

        :param sender: The object that communicates with the RPi.
        :param direction: The direction to move (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Any cells that the robot has stepped on that it had not yet before.
        """
        self.turn_robot(sender, direction)

        sender.send_arduino(get_arduino_cmd(FORWARD))

        if self.facing == NORTH:
            self.center += ROW_LENGTH
        elif self.facing == EAST:
            self.center += 1
        elif self.facing == SOUTH:
            self.center -= ROW_LENGTH
        elif self.facing == WEST:
            self.center -= 1

        updated_cells = self.mark_robot_standing()

        sender.wait_arduino('M')

        if ARROW_SCAN:
            self.check_arrow(sender)

        return updated_cells

    def check_free(self, direction):
        """
        Check if the adjacent cells in the chosen direction have obstacles.

        :param direction: The direction to check (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: True if the robot is able to take one step in that direction, false otherwise
        """
        true_bearing = (self.facing + direction) % 4

        robot_cells = get_robot_cells(self.center)

        try:
            if true_bearing == NORTH:
                y, x = get_matrix_coords(robot_cells[0])
                y += 1
                if y < 0 or x < 0:
                    raise IndexError
                return not (self.discovered_map[y][x] == 1 or self.discovered_map[y][x + 1] == 1
                            or self.discovered_map[y][x + 2] == 1)
            elif true_bearing == EAST:
                y, x = get_matrix_coords(robot_cells[2])
                x += 1
                if y < 2 or x < 0:
                    raise IndexError
                return not (self.discovered_map[y][x] == 1 or self.discovered_map[y - 1][x] == 1
                            or self.discovered_map[y - 2][x] == 1)
            elif true_bearing == SOUTH:
                y, x = get_matrix_coords(robot_cells[6])
                y -= 1
                if y < 0 or x < 0:
                    raise IndexError
                return not (self.discovered_map[y][x] == 1 or self.discovered_map[y][x + 1] == 1
                            or self.discovered_map[y][x + 2] == 1)
            elif true_bearing == WEST:
                y, x = get_matrix_coords(robot_cells[0])
                x -= 1
                if y < 2 or x < 0:
                    raise IndexError
                return not (self.discovered_map[y][x] == 1 or self.discovered_map[y - 1][x] == 1
                            or self.discovered_map[y - 2][x] == 1)
        except IndexError:
            return False

    def is_arrow_possible(self):
        """
        Check if it is possible to have arrows in the chosen direction.

        The method also takes into consideration obstacles that have already been scanned for arrows. Will only return
        true if there are faces that have not been scanned that are facing the robot.

        :return: True if there are unscanned faces of obstacles in the path of the RPi camera, false otherwise.
        """
        arrow_range = 1
        y, x = get_matrix_coords(self.center)
        discovered_map = self.discovered_map
        arrow_map = self._arrow_map
        facing = self.facing

        try:
            for distance in range(2, arrow_range + 2):
                if facing == NORTH:
                    new_x = x - distance
                    if new_x < 0:
                        raise IndexError
                    print('checking %s,%s %s,%s %s,%s' % (y, new_x, y + 1, new_x, y - 1, new_x))
                    obstacles = [discovered_map[y][new_x] == 1, discovered_map[y + 1][new_x] == 1,
                                 discovered_map[y - 1][new_x] == 1]
                    marked = [arrow_map[y][new_x][facing], arrow_map[y + 1][new_x][facing],
                              arrow_map[y][new_x][facing]]
                    if any(obstacles) and not any(marked):
                        self._mark_arrow_taken(y, new_x, facing)
                        self._mark_arrow_taken(y + 1, new_x, facing)
                        self._mark_arrow_taken(y - 1, new_x, facing)
                        return True
                elif facing == EAST:
                    new_y = y + distance
                    print('checking %s,%s %s,%s %s,%s' % (new_y, x, new_y, x + 1, new_y, x - 1))
                    obstacles = [discovered_map[new_y][x] == 1, discovered_map[new_y][x + 1] == 1,
                                 discovered_map[new_y][x - 1] == 1]
                    marked = [arrow_map[new_y][x][facing], arrow_map[new_y][x + 1][facing],
                              arrow_map[new_y][x - 1][facing]]
                    if any(obstacles) and not any(marked):
                        self._mark_arrow_taken(new_y, x, facing)
                        self._mark_arrow_taken(new_y, x + 1, facing)
                        self._mark_arrow_taken(new_y, x - 1, facing)
                        return True
                elif facing == SOUTH:
                    new_x = x + distance
                    print('checking %s,%s %s,%s %s,%s' % (y, new_x, y + 1, new_x, y - 1, new_x))
                    obstacles = [discovered_map[y][new_x] == 1, discovered_map[y + 1][new_x] == 1,
                                 discovered_map[y - 1][new_x] == 1]
                    marked = [arrow_map[y][new_x][facing], arrow_map[y + 1][new_x][facing],
                              arrow_map[y][new_x][facing]]
                    if any(obstacles) and not any(marked):
                        self._mark_arrow_taken(y, new_x, facing)
                        self._mark_arrow_taken(y + 1, new_x, facing)
                        self._mark_arrow_taken(y - 1, new_x, facing)
                        return True
                elif facing == WEST:
                    new_y = y - distance
                    if new_y < 0:
                        raise IndexError
                    print('checking %s,%s %s,%s %s,%s' % (new_y, x, new_y, x + 1, new_y, x - 1))
                    obstacles = [discovered_map[new_y][x] == 1, discovered_map[new_y][x + 1] == 1,
                                 discovered_map[new_y][x - 1] == 1]
                    marked = [arrow_map[new_y][x][facing], arrow_map[new_y][x + 1][facing],
                              arrow_map[new_y][x - 1][facing]]
                    if any(obstacles) and not any(marked):
                        self._mark_arrow_taken(new_y, x, facing)
                        self._mark_arrow_taken(new_y, x + 1, facing)
                        self._mark_arrow_taken(new_y, x - 1, facing)
                        return True
            return False
        except IndexError:
            return False

    def check_arrow(self, sender):
        """
        Send the RPi a message to take a picture to check for arrows.

        Check if there are potential unscanned arrows in the field of view of the RPi camera.

        :param sender: The object that communicates with the RPi.
        :return: N/A
        """
        if self.is_arrow_possible():
            y, x = get_matrix_coords(self.center)
            msg = '%s,%s,%s' % (x, y, self.facing)
            enable_print()
            sender.send_rpi(msg)
            sender.wait_arduino('Y')
            disable_print()

        else:
            print(get_matrix_coords(self.center), 'false')

    def get_sensor_readings(self, sender):
        """
        Send a message to the Arduino to take sensor readings.


        The Arduino will take 11 readings from its sensors and return the distance at which it detects an obstacle
        for each sensor. 0 indicates no obstacle detected.

        The readings are split into a list of lists such that each inner list is one reading taken from each sensor.
        The resulting matrix is then transposed such that each inner list is now all readings taken from one
        sensor.

        The sensors are iterated through and the number of times a cell is detected as an obstacle is added to its
        running count. The total number of scans the cell has received is added to its running count of scans.

        The counts are weighted by distance, with the weight halving for every unit further away from the robot that
        the reading is taken.

        :param sender: The object that communicates with the RPi
        :return: The updated cell values and indexes.
        """
        mark_probability = self._mark_probability

        sender.send_arduino('g')
        readings = sender.wait_arduino(self._readings_regex, is_regex=True)
        readings = readings.split(',')
        del readings[-1]

        readings = [int(x) for x in readings]

        # split readings list into len(sensors)-sized chunks
        readings = [readings[i:i + len(self._sensors)] for i in range(0, len(readings), len(self._sensors))]

        # transpose list so that each row is the list of readings for that sensor
        readings = [[row[i] for row in readings] for i, _ in enumerate(readings[0])]

        robot_cells = get_robot_cells(self.center)
        sensors = self._sensors[:]
        sensor_index = sensors.index
        updated_cells = {}

        for sensor in sensors:
            true_facing = (sensor["facing"] + self.facing) % 4

            if sensor["mount_loc"] != CS:
                offset = self.facing * 2
                true_mounting = (sensor["mount_loc"] + offset) % 8
            else:
                true_mounting = CS

            if true_mounting == NWS:
                origin = robot_cells[0]
            elif true_mounting == NS:
                origin = robot_cells[1]
            elif true_mounting == NES:
                origin = robot_cells[2]
            elif true_mounting == WS:
                origin = robot_cells[3]
            elif true_mounting == ES:
                origin = robot_cells[5]
            elif true_mounting == SWS:
                origin = robot_cells[6]
            elif true_mounting == SS:
                origin = robot_cells[7]
            elif true_mounting == SES:
                origin = robot_cells[8]
            elif true_mounting == CS:
                origin = robot_cells[4]

            y, x = get_matrix_coords(origin)
            read_range = list(range(sensor["blind_spot"] + 1, sensor["range"] + 1))

            reading = readings[sensor_index(sensor)]
            print('Sensor', sensor_index(sensor))

            weight = 4
            for cell in read_range:
                try:
                    if true_facing == NORTH:
                        to_explore = (y + cell, x)
                    elif true_facing == EAST:
                        to_explore = (y, x + cell)
                    elif true_facing == SOUTH:
                        to_explore = (y - cell, x)
                    elif true_facing == WEST:
                        to_explore = (y, x - cell)

                    if to_explore[0] < 0 or to_explore[1] < 0:
                        print('ie')
                        raise IndexError

                    cell_index = get_grid_index(to_explore[0], to_explore[1])

                    if mark_probability(cell_index, weight * reading.count(cell), weight * self.num_sensor_readings):
                        raise IndexError

                    weight /= 2

                except IndexError:
                    break
            print('br')

        print(updated_cells)
        return updated_cells

    def get_explore_string(self):
        """ Build and return the MDF string of the exploration status at the time of calling this function. """
        exploration_status = self.exploration_status[:]

        explore_str = ''.join(str(grid) for row in exploration_status for grid in row)

        explore_status_string = '11%s11' % explore_str
        explore_status_string = str(hex(int(explore_status_string, 2)))

        return explore_status_string[2:]

    def get_map_string(self):
        """ Build and return the MDF string of the robot's internal map at the time of calling this function. """
        discovered_map = self.discovered_map[:]

        map_str = ''.join(str(grid) for row in discovered_map for grid in row if grid != 2)

        pad_length = (8 - ((len(map_str) + 4) % 8)) % 8
        pad = '0' * pad_length

        map_string = '1111%s%s' % (map_str, pad)
        map_string = str(hex(int(map_string, 2)))

        map_string = map_string[3:]
        map_string += '0'

        return map_string
