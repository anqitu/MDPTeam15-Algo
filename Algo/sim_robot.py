from Utils.utils import *
from time import time

"""This module defines the simulated Robot class."""

class Robot:
    """
    This class is the simulation robot.
    """
    def __init__(self, exploration_status, facing, discovered_map, real_map):
        """Initialize the robot."""
        self.is_fast_path = False
        self.exploration_status = exploration_status
        self.center = START
        self.facing = facing
        self.discovered_map = discovered_map
        self.probability_map = [[[0.0, 0.0] for _ in range(ROW_LENGTH)] for _ in range(COL_LENGTH)]
        self.arrow_taken_status = [[[0, 0, 0, 0] for _ in range(ROW_LENGTH)] for _ in range(COL_LENGTH)]
        self.arrow_taken_positions = []
        self.arrows = []
        self.arrows_arduino = []
        self.arrows_results = []
        self.sensors = [
            #   2 3 4
            # 1       5
            # 0
            #
            {"mount_loc": WS, "facing": WEST, "range": 3, "blind_spot": 0},
            {"mount_loc": NWS, "facing": WEST, "range": 3, "blind_spot": 0},
            {"mount_loc": NWS, "facing": NORTH, "range": 3, "blind_spot": 0},
            {"mount_loc": NS, "facing": NORTH, "range": 3, "blind_spot": 0},
            {"mount_loc": NES, "facing": NORTH, "range": 3, "blind_spot": 0},
            {"mount_loc": NES, "facing": EAST, "range": 6, "blind_spot": 0}
        ]
        self.real_map = real_map

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
        print('Current Sesnsor Reading: x, y, count, total: {}'.format((x, y, count, total)))

        if self.probability_map[y][x][0] == 1.0 and self.probability_map[y][x][1] == 0.0:
            print('perm')
            return None, None

        # Update counts
        self.probability_map[y][x][0] += count
        self.probability_map[y][x][1] += total

        prob_obstacle = self.probability_map[y][x][0]
        prob_total = self.probability_map[y][x][1]
        value = int(prob_obstacle / prob_total >= 0.5)

        print('Cumulative Sesnsor Reading: x, y, prob_obstacle, prob_total: {}'.format((x, y, prob_obstacle, prob_total)))

        if not self.exploration_status[y][x]:
            self.exploration_status[y][x] = 1

        if self.discovered_map[y][x] != value:
            self.discovered_map[y][x] = value
            return get_grid_index(y, x), int(prob_obstacle / prob_total > 0.5)

        return None, None

    def _mark_permanent(self, cell):
        """
        Mark a cell as a guaranteed non-obstacle.

        Called when the robot walks over a cell.

        :param cell: The number of the cell being marked.
        :return: True if success. No definition of failure provided, however it is easy to add if required.
        """
        y, x = get_matrix_coords(cell)

        self.probability_map[y][x][0] = 1.0
        self.probability_map[y][x][1] = 0.0

        if not self.exploration_status[y][x]:
            self.exploration_status[y][x] = 1

        self.discovered_map[y][x] = 0

        return True

    # def _mark_explored(self, cell_index):
    #     """
    #     Discover the cell if it is not already explored.
    #     Update exploration status as 1
    #     Set the cell in discovered_map as the one in real_map
    #
    #     :param cell_index: cell index.
    #     :return: N/A
    #     """
    #     y, x = get_matrix_coords(cell_index)
    #
    #     # if exploration_status not 0
    #     if not self.exploration_status[y][x]:
    #         self.exploration_status[y][x] = 1
    #         self.discovered_map[y][x] = int(self.real_map[19 - y][x] != 0)
    #         return get_grid_index(y, x), self.discovered_map[y][x]
    #     return None, None

    def _mark_arrow_taken(self, y, x, camera_facing):
        """
        Mark the face a cell having its picture taken by the arrow recognizer.

        :param y: The y-coordinate of the cell to be marked.
        :param x: The x-coordinate of the cell to be marked.
        :param camera_facing: The facing of the camera when the photo was taken.
        :return: True if success. No definition of failure provided, however it is easy to add if required.
        """

        self.arrow_taken_status[y][x][camera_facing] = 1
        print('Mark Arrow Taken at {}'.format((x, y, DIRECTIONS[camera_facing])))

        return True

    # def _take_arrow_images(self, msg):
    #     """
    #     Mark position of a detected arrow
    #
    #     :param msg: The y-coordinate, x-coordinate of the cell to be marked and The facing of the robot when the photo was taken.
    #     """
    #     self.arrow_taken_positions.append(msg)
    #     print('Take Arrow Image at {}'.format(msg))


    def _mark_arrows(self, position):
        y, x, facing = tuple([int(_) for _ in position.split(',')])
        print('Recognizing Image taken with robot position@ {}'.format((x, y, DIRECTIONS[facing])))
        discovered_map = self.discovered_map

        camera_facing = (facing + CAMERA_FACING) % 4

        try:
            distance = 2
            if camera_facing == WEST:
                new_x = x - distance
                if new_x < 0:
                    raise IndexError
                for i, j in [(new_x, y - 1), (new_x, y)]:
                    print('Check Arrow @ {}'.format((i, j, DIRECTIONS[camera_facing])))
                    if discovered_map[j][i] == 1 and int(self.real_map[19-j][i] == camera_facing + 2) == 1:
                        self.arrows.append((j, i, camera_facing))
                        self.arrows_arduino.append(','.join([str(i), str(19-j), str(camera_facing)]))
                        print('Detected Arrows @ {}'.format((i, j, DIRECTIONS[camera_facing])))
            elif camera_facing == NORTH:
                new_y = y + distance
                if new_y > 19:
                    raise IndexError
                for i, j in [(x - 1, new_y), (x, new_y)]:
                    print('Check Arrow @ {}'.format((i, j, DIRECTIONS[camera_facing])))
                    if discovered_map[j][i] == 1 and int(self.real_map[19-j][i] == camera_facing + 2) == 1:
                        self.arrows.append((j, i, camera_facing))
                        self.arrows_arduino.append(','.join([str(i), str(19-j), str(camera_facing)]))
                        print('Detected Arrows @ {}'.format((i, j, DIRECTIONS[camera_facing])))
            elif camera_facing == EAST:
                new_x = x + distance
                if new_x > 14:
                    raise IndexError
                for i, j in [(new_x, y + 1), (new_x, y)]:
                    print('Check Arrow @ {}'.format((i, j, DIRECTIONS[camera_facing])))
                    if discovered_map[j][i] == 1 and int(self.real_map[19-j][i] == camera_facing + 2) == 1:
                        self.arrows.append((j, i, camera_facing))
                        self.arrows_arduino.append(','.join([str(i), str(19-j), str(camera_facing)]))
                        print('Detected Arrows @ {}'.format((i, j, DIRECTIONS[camera_facing])))
            elif camera_facing == SOUTH:
                new_y = y - distance
                if new_y < 0:
                    raise IndexError
                for i, j in [(x + 1, new_y), (x, new_y)]:
                    print('Check Arrow @ {}'.format((i, j, DIRECTIONS[camera_facing])))
                    if discovered_map[j][i] == 1 and int(self.real_map[19-j][i] == camera_facing + 2) == 1:
                        self.arrows.append((j, i, camera_facing))
                        self.arrows_arduino.append(','.join([str(i), str(19-j), str(camera_facing)]))
                        print('Detected Arrows @ {}'.format((i, j, DIRECTIONS[camera_facing])))
        except IndexError:
            return

    def in_efficiency_limit(self):
        """
        Check if the robot is in a position that would indicate that it had just moved past a 1-width obstacle
        that is against the wall of the maze.

        :return: True if the above condition is fulfilled, false otherwise.
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
        for cell in robot_cells:
            if self._mark_permanent(cell):
                updated_cells[cell] = 0

        return updated_cells

    def get_completion_percentage(self):
        """
        Calculate how much of the maze the robot has explored in percentage.

        :return: The percentage of the map the robot has explored.
        """
        count = 0
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
        return self.get_completion_percentage() >= float(explore_limit) \
            or float(time() - start_time >= time_limit)

    def turn_robot(self, direction):
        """
        Turn the robot in a chosen direction.

        :param direction: The direction to turn (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: N/A
        """
        if direction == FORWARD:
            return

        self.facing = (self.facing + direction) % 4

        if IS_ARROW_SCAN and not self.is_fast_path:
            self.check_arrow()

    def move_robot(self, direction):
        """
        Move the robot 1 step in a chosen direction. Assume step is not an obstacle.

        :param direction: The direction to move (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Any cells that the robot has stepped on that it had not yet before.
        """
        self.turn_robot(direction)

        if self.facing == NORTH:
            self.center += ROW_LENGTH
        elif self.facing == EAST:
            self.center += 1
        elif self.facing == SOUTH:
            self.center -= ROW_LENGTH
        elif self.facing == WEST:
            self.center -= 1

        updated_cells = self.mark_robot_standing()

        if IS_ARROW_SCAN and not self.is_fast_path:
            self.check_arrow()

        return updated_cells

    def move_robot_algo(self, direction):
        """
        Turn the algo robot in a chosen direction or forward it

        :param direction: The direction to turn (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Nothing. Stops the method if the direction is FORWARD to save time as the robot does not need to turn.
        """
        if direction == FORWARD:
            if self.facing == NORTH:
                self.center += ROW_LENGTH
            elif self.facing == EAST:
                self.center += 1
            elif self.facing == SOUTH:
                self.center -= ROW_LENGTH
            elif self.facing == WEST:
                self.center -= 1
        else:
            self.facing = (self.facing + direction) % 4

    def check_free(self, direction):
        """
        Check if the adjacent cells in the chosen direction have obstacles.

        :param direction: he direction to check (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: true if the robot is able to take one step in that direction, false otherwise
        """

        print('Check Free......')
        print('Direction: {}'.format(MOVEMENTS[direction]))

        true_bearing = (self.facing + direction) % 4
        robot_cells = get_robot_cells(self.center)

        try:
            if true_bearing == NORTH:
                y, x = get_matrix_coords(robot_cells[0])
                y += 1
                print('Cell to check : {}'.format((x, y, x+1, y, x+2, y)))
                if y < 0 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y][x + 1] == 1
                            or self.discovered_map[y][x + 2] == 1)
                print('North: ' + str(is_free))
                return is_free
            elif true_bearing == EAST:
                y, x = get_matrix_coords(robot_cells[2])
                x += 1
                print('Cell to check : {}'.format((x, y, x, y-1, x, y-2)))
                if y < 2 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y - 1][x] == 1
                            or self.discovered_map[y - 2][x] == 1)
                print('East: ' + str(is_free))
                return is_free
            elif true_bearing == SOUTH:
                y, x = get_matrix_coords(robot_cells[6])
                y -= 1
                print('Cell to check : {}'.format((x, y, x+1, y, x+2, y)))
                if y < 0 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y][x + 1] == 1
                            or self.discovered_map[y][x + 2] == 1)
                print('South: ' + str(is_free))
                return is_free
            elif true_bearing == WEST:
                y, x = get_matrix_coords(robot_cells[0])
                x -= 1
                print('Cell to check : {}'.format((x, y, x, y-1, y-2, x)))
                if y < 2 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y - 1][x] == 1
                            or self.discovered_map[y - 2][x] == 1)
                print('West: ' + str(is_free))
                return is_free
        except IndexError:
            return False

    def is_arrow_possible(self):
        """
        # Camera put on the west of the robot to detect the left and middle image

           # # #
        I2 # # #
        I1 # # #

        Check if it is possible to have arrows in the chosen direction.

        The method also takes into consideration obstacles that have already been scanned for arrows. Will only return
        true if there are faces that have not been scanned that are facing the robot.

        :return: True if there are unscanned faces of obstacles in the path of the RPi camera, false otherwise.
        """
        y, x = get_matrix_coords(self.center)
        discovered_map = self.discovered_map
        arrow_taken_status = self.arrow_taken_status
        facing = self.facing
        camera_facing = (facing + CAMERA_FACING) % 4

        try:
            distance = 2
            if camera_facing == WEST:
                new_x = x - distance
                if new_x < 0:
                    raise IndexError

                for i, j in [(new_x, y - 1), (new_x, y)]:
                    print('Checking %s,%s' % (i, j))
                    if discovered_map[j][i] == 1 and not arrow_taken_status[j][i][camera_facing]:
                        self._mark_arrow_taken(j, i, camera_facing)
                        return True
            elif camera_facing == NORTH:
                new_y = y + distance
                if new_y > 19:
                    raise IndexError
                for i, j in [(x - 1, new_y), (x, new_y)]:
                    print('Checking %s,%s' % (i, j))
                    if discovered_map[j][i] == 1 and not arrow_taken_status[j][i][camera_facing]:
                        self._mark_arrow_taken(j, i, camera_facing)
                        return True
            elif camera_facing == EAST:
                new_x = x + distance
                if new_x > 14:
                    raise IndexError
                for i, j in [(new_x, y + 1), (new_x, y)]:
                    print('Checking %s,%s' % (i, j))
                    if discovered_map[j][i] == 1 and not arrow_taken_status[j][i][camera_facing]:
                        self._mark_arrow_taken(j, i, camera_facing)
                        return True
            elif camera_facing == SOUTH:
                new_y = y - distance
                if new_y < 0:
                    raise IndexError
                for i, j in [(x + 1, new_y), (x, new_y)]:
                    print('Checking %s,%s' % (i, j))
                    if discovered_map[j][i] == 1 and not arrow_taken_status[j][i][camera_facing]:
                        self._mark_arrow_taken(j, i, camera_facing)
                        return True
        except IndexError:
            return False


    def check_arrow(self):
        """
        Send the RPi a message to take a picture to check for arrows.

        Check if there are potential unscanned arrows in the field of view of the RPi camera.
        :return: N/A
        """
        y, x = get_matrix_coords(self.center)

        if self.is_arrow_possible():
            print('Arrow Possible @ Robot Position: {}'.format((x, y, DIRECTIONS[self.facing])))
            position = '%s,%s,%s' % (y, x, self.facing)
            self._mark_arrows(position)
        else:
            print('Arrow Not Possible @ Robot Position: {}'.format((x, y, DIRECTIONS[self.facing])))

    def return_sensor_readings(self):
        """
        Get simulated sensor readings by comparing the cells that are to be explored
        by the virtual sensors against the map provided.
        """
        print('Return Sensor Readings...')
        readings = [0] * 6
        sensors = self.sensors[:]
        sensor_index = sensors.index

        for sensor in self.sensors:
            print('Sensor', sensor_index(sensor))
            true_facing = (sensor["facing"] + self.facing) % 4

            if sensor["mount_loc"] != CS:
                offset = self.facing * 2
                true_mounting = (sensor["mount_loc"] + offset) % 8
            else:
                true_mounting = CS

            robot_cells = get_robot_cells(self.center)

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
            cover_range = list(range(1, sensor["range"] + 1))

            for cell in cover_range:
                try:
                    if true_facing == NORTH:
                        to_explore = (y + cell, x)
                    elif true_facing == EAST:
                        to_explore = (y, x + cell)
                    elif true_facing == SOUTH:
                        to_explore = (y - cell, x)
                    elif true_facing == WEST:
                        to_explore = (y, x - cell)

                    if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
                        readings[sensor_index(sensor)] = cell
                        print('ie')
                        raise IndexError

                    if self.real_map[19 - to_explore[0]][to_explore[1]] != 0:
                        print('Obstacle @ Cell {}'.format(cell))
                        readings[sensor_index(sensor)] = cell
                        raise IndexError

                except IndexError:
                    break
        readings = ','.join([str(reading) for reading in readings]) + ','
        print(readings)
        return readings

    def get_sensor_readings(self):
        """
        Send a message to the Arduino to take sensor readings.

        The sensors are iterated through and the number of times a cell is detected as an obstacle is added to its
        running count. The total number of scans the cell has received is added to its running count of scans.

        The counts are weighted by distance, with the weight halving for every unit further away from the robot that
        the reading is taken.

        :return: The updated cell values and indexes.
        """
        readings = self.return_sensor_readings()
        readings = readings.split(',')
        del readings[-1]

        readings = [int(x) for x in readings]

        robot_cells = get_robot_cells(self.center)
        sensors = self.sensors[:]
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
            cover_range = list(range(1, sensor["range"] + 1))
            read_range = list(range(sensor["blind_spot"] + 1, sensor["range"] + 1))
            blind_range = list(range(1, sensor["blind_spot"] + 1))

            reading = readings[sensor_index(sensor)]
            print('Sensor', sensor_index(sensor))

            weight = 4

            # If reading is 0, means no obstacle in the covered range
            if reading == 0:
                print('No Obstacle in Covered Range')
                for cell in cover_range:
                    try:
                        if true_facing == NORTH:
                            to_explore = (y + cell, x)
                        elif true_facing == EAST:
                            to_explore = (y, x + cell)
                        elif true_facing == SOUTH:
                            to_explore = (y - cell, x)
                        elif true_facing == WEST:
                            to_explore = (y, x - cell)

                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
                            print('ie')
                            raise IndexError

                        cell_index = get_grid_index(to_explore[0], to_explore[1])

                        updated_cell, value = self._mark_probability(cell_index, 0 * weight, 1 * weight)
                        if updated_cell is not None:
                            updated_cells[updated_cell] = value

                        weight = max(weight/2, 1)

                    except IndexError:
                        break
                print('br')

            # if reading in the read range, mark cells as 0 until the obstacle cell
            elif reading in read_range:
                print('Has Obstacle in Covered Range')

                # If the robot is able to observe onstacle in covered range, there is no obstacle in the blind spot.
                for cell in blind_range:
                    try:
                        if true_facing == NORTH:
                            to_explore = (y + cell, x)
                        elif true_facing == EAST:
                            to_explore = (y, x + cell)
                        elif true_facing == SOUTH:
                            to_explore = (y - cell, x)
                        elif true_facing == WEST:
                            to_explore = (y, x - cell)

                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
                            print('ie')
                            raise IndexError

                        cell_index = get_grid_index(to_explore[0], to_explore[1])
                        updated_cell, value = self._mark_probability(cell_index, 0, 1 * weight)
                        if updated_cell is not None:
                            updated_cells[updated_cell] = value
                    except IndexError:
                        break

                # Check for cells in read range
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

                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
                            print('ie')
                            raise IndexError

                        cell_index = get_grid_index(to_explore[0], to_explore[1])

                        updated_cell, value = self._mark_probability(cell_index, int(reading == cell) * weight, 1 * weight)
                        if updated_cell is not None:
                            updated_cells[updated_cell] = value

                        # If the current cell is the one with obstacle, break the loop
                        if self.discovered_map[to_explore[0]][to_explore[1]] == 1:
                            raise IndexError

                        weight /= 2

                    except IndexError:
                        break
                print('br')

            else:
                print('Unacceptable Reading')
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
        pad_length = (4 - ((len(map_str) + 4) % 4)) % 4
        pad = '0' * pad_length
        map_string = '1111%s%s' % (map_str, pad)
        map_string = str(hex(int(map_string, 2)))
        map_string = map_string[3:]
        return map_string
