from Algo.fastest_path import *

"""This module defines the Exploration class that handles the exploration algorithm, along with Exceptions used."""
class Exploration:
    """
    This class defines and handles the exploration algorithm.
    """
    def __init__(self, robot, start_time, is_arrow_scan, exploration_limit=100, time_limit=360):
        self._robot = robot
        self._start_time = start_time
        self.is_arrow_scan = is_arrow_scan
        self._exploration_limit = exploration_limit
        self._time_limit = time_limit
        self._auto_update = True

    def _get_nearest_unexplored(self):
        """
        Find the unexplored cell with the shortest Manhattan Distance from the current center of the robot.

        :return: The xy coordinates of the nearest unexplored cell.
        """
        min_dist = NUM_ROWS * NUM_COLS + 1
        nearest = (-1, -1)
        y, x = get_matrix_coords(self._robot.center)
        for i in range(NUM_ROWS):
            for j in range(NUM_COLS):
                if self._robot.discovered_map[i][j] == 2:
                    dist = abs(y - i) + abs(x - j)

                    if dist < min_dist:
                        min_dist = dist
                        nearest = (i, j)

        return nearest[0], nearest[1]

    def _get_unexplored(self):
        unexplored_coors = {}
        y, x = get_matrix_coords(self._robot.center)
        for i in range(NUM_ROWS):
            for j in range(NUM_COLS):
                if self._robot.discovered_map[i][j] == 2:
                    unexplored_coors[(i, j)] = abs(y - i) + abs(x - j)
        return sorted(unexplored_coors.items(), key=lambda kv: kv[1])


    def start(self):
        """
        Simulate exploring a maze with a virtual robot.

        :return: True when exploration is complete and the robot is back in the start zone.
        """
        yield self._robot.mark_robot_standing()  # Mark initial robot space explored

        is_back_at_start = False
        is_leave_start = False
        while True:
            try:
                while not is_back_at_start:
                    updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(self.is_arrow_scan)
                    yield updated_cells

                    if is_blind_range_undetected_obstacle:
                        if self._robot.check_free(LEFT) or self._robot.check_free(FORWARD):
                            self._robot.turn_robot(RIGHT)
                            print('Blind Range Undetected Obstacle Observed: Turn right to get sensor reading')
                            yield RIGHT, TURN, {}

                            is_complete = False
                            yield is_complete

                            is_back_at_start = False
                            yield is_back_at_start

                            updated_cells = self._robot.get_sensor_readings_blind_range()
                            yield updated_cells

                            print('Turn Left to get back to original track')
                            self._robot.turn_robot(LEFT)
                            yield LEFT, TURN, {}
                            yield is_complete
                            yield is_back_at_start

                            updated_cells = {}
                            yield updated_cells

                    in_efficiency_limit = self._robot.in_efficiency_limit()

                    if not in_efficiency_limit:

                        if self._robot.check_free(LEFT):
                            updated_cells = self._robot.move_robot(LEFT)
                            print('LEFT Free')
                            yield LEFT, MOVE, updated_cells
                        elif self._robot.check_free(FORWARD):
                            print('Forward Free')
                            updated_cells = self._robot.move_robot(FORWARD)
                            yield FORWARD, MOVE, updated_cells
                        else:
                            self._robot.turn_robot(RIGHT)
                            yield RIGHT, TURN, {}
                    else:
                        print('Robot in efficiency limit.... ')
                        if self._robot.check_free(FORWARD):
                            print('Forward Free')
                            updated_cells = self._robot.move_robot(FORWARD)
                            yield FORWARD, MOVE, updated_cells
                        elif self._robot.check_free(LEFT):
                            updated_cells = self._robot.move_robot(LEFT)
                            yield LEFT, MOVE, updated_cells

                            is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                            yield is_complete

                            if is_complete:
                                raise ExploreComplete

                            if self._robot.center == START:
                                is_back_at_start = True
                            yield is_back_at_start
                            if is_back_at_start:
                                break
                            updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(self.is_arrow_scan)
                            yield updated_cells

                            self._robot.turn_robot(BACKWARD)
                            yield BACKWARD, TURN, {}
                        else:
                            self._robot.turn_robot(RIGHT)
                            yield RIGHT, TURN, {}


                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                    yield is_complete

                    if is_complete:
                        raise ExploreComplete

                    if self._robot.center != START:
                        is_leave_start = True

                    if self._robot.center == START and is_leave_start and self._robot.get_completion_count() > 100:
                        is_back_at_start = True

                    yield is_back_at_start

                if self._robot.is_complete_after_back_to_start(self._exploration_limit, self._start_time, self._time_limit):
                    raise ExploreComplete

                while True:
                    try:
                        unexplored_coors = self._get_unexplored()
                        for unexplored_coor in unexplored_coors:
                            nearest_unexplored_y, nearest_unexplored_x = unexplored_coor[0]
                            center_y, center_x = get_matrix_coords(self._robot.center)
                            print('nearest_unexplored_y, nearest_unexplored_x: {}'.format((nearest_unexplored_y, nearest_unexplored_x)))

                            print('Finding shortest path moves to nearest unexplored......')
                            moves = get_shortest_path_moves(self._robot,
                                                            (center_y, center_x),
                                                            (nearest_unexplored_y, nearest_unexplored_x))
                            print('Shortest path moaves to nearest unexplored: {}'.format(moves))

                            if not moves:  # Check adjacent cells
                                print('WARNING: Cannot find shortest path moves to nearest unexplored')

                                print('Finding shortest valid path moves to adjacent cells......')
                                robot_cell_index = get_grid_index(nearest_unexplored_y, nearest_unexplored_x)
                                adjacent_cells = get_robot_cells(robot_cell_index)
                                del adjacent_cells[4]

                                adj_order = [5, 6, 7, 3, 4, 0, 1, 2]
                                adjacent_cells = [adjacent_cells[i] for i in adj_order]

                                moves = get_shortest_valid_path(self._robot,
                                                                self._robot.center, adjacent_cells)

                            if moves:
                                break

                        if not moves:
                            print('WARNING: Cannot find shortest path moves to unexplored')
                            raise PathNotFound

                        for move in moves:
                            updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(self.is_arrow_scan)
                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                        self._time_limit)
                                yield "updated", updated_cells, is_complete

                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated

                            if is_blind_range_undetected_obstacle:
                                self._robot.turn_robot(RIGHT)
                                print('-' * 50)
                                print('Blind Range Undetected Obstacle Observed: Turn right to get sensor reading')
                                yield "turned", RIGHT, False

                                updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(self.is_arrow_scan)

                                if updated_cells:
                                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                            self._time_limit)
                                    yield "updated", updated_cells, is_complete

                                    if is_complete:
                                        raise ExploreComplete
                                    raise CellsUpdated

                                print('Turn Left to get back to original track')
                                self._robot.turn_robot(LEFT)
                                yield "turned", LEFT, False

                            updated_cells = self._robot.move_robot(move)
                            yield "moved", move, False

                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                        self._time_limit)
                                yield "updated", updated_cells, is_complete

                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated

                    except CellsUpdated:
                        continue

            except ExploreComplete:
                break
            except PathNotFound:
                yield "invalid", None, None
                break

        # Return to start after completion
        center_y, center_x = get_matrix_coords(self._robot.center)
        start_y, start_x = get_matrix_coords(START)

        moves = get_shortest_path_moves(self._robot,
                                        (center_y, center_x), (start_y, start_x), is_give_up=True)

        if not moves:
            return True

        else:
            for move in moves:
                self._robot.move_robot(move)
                yield move

        return True

    def start_real(self, sender):
        """
        Explore the maze with the physical robot.

        :param sender: The object that communicates with the RPi.
        :return: True when exploration is complete.
        """
        yield self._robot.mark_robot_standing()  # Mark initial robot space explored

        is_back_at_start = False
        is_leave_start = False
        while True:
            try:
                # Left-wall-hugging until loop
                sender.send_arduino(ARDUINO_SENSOR)
                readings = sender.wait_arduino(ARDUINO_READINGS_REGEX, is_regex=True)

                while not is_back_at_start:
                    updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(readings, sender, self.is_arrow_scan)
                    yield updated_cells

                    if is_blind_range_undetected_obstacle:
                        if self._robot.check_free(LEFT) or self._robot.check_free(FORWARD):
                            readings = self._robot.turn_robot(sender, RIGHT, self.is_arrow_scan)
                            print('Blind Range Undetected Obstacle Observed: Turn right to get sensor reading')
                            yield RIGHT, TURN, {}

                            is_complete = False
                            yield is_complete

                            is_back_at_start = False
                            yield is_back_at_start

                            updated_cells = self._robot.get_sensor_readings_blind_range(readings, sender)
                            yield updated_cells

                            print('Turn Left to get back to original track')
                            self._robot.turn_robot(sender, LEFT, self.is_arrow_scan)
                            yield LEFT, TURN, {}
                            yield is_complete
                            yield is_back_at_start

                            updated_cells = {}
                            yield updated_cells

                    if self._robot.check_free(LEFT):
                        readings, updated_cells = self._robot.move_robot(sender, LEFT, self.is_arrow_scan)
                        print('LEFT Free')
                        yield LEFT, MOVE, updated_cells
                    elif self._robot.check_free(FORWARD):
                        print('Forward Free')
                        readings, updated_cells = self._robot.move_robot(sender, FORWARD, self.is_arrow_scan)
                        yield FORWARD, MOVE, updated_cells
                    else:
                        readings = self._robot.turn_robot(sender, RIGHT, self.is_arrow_scan)
                        yield RIGHT, TURN, {}

                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                    yield is_complete

                    if is_complete:
                        raise ExploreComplete

                    if self._robot.center != START:
                        is_leave_start = True

                    if self._robot.center == START and is_leave_start and self._robot.get_completion_count() > 100:
                        is_back_at_start = True

                    yield is_back_at_start

                if self._robot.is_complete_after_back_to_start(self._exploration_limit, self._start_time, self._time_limit):
                    raise ExploreComplete

                # Finding shortest path to nearest unexplored square
                while True:
                    try:
                        unexplored_coors = self._get_unexplored()
                        for unexplored_coor in unexplored_coors:
                            nearest_unexplored_y, nearest_unexplored_x = unexplored_coor[0]
                            center_y, center_x = get_matrix_coords(self._robot.center)
                            print('nearest_unexplored_y, nearest_unexplored_x: {}'.format((nearest_unexplored_y, nearest_unexplored_x)))

                            print('Finding shortest path moves to nearest unexplored......')
                            moves = get_shortest_path_moves(self._robot,
                                                            (center_y, center_x),
                                                            (nearest_unexplored_y, nearest_unexplored_x))
                            print('Shortest path moaves to nearest unexplored: {}'.format(moves))

                            if not moves:  # Check adjacent cells
                                print('WARNING: Cannot find shortest path moves to nearest unexplored')

                                print('Finding shortest valid path moves to adjacent cells......')
                                robot_cell_index = get_grid_index(nearest_unexplored_y, nearest_unexplored_x)
                                adjacent_cells = get_robot_cells(robot_cell_index)
                                del adjacent_cells[4]

                                adj_order = [5, 6, 7, 3, 4, 0, 1, 2]
                                adjacent_cells = [adjacent_cells[i] for i in adj_order]

                                moves = get_shortest_valid_path(self._robot,
                                                                self._robot.center, adjacent_cells)

                            if moves:
                                break

                        if not moves:
                            print('WARNING: Cannot find shortest path moves to unexplored')
                            raise PathNotFound

                        sender.send_arduino(ARDUINO_SENSOR)
                        readings = sender.wait_arduino(ARDUINO_READINGS_REGEX, is_regex=True)

                        for move in moves:

                            updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(readings, sender, self.is_arrow_scan)
                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                        self._time_limit)
                                yield "updated", updated_cells, is_complete

                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated

                            if is_blind_range_undetected_obstacle:
                                self._robot.turn_robot(sender, RIGHT, self.is_arrow_scan)
                                print('-' * 50)
                                print('Blind Range Undetected Obstacle Observed: Turn right to get sensor reading')
                                yield "turned", RIGHT, False

                                updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(readings, sender, self.is_arrow_scan)

                                if updated_cells:
                                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                            self._time_limit)
                                    yield "updated", updated_cells, is_complete

                                    if is_complete:
                                        raise ExploreComplete
                                    raise CellsUpdated

                                print('Turn Left to get back to original track')
                                self._robot.turn_robot(sender, LEFT, self.is_arrow_scan)
                                yield "turned", LEFT, False

                            readings, updated_cells = self._robot.move_robot(sender, move, self.is_arrow_scan)
                            yield "moved", move, False

                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                        self._time_limit)
                                yield "updated", updated_cells, is_complete

                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated

                    except CellsUpdated:
                        continue

            except ExploreComplete:
                break
            except PathNotFound:
                yield "invalid", None, None
                break


        center_y, center_x = get_matrix_coords(self._robot.center)
        start_y, start_x = get_matrix_coords(START)

        moves = get_shortest_path_moves(self._robot,
                                        (center_y, center_x), (start_y, start_x), is_give_up=True)

        if not moves:
            return True

        else:
            for move in moves:
                self._robot.move_robot(sender, move, self.is_arrow_scan)
                yield move
        return True


class ExploreComplete(Exception):
    """
    This exception is raised when exploration is complete.
    """
    def __init__(self, message="Exploration complete!"):
        print(message)


class CellsUpdated(Exception):
    """
    This exception is raised when there has been an update in the robot's internal map
    """
    pass


class PathNotFound(Exception):
    """
    This exception is raised when a path to the location cannot be found.
    """
    def __init__(self):
        print("Valid path not found!")
