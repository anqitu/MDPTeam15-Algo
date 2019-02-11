from Algo.fastest_path import *

"""This module defines the Exploration class that handles the exploration algorithm, along with Exceptions used."""

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


class Exploration:
    """
    This class defines and handles the exploration algorithm.
    """
    def __init__(self, robot, start_time, exploration_limit=100, time_limit=360):
        self._robot = robot
        self._start_time = start_time
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
        for i in range(NUM_ROWS):
            for j in range(NUM_COLS):
                if self._robot.discovered_map[::-1][i][j] == 2:
                    y, x = get_matrix_coords(self._robot.center)
                    dist = abs(y - i) + abs(x - j)

                    if dist < min_dist:
                        min_dist = dist
                        nearest = (i, j)

        return nearest[0], nearest[1]

    def start(self):
        """
        Simulate exploring a maze with a virtual robot.

        :return: True when exploration is complete and the robot is back in the start zone.
        """
        yield self._robot.mark_robot_standing()  # Mark initial robot space explored

        is_back_at_start = False
        while True:
            try:
                while not is_back_at_start:
                    print('-' * 50)
                    updated_cells = self._robot.get_sensor_readings()
                    yield updated_cells

                    print('Check Free Function')
                    print('Cells: {}'.format(get_robot_cells(self._robot.center)))
                    print('Facing: {}'.format(self._robot.facing))

                    print('Exploration Status Map: :')
                    for _ in self._robot.exploration_status:
                        print(_)

                    print('Discovered Map: :')
                    for _ in self._robot.discovered_map:
                        print(_)

                    if self._robot.check_free(LEFT) and not \
                            self._robot.in_efficiency_limit():
                    # if self._robot.check_free(LEFT):
                        updated_cells = self._robot.move_robot(LEFT)
                        print('LEFT Free')
                        yield LEFT, MOVE, updated_cells
                    elif self._robot.check_free(FORWARD):
                        print('Forward Free')
                        updated_cells = self._robot.move_robot(FORWARD)
                        yield FORWARD, MOVE, updated_cells
                    else:
                        if self._robot.in_efficiency_limit():
                            print('self._robot.in_efficiency_limit(): ' + str(True))

                            if self._robot.check_free(LEFT):

                                updated_cells = self._robot.move_robot(LEFT)
                                yield LEFT, MOVE, updated_cells
                                yield self._robot.is_complete(self._exploration_limit, self._start_time,
                                                              self._time_limit)
                                if self._robot.is_complete(self._exploration_limit, self._start_time,
                                                           self._time_limit):
                                    raise ExploreComplete

                                if self._robot.center == START:
                                    is_back_at_start = True
                                yield is_back_at_start
                                if is_back_at_start:
                                    break
                                updated_cells = self._robot.get_sensor_readings()
                                yield updated_cells
                                self._robot.turn_robot(BACKWARD)
                                yield BACKWARD, TURN, {}

                            # Start (Add by Anqi)
                            else:
                                self._robot.turn_robot(RIGHT)
                                yield RIGHT, TURN, {}
                            # End (Add by Anqi)

                        else:
                            self._robot.turn_robot(RIGHT)
                            yield RIGHT, TURN, {}

                    # print('self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)')
                    yield self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)

                    if self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit):
                        raise ExploreComplete

                    if self._robot.center == START:
                        is_back_at_start = True

                    yield is_back_at_start

                while True:
                    try:
                        nearest_unexplored_y, nearest_unexplored_x = self._get_nearest_unexplored()
                        center_y, center_x = get_matrix_coords(self._robot.center)
                        print('nearest_unexplored_y, nearest_unexplored_x: {}'.format((nearest_unexplored_y, nearest_unexplored_x)))

                        moves = get_shortest_path_moves(self._robot,
                                                        (center_y, center_x),
                                                        (nearest_unexplored_y, nearest_unexplored_x))
                        print('Moves to nearest unexplored: {}'.format(moves))

                        if not moves:  # Check adjacent cells
                            adjacent_cells = get_robot_cells(get_grid_index(nearest_unexplored_y, nearest_unexplored_x))
                            del adjacent_cells[4]

                            adj_order = [5, 6, 7, 3, 4, 0, 1, 2]
                            adjacent_cells = [adjacent_cells[i] for i in adj_order]

                            moves = get_shortest_valid_path(self._robot,
                                                            self._robot.center, adjacent_cells)

                            # Check adjacent cells of SW/SE/NW/NE cells
                            if not moves:
                                swsenwne_cells = [adjacent_cells[0], adjacent_cells[2], adjacent_cells[5],
                                                  adjacent_cells[7]]

                                for cell in swsenwne_cells:
                                    double_adjacent_cells = get_robot_cells(cell)
                                    if swsenwne_cells.index(cell) == 0:
                                        unwanted = set(double_adjacent_cells[1:3])
                                        unwanted.update(double_adjacent_cells[4:6])
                                    elif swsenwne_cells.index(cell) == 1:
                                        unwanted = set(double_adjacent_cells[0:2])
                                        unwanted.update(double_adjacent_cells[3:5])
                                    elif swsenwne_cells.index(cell) == 2:
                                        unwanted = set(double_adjacent_cells[4:6])
                                        unwanted.update(double_adjacent_cells[7:9])
                                    elif swsenwne_cells.index(cell) == 3:
                                        unwanted = set(double_adjacent_cells[3:5])
                                        unwanted.update(double_adjacent_cells[6:8])

                                    double_adjacent_cells = [e for e in double_adjacent_cells if e not in unwanted]

                                    moves = get_shortest_valid_path(self._robot,
                                                                    self._robot.center, double_adjacent_cells)
                                    if moves:
                                        break

                        if not moves:
                            raise PathNotFound

                        for move in moves:
                            updated_cells = self._robot.get_sensor_readings()
                            if updated_cells:
                                yield "updated", \
                                      updated_cells, \
                                      self._robot.is_complete(self._exploration_limit, self._start_time,
                                                              self._time_limit)
                                if self._robot.is_complete(self._exploration_limit, self._start_time,
                                                           self._time_limit):

                                    print('Is_Complete: ')
                                    print('Exploration Status Map: :')
                                    for _ in self._robot.exploration_status:
                                        print(_)

                                    print('Discovered Map: :')
                                    for _ in self._robot.discovered_map:
                                        print(_)

                                    raise ExploreComplete
                                raise CellsUpdated
                            self._robot.move_robot(move)
                            yield "moved", move, False

                    except CellsUpdated:
                        continue

            except ExploreComplete:
                break
            except PathNotFound:
                yield "invalid", None, None
                break

        # Return to start after completion

        print("Returning to Start...")

        center_y, center_x = get_matrix_coords(self._robot.center)
        start_y, start_x = get_matrix_coords(START)

        moves = get_shortest_path_moves(self._robot,
                                        (center_y, center_x), (start_y, start_x), is_give_up=True)

        print(moves)

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
        while True:
            try:
                # Left-wall-hugging until loop
                while not is_back_at_start:
                    updated_cells = self._robot.get_sensor_readings(sender)
                    yield updated_cells

                    if self._robot.check_free(LEFT):
                        updated_cells = self._robot.move_robot(sender, LEFT)
                        yield LEFT, MOVE, updated_cells
                    elif self._robot.check_free(FORWARD):
                        updated_cells = self._robot.move_robot(sender, FORWARD)
                        yield FORWARD, MOVE, updated_cells
                    elif self._robot.check_free(RIGHT):
                        updated_cells = self._robot.move_robot(sender, RIGHT)
                        yield RIGHT, MOVE, updated_cells
                    else:
                        self._robot.turn_robot(sender, BACKWARD)
                        yield BACKWARD, TURN, {}

                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                    yield is_complete
                    if is_complete:
                        raise ExploreComplete

                    if self._robot.center == START:
                        is_back_at_start = True
                        print("looped")

                    yield is_back_at_start

                if self._robot.get_completion_percentage() >= 97.5:
                    return True

                # Finding shortest path to nearest unexplored square
                while True:
                    try:
                        nearest_unexplored_y, nearest_unexplored_x = self._get_nearest_unexplored()
                        center_y, center_x = get_matrix_coords(self._robot.center)

                        moves = get_shortest_path_moves(self._robot,
                                                        (center_y, center_x),
                                                        (nearest_unexplored_y, nearest_unexplored_x))

                        # Check adjacent cells
                        if not moves:
                            adjacent_cells = get_robot_cells(get_grid_index(nearest_unexplored_y, nearest_unexplored_x))
                            del adjacent_cells[4]

                            adj_order = [5, 6, 7, 3, 4, 0, 1, 2]
                            adjacent_cells = [adjacent_cells[i] for i in adj_order]

                            moves = get_shortest_valid_path(self._robot,
                                                            self._robot.center, adjacent_cells)

                            # Check adjacent cells of SW/SE/NW/NE cells
                            if not moves:
                                swsenwne_cells = [adjacent_cells[0], adjacent_cells[2], adjacent_cells[5],
                                                  adjacent_cells[7]]

                                for cell in swsenwne_cells:
                                    double_adjacent_cells = get_robot_cells(cell)
                                    if swsenwne_cells.index(cell) == 0:
                                        unwanted = set(double_adjacent_cells[1:3])
                                        unwanted.update(double_adjacent_cells[4:6])
                                    elif swsenwne_cells.index(cell) == 1:
                                        unwanted = set(double_adjacent_cells[0:2])
                                        unwanted.update(double_adjacent_cells[3:5])
                                    elif swsenwne_cells.index(cell) == 2:
                                        unwanted = set(double_adjacent_cells[4:6])
                                        unwanted.update(double_adjacent_cells[7:9])
                                    elif swsenwne_cells.index(cell) == 3:
                                        unwanted = set(double_adjacent_cells[3:5])
                                        unwanted.update(double_adjacent_cells[6:8])

                                    double_adjacent_cells = [e for e in double_adjacent_cells if e not in unwanted]

                                    moves = get_shortest_valid_path(self._robot,
                                                                    self._robot.center, double_adjacent_cells)
                                    if moves:
                                        break

                        if not moves:
                            raise PathNotFound

                        updated_cells = self._robot.get_sensor_readings(sender)
                        for move in moves:
                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                                      self._time_limit)
                                yield "updated", updated_cells, is_complete
                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated
                            updated_cells = self._robot.move_robot(sender, move)
                            yield "moved", move, False

                    except CellsUpdated:
                        continue

            except ExploreComplete:
                break
            except PathNotFound:
                yield "invalid", None, None
                break

        # Return to start after completion

        print("Returning to Start...")

        while True:
            center_y, center_x = get_matrix_coords(self._robot.center)
            start_y, start_x = get_matrix_coords(START)

            try:
                moves = get_shortest_path_moves(self._robot,
                                                (center_y, center_x), (start_y, start_x), is_give_up=True)

                if not moves:
                    break

                for move in moves:
                    # In case of undetected blocks in the path home
                    updated_cells = self._robot.get_sensor_readings(sender)
                    if updated_cells:
                        yield move
                        raise CellsUpdated
                    self._robot.move_robot(sender, move)
                    yield move

            except IndexError:
                break
            except CellsUpdated:
                continue

        return True

    def start_real_efficient(self, sender):
        """
        Experimental explore function; implements more efficient movement patterns.
        Assumes robot movement/turning is reliably accurate.

        :param sender: Communicates with RPi
        :return: When explore is completed and robot is back at start, return to controller
        """
        yield self._robot.mark_robot_standing()  # Mark initial robot space explored

        is_back_at_start = False
        while True:
            try:
                while not is_back_at_start:
                    updated_cells = self._robot.get_sensor_readings(sender)
                    yield updated_cells

                    if self._robot.check_free(LEFT) and not self._robot.in_efficiency_limit():
                        updated_cells = self._robot.move_robot(sender, LEFT)
                        yield LEFT, MOVE, updated_cells
                    elif self._robot.check_free(FORWARD):
                        updated_cells = self._robot.move_robot(sender, FORWARD)
                        yield FORWARD, MOVE, updated_cells
                    else:
                        if self._robot.in_efficiency_limit():
                            if self._robot.check_free(LEFT):
                                updated_cells = self._robot.move_robot(sender, LEFT)
                                yield LEFT, MOVE, updated_cells
                                yield self._robot.is_complete(self._exploration_limit, self._start_time,
                                                              self._time_limit)
                                if self._robot.is_complete(self._exploration_limit, self._start_time,
                                                           self._time_limit):
                                    raise ExploreComplete

                                if self._robot.center == START:
                                    is_back_at_start = True
                                yield is_back_at_start
                                if is_back_at_start:
                                    break
                                updated_cells = self._robot.get_sensor_readings(sender)
                                yield updated_cells
                                self._robot.turn_robot(sender, RIGHT)
                                self._robot.turn_robot(sender, RIGHT)
                                yield BACKWARD, TURN, {}
                        else:
                            self._robot.turn_robot(sender, RIGHT)
                            yield RIGHT, TURN, {}

                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                    yield is_complete
                    if is_complete:
                        raise ExploreComplete

                    if self._robot.center == START:
                        is_back_at_start = True
                        print("looped")

                    yield is_back_at_start

                if self._robot.get_completion_percentage() >= 97.5:
                    return True

                # Finding shortest path to nearest unexplored square
                while True:
                    try:
                        nearest_unexplored_y, nearest_unexplored_x = self._get_nearest_unexplored()
                        center_y, center_x = get_matrix_coords(self._robot.center)

                        moves = get_shortest_path_moves(self._robot,
                                                        (center_y, center_x),
                                                        (nearest_unexplored_y, nearest_unexplored_x))

                        if not moves:  # Check adjacent cells
                            adjacent_cells = get_robot_cells(get_grid_index(nearest_unexplored_y, nearest_unexplored_x))
                            del adjacent_cells[4]

                            adj_order = [5, 6, 7, 3, 4, 0, 1, 2]
                            adjacent_cells = [adjacent_cells[i] for i in adj_order]

                            moves = get_shortest_valid_path(self._robot,
                                                            self._robot.center, adjacent_cells)

                            # Check adjacent cells of SW/SE/NW/NE cells
                            if not moves:
                                swsenwne_cells = [adjacent_cells[0], adjacent_cells[2], adjacent_cells[5],
                                                  adjacent_cells[7]]

                                for cell in swsenwne_cells:
                                    double_adjacent_cells = get_robot_cells(cell)
                                    if swsenwne_cells.index(cell) == 0:
                                        unwanted = set(double_adjacent_cells[1:3])
                                        unwanted.update(double_adjacent_cells[4:6])
                                    elif swsenwne_cells.index(cell) == 1:
                                        unwanted = set(double_adjacent_cells[0:2])
                                        unwanted.update(double_adjacent_cells[3:5])
                                    elif swsenwne_cells.index(cell) == 2:
                                        unwanted = set(double_adjacent_cells[4:6])
                                        unwanted.update(double_adjacent_cells[7:9])
                                    elif swsenwne_cells.index(cell) == 3:
                                        unwanted = set(double_adjacent_cells[3:5])
                                        unwanted.update(double_adjacent_cells[6:8])

                                    double_adjacent_cells = [e for e in double_adjacent_cells if e not in unwanted]

                                    moves = get_shortest_valid_path(self._robot,
                                                                    self._robot.center, double_adjacent_cells)
                                    if moves:
                                        break

                        if not moves:
                            raise PathNotFound

                        updated_cells = self._robot.get_sensor_readings(sender)
                        for move in moves:
                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                                      self._time_limit)
                                yield "updated", updated_cells, is_complete
                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated
                            updated_cells = self._robot.move_robot(sender, move)
                            yield "moved", move, False

                    except CellsUpdated:
                        continue

            except ExploreComplete:
                break
            except PathNotFound:
                yield "invalid", None, None
                break

        # Return to start after completion

        print("Returning to Start...")

        while True:
            center_y, center_x = get_matrix_coords(self._robot.center)
            start_y, start_x = get_matrix_coords(START)

            try:
                moves = get_shortest_path_moves(self._robot,
                                                (center_y, center_x), (start_y, start_x), is_give_up=True)

                if not moves:
                    break

                for move in moves:
                    # In case of undetected blocks in the path home
                    updated_cells = self._robot.get_sensor_readings(sender)
                    if updated_cells:
                        yield move
                        raise CellsUpdated
                    self._robot.move_robot(sender, move)
                    yield move

            except IndexError:
                break
            except CellsUpdated:
                continue

        return True
