from Utils.utils import *

list(range(0, len(readings), 6))

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

                updated_cells = self._robot.get_sensor_readings()
                yield updated_cells

                in_efficiency_limit = self._robot.in_efficiency_limit()

                # If in efficient limit, check forward first, otherwise, check left first
                if not in_efficiency_limit and self._robot.check_free(LEFT):
                    updated_cells = self._robot.move_robot(LEFT)
                    print('LEFT Free')
                    yield LEFT, MOVE, updated_cells
                elif self._robot.check_free(FORWARD):
                    print('Forward Free')
                    updated_cells = self._robot.move_robot(FORWARD)
                    yield FORWARD, MOVE, updated_cells
                else:
                    if in_efficiency_limit:
                        print('Robot in efficiency limit.... ')

                        if self._robot.check_free(LEFT):

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

                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                yield is_complete

                if is_complete:
                    raise ExploreComplete

                if self._robot.center == START:
                    is_back_at_start = True

                yield is_back_at_start

            while True:
                try:
                    nearest_unexplored_y, nearest_unexplored_x = self._get_nearest_unexplored()
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

                        # Check adjacent cells of SW/SE/NW/NE cells
                        if not moves:
                            print('WARNING: Cannot find shortest valid path moves to adjacent cells')

                            print('Finding shortest valid path moves to adjacent cells of SW/SE/NW/NE cells......')
                            swsenwne_cells = [adjacent_cells[0], adjacent_cells[2], adjacent_cells[5],
                                              adjacent_cells[7]]

                            for cell in swsenwne_cells:
                                double_adjacent_cells = get_robot_cells(cell)
                                double_adjacent_cells = [e for e in double_adjacent_cells if e not in adjacent_cells]
                                moves = get_shortest_valid_path(self._robot,
                                                                self._robot.center, double_adjacent_cells)
                                if moves:
                                    break

                    if not moves:
                        print('WARNING: Cannot find shortest path moves to unexplored')
                        raise PathNotFound

                    for move in moves:
                        updated_cells = self._robot.get_sensor_readings()
                        if updated_cells:
                            is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                    self._time_limit)
                            yield "updated", updated_cells, is_complete

                            if is_complete:
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
