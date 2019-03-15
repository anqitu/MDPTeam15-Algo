from tkinter import *
from tkinter.filedialog import askopenfilename
import re
from time import time, sleep

from Utils.utils import *
from Algo.exploration import Exploration
from Algo.fastest_path import *
from Utils.constants import *

import threading
from ast import literal_eval
from Connections.connection_client import Message_Handler


"""This module defines the main GUI window for the robot simulation."""

__author__ = 'MDPTeam15'


class TimeUp(Exception):
    """Raised when the time limit has reached."""
    def __init__(self):
        print("TIME'S UP (GUI)")


class Window(Frame):
    """
    This class is the main GUI window.
    """

    _grid_size = 30                     # size of one grid square in pixels

    def __init__(self, master):
        enable_print()

        """Initializes the GUI."""
        Frame.__init__(self, master)

        self._master = master
        self._filename = ''

        print("Init window starting")
        self._init_window()
        print("Init window completed")

        """
        Initialize the Controller class.
        """
        self._is_sim = IS_SIMULATE_MODE

        if self._is_sim:
            print('Simulation run')
            print('Loading Map......')

            import tkinter as tk
            root = tk.Tk()
            root.withdraw()

            while not self._load_map():
                while True:
                    ans = input("Try again? (y/n) ").strip()
                    if ans.lower() == 'y':
                        break
                    elif ans.lower() == 'n':
                        exit()

            self._timestep = float(input("Timestep: ").strip())
            from Algo.sim_robot import Robot
            self._robot = Robot(exploration_status=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)],
                                facing=NORTH,
                                discovered_map=[[2] * ROW_LENGTH for _ in range(COL_LENGTH)],
                                real_map=[[]])
        else:
            print('Real run')
            from Algo.real_robot import Robot
            self._robot = Robot(exploration_status=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)],
                                facing=NORTH,
                                discovered_map=[[2] * ROW_LENGTH for _ in range(COL_LENGTH)])

            self._paint_map()

        self._explore_limit = COMPLETION_THRESHOLD
        self._time_limit = TIME_LIMITE

        # Initialize connention client thread
        self._sender = Message_Handler(self._receive_handler)
        self._auto_update = True

        print('Init complete!')
        self._sender.send_rpi("Hello from PC to RPi\n")
        self._sender.send_arduino("Hello from PC to Arduino\n")
        self._sender.send_android("Hello from PC to Android\n")

        self._facing = self._robot.facing
        self._draw_robot(START, self._facing)

        self.is_arrow_scan = IS_ARROW_SCAN

        disable_print()

    def _init_window(self):
        """
        Load all window elements.
        """
        self._master.title("MDP Team 15 Robot Simulation")

        self.pack(fill=BOTH, expand=1)

        bg_frame = Frame(self)
        bg_frame.pack(fill=X, padx=90)

        self._explore_label = Label(bg_frame, text="Explore Completion:")
        self._explore_label.grid(row=0, column=0)

        self._completion_label = Label(bg_frame, text="0")
        self._completion_label.grid(row=0, column=1)

        self._time_limit_label = Label(bg_frame, text="Time Spent(seconds):")
        self._time_limit_label.grid(row=1, column=0)

        self._time_spent_label = Label(bg_frame, text="0.0s")
        self._time_spent_label.grid(row=1, column=1)

        self._canvas = Canvas(self, height=COL_LENGTH * self._grid_size + 1, width=ROW_LENGTH * self._grid_size + 1,
                              borderwidth=0, highlightthickness=0, background='#ffffff')
        self._canvas.pack(padx=20, pady=20)

        # Draw grid
        self._draw_grid()

    def _receive_handler(self, msg):
        """
        Parse and handle messages from the Android device.

        :param msg: The message received from the Android device.
        :return: N/A
        """
        if msg[0:8] == ANDROID_WAYPOINT:
            self._set_way_point(msg[8:])
        elif msg == ANDROID_CALIBRATE:
            thread = threading.Thread(target=self._calibrate)
            thread.daemon = True
            thread.start()
            enable_print()
            print('Start CALIBRATION')
            disable_print()
        elif msg == ANDROID_EXPLORE:
            thread = threading.Thread(target=self._explore)
            thread.daemon = True
            thread.start()
            enable_print()
            print('Start EXPLORATION')
            disable_print()
        elif msg == ANDROID_MOVE_FAST_PATH:
            thread = threading.Thread(target=self._move_fastest_path)
            thread.daemon = True
            thread.start()
            enable_print()
            print('Start FAST PATH')
            disable_print()
        elif msg == ANDROID_LOAD_EXPLORE_MAP:
            thread = threading.Thread(target=self._load_explore_map)
            thread.daemon = True
            thread.start()
            enable_print()
            print('LOAD EXPLORE MAP')
            disable_print()
        elif msg == ANDROID_FORWARD:
            self._sender.send_arduino(ARDUINO_FORWARD)
        elif msg == ANDROID_TURN_LEFT:
            self._sender.send_arduino(ARDUINO_TURN_LEFT)
        elif msg == ANDROID_TURN_RIGHT:
            self._sender.send_arduino(ARDUINO_TURN_RIGHT)
        elif msg == ANDROID_TURN_TO_BACKWARD:
            self._sender.send_arduino(ARDUINO_TURN_TO_BACKWARD)
        elif msg == 'arrow_on':
            self.is_arrow_scan = True
        elif msg == 'arrow_off':
            self.is_arrow_scan = False

    def _load_explore_map(self):
        if self._is_sim:
            from Algo.sim_robot import Robot
            self._robot = Robot(exploration_status=EXPLORE_STATUS_MAP,
                                facing=NORTH,
                                discovered_map=EXPLORATION_OBSTACLE_MAP,
                                real_map=[[]])
        else:
            from Algo.real_robot import Robot
            self._robot = Robot(exploration_status=EXPLORE_STATUS_MAP,
                                facing=NORTH,
                                discovered_map=EXPLORATION_OBSTACLE_MAP)

        cells = [item for sublist in EXPLORATION_OBSTACLE_MAP for item in sublist]
        updated_cells = {i+1: cells[i] for i in range(len(cells))}
        self._update_cells(updated_cells)
        self._update_android()

        self._calibrate_after_exploration()

    def _set_way_point(self, coordinate):
        """
        Set the waypoint coordinates.

        :param coordinate: The coordinates received from the Android device.
        :return: N/A
        """
        enable_print()
        (col, row) = literal_eval(coordinate)
        self._way_point = (19 - row, col)
        print('Set Waypoint: {}'.format(self._way_point))
        self._mark_way_point(get_grid_index(19 - row, col))
        disable_print()

    def _calibrate(self):
        """
        Calibrate the robot.

        :return: N/A
        """
        self._robot.calibrate(self._sender)

        # if self._is_sim:
        #     self._robot.calibrate()
        # else:
        #     self._robot.calibrate(self._sender)

    def _update_android(self):
        """
        Send the latest updates to the Android device.

        :return: N/A
        """
        msgs = []
        # Send the latest MDF strings to the Android device.
        msgs.append('"exploreMap":"%s"'%self._robot.get_explore_string())
        msgs.append('"obstacleMap":"%s"'%self._robot.get_map_string())
        y, x = get_matrix_coords(self._robot.center)
        msgs.append('"robotPosition":"%s,%s,%s"' % (str(x), str(19 - y), str(self._robot.facing)))
        msgs.append('"arrowPosition":"{}"'.format(';'.join(self._robot.arrows_arduino)))

        self._sender.send_android('{' + ','.join(msgs) + '}')

    def _explore(self):
        """Start the exploration."""

        start_time = time()
        # if self._is_sim:
        #     self._robot.real_map = self._grid_map
        exploration = Exploration(self._robot, start_time, self.is_arrow_scan, self._explore_limit, self._time_limit)

        if self._is_sim:
            run = exploration.start()
        else:
            run = exploration.start_real(self._sender)

        initial_pos = next(run)
        self._update_cells(initial_pos)
        self._update_android()

        while True:
            try:
                # Exploration until completion
                while True:
                    print('=' * 100)

                    updated_cells = run.send(0)

                    print('-' * 50)
                    print('updated_cells (sensor_readings): {}'.format(updated_cells)) # sensor_reading

                    self._update_cells(updated_cells)
                    self._update_android()

                    direction, move_or_turn, updated_cells = run.send(0)
                    print('direction, move_or_turn, updated_cells (robot standing): {}'.format((MOVEMENTS[direction], MOVE_TURN[move_or_turn], updated_cells)))

                    # if self._is_sim:
                    #     sleep(self._timestep)

                    # if IS_SLEEP:
                    #     sleep(SLEEP_SEC)

                    self._time_spent_label.config(text="%.2f" % get_time_elapsed(start_time) + "s")
                    self._update_cells(updated_cells)

                    if move_or_turn == MOVE:
                        self._move_robot(direction)
                    elif move_or_turn == TURN:
                        self._turn_head(self._facing, direction)

                    self._update_android()
                    print_map_info(self._robot)

                    is_complete = run.send(0)
                    if is_complete:
                        self._update_android()

                        enable_print()
                        print_map_info(self._robot)
                        disable_print()
                        break

                    is_back_at_start = run.send(0)
                    if is_back_at_start:

                        enable_print()
                        print('Back to start......')
                        print_map_info(self._robot)
                        disable_print()

                        # Move to unexplored area
                        while True:
                            print('=' * 100)
                            updated_or_moved_or_turned, value, is_complete = run.send(0)
                            # if self._is_sim:
                            #     sleep(self._timestep)
                            # if IS_SLEEP:
                            #     sleep(SLEEP_SEC)

                            self._time_spent_label.config(text="%.2f" % get_time_elapsed(start_time) + "s")

                            print('-' * 50)
                            if updated_or_moved_or_turned == "updated":
                                self._update_cells(value)
                                self._update_android()
                                print('updated_cells: {}'.format(value)) # sensor_reading
                            elif updated_or_moved_or_turned == "moved":
                                self._move_robot(value)
                                self._update_android()
                                print('moved robot: {}'.format(MOVEMENTS[value])) # sensor_reading
                            elif updated_or_moved_or_turned == "turned":
                                self._turn_head(self._facing, value)
                                self._update_android()
                                print('turned robot: {}'.format(MOVEMENTS[value])) # sensor_reading
                            else:
                                # invalid (no path find)
                                break

                            if is_complete:
                                self._update_android()

                                enable_print()
                                print_map_info(self._robot)
                                disable_print()
                                break
                            print_map_info(self._robot)

                        break

                # Returning to start after completion
                enable_print()
                print("Returning to Start...")
                disable_print()
                while True:
                    direction = run.send(0)
                    # if self._is_sim:
                    #     sleep(self._timestep)
                    # if IS_SLEEP:
                    #     sleep(SLEEP_SEC)

                    self._move_robot(direction)
                    self._update_android()

            except StopIteration:
                print_map_info(self._robot)
                break

        enable_print()
        print('Exploration Done')
        disable_print()

        self._calibrate_after_exploration()

        if self.is_arrow_scan:
            if self._robot.arrows:
                for y, x, facing in self._robot.arrows:
                    self._draw_arrow(get_grid_index(y, x), facing)

        self._update_android()
        enable_print()
        print_map_info(self._robot)
        disable_print()

    def _calibrate_after_exploration(self):
        """
        Post-exploration calibration to prepare the robot for the fastest path.

        :return: N/A
        """
        enable_print()
        print('Calibrating for fast path...')
        disable_print()
        self._fastest_path = self._find_fastest_path()

        if self._is_sim:
            sleep(self._timestep)
            self._robot.turn_robot(self._fastest_path[0])
            self._turn_head(self._facing, self._fastest_path[0])
        else:
            if self._fastest_path[0] != FORWARD:
                print('Turning Robot')
                self._robot.turn_robot(self._sender, self._fastest_path[0], self.is_arrow_scan)
                print('Robot Turned')
                self._turn_head(self._facing, self._fastest_path[0])

        self._fastest_path[0] = FORWARD
        self._update_android()

        enable_print()
        print('Calibrating Done!')
        disable_print()

    def _find_fastest_path(self):
        """Calculate and return the set of moves required for the fastest path."""
        from Algo.sim_robot import Robot
        clone_robot = Robot(exploration_status=self._robot.exploration_status,
                            facing=self._robot.facing,
                            discovered_map=self._robot.discovered_map,
                            real_map=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)])

        fastest_path_start_way_point = get_shortest_path_moves(clone_robot,
                                                               start=(1, 1),
                                                               goal=self._way_point)

        if fastest_path_start_way_point:
            for move in fastest_path_start_way_point:
                clone_robot.move_robot(move)

        before_way_point = previous_cell(clone_robot.center, clone_robot.facing)

        fastest_path_way_point_goal = get_shortest_path_moves(clone_robot,
                                                              start=self._way_point,
                                                              goal=(18, 13),
                                                              before_start_point=before_way_point)

        return fastest_path_start_way_point + fastest_path_way_point_goal

    def _move_fastest_path(self):
        """Move the robot along the fastest path."""
        if self._fastest_path:
            self._robot.is_fast_path = True

            if self._is_sim:
                move_strs = get_fastest_path_moves(self._fastest_path)
                for move_str in move_strs:
                    self._sender.send_arduino(move_str)

                    for cmd in move_str:
                        sleep(self._timestep)
                        self._robot.move_robot_algo(convert_arduino_cmd_to_direction(cmd))
                        if convert_arduino_cmd_to_direction(cmd) == FORWARD:
                            self._move_robot(convert_arduino_cmd_to_direction(cmd))
                        else:
                            self._turn_head(self._facing, convert_arduino_cmd_to_direction(cmd))
                        self._update_android()

            else:
                move_strs = get_fastest_path_moves(self._fastest_path)
                for move_str in move_strs:
                    sleep(FAST_PATH_SLEEP_SEC)
                    self._sender.send_arduino(move_str)

                    for cmd in move_str:
                        self._sender.wait_arduino(ARDUIMO_MOVED)

                        self._robot.move_robot_algo(convert_arduino_cmd_to_direction(cmd))
                        if convert_arduino_cmd_to_direction(cmd) == FORWARD:
                            self._move_robot(convert_arduino_cmd_to_direction(cmd))
                        else:
                            self._turn_head(self._facing, convert_arduino_cmd_to_direction(cmd))

                        self._update_android()

            enable_print()
            print('Reached GOAL!')
            disable_print()
        else:
            enable_print()
            print("No valid path")
            disable_print()

    def _draw_grid(self):
        """Draw the virtual maze."""
        self._grid_squares = []

        for y in range(COL_LENGTH):
            temp_row = []
            for x in range(ROW_LENGTH):
                temp_square = self._canvas.create_rectangle(x * self._grid_size, (COL_LENGTH - 1 - y) * self._grid_size,
                                                            (x + 1) * self._grid_size,
                                                            (COL_LENGTH - y) * self._grid_size, width=3)
                temp_row.append(temp_square)

            self._grid_squares.append(temp_row)

    def _draw_robot(self, location, facing):
        """Draw the robot in a given location with a given facing."""
        if location in BORDERS[NORTH] or location in BORDERS[SOUTH] \
                or location in BORDERS[EAST] or location in BORDERS[WEST]:
            print("invalid location")
            return

        top_left_grid = self._canvas.coords(location + ROW_LENGTH - 1)
        x = top_left_grid[0]
        y = top_left_grid[1]

        self._robot_graphic = self._canvas.create_oval(x, y, x + (3 * self._grid_size), y + (3 * self._grid_size),
                                                       width=2, fill="#354458", outline="#252a33")
        self._draw_head(location, facing)

    def _draw_head(self, location, facing):
        """Draw the head of the robot based on the robot's location and facing."""
        if facing == NORTH:
            corner = self._canvas.coords(location + ROW_LENGTH)
        elif facing == SOUTH:
            corner = self._canvas.coords(location - ROW_LENGTH)
        elif facing == EAST:
            corner = self._canvas.coords(location + 1)
        else:
            corner = self._canvas.coords(location - 1)
        x, y = corner[0], corner[1]
        self._head = self._canvas.create_oval(x + (self._grid_size // 3), y + (self._grid_size // 3),
                                              x + (2 * (self._grid_size // 3)) + 1,
                                              y + (2 * (self._grid_size // 3)) + 1,
                                              width=0, fill="#7acdc8")


    def _turn_head(self, facing, direction):
        """Move the graphical head of the robot in a certain direction."""
        if facing == NORTH:
            if direction == LEFT:
                self._canvas.move(self._head, -self._grid_size, self._grid_size)
                self._facing = WEST
            elif direction == RIGHT:
                self._canvas.move(self._head, self._grid_size, self._grid_size)
                self._facing = EAST
            elif direction == BACKWARD:
                self._canvas.move(self._head, 0, self._grid_size * 2)
                self._facing = SOUTH
        elif facing == SOUTH:
            if direction == LEFT:
                self._canvas.move(self._head, self._grid_size, -self._grid_size)
                self._facing = EAST
            elif direction == RIGHT:
                self._canvas.move(self._head, -self._grid_size, -self._grid_size)
                self._facing = WEST
            elif direction == BACKWARD:
                self._canvas.move(self._head, 0, -self._grid_size * 2)
                self._facing = NORTH
        elif facing == EAST:
            if direction == LEFT:
                self._canvas.move(self._head, -self._grid_size, -self._grid_size)
                self._facing = NORTH
            elif direction == RIGHT:
                self._canvas.move(self._head, -self._grid_size, self._grid_size)
                self._facing = SOUTH
            elif direction == BACKWARD:
                self._canvas.move(self._head, -self._grid_size * 2, 0)
                self._facing = WEST
        else:
            if direction == LEFT:
                self._canvas.move(self._head, self._grid_size, self._grid_size)
                self._facing = SOUTH
            elif direction == RIGHT:
                self._canvas.move(self._head, self._grid_size, -self._grid_size)
                self._facing = NORTH
            elif direction == BACKWARD:
                self._canvas.move(self._head, self._grid_size * 2, 0)
                self._facing = EAST

        self.update()

    def _move_robot(self, direction):
        """Move the graphical robot in a certain direction."""
        switcher = {
            NORTH: self._north_facing_move,
            SOUTH: self._south_facing_move,
            EAST: self._east_facing_move,
            WEST: self._west_facing_move
        }

        f = switcher.get(self._facing)

        if direction == BACKWARD:
            self._turn_head(self._facing, RIGHT)
            self._turn_head(self._facing, RIGHT)
        elif direction != FORWARD:
            self._turn_head(self._facing, direction)

        f(direction)

    def _north_facing_move(self, direction):
        """Move the graphical robot as it is facing north."""
        switcher = {
            FORWARD: self._move_up,
            LEFT: self._move_left,
            RIGHT: self._move_right,
            BACKWARD: self._move_down
        }

        f = switcher.get(direction)

        f()

    def _south_facing_move(self, direction):
        """Move the graphical robot as it is facing south."""
        switcher = {
            FORWARD: self._move_down,
            LEFT: self._move_right,
            RIGHT: self._move_left,
            BACKWARD: self._move_up
        }

        f = switcher.get(direction)

        f()

    def _east_facing_move(self, direction):
        """Move the graphical robot as it is facing east."""
        switcher = {
            FORWARD: self._move_right,
            LEFT: self._move_up,
            RIGHT: self._move_down,
            BACKWARD: self._move_left
        }

        f = switcher.get(direction)

        f()

    def _west_facing_move(self, direction):
        """Move the graphical robot as it is facing west."""
        switcher = {
            FORWARD: self._move_left,
            LEFT: self._move_down,
            RIGHT: self._move_up,
            BACKWARD: self._move_right
        }

        f = switcher.get(direction)

        f()

    def _move_up(self):
        """Move the graphical robot toward the top of the maze regardless of its facing."""
        self._canvas.move(self._robot_graphic, 0, -self._grid_size)
        self._canvas.move(self._head, 0, -self._grid_size)
        self.update()

    def _move_left(self):
        """Move the graphical robot toward the left of the maze regardless of its facing."""
        self._canvas.move(self._robot_graphic, -self._grid_size, 0)
        self._canvas.move(self._head, -self._grid_size, 0)
        self.update()

    def _move_right(self):
        """Move the graphical robot toward the right of the maze regardless of its facing."""
        self._canvas.move(self._robot_graphic, self._grid_size, 0)
        self._canvas.move(self._head, self._grid_size, 0)
        self.update()

    def _move_down(self):
        """Move the graphical robot toward the right of the maze regardless of its facing."""
        self._canvas.move(self._robot_graphic, 0, self._grid_size)
        self._canvas.move(self._head, 0, self._grid_size)
        self.update()

    def _update_cells(self, updated_cells):
        """Repaint the cells that have been updated."""
        start_cells = get_robot_cells(START)
        goal_cells = get_robot_cells(GOAL)
        for cell, value in updated_cells.items():
            if cell in start_cells:
                self.mark_cell(cell, START_AREA)
            elif cell in goal_cells:
                self.mark_cell(cell, GOAL_AREA)
            elif not value:
                self.mark_cell(cell, EXPLORED)
            else:
                self.mark_cell(cell, OBSTACLE)

        if self.is_arrow_scan:
            for y, x, facing in self._robot.arrows:
                self._draw_arrow(get_grid_index(y, x), facing)

        self._completion_label.config(text=(str(self._robot.get_completion_count())))

    def _load_map(self):
        """
        Load a map descriptor text file.

        :return: True if the file exists and is able to be successfully parsed, false otherwise.
        """
        self._filename = askopenfilename(title="Select Map Descriptor", filetypes=[("Text Files (*.txt)", "*.txt")])

        if self._filename:
            print(self._filename)
            if self._parse_map(self._filename):
                self._paint_map()
                return True
            print("File %s cannot be parsed" % self._filename)
            return False
        print("File %s does not exist" % self._filename)
        return False

    def _mark_way_point(self, grid_num):
        """
        Mark a grid as obstructed

        :param grid_num: ID of the grid to be marked
        :return: N/A
        """
        self._canvas.itemconfig(grid_num, fill="#ffc700")
        self.update()

    def _parse_map(self, filename):
        """
        Parse a map descriptor text file

        :param filename: The name of the map descriptor file
        :return: True if the file is able to be successfully parsed, false otherwise.
        """
        file = open(filename, mode="r")
        map_str = file.read()

        match = re.fullmatch("[012345\n]*", map_str)
        if match:
            self._grid_map = []
            row_strings = map_str.split("\n")
            for row_string in row_strings[:NUM_ROWS]:
                grid_row = []
                for char in row_string:
                    bit = int(char)
                    grid_row.append(bit)
                self._grid_map.append(grid_row)
            return True

        return False

    def _paint_map(self):
        """Paint the unexplored map on the grid."""
        for i in range(NUM_ROWS):
            for j in range(NUM_COLS):
                grid_num = i * ROW_LENGTH + j + 1
                self.mark_cell(grid_num, UNEXPLORED)

    def mark_cell(self, cell_index, cell_type):
        """Mark a cell as a certain type."""
        self._canvas.itemconfig(cell_index, fill=cell_type)
        self.update()

    def _draw_arrow(self, location, facing):
        top_left_grid = self._canvas.coords(location)
        x, y = top_left_grid[0], top_left_grid[1]

        if facing == NORTH:
            points = [x, y + self._grid_size, x + self._grid_size, y + self._grid_size, x + self._grid_size/2, y]
        elif facing == SOUTH:
            points = [x, y, x + self._grid_size, y, x + self._grid_size/2, y + self._grid_size]
        elif facing == EAST:
            points = [x, y, x, y + self._grid_size, x + self._grid_size, y + self._grid_size/2]
        else:
            points = [x + self._grid_size, y, x + self._grid_size, y + self._grid_size, x, y + self._grid_size/2]

        self._canvas.create_polygon(points, fill='gold', width=3)


def get_time_elapsed(start_time):
    """Get the elapsed time."""
    return float(time() - start_time)
