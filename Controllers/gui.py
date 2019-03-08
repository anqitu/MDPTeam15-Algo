from tkinter import *
from tkinter.filedialog import askopenfilename
import re
from time import time, sleep

from Utils.utils import *
from Algo.exploration import Exploration
from Algo.fastest_path import *
from Utils.constants import *
from Algo.sim_robot import Robot

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
        """Initializes the GUI."""
        Frame.__init__(self, master)

        self._master = master

        enable_print()
        print("Init window starting")
        self._init_window()
        print("Init window completed")
        disable_print()

    def _init_window(self):
        """
        Load all window elements.
        """
        self._master.title("MDP Team 15 Robot Simulation")

        self.pack(fill=BOTH, expand=1)

        bg_frame = Frame(self)
        bg_frame.pack(fill=X, padx=90)

        self._timestep_label = Label(bg_frame, text="Timestep(seconds):")
        self._timestep_label.grid(row=0, column=0)

        self._timestep_entry = Entry(bg_frame, width=5, justify='center')
        self._timestep_entry.insert(END, "0.1")
        self._timestep_entry.grid(row=0, column=1)

        self._explore_label = Label(bg_frame, text="Explore Cutoff:")
        self._explore_label.grid(row=1, column=0)

        self._explore_entry = Entry(bg_frame, width=5, justify='center')
        self._explore_entry.insert(END, str(int(COMPLETION_THRESHOLD * 300)))
        self._explore_entry.grid(row=1, column=1)

        self._percentage_completion_label = Label(bg_frame, text="0.0%")
        self._percentage_completion_label.grid(row=1, column=2)

        self._time_limit_label = Label(bg_frame, text="Time Limit(seconds):")
        self._time_limit_label.grid(row=2, column=0)

        self._time_limit_entry = Entry(bg_frame, width=5, justify='center')
        self._time_limit_entry.insert(END, "60")
        self._time_limit_entry.grid(row=2, column=1)

        self._time_spent_label = Label(bg_frame, text="0.0s")
        self._time_spent_label.grid(row=2, column=2)

        self._loadBtn = Button(bg_frame, text="Load Map", command=self._load_map)
        self._loadBtn.grid(row=3, column=0)

        self._button = Button(bg_frame, text="Explore", command=self._explore)
        self._button.grid(row=3, column=1)

        self._fp_button = Button(bg_frame, text="Move Fastest Path", command=self._move_fastest_path)
        self._fp_button.grid(row=3, column=2)

        self._canvas = Canvas(self, height=COL_LENGTH * self._grid_size + 1, width=ROW_LENGTH * self._grid_size + 1,
                              borderwidth=0, highlightthickness=0, background='#ffffff')
        self._canvas.pack(padx=20, pady=20)

        # Draw grid
        self._draw_grid()

        self._robot = Robot(exploration_status=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)],
                            facing=NORTH,
                            discovered_map=[[2] * ROW_LENGTH for _ in range(COL_LENGTH)],
                            real_map=[[]])

        # Draw robot
        self._facing = self._robot.facing
        self._draw_robot(START, self._facing)

    def _explore(self):
        """Start the exploration."""
        start_time = time()

        time_limit = float(self._time_limit_entry.get().strip())
        explore_limit = float(self._explore_entry.get().strip())
        timestep = float(self._timestep_entry.get().strip())

        self._robot.real_map = self._grid_map
        exploration = Exploration(self._robot, start_time, explore_limit, time_limit)

        run = exploration.start()

        initial_pos = next(run)
        self._update_cells(initial_pos)

        while True:
            try:
                # Exploration until completion
                while True:
                    print('-' * 50)

                    updated_cells = run.send(0)
                    print('updated_cells (sensor_readings): {}'.format(updated_cells)) # sensor_reading

                    self._update_cells(updated_cells)

                    print_map_info(self._robot)

                    direction, move_or_turn, updated_cells = run.send(0)
                    print('direction, move_or_turn, updated_cells (robot standing): {}'.format((MOVEMENTS[direction], MOVE_TURN[move_or_turn], updated_cells)))

                    sleep(timestep)
                    self._time_spent_label.config(text="%.2f" % get_time_elapsed(start_time) + "s")
                    self._update_cells(updated_cells)

                    if move_or_turn == MOVE:
                        self._move_robot(direction)
                    elif move_or_turn == TURN:
                        self._turn_head(self._facing, direction)

                    is_complete = run.send(0)
                    if is_complete:
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
                            updated_or_moved, value, is_complete = run.send(0)
                            sleep(timestep)
                            self._time_spent_label.config(text="%.2f" % get_time_elapsed(start_time) + "s")

                            print_map_info(self._robot)

                            if updated_or_moved == "updated":
                                self._update_cells(value)
                            elif updated_or_moved == "moved":
                                self._move_robot(value)
                            else:
                                # invalid (no path find)
                                break

                            if is_complete:
                                enable_print()
                                print_map_info(self._robot)
                                disable_print()
                                break
                        break

                # Returning to start after completion
                enable_print()
                print("Returning to Start...")
                disable_print()

                while True:
                    direction = run.send(0)
                    sleep(timestep)
                    self._move_robot(direction)

            except StopIteration:
                print_map_info(self._robot)
                break

        enable_print()
        print('Exploration Done')
        disable_print()

        self._calibrate_after_exploration()

        if IS_ARROW_SCAN:
            # self._robot.postprocess_arrow_images()
            if self._robot.arrows:
                for y, x, facing in self._robot.arrows:
                    self._draw_arrow(get_grid_index(y, x), facing)

        enable_print()
        print_map_info(self._robot)
        disable_print()

    def _calibrate_after_exploration(self):
        """
        Post-exploration calibration to prepare the robot for the fastest path.

        :return: N/A
        """
        enable_print()
        print('Calibrating...')
        disable_print()

        timestep = float(self._timestep_entry.get().strip())
        self._fastest_path = self._find_fastest_path()

        sleep(timestep)
        self._robot.turn_robot(self._fastest_path[0])
        self._turn_head(self._facing, self._fastest_path[0])

        self._fastest_path[0] = FORWARD

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
            timestep = float(self._timestep_entry.get().strip())
            for move in self._fastest_path:
                sleep(timestep)
                self._robot.move_robot(move)
                self._move_robot(move)
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

                self._canvas.tag_bind(temp_square, "<Button-1>",
                                      lambda event, arg=temp_square: self._on_grid_click(event, arg))
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

        if IS_ARROW_SCAN:
            for y, x, facing in self._robot.arrows:
                self._draw_arrow(get_grid_index(y, x), facing)

        self._percentage_completion_label.config(text=("%.2f" % self._robot.get_completion_percentage() + "%"))

    def _load_map(self):
        """
        Load a map descriptor text file.

        :return: True if the file exists and is able to be successfully parsed, false otherwise.
        """
        filename = askopenfilename(title="Select Map Descriptor", filetypes=[("Text Files (*.txt)", "*.txt")])

        if filename:
            print(filename)
            if self._parse_map(filename):
                self._paint_map()
                return True
            print("File %s cannot be parsed" % filename)
            return False
        print("File %s does not exist" % filename)
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
            for row_string in row_strings:
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

    def _on_grid_click(self, event, arg):
        """Mark a cell as the waypoint."""

        self._way_point = (19 - int(event.y/30), int(event.x/30))
        self._mark_way_point(self._way_point[0] * ROW_LENGTH + self._way_point[1] + 1)
        event.widget.itemconfig(arg, activefill="#00ffff")


def get_time_elapsed(start_time):
    """Get the elapsed time."""
    return float(time() - start_time)
