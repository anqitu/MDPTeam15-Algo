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

"""This module defines the controller that sends messages to the Android."""

class Controller:
    """
    This class is the controller that relays messages to the Android.
    """
    def __init__(self):
        enable_print()

        """
        Initialize the Controller class.
        """
        self._filename = ''

        print('Real run')
        from Algo.real_robot import Robot
        self._robot = Robot(exploration_status=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)],
                            facing=NORTH,
                            discovered_map=[[2] * ROW_LENGTH for _ in range(COL_LENGTH)])

        self._explore_limit = COMPLETION_THRESHOLD
        self._time_limit = TIME_LIMITE

        # Initialize connention client thread
        self._sender = Message_Handler(self._receive_handler)
        self._auto_update = True

        print('Init complete!')
        self._sender.send_rpi("Hello from PC to RPi\n")
        self._sender.send_arduino("Hello from PC to Arduino\n")
        self._sender.send_android("Hello from PC to Android\n")
        disable_print()

        self.is_arrow_scan = IS_ARROW_SCAN

        self._set_way_point('3,17')

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
        elif msg == 'S':
            self._sender.send_arduino(ARDUINO_SENSOR)
        elif msg == ANDROID_BATTERY_DRAINER:
            thread = threading.Thread(target=self._battery_drainer)
            thread.daemon = True
            thread.start()
            enable_print()
            print('START BATTERY DRAINER')
            disable_print()

    def _load_explore_map(self):

        from Algo.real_robot import Robot
        self._robot = Robot(exploration_status=EXPLORE_STATUS_MAP,
                            facing=NORTH,
                            discovered_map=EXPLORATION_OBSTACLE_MAP)

        cells = [item for sublist in EXPLORATION_OBSTACLE_MAP for item in sublist]
        updated_cells = {i+1: cells[i] for i in range(len(cells))}
        self._update_android()

        self._calibrate()
        sleep(1)

        self._calibrate_after_exploration()
        sleep(1)

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
        disable_print()

    def _calibrate(self):
        """
        Calibrate the robot.

        :return: N/A
        """
        for move in ['C', 'S', 'L', 'D', 'C', 'L', 'D', 'C']:
            self._sender.send_arduino(move)
            self._sender.wait_arduino(ARDUIMO_MOVED)

        enable_print()
        print('Calibrating Done!')
        disable_print()

    def _battery_drainer(self):
        for j in range(2):
            for i in range(min(BATTERY_DRAINER_STEP_Y, 17)):
                self._robot.move_robot(self._sender, FORWARD)
            self._sender.send_arduino(BATTERY_DRAINER_TURN)
            self._sender.wait_arduino(ARDUIMO_MOVED)
            for i in range(min(BATTERY_DRAINER_STEP_X, 12)):
                self._robot.move_robot(self._sender, FORWARD)
            self._sender.send_arduino(BATTERY_DRAINER_TURN)
            self._sender.wait_arduino(ARDUIMO_MOVED)

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
        exploration = Exploration(self._robot, start_time, self.is_arrow_scan, self._explore_limit, self._time_limit)

        run = exploration.start_real(self._sender)

        initial_pos = next(run)
        self._update_android()

        while True:
            try:
                # Exploration until completion
                while True:
                    print('=' * 100)

                    updated_cells = run.send(0)

                    print('-' * 50)
                    print('updated_cells (sensor_readings): {}'.format(updated_cells)) # sensor_reading

                    self._update_android()

                    direction, move_or_turn, updated_cells = run.send(0)
                    print('direction, move_or_turn, updated_cells (robot standing): {}'.format((MOVEMENTS[direction], MOVE_TURN[move_or_turn], updated_cells)))

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

                            print('-' * 50)
                            if updated_or_moved_or_turned == "updated":
                                self._update_android()
                                print('updated_cells: {}'.format(value)) # sensor_reading
                            elif updated_or_moved_or_turned == "moved":
                                self._update_android()
                                print('moved robot: {}'.format(MOVEMENTS[value])) # sensor_reading
                            elif updated_or_moved_or_turned == "turned":
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
                    self._update_android()

            except StopIteration:
                print_map_info(self._robot)
                break

        enable_print()
        print('Exploration Done')
        disable_print()

        self._calibrate()
        sleep(1)

        self._calibrate_after_exploration()
        sleep(1)

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

        if self._fastest_path[0] != FORWARD:
            print('Turning Robot')
            self._robot.turn_robot(self._sender, self._fastest_path[0], self.is_arrow_scan)
            print('Robot Turned')

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
            self._moves_arduino = get_fastest_path_moves(self._fastest_path)
            moves_ardiono_with_calibration = add_calibration_to_arduino_moves(self._moves_arduino, self._robot)

            thread = threading.Thread(target=self._update_android_fast_path)
            thread.daemon = True
            thread.start()

            # self._sender.send_arduino(''.join(moves_ardiono_with_calibration))
            self._sender.send_arduino(''.join([s for s in moves_ardiono_with_calibration if s !='C']))

            # for moves in moves_ardiono_with_calibration:
            #     self._sender.send_arduino(moves)
            #     self._sender.wait_arduino(ARDUIMO_MOVED)

            enable_print()
            print('Reached GOAL!')
            disable_print()
        else:
            enable_print()
            print("No valid path")
            disable_print()

    def _update_android_fast_path(self):
        for move in ''.join(self._moves_arduino):
            sleep(0.7)

            self._robot.move_robot_algo(convert_arduino_cmd_to_direction(move))
            self._update_android()

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
