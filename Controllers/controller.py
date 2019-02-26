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

            # self._timestep = 0.1
            self._timestep = float(input("Timestep: ").strip())
            self._explore_limit = 100
            # self._explore_limit = float(input("Coverage Limit: ").strip())
            # self._time_limit = float(input("Time Limit: ").strip())
            self._time_limit = 100
            # self._time_limit = 100

            from Algo.sim_robot import Robot
            self._robot = Robot(exploration_status=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)],
                                facing=NORTH,
                                discovered_map=[[2] * ROW_LENGTH for _ in range(COL_LENGTH)],
                                real_map=[[]])
        else:
            print('Real run')

            self._explore_limit = 100.0
            self._time_limit = 720
            from Algo.real_robot import Robot
            self._robot = Robot(exploration_status=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)],
                                facing=NORTH,
                                discovered_map=[[2] * ROW_LENGTH for _ in range(COL_LENGTH)])

        # Initialize connention client thread
        self._sender = Message_Handler(self._receive_handler)
        self._auto_update = True

        print('Init complete!')
        self._sender.send_rpi("Hello from PC to RPi\n")
        self._sender.send_arduino("Hello from PC to Arduino\n")
        self._sender.send_android("Hello from PC to Android\n")
        disable_print()

    def _receive_handler(self, msg):
        """
        Parse and handle messages from the Android device.

        :param msg: The message received from the Android device.
        :return: N/A
        """
        if msg == ANDROID_CALIBRATE:
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
        elif msg == ANDROID_AUTO_UPDATE_TRUE:
            self._auto_update = True
        elif msg == ANDROID_AUTO_UPDATE_FALSE:
            self._auto_update = False
            self._update_android(True, True, override=True)
        elif msg[0:8] == ANDROID_WAYPOINT:
            self._set_way_point(msg[8:])
        elif msg == ANDROID_MOVE_FAST_PATH:
            thread = threading.Thread(target=self._move_fastest_path)
            thread.daemon = True
            thread.start()
        elif msg == ANDROID_FORWARD:
            self._sender.send_arduino(ARDUINO_MOVE_FORWARD)
        elif msg == ANDROID_TURN_LEFT:
            self._sender.send_arduino(ARDUINO_TURN_LEFT)
        elif msg == ANDROID_TURN_RIGHT:
            self._sender.send_arduino(ARDUINO_TURN_RIGHT)
        elif msg == ANDROID_BACK:
            self._sender.send_arduino(ARDUINO_BACKWARD)
        elif msg == ANDROID_SENSOR:
            self._sender.send_arduino(ARDUINO_SENSOR)


    def _set_way_point(self, coordinate):
        """
        Set the waypoint coordinates.

        :param coordinate: The coordinates received from the Android device.
        :return: N/A
        """
        enable_print()
        print('Set Waypoint: {}'.format(coordinate))
        (col, row) = literal_eval(coordinate)
        self._way_point = (row, col)
        disable_print()

    def _calibrate(self):
        """
        Calibrate the robot.

        :return: N/A
        """
        if not self._is_sim:
            self._sender.send_android('{"status":"calibrating"}')
            self._sender.send_arduino('z')
            self._sender.wait_arduino('D')
            self._robot.turn_robot(self._sender, RIGHT)
            self._robot.get_sensor_readings(self._sender)
            self._robot.turn_robot(self._sender, LEFT)
            self._sender.send_arduino('z')
            self._sender.wait_arduino('D')
        self._sender.send_android('{"status":"calibrating done"}')

    def _update_map_android(self):
        """
        Send the latest MDF strings to the Android device.

        :return: N/A
        """
        self._sender.send_android('{"exploreMap":"%s","obstacleMap":"%s"}' % (self._robot.get_explore_string(), self._robot.get_map_string()))

    def _update_coords_android(self):
        """
        Send the latest coordinates and facing of the robot to the Android device.

        :return: N/A
        """
        y, x = get_matrix_coords(self._robot.center)
        self._sender.send_android('{"robotPosition":[%s,%s,%s]}' % (str(x), str(y), str(self._robot.facing)))

    def _update_android(self, is_update_map, is_update_coords, override=False):
        """
        Send the latest updates to the Android device.

        :param is_update_map: Whether the latest MDF string should be updated.
        :param is_update_coords: Whether the latest coordinates and facing should be updated.
        :param override: If this is set to true, the update will be sent even when the Android is not in auto-update.
        :return: N/A
        """
        if self._auto_update or override:
            if is_update_map:
                self._update_map_android()
            if is_update_coords:
                self._update_coords_android()

    def _explore(self):
        """
        Conduct the exploration.

        :return: N/A
        """
        enable_print()

        start_time = time()
        if self._is_sim:
            self._robot.real_map = self._grid_map
        exploration = Exploration(self._robot, start_time, self._explore_limit, self._time_limit)

        if self._is_sim:
            run = exploration.start()
        else:
            run = exploration.start_real(self._sender)

        next(run) # Robot Standing
        self._update_android(True, True)
        while True:
            try:
                # Exploration until completion
                while True:
                    print('-' * 50)

                    run.send(0) # empty updated_cells
                    self._update_android(True, True)

                    print_map_info(self._robot)

                    run.send(0) # robot movement
                    if self._is_sim:
                        sleep(self._timestep)

                    self._update_android(True, True)

                    is_complete = run.send(0) # is_complete
                    if is_complete:
                        enable_print()
                        print_map_info(self._robot)
                        disable_print()
                        break

                    is_back_at_start = run.send(0) # is_back_to_start
                    if is_back_at_start:

                        enable_print()
                        print('Back to start......')
                        print_map_info(self._robot)
                        disable_print()

                        while True:
                            updated_or_moved, value, is_complete = run.send(0)
                            print_map_info(self._robot)

                            if self._is_sim:
                                sleep(self._timestep)
                            if updated_or_moved == "updated" or updated_or_moved == "moved":
                                self._update_android(True, True)
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
                    run.send(0)
                    if self._is_sim:
                        sleep(self._timestep)

                    self._update_android(True, True)

            # Raised by an iterator’s next() method to signal that there are no further values.
            except StopIteration:
                if not self._is_sim:
                    # Adjust robot position to face to North
                    enable_print()
                    print_map_info(self._robot)
                    print('Returned to start!')

                    self._sender.send_arduino('j')
                    self._sender.wait_arduino('U')
                    if self._robot.facing == WEST:
                        print('ENTERED FACING WEST')
                        sleep(0.05)
                        self._robot.turn_robot(self._sender, LEFT)
                        self._sender.send_arduino('j')
                        self._sender.wait_arduino('U')
                        self._robot.turn_robot(self._sender, RIGHT)
                        sleep(0.05)
                        self._robot.turn_robot(self._sender, RIGHT)
                        sleep(0.05)
                        self._sender.send_arduino('x')
                        self._sender.wait_arduino('D')
                    elif self._robot.facing == SOUTH:
                        print('ENTERED FACING SOUTH')
                        sleep(0.05)
                        self._robot.turn_robot(self._sender, RIGHT)
                        self._sender.send_arduino('j')
                        self._sender.wait_arduino('U')
                        sleep(0.05)
                        self._robot.turn_robot(self._sender, RIGHT)
                        sleep(0.05)
                        self._sender.send_arduino('x')
                        self._sender.wait_arduino('D')

                    disable_print()
                break

        enable_print()
        print('Calibrating...')
        self._calibrate_after_exploration()
        disable_print()

    def _calibrate_after_exploration(self):
        """
        Post-exploration calibration to prepare the robot for the fastest path.

        :return: N/A
        """
        self._fastest_path = self._find_fastest_path()

        if self._is_sim:
            sleep(self._timestep)
            self._robot.turn_robot(self._fastest_path[0])
        else:
            self._robot.turn_robot(self._sender, self._fastest_path[0])
        self._fastest_path[0] = FORWARD
        self._update_android(True, True)
        self._sender.send_android('{"status":"explore done"}')
        self._sender.send_android('{"status":"calibrating done"}')

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
        """Move along the fastest path to the goal zone."""
        if self._fastest_path:
            move_str = get_fastest_path_move_string(self._fastest_path)
            print('Move String: {}'.format(move_str))

            if self._is_sim:
                for move in self._fastest_path:
                    sleep(self._timestep)
                    self._robot.move_robot(move)
                    self._update_android(False, True)
            else:
                # 'nn/a/n/d/nnnnn'.split('/') = ['nn', 'a', 'n', 'd', 'nnnnn']
                move_str = get_fastest_path_move_string(self._fastest_path)
                moves = move_str.split('/')
                for move in moves:
                    move_len = len(move)
                    if move[0] == 'n':
                        move = move[:-1] + 't'
                    elif move[0] == 'a' or move[0] == 'd':
                        pass
                    self._sender.send_arduino(move)
                    for i in range(move_len):
                        self._sender.wait_arduino('M')
            enable_print()
            print('GOAL Reached!')
            disable_print()
        else:
            enable_print()
            print("No valid path")
            disable_print()

    def _load_map(self):
        """
        Load a map descriptor text file.

        :return: True if the file exists and is able to be successfully parsed, false otherwise.
        """
        filename = askopenfilename(title="Select Map Descriptor", filetypes=[("Text Files (*.txt)", "*.txt")])

        if filename:
            print(filename)
            if self._parse_map(filename):
                return True
            print("File %s cannot be parsed" % filename)
            return False
        print("File %s does not exist" % filename)
        return False

    def _parse_map(self, filename):
        """
        Parse a map descriptor text file

        :param filename: The name of the map descriptor file
        :return: True if the file is able to be successfully parsed, false otherwise.
        """
        file = open(filename, mode="r")
        map_str = file.read()

        match = re.fullmatch("[01\n]*", map_str)
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
