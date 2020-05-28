# -*- coding: utf-8 -*-
"""
Created on Thu Oct 04 17:53:55 2018

@author: Quantum Engineer
"""

import sys
import time
import zprocess
import h5py
import numpy as np
import matplotlib.pyplot as plt
import os
import clr
from device_server import DeviceServer

sys.path.append(r"C:\Program Files\Thorlabs\Kinesis")
clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.InertialMotorCLI")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *


class KCubeController:
    def __init__(self, serial_number):
        DeviceManagerCLI.BuildDeviceList()
        serial_numbers = DeviceManagerCLI.GetDeviceList()
        print(f"Found serial numbers of conneccted controllers: {serial_numbers}")
        self.device = KCubeInertialMotor.CreateDevice(serial_number)
        self.device.Connect(serial_number)
        device_info = self.device.GetDeviceInfo()
        print(f"Connected to device {device_info.Name} with serial number {serial_number}")
        return

    def get_actual_positions(self):
        channels = [1, 2, 3, 4]
        self.positions = [self.device.GetPosition(i) for i in channels]
        return self.positions

    def move_to_positions(self, positions):
        channels = [1, 2, 3, 4]
        assert len(positions) == len(channels), "position array has incorrect length"
        for channel, position, current_pos in zip(channels, positions, self.positions):
            if position !=current_pos:
                self.device.MoveTo(channel, position, 0)


class KinesisServer(DeviceServer):
    def __init__(self, variable_names, serial_numbers, port):
        super().__init__(port)
        print("Starting Kinesis Server")
        self.serials = serial_numbers
        self.variables = variable_names
        self.controllers = [KCubeController(i) for i in self.serials]

    def transition_to_buffered(self, h5_filepath):
        """
        Get positions from h5_filepath
        Get positions from picomotors
        If positions are different move picomotors
        """
        with h5py.File(h5_filepath) as f:
            desired_positions = [
                self.get_desired_positions(i, f) for i in self.variables
            ]

        print(f"Desired positions are: {desired_positions}")
        actual_positions = [i.get_actual_positions() for i in self.controllers]
        print(f"Actual positions are: {actual_positions}")
        for controller, positions in zip(self.controllers, desired_positions):
            controller.move_to_positions(positions)
        print("transition to buffered")

    def get_desired_positions(self, variables, hf):
        desired_positions = [hf["globals"].attrs[i] for i in variables]
        return desired_positions

    def transition_to_static(self, h5_filepath):
        """To be overridden by subclasses. Do any post processing after a
        shot, eg computing optical depth, fits, displaying images, saving
        images and results to the h5 file, returning cameras to an idle
        state."""
        print("transition to static")

    def abort(self):
        """To be overridden by subclasses. Return cameras and any other state
        to one in which transition_to_buffered() can be called again. abort()
        will be called if there was an exception in either
        transition_to_buffered() or transtition_to_static(), and so should
        ideally be written to return things to a sensible state even if those
        methods did not complete. Like any cleanup function, abort() should
        proceed to further cleanups even if earlier cleanups fail. As such it
        should make liberal use of try: except: blocks, so that an exception
        in performing one cleanup operation does not stop it from proceeding
        to subsequent cleanup operations"""
        print("abort")


serials = ["97100362", "97100395"]
global_names = [["Kinesis_Ch1", "Kinesis_Ch2", "Kinesis_Ch3", "Kinesis_Ch4"]]
port = 7426
assert len(serials) == len(global_names), "List of globals must be the same length as serial numbers"
kserver = KinesisServer(global_names, serials, port)
kserver.shutdown_on_interrupt()
