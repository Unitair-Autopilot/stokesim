"""
Definition of the Joystick class.

"""
from __future__ import division
import numpy as np
from threading import Thread
from collections import deque
from inputs import devices, get_gamepad  # sudo apt install python-pip && pip install inputs --user

##################################################

class Command(object):
    """
    Container for what the Joystick class records.

    """
    def __init__(self):
        self.throttle = np.float64(0)
        self.roll = np.float64(0)
        self.pitch = np.float64(0)
        self.yaw = np.float64(0)
        self.left_vertical = np.float64(0)
        self.left_trigger = np.float64(0)

    def as_array(self):
        return np.array([self.throttle,
                         self.roll,
                         self.pitch,
                         self.yaw,
                         self.left_vertical,
                         self.left_trigger], dtype=np.float64)

##################################################

class Joystick(object):
    """
    User interface for remote-controlling.
    Call start_joystick_thread to begin filling an internal buffer with user input.
    Call get_command to execute / clear the buffer and get the current relevant Command object.
    Call stop_joystick_thread when done!

    stick_deadband:   fraction of analog joystick travel that should be treated as zero
    trigger_deadband: fraction of analog trigger travel that should be treated as zero
    max_buffer_size:  maximum number of user commands that should be stored before dropping old ones
    button_callbacks: dictionary of callback functions keyed by button names (A, B, X, Y, L, R, SL, SR, DV, DH, K)

    """
    def __init__(self, stick_deadband=0.05, trigger_deadband=0.05, max_buffer_size=200, button_callbacks={}):
        self.stick_deadband = float(stick_deadband)
        self.trigger_deadband = float(trigger_deadband)
        self.max_buffer_size = int(max_buffer_size)
        self.button_callbacks = button_callbacks

        # Valid input device names in priority order
        self.valid_device_names = ["Microsoft X-Box One pad (Firmware 2015)",
                                   "PowerA Xbox One wired controller",
                                   "Logitech Gamepad F310"]

        # Set valid input device
        self.input_device = None
        for valid_device_name in self.valid_device_names:
            if self.input_device is not None: break
            for device in devices:
                if device.name == valid_device_name:
                    self.input_device = device.name
                    print("Hello, human! Ready to read from {}.".format(device.name))
                    break
        if self.input_device is None: raise IOError("FATAL: No valid input device is connected!")

        # Digital button code names
        self.button_codes = {"BTN_SOUTH": "A", "BTN_EAST": "B", "BTN_NORTH": "X", "BTN_WEST": "Y",
                             "BTN_TL": "L", "BTN_TR": "R", "BTN_SELECT": "SL", "BTN_START": "SR",
                             "ABS_HAT0Y": "DV", "ABS_HAT0X": "DH", "BTN_MODE": "K"}

        # Analog input characteristics
        self.max_stick = 32767
        if self.input_device == "Logitech Gamepad F310": self.max_trigger = 255
        else: self.max_trigger = 1023
        self.min_stick = int(self.stick_deadband * self.max_stick)
        self.min_trigger = int(self.trigger_deadband * self.max_trigger)

        # Internals
        self.command = None
        self.joystick_thread = None
        self.stay_alive = False
        self.buffer = deque([])
        self.buffer_size_flag = False

    def get_command(self):
        """
        Executes / clears the input buffer and returns the current relevant Command object.

        """
        if self.joystick_thread is None: raise AssertionError("FATAL: Cannot get_command without active joystick thread!")
        while self.buffer:
            event = self.buffer.pop()
            if event.code == "ABS_Y": self.command.left_vertical = -self._stick_frac(event.state)
            elif event.code == "ABS_X": self.command.roll = self._stick_frac(event.state)
            elif event.code == "ABS_RY": self.command.pitch = -self._stick_frac(event.state)
            elif event.code == "ABS_RX": self.command.yaw = -self._stick_frac(event.state)
            elif event.code == "ABS_Z": self.command.left_trigger = self._trigger_frac(event.state)
            elif event.code == "ABS_RZ": self.command.throttle = self._trigger_frac(event.state)
            elif event.code in self.button_codes:
                self.button_callbacks.get(self.button_codes[event.code], lambda val: None)(event.state)
        return self.command

    def start_joystick_thread(self):
        """
        Starts a thread that reads user input into the internal buffer.

        """
        if self.stay_alive:
            print("----------")
            print("WARNING: Joystick thread already running!")
            print("Cannot start another.")
            print("----------")
            return
        self.command = Command()
        self.stay_alive = True
        if self.input_device in ["Microsoft X-Box One pad (Firmware 2015)",
                                 "PowerA Xbox One wired controller",
                                 "Logitech Gamepad F310"]:
            self.joystick_thread = Thread(target=self._listen_xbox)
        else:
            raise IOError("FATAL: No listener function has been implemented for device {}.".format(self.input_device))
        print("Joystick thread has begun!")
        self.joystick_thread.daemon = True
        self.joystick_thread.start()

    def stop_joystick_thread(self):
        """
        Terminates the joystick's user input reading thread and clears the buffer.

        """
        self.stay_alive = False
        if self.joystick_thread is not None:
            print("Joystick thread terminating on next input!")
            self.joystick_thread.join()
            self.joystick_thread = None
        while self.buffer:
            self.buffer.pop()
        self.buffer_size_flag = False
        self.command = None

    def _listen_xbox(self):
        try:
            while self.stay_alive:
                self.buffer.appendleft(get_gamepad()[0])  # this is blocking (hence need for threading)
                if len(self.buffer) > self.max_buffer_size:
                    if not self.buffer_size_flag:
                        self.buffer_size_flag = True
                        print("----------")
                        print("WARNING: Joystick input buffer reached {} entries.".format(self.max_buffer_size))
                        print("Dropping old commands.")
                        print("----------")
                    self.buffer.pop()
        finally:
            print("Joystick thread terminated!")
            self.joystick_thread = None

    def _stick_frac(self, val):
        if abs(val) > self.min_stick:
            return np.divide(val, self.max_stick, dtype=np.float64)
        return np.float64(0)

    def _trigger_frac(self, val):
        if abs(val) > self.min_trigger:
            return np.divide(val, self.max_trigger, dtype=np.float64)
        return np.float64(0)
