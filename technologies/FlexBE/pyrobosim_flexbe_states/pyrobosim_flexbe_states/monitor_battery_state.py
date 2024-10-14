#!/usr/bin/env python

# Copyright 2024 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""FlexBE State to monitor PyRoboSim robot battery."""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy.qos import QOS_LOSSY

from pyrobosim_msgs.msg import RobotState

from rclpy.duration import Duration
from rclpy.time import Time


class MonitorBatteryState(EventState):
    """
    FlexBE state to monitor PyRoboSim robot battery.

    Elements defined here for UI
    Parameters
    -- low_battery_level   Low level to generate outcome(default=15.0)
    -- high_battery_level  High level to generate outcome (default=105.0)
    -- state_topic         Robot state topic name (default= 'robot/robot_state')
    -- timeout             Max timeout between messages

    Outputs
    <= battery_level       Battery level check
    <= failed              Failed to retrieve state

    User data
    #> battery_level  float   Current battery level
    """

    def __init__(self,
                 low_battery_level=15.0,
                 high_battery_level=105.0,  # Default to being concerned with low-level only
                 state_topic='robot/robot_state',
                 timeout=2.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['battery_level', 'failed'],
                         input_keys=[],
                         output_keys=['battery_level'])

        self._low_battery_level = low_battery_level
        self._high_battery_level = high_battery_level
        self._state_topic = state_topic
        self._timeout = Duration(seconds=timeout)

        self._qos = QOS_LOSSY
        self._connected = False
        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._state_sub = None
        self._battery_level = None
        self._last_msg = None

    def on_start(self):
        """Call when behavior starts."""
        Logger.localinfo(f" on_start  '{self}' - '{self.path}' : subscribe to '{self._state_topic}' ")
        self._connect()

    def on_stop(self):
        """Call when behavior stops."""
        if self._connected:
            # Unsubscribe topic when behavior stops
            Logger.localinfo(f"  '{self}' : unsubscribe from '{self._state_topic}' ")
            ProxySubscriberCached.unsubscribe_topic(self._state_topic)
            self._connected = False
        Logger.localinfo(f" on_stop  '{self}' - '{self.path}'")

    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the data from prior executions
        Logger.localinfo(f" on_enter  '{self}' - '{self.path}' ...")
        self._return = None
        self._battery_level = None
        self._last_msg = None
        if not self._connected:
            self._connect()  # Retry
            if not self._connected:
                Logger.localinfo(f"Failed to connect to '{self._state_topic}' ")
                self._return = 'failed'

        self._state_sub.remove_last_msg(self._state_topic)  # Force us to get a new message
        self._msg_time = self._node.get_clock().now()

    def on_exit(self, userdata):
        """Call when state deactivates."""
        # make sure to reset the data from prior executions
        Logger.localinfo(f" on_exit  '{self}' - '{self.path}' ...")

    def on_pause(self):
        """Execute each time this state is paused."""
        Logger.localinfo(f"on_pause '{self}' - '{self.path}' ...")

    def on_resume(self, userdata):
        """Execute each time this state is resumed."""
        Logger.localinfo(f"on_resume '{self}' - '{self.path}' ...")

    def execute(self, userdata):
        """
        Call periodically while this state is active.

        Check if the action has been finished and evaluate the result.
        """
        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        if self._state_sub.has_msg(self._state_topic):
            last_msg = self._state_sub.get_last_msg(self._state_topic)
            if last_msg is not self._last_msg:
                # Retain last message in proxy for other uses
                self._last_msg = last_msg
                self._battery_level = self._last_msg.battery_level
                self._msg_time = Time(seconds=last_msg.header.stamp.sec,
                                      nanoseconds=last_msg.header.stamp.nanosec,
                                      clock_type=self._node.get_clock().clock_type)  # Assumes messages are consistent!

        elapsed = self._node.get_clock().now() - self._msg_time
        if elapsed > self._timeout:
            self._return = 'failed'

        if self._battery_level is not None:
            if self._battery_level < self._low_battery_level:
                Logger.logwarn(f'Low battery level = {self._battery_level:.2f}')
                self._return = 'battery_level'
            if self._battery_level > self._high_battery_level:
                Logger.logwarn(f'High battery level = {self._battery_level:.2f}')
                self._return = 'battery_level'

        userdata.battery_level = self._battery_level

        # If the action has not yet finished, None outcome will be returned and the state stays active.
        return self._return

    def _connect(self):
        """Connect to publisher."""
        try:
            self._state_sub = ProxySubscriberCached({self._state_topic: RobotState}, qos=self._qos, inst_id=id(self))
            self._connected = True
            return True
        except Exception:  # pylint: disable=W0703
            return False
