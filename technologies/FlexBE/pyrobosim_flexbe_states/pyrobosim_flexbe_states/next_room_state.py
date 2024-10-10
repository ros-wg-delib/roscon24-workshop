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

"""FlexBE State to determine the next room along the way to goal in PyRoboSim world."""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy.qos import QOS_LOSSY

from geometry_msgs.msg import Pose

from pyrobosim_flexbe_utilities.world_configuration import WorldConfiguration

from pyrobosim_msgs.msg import RobotState

from rclpy.duration import Duration
from rclpy.time import Time


class NextRoomState(EventState):
    """
    FlexBE state to determine next room to move given pyrobosim world definition.

    Elements defined here for UI
    Parameters
    -- world_definition          world definition (default='world4')
    -- package_name        package name (default='delib_ws_worlds')
    -- sub_folder          Subfolder for world definition(default='worlds')
    -- state_topic         Robot state topic name (default= 'robot/robot_state')
    -- timeout             Max timeout between messages

    Outputs
    <= done                Next room goal is set
    <= failed              Failed to retrieve goal

    User data
    ># goal           string  where we want to go overall
    #> room           string  where we need to go next
    #> hallway        string  connecting hallway (door)
    """

    def __init__(self,
                 world='world4',
                 package_name='delib_ws_worlds',
                 sub_folder='worlds',
                 state_topic='robot/robot_state',
                 timeout=2.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=['goal'],
                         output_keys=['room', 'hallway'])

        self._state_topic = state_topic
        self._timeout = Duration(seconds=timeout)
        self._qos = QOS_LOSSY
        self._connected = False
        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._state_sub = None
        self._last_msg = None
        self._last_visited = None
        self._last_pose = None

        self._world = world

        # Load structure of world as needed
        self._world_config = WorldConfiguration(world, package_name, sub_folder)

    def on_start(self):
        """Call when behavior starts."""
        Logger.localinfo(f"on_start '{self}' - '{self.path}' : subscribe to '{self._state_topic}' ")
        self._connect()

    def on_stop(self):
        """Call when behavior stops."""
        if self._connected:
            # Unsubscribe topic when behavior stops
            Logger.localinfo(f"  '{self}' : unsubscribe from '{self._state_topic}' ")
            ProxySubscriberCached.unsubscribe_topic(self._state_topic)
            self._connected = False
        Logger.localinfo(f"on_stop '{self}' - '{self.path}' ...")

    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the data from prior executions
        Logger.localinfo(f"on_enter '{self}' - '{self.path}' ...")

        self._return = None
        self._last_msg = None
        self._last_visited = None
        self._last_pose = None

        if not self._connected:
            self._connect()  # Retry
            if not self._connected:
                Logger.localinfo(f"Failed to connect to '{self._state_topic}' ")
                self._return = 'failed'

        self._state_sub.remove_last_msg(self._state_topic)  # Force us to get a new message
        self._msg_time = self._node.get_clock().now()

    def on_exit(self, userdata):
        """Execute when state is deactivated."""
        Logger.localinfo(f"on_exit '{self}' - '{self.path}' ...")

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
            self._last_visited = last_msg.last_visited_location
            self._last_pose = last_msg.pose
            self._msg_time = Time(seconds=last_msg.header.stamp.sec,
                                  nanoseconds=last_msg.header.stamp.nanosec,
                                  clock_type=self._node.get_clock().clock_type)  # Assumes messages are consistent!

        elapsed = self._node.get_clock().now() - self._msg_time
        if elapsed > self._timeout:
            Logger.logerr('Failed to get robot state message for last visited')
            self._return = 'failed'
            return self._return

        if self._last_visited is None:
            return None

        try:
            # For now hope we get lucky choosing index [0] if multiple rooms returned
            target_room = self._world_config.get_rooms(userdata.goal, self._world)[0]
            current_room = self._world_config.get_rooms(self._last_visited, self._world, self._last_pose)[0]

            path = self._world_config.search(current_room, target_room, self._world)
            if path is not None and len(path) > 1:
                userdata.room = path[1]  # next room ([0] is current)
                userdata.hallway = self._world_config.get_hallway(current_room, path[1], self._world)
                Logger.loginfo(f" travel '{current_room}' to '{userdata.room}' via '{userdata.hallway}'")
                if userdata.room == 'dining':
                    userdata.room = Pose()
                    userdata.room.position.x = -1.0  # a valid pose to left of dining table
                    Logger.localinfo(f'Hack: Dining is not reachable so replace with reachable pose {userdata.room}!')
                self._return = 'done'
            else:
                Logger.logerr(f"Failed to find valid path from '{current_room}' to '{target_room}'")
                self._return = 'failed'

        except Exception as exc:
            Logger.logerr(f'Error processing room request:\n    {exc}')
            self._return = 'failed'

        # If the action has not yet finished, None outcome will be returned and the state stays active
        # while we wait on another robot state message.
        return self._return

    def _connect(self):
        """Connect to publisher."""
        try:
            self._state_sub = ProxySubscriberCached({self._state_topic: RobotState}, qos=self._qos, inst_id=id(self))
            self._connected = True
            return True
        except Exception:  # pylint: disable=W0703
            return False
