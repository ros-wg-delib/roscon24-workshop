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

"""Check pyrobosim door FlexBE state."""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from pyrobosim_msgs.srv import RequestWorldState

from rclpy.duration import Duration


class CheckDoorState(EventState):
    """
    This state checks the status of given "openable" in pyrobosim.

    Elements defined here for UI
    Parameters
    -- call_timeout  float      Timeout for completion (default: 3.0 seconds)
    -- wait_timeout  float      Duration to wait for service to become available (default: 3.0 seconds)
    -- service_name  string     Service name (default: `request_world_state`)
    -- robot         string     Robot name (default: '' request world state)

    Outputs
    <= open             Service call returned result as expected
    <= closed           Failed to make service call successfully
    <= failed           Service call did not return timely result

    User data
    ># name     string  Name of openable (e.g. 'hall_room1_room2', 'fridge_storage')
    #> msg      string  Output message
    """

    def __init__(self, call_timeout=3.0, wait_timeout=3.0, service_name='request_world_state', robot=''):
        """Declare outcomes, input_keys, and output_keys by calling the EventState super constructor."""
        super().__init__(outcomes=['open', 'closed', 'failed'],
                         input_keys=['name'],
                         output_keys=['msg'])

        # No longer needed in 4.0+ ProxyServiceCaller.initialize(CheckDoorState._node)

        # Store state parameters for later use.
        self._call_timeout = Duration(seconds=call_timeout)
        self._wait_timeout = Duration(seconds=wait_timeout)

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None
        self._return = None  # Track the outcome so we can detect if transition is blocked
        self._service_called = False

        self._srv_topic = service_name
        self._srv_result = None

        self._srv_request = RequestWorldState.Request()
        self._srv_request.robot = robot

        self._error = None
        self._srv = None

    def on_start(self):
        """Execute when behavior starts."""
        # Set up the proxy now, but do not wait on the service just yet
        Logger.localinfo(f" on_start  '{self}' - '{self.path}' ...")
        self._srv = ProxyServiceCaller({self._srv_topic: RequestWorldState}, wait_duration=0.0)

    def on_stop(self):
        """Execute when behavior stops."""
        # Remove the proxy client if no longer in use
        ProxyServiceCaller.remove_client(self._srv_topic)
        self._srv = None
        Logger.localinfo(f" on_stop  '{self}' - '{self.path}' ...")

    def on_pause(self):
        """Execute each time this state is paused."""
        Logger.localinfo(f"on_pause '{self}' - '{self.path}' ...")

    def on_resume(self, userdata):
        """Execute each time this state is resumed."""
        Logger.localinfo(f"on_resume '{self}' - '{self.path}' ...")

    def execute(self, userdata):
        """
        Execute this method periodically while the state is active.

        If no outcome is returned, the state will stay active.
        """
        if self._return:
            # We have completed the state, and therefore must be blocked by autonomy level
            return self._return

        if self._service_called:
            # Waiting for result.
            # We will do this in a non-blocking way
            if self._srv.done(self._srv_topic):
                result = self._srv.result(self._srv_topic)  # grab result
                self._process_result(result, userdata.name)
                if self._return == 'failed':
                    userdata.msg = f"{self._name}: world state did not find '{userdata.name}'!"
                return self._return
            else:
                elapsed = self._node.get_clock().now() - self._start_time
                if elapsed > self._call_timeout:
                    # Failed to return call in timely manner
                    self._return = 'failed'
                    userdata.msg = f"'{self._name}': Service '{self._srv_topic}' call timed out!"
        else:
            # Waiting for service to become available in non-blocking manner
            if self._srv.is_available(self._srv_topic, wait_duration=0.0):
                Logger.localinfo(f"'{self._name}': Service '{self._srv_topic}' is now available - making service call!")
                self._do_service_call()
                # Process the result on next execute call (so some delay)
            else:
                elapsed = self._node.get_clock().now() - self._start_time
                if elapsed > self._wait_timeout:
                    # Failed to return call in timely manner
                    self._return = 'failed'
                    userdata.msg = f"'{self._name}': Service '{self._srv_topic}' is unavailable!"

        return self._return

    def on_enter(self, userdata):
        """
        Call this method when the state becomes active.

        i.e. a transition from another state to this one is taken.
        """
        Logger.localinfo(f" on_enter  '{self}' - '{self.path}' ...")
        if 'name' in userdata and isinstance(userdata.name, str):
            userdata.msg = ''
            self._start_time = self._node.get_clock().now()
            self._return = None  # reset the completion flag
            self._service_called = False
            try:
                #                                          Don't block just yet
                if self._srv.is_available(self._srv_topic, wait_duration=0.0):
                    self._do_service_call()
                else:
                    Logger.logwarn(f"'{self._name}': Service '{self._srv_topic}' is not yet available ...")
            except Exception as exc:  # pylint: disable=W0703
                Logger.logerr(f"'{self._name}': Service '{self._srv_topic}' exception {type(exc)} - {str(exc)}")
                self._return = 'failed'
        else:
            userdata.msg = f"'{self}' - userdata must contain 'name' field!"
            self._return = 'failed'
            return

    def on_exit(self, userdata):
        """Call when state deactivates."""
        # make sure to reset the data from prior executions
        Logger.localinfo(f" on_exit  '{self}' - '{self.path}' ...")

    def _do_service_call(self):
        """Make the service call using async non-blocking."""
        try:
            Logger.localinfo(f"'{self._name}': Calling service '{self._srv_topic}' ...")
            self._srv_result = self._srv.call_async(self._srv_topic, self._srv_request, wait_duration=0.0)
            self._start_time = self._node.get_clock().now()  # Reset timer for call timeout
            self._service_called = True
        except Exception as exc:
            Logger.logerr(f"'{self._name}': Service '{self._srv_topic}' exception {type(exc)} - {str(exc)}")
            raise exc

    def _process_result(self, result, name):
        """Look for name in result."""
        world_state = result.state
        if name.startswith('hall_'):
            # This is a door between two rooms
            possible_rooms = tuple(name[len('hall_'):].split('_'))
            alternate_name = f'hall_{possible_rooms[1]}_{possible_rooms[0]}'
            # Check if hallway door
            for loc in world_state.hallways:
                if loc.name == name or loc.name == alternate_name:
                    if loc.is_open:
                        self._return = 'open'
                    else:
                        self._return = 'closed'
                    return

        endings = ['_dock', '_tabletop', '_disposal', '_storage']
        location = next((name.replace(ending, '') for ending in endings if name.endswith(ending)), name)

        for loc in world_state.locations:
            if loc.name == location:
                if loc.is_open:
                    self._return = 'open'
                else:
                    self._return = 'closed'
                return

        Logger.localinfo(f"'{name}' not found at location '{location}' or hallway!")
        self._return = 'failed'
