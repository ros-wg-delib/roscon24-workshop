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

"""FlexBE State to request local objects for PyRoboSim robot."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from pyrobosim_msgs.action import ExecuteTaskAction
from pyrobosim_msgs.msg import ExecutionResult, TaskAction
from pyrobosim_msgs.srv import RequestWorldState

from rclpy.duration import Duration


class DetectLocalObjectsState(EventState):
    """
    FlexBE state to request local objects for PyRoboSim robot.

    Elements defined here for UI
    Parameters
    -- filter              Item type to look for (default=None (detect all))
    -- action_topic        Action topic name (default= '/execute_action')
    -- server_topic        Server topic to request observation (default='/request_world_state')
    -- robot_name          Name of robot (default='robot')
    -- server_timeout      Wait for action server timeout in seconds (default = 2s)
    -- call_timeout        Wait for service response to request (default=1s)

    Outputs
    <= done                Successfully retrieved items
    <= failed              Failed to plan path

    User data
    #> msg      string     Result message
    #> items    list       Detected items
    """

    def __init__(self, filter=None,
                 action_topic='/execute_action',
                 server_topic='/request_world_state',
                 robot_name='robot',
                 server_timeout=1.0,
                 call_timeout=1.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=['goal'],
                         output_keys=['msg', 'items'])

        self._goal = None

        self._filter = filter
        self._action_topic = action_topic
        self._srv_topic = server_topic
        self._robot_name = robot_name

        self._server_timeout_sec = server_timeout
        self._call_timeout = Duration(seconds=call_timeout)

        self._srv_client = None
        self._act_client = None
        self._srv_start_time = None  # Reset timer for call timeout
        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._service_called = False

    def on_start(self):
        """Execute when behavior starts."""
        # Set up the proxy now, but do not wait on the service just yet
        Logger.localinfo(f" on_start  '{self}' - '{self.path}' ...")
        self._srv_client = ProxyServiceCaller({self._srv_topic: RequestWorldState}, wait_duration=0.0)
        self._act_client = ProxyActionClient({self._action_topic: ExecuteTaskAction},
                                             wait_duration=0.0)  # no need to wait here, we'll check on_enter

    def on_stop(self):
        """Execute when behavior stops."""
        # Remove the proxy client if no longer in use
        ProxyServiceCaller.remove_client(self._srv_topic)
        ProxyActionClient.remove_client(self._action_topic)
        self._srv_client = None
        self._act_client = None
        Logger.localinfo(f" on_stop  '{self}' - '{self.path}' ...")

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

        if self._service_called:
            # Waiting for result from world state
            # We will do this in a non-blocking way
            if self._srv_client.done(self._srv_topic):
                result = self._srv_client.result(self._srv_topic)  # grab empty result, but nothing to check here presume success
                Logger.localinfo(f"Got {type(result)} from service call to '{self._srv_topic} ' ...")
                userdata.items = self._process_srv_response(result)
                Logger.localinfo(f"   returned '{self._return}' with {len(userdata.items) if userdata.items else 0}")
            elif self._node.get_clock().now().nanoseconds - self._srv_start_time.nanoseconds > self._call_timeout.nanoseconds:
                # Failed to return call in timely manner
                self._return = 'failed'
                Logger.logerr(f"'{self._name}': Service '{self._srv_topic}' call timed out!")
            return self._return

        # We have not finished detection yet
        try:
            status = self._act_client.get_status(self._action_topic)  # get status before clearing result
            if self._act_client.has_result(self._action_topic):
                # Check if the action has been finished
                result = self._act_client.get_result(self._action_topic, clear=True)
                Logger.localinfo(f"  '{self}' : '{self._action_topic}' returned {status} {result}")
                if status == GoalStatus.STATUS_SUCCEEDED:
                    result_status = result.execution_result.status
                    userdata.items = None
                    if result_status == ExecutionResult.SUCCESS:
                        if self._srv_client.is_available(self._srv_topic, wait_duration=0.0):
                            # Non-blocking check for availability
                            Logger.localinfo(f"'{self}' - detection completed - now request items from service result")
                            self._do_service_call()
                        elif self._srv_start_time is None:
                            Logger.localinfo(f"'{self}' - detection completed - wait for world state server to become available")
                            self._srv_start_time = self._node.get_clock().now()  # Reset timer for call timeout
                        elif self._node.get_clock().now().nanoseconds - self._srv_start_time.nanoseconds > self._call_timeout.nanoseconds:
                            # Failed to return call in timely manner
                            Logger.logerr(f"'{self._name}': Service '{self._srv_topic}' is not available!")
                            self._return = 'failed'
                        # Otherwise waiting for service to become available

            # Otherwise check for action status change
            if status == GoalStatus.STATUS_CANCELED:
                Logger.loginfo(f" '{self}' : '{self._action_topic}' - detection goal was canceled! ")
                self._return = 'failed'
            elif status == GoalStatus.STATUS_ABORTED:
                Logger.loginfo(f" '{self}' : '{self._action_topic}' -  detection goal was aborted! ")
                self._return = 'failed'
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f"'{self._name}': '{self._action_topic}' exception {type(exc)} - {str(exc)}")
            self._return = 'failed'

        # If the action has not yet finished, None outcome will be returned and the state stays active.
        return self._return

    def _do_service_call(self):
        """Make the service call using async non-blocking."""
        try:
            Logger.localinfo(f"'{self._name}': Calling service '{self._srv_topic}' ...")
            srv_request = RequestWorldState.Request(robot=self._robot_name)
            self._srv_client.call_async(self._srv_topic, srv_request, wait_duration=self._call_timeout.nanoseconds * 1e-9)
            self._service_called = True
        except Exception as exc:
            Logger.logerr(f"'{self._name}': Service '{self._srv_topic}' exception {type(exc)} - {str(exc)}")
            raise exc

    def _process_srv_response(self, response):

        robot = None
        for rbot in response.state.robots:
            if rbot.name == self._robot_name:
                robot = rbot
                break

        if robot is None:
            Logger.logerr(f"Robot '{self._robot_name}' was not found in world state!")
            self._return = 'failed'
            return None

        Logger.localinfo(f"Robot '{self._robot_name}' at location '{robot.last_visited_location}'")

        local_objects = []
        for obj in response.state.objects:
            if obj.parent == robot.last_visited_location:
                if self._filter is None:
                    local_objects.append(obj.name)
                elif obj.name.startswith(self._filter):
                    local_objects.append(obj.name)
        self._return = 'done'
        return tuple(local_objects)

    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the data from prior executions
        Logger.localinfo(f" on_enter  '{self}' - '{self.path}' ...")
        self._srv_start_time = None  # Reset timer for call timeout
        self._return = None
        self._service_called = False
        userdata.items = None
        self._act_client.remove_result(self._action_topic)  # clear any prior result from action server

        # Send the goal.
        try:
            Logger.localinfo(f"Send detection request for '{self._robot_name}' using '{self._filter}' ")
            goal = ExecuteTaskAction.Goal()
            if self._filter is None:
                goal.action = TaskAction(robot=self._robot_name,
                                         type='detect',
                                         )
            else:
                goal.action = TaskAction(robot=self._robot_name,
                                         type='detect',
                                         object=self._filter
                                         )
            self._act_client.send_goal(self._action_topic, goal, wait_duration=self._server_timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            # Since a state failure not necessarily causes a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn(f"Failed to send the '{self}' command:\n  {type(exc)} - {exc}")
            self._return = 'failed'

    def on_exit(self, userdata):
        """Call when state is deactivated."""
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if self._act_client.is_active(self._action_topic):
            self._act_client.cancel(self._action_topic)
            # Check for action status change
            status = self._act_client.get_status(self._action_topic)  # get status before clearing result
            if status == GoalStatus.STATUS_CANCELED:
                Logger.loginfo(f" '{self}' : '{self._action_topic}' - request to plan was canceled! ")
                self._return = 'failed'
            else:
                status_string = self._act_client.get_status_string(self._action_topic)
                Logger.loginfo(f" '{self}' : '{self._action_topic}' - Requested to cancel "
                               f"an active plan request ('{status_string}').")

        # Local message are shown in terminal but not the UI
        if self._return == 'done':
            Logger.localinfo('Successfully retrieved detected items.')
        else:
            Logger.localwarn('Failed to detect items.')
        Logger.localinfo(f" on_exit  '{self}' - '{self.path}' ...")
