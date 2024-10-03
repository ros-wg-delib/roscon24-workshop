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

"""FlexBE State to command PyRoboSim robot to detect objects at the current location."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from pyrobosim_msgs.action import DetectObjects
from pyrobosim_msgs.msg import ExecutionResult, TaskAction

from rclpy.duration import Duration


class DetectObjectsState(EventState):
    """
    FlexBE state to request local objects for PyRoboSim robot.

    Elements defined here for UI
    Parameters
    -- filter              Item type to look for (default=None (detect all))
    -- action_topic        Action topic name (default= 'robot/detect_objects')
    -- timeout             Total time for action server to complete in seconds (default = 2s)

    Outputs
    <= done                Successfully retrieved 1 or more items
    <= nothing             No objects were detected
    <= failed              Action failed

    User data
    #> msg      string     Result message
    #> items    list       Detected items
    """

    def __init__(self, filter=None,
                 action_topic='robot/detect_objects',
                 timeout=1.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed', 'nothing'],
                         input_keys=['goal'],
                         output_keys=['msg', 'items'])

        self._goal = None

        self._filter = filter
        self._action_topic = action_topic

        self._timeout = Duration(seconds=timeout)

        self._act_client = None
        self._start_time = None  # Reset timer for call timeout
        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._service_called = False

    def on_start(self):
        """Call when behavior starts."""
        # Set up the proxy now, but do not wait on the service just yet
        Logger.localinfo(f" on_start  '{self}' - '{self.path}' ...")
        self._act_client = ProxyActionClient({self._action_topic: DetectObjects},
                                             wait_duration=0.0)  # no need to wait here, we'll check on_enter

    def on_stop(self):
        """Call when behavior stops."""
        # Remove the proxy client if no longer in use
        ProxyActionClient.remove_client(self._action_topic)
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

        # We have not finished detection yet
        try:
            status = self._act_client.get_status(self._action_topic)  # get status before clearing result
            if self._act_client.has_result(self._action_topic):
                # Check if the action has been finished
                result = self._act_client.get_result(self._action_topic, clear=True)
                if status == GoalStatus.STATUS_SUCCEEDED:
                    result_status = result.execution_result.status
                    userdata.msg = result.execution_result.message
                    if result_status == ExecutionResult.SUCCESS:
                        Logger.localinfo(f"  '{self}' : '{self._action_topic}' "
                                         f'detected {len(result.detected_objects)} objects')
                        if len(result.detected_objects) == 0:
                            userdata.items = None
                            userdata.msg = f"'{self}': found no objects at location!"
                            self._return = 'nothing'
                        else:
                            objects = result.detected_objects
                            names = []
                            for obj in objects:
                                names.append(obj.name)

                            userdata.items = names
                            self._return = 'done'
                    else:
                        Logger.localwarn(f"'{self}' : '{self._action_topic}' -"
                                         f" failed to detect objects ({result_status}) '{userdata.msg}'")
                        userdata.items = None
                        self._return = 'failed'
            elif self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
                # Failed to return call in timely manner
                self._return = 'failed'
                userdata.msg = f"'{self}': failed to get objects within timeout!"
                Logger.localwarn(userdata.msg)

            # Otherwise check for action status change
            if status == GoalStatus.STATUS_CANCELED:
                Logger.loginfo(f" '{self}' : '{self._action_topic}' - detection goal was canceled! ")
                self._return = 'failed'
            elif status == GoalStatus.STATUS_ABORTED:
                Logger.loginfo(f" '{self}' : '{self._action_topic}' -  detection goal was aborted! ")
                self._return = 'failed'
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f"'{self}': {self._action_topic} exception {type(exc)} - {str(exc)}")
            self._return = 'failed'

        # If the action has not yet finished, None outcome will be returned and the state stays active.
        return self._return

    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the data from prior executions
        Logger.localinfo(f" on_enter  '{self}' - '{self.path}' ...")
        self._return = None
        userdata.items = None
        self._act_client.remove_result(self._action_topic)  # clear any prior result from action server
        self._start_time = self._node.get_clock().now()

        # Send the goal.
        try:
            Logger.localinfo(f"Send detection request to '{self._action_topic}' using '{self._filter}' ")
            if self._filter is None:
                goal = DetectObjects.Goal()
            else:
                goal = DetectObjects.Goal(target_object=self._filter)
            self._act_client.send_goal(self._action_topic, goal, wait_duration=self._timeout.nanoseconds * 1e-9)
        except Exception as exc:  # pylint: disable=W0703
            # Since a state failure not necessarily causes a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn(f"Failed to send the '{self}' command:\n  {type(exc)} - {exc}")
            self._return = 'failed'

    def on_exit(self, userdata):
        """Call when state is deactivated."""
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when
        # the operator manually triggers an outcome.

        if self._act_client.is_active(self._action_topic):
            self._act_client.cancel(self._action_topic)
            # Check for action status change
            status = self._act_client.get_status(self._action_topic)  # get status before clearing result
            if status == GoalStatus.STATUS_CANCELED:
                # log messages are shown on the UI and terminal (keep to minimum for normal operation)
                Logger.loginfo(f" '{self}' : '{self._action_topic}' - request for objects was canceled! ")
                self._return = 'failed'
            else:
                status_string = self._act_client.get_status_string(self._action_topic)
                Logger.loginfo(f" '{self}' : '{self._action_topic}' - Requested to cancel "
                               f'an active detection request ({status_string}).')

        # Local message are shown in terminal but not the UI
        # Generally avoid doing this except when debugging
        if self._return == 'done':
            Logger.localinfo('Successfully retrieved detected items.')
        else:
            Logger.localwarn('Failed to detect items.')
        Logger.localinfo(f" on_exit  '{self}' - '{self.path}' ...")
