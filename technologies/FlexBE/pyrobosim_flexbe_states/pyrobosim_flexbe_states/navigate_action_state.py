#!/usr/bin/env python

# Copyright 2024 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""FlexBE State to navigate PyRoboSim robot to target location."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from pyrobosim_msgs.action import ExecuteTaskAction
from pyrobosim_msgs.msg import ExecutionResult, TaskAction

from rclpy.duration import Duration


class NavigateActionState(EventState):
    """
    FlexBE state to navigate PyRoboSim robot to target location.

    Elements defined here for UI
    Parameters
    -- target_location     Location to navigate to
    -- robot_name          Robot name (default= 'robot')
    -- action_topic        Action topic name (default= '/execute_action')
    -- server_timeout      Wait for server timeout in seconds (default = 5s)
    -- navigate_timeout    Timeout for action completion (default=None)
    Outputs
    <= done                Navigation is complete
    <= planning_failed     Failed to plan path.
    <= motion_failed       Failed for some reason.
    <= canceled            User canceled before completion.
    <= timeout             Failed to navigate in timely manner

    #> msg                 Output message
    """

    def __init__(self, target_location, robot_name='robot',
                 action_topic='/execute_action', server_timeout=5.0,
                 navigate_timeout=None):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'planning_failed', 'motion_failed', 'canceled', 'timeout'],
                         input_keys=[],
                         output_keys=['msg'])

        self._goal = ExecuteTaskAction.Goal()
        self._goal.action = TaskAction(robot=robot_name,
                                       type='navigate',
                                       target_location=target_location,
                                       )

        self._timeout = None
        if navigate_timeout is not None:
            self._timeout = Duration(seconds=navigate_timeout)
        self._server_timeout_sec = float(server_timeout)
        self._topic = action_topic

        self._client = ProxyActionClient({self._topic: ExecuteTaskAction},
                                         wait_duration=0.0)  # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._start_time = None
        self._elapsed = None

    def execute(self, userdata):
        """
        Call periodically while this state is active.

        Check if the action has been finished and evaluate the result.
        """
        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        status = self._client.get_status(self._topic)  # get status before clearing result
        if self._client.has_result(self._topic):
            # Check if the action has been finished
            result = self._client.get_result(self._topic, clear=True)
            Logger.localinfo(f"  '{self}' : '{self._topic}' returned {status} {result}")
            if status == GoalStatus.STATUS_SUCCEEDED:
                result = result.execution_result
                self._elapsed = (self._node.get_clock().now() - self._start_time).nanoseconds * 1e-9
                userdata.msg = result.message

                if result.status == ExecutionResult.SUCCESS:
                    self._return = 'done'
                elif result.status in (ExecutionResult.PRECONDITION_FAILURE,
                                       ExecutionResult.PLANNING_FAILURE):
                    Logger.logwarn(f"{self} : '{self._topic}' - planning failure '{result.message}'")
                    self._return = 'planning_failed'
                elif result.status in (ExecutionResult.EXECUTION_FAILURE):
                    Logger.logwarn(f"{self} : '{self._topic}' - execution failure '{result.message}'")
                    self._return = 'motion_failed'
                elif result.status in (ExecutionResult.CANCELED):
                    Logger.logwarn(f"{self} : '{self._topic}' - canceled '{result.message}'")
                    self._return = 'canceled'
                else:
                    Logger.logwarn(f"{self} : '{self._topic}' -"
                                   f" unknown failure ({result.status}) '{result.message}'")
                    self._return = 'motion_failed'
                return self._return

        # Otherwise check for status change
        if status == GoalStatus.STATUS_CANCELED:
            Logger.loginfo(f" '{self}' : '{self._topic}' - goal was canceled! ")
            self._return = 'canceled'
        elif status == GoalStatus.STATUS_ABORTED:
            Logger.loginfo(f" '{self}' : '{self._topic}' -  goal was aborted! ")
            self._return = 'canceled'
        elif self._timeout is not None:
            if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
                # Checking for timeout after we check for goal response
                Logger.loginfo(f" '{self}' : '{self._topic}' - navigation timed out! ")
                self._return = 'timeout'

        # If the action has not yet finished, None outcome will be returned and the state stays active.
        return self._return

    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the return state since a previous state execution might have failed
        Logger.localinfo(f"on_enter '{self}' - '{self.path}' ...")
        self._return = None
        self._client.remove_result(self._topic)  # clear any prior result from action server

        # Recording the start time to set motion duration output
        self._start_time = self._node.get_clock().now()

        # Send the goal.
        try:
            Logger.loginfo(f"Request to navigate to '{self._goal.action.target_location}'")
            self._client.send_goal(self._topic, self._goal, wait_duration=self._server_timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            # Since a state failure does not necessarily cause a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log
            # enables the operator to collapse details in the GUI.
            Logger.logwarn(f"Failed to send the '{self}' command:\n  {type(exc)} - {exc}")
            self._return = 'planning_failed'

    def on_exit(self, userdata):
        """Call when state is deactivated."""
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if self._client.is_active(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo(f" '{self}' : '{self._topic}' -Cancelled active action goal.")

        if self._return == 'done':
            Logger.loginfo(f'Successfully completed motion in {self._elapsed:.3f} seconds.')
        else:
            Logger.logwarn('Failed to complete motion.')
        Logger.localinfo(f"on_exit '{self}' - '{self.path}' ...")

        # Choosing to remove in on_enter and retain in proxy for now
        # Either choice can be valid.
        # if self._client.has_result(self._topic):
        #     # remove the old result so we are ready for the next time
        #     # and don't prematurely return
        #     self._client.remove_result(self._topic)

    def on_pause(self):
        """Execute each time this state is paused."""
        Logger.localinfo(f"on_pause '{self}' - '{self.path}' ...")

    def on_resume(self, userdata):
        """Execute each time this state is resumed."""
        Logger.localinfo(f"on_resume '{self}' - '{self.path}' ...")

    def on_start(self):
        """Call when behavior starts."""
        Logger.localinfo(f" on_start  '{self}' - '{self.path}' ")

    def on_stop(self):
        """Call when behavior stops."""
        Logger.localinfo(f" on_stop  '{self}' - '{self.path}'")
