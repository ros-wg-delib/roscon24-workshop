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

"""FlexBE State to command PyRoboSim robot to pick up object at current location."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from pyrobosim_msgs.action import ExecuteTaskAction
from pyrobosim_msgs.msg import ExecutionResult, TaskAction

from rclpy.duration import Duration


class PickActionState(EventState):
    """
    FlexBE state to pick action for PyRoboSim robot

    Elements defined here for UI
    Parameters
    -- robot_name          Robot name (default= 'robot')
    -- action_topic        Action topic name (default= '/execute_action')
    -- timeout             Total time to wait for action to complete in seconds (default = 2s)

    Outputs
    <= done                Successfully grabbed object.
    <= failed              Failed to pick object

    User data
    ># object     string   Desired object to pick up
    #> msg        string   Result message
    """

    def __init__(self, robot_name="robot",
                 action_topic='/execute_action',
                 timeout=2.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=['object'],
                         output_keys=['msg'])

        self._goal = ExecuteTaskAction.Goal()
        self._goal.action = TaskAction(robot=robot_name, type="pick")

        self._topic = action_topic
        self._timeout = Duration(seconds=timeout)

        self._client = ProxyActionClient({self._topic: ExecuteTaskAction},
                                         wait_duration=0.0)  # pass required clients as dict (topic: type)

        self._start_time = None
        self._return = None  # Retain return value in case the outcome is blocked by operator

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
            result = self._client.get_result(self._topic, clear=True).execution_result
            userdata.msg = result.message  # Output message
            if status == GoalStatus.STATUS_SUCCEEDED:
                if result.status == ExecutionResult.SUCCESS:
                    Logger.localinfo(f"{self} - successfully picked '{userdata.object}' ")
                    self._return = 'done'
                elif result.status in (ExecutionResult.PRECONDITION_FAILURE,
                                       ExecutionResult.PLANNING_FAILURE,
                                       ExecutionResult.EXECUTION_FAILURE):
                    Logger.logwarn(f"{self} : '{self._topic}' - pick failure '{result.message}'")
                    self._return = 'failed'
                elif result.status in (ExecutionResult.CANCELED):
                    Logger.logwarn(f"{self} : '{self._topic}' - canceled '{result.message}'")
                    self._return = 'failed'
                else:
                    Logger.logwarn(f"{self} : '{self._topic}' -"
                                   f" unknown failure ({result.status}) '{result.message}'")
                    self._return = 'failed'
                return self._return
            else:
                Logger.logwarn(f"{self} : '{self._topic}' - invalid action status '{result.message}'")
                self._return = 'failed'
        elif self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # Failed to return call in timely manner
            self._return = 'failed'
            userdata.msg = f"{self._name}: failed to pick object within timeout!"
            Logger.localwarn(userdata.msg)

        # Otherwise check for status change
        if status == GoalStatus.STATUS_CANCELED:
            Logger.loginfo(f" '{self}' : '{self._topic}' - goal was canceled! ")
            self._return = 'failed'
        elif status == GoalStatus.STATUS_ABORTED:
            Logger.loginfo(f" '{self}' : '{self._topic}' -  goal was aborted! ")
            self._return = 'failed'

        # If the action has not yet finished, None outcome will be returned and the state stays active.
        return self._return


    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the error state since a previous state execution might have failed
        self._return = None
        self._client.remove_result(self._topic)  # clear any prior result from action server

        if 'object' not in userdata:
            self._return = 'failed'
            userdata.msg = f"PickActionState '{self}' requires userdata.object key!"
            Logger.logwarn(userdata.msg)
            return

        # Send the goal.
        try:
            self._goal.action.object = userdata.object
            self._client.send_goal(self._topic, self._goal, wait_duration=self._timeout.nanoseconds*1e-9)
            self._start_time = self._node.get_clock().now()
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

        if self._client.is_active(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo(f" '{self}' : '{self._topic}' -Cancelled active action goal.")

        # Local message are shown in terminal but not the UI
        if self._return == 'done':
            Logger.localinfo(f'Successfully completed pick action.')
        else:
            Logger.localwarn('Failed to complete pick action.')

        # Choosing to remove in on_enter and retain in proxy for now
        # Either choice can be valid.
        # if self._client.has_result(self._topic):
        #     # remove the old result so we are ready for the next time
        #     # and don't prematurely return
        #     self._client.remove_result(self._topic)
