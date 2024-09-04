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

"""FlexBE State to command PyRoboSim robot to open or close a door at current location."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.core import StateError
from flexbe_core.proxy import ProxyActionClient

from pyrobosim_msgs.action import ExecuteTaskAction
from pyrobosim_msgs.msg import ExecutionResult, TaskAction

from rclpy.duration import Duration


class DoorActionState(EventState):
    """
    FlexBE state to perform open/close action for PyRoboSim robot at current location.

    Elements defined here for UI
    Parameters
    -- action_type         Action either 'open' or 'close'
    -- robot_name          Robot name (default= 'robot')
    -- action_topic        Action topic name (default= '/execute_action')
    -- timeout             Total time to wait for action to complete in seconds (default = 2s)

    Outputs
    <= done                Successfull completion
    <= failed              Failed to perform action

    User data
    #> msg        string   Result message
    """

    def __init__(self,
                 action_type='open',
                 robot_name='robot',
                 action_topic='/execute_action',
                 timeout=2.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=[],
                         output_keys=['msg'])

        if action_type not in ('open', 'close'):
            # Fail on behavior build if not valid action
            msg = f"State '{self}' : Invalid action type '{action_type}' - must be 'open' or 'close'"
            Logger.logerr(msg)
            raise StateError(msg)

        self._action_type = action_type
        self._action_outcome = {'open': 'opened', 'close': 'closed'}[action_type]  # grammar

        self._goal = ExecuteTaskAction.Goal()
        self._goal.action = TaskAction(robot=robot_name, type=action_type)

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
                    Logger.localinfo(f"{self} - successfully {self._action_outcome}!")
                    self._return = 'done'
                elif result.status in (ExecutionResult.PRECONDITION_FAILURE,
                                       ExecutionResult.PLANNING_FAILURE,
                                       ExecutionResult.EXECUTION_FAILURE):
                    Logger.logwarn(f"{self} : '{self._topic}' - failed to '{self._action_type}' command - '{result.message}'")
                    self._return = 'failed'
                elif result.status in (ExecutionResult.CANCELED):
                    Logger.logwarn(f"{self} : '{self._topic}' - canceled '{self._action_type}' command -'{result.message}'")
                    self._return = 'failed'
                else:
                    Logger.logwarn(f"{self} : '{self._topic}' -"
                                   f" unknown failure to '{self._action_type}' command - ({result.status}) '{result.message}'")
                    self._return = 'failed'
                return self._return
            else:
                Logger.logwarn(f"{self} : '{self._topic}' - command invalid action status to '{self._action_type}' '{result.message}'")
                self._return = 'failed'
        elif self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # Failed to return call in timely manner
            self._return = 'failed'
            userdata.msg = f"{self._name}: failed to '{self._action_type}' within timeout!"
            Logger.localwarn(userdata.msg)

        # Otherwise check for status change
        if status == GoalStatus.STATUS_CANCELED:
            Logger.loginfo(f" '{self}' : '{self._topic}' - goal was canceled! ")
            userdata.msg = f"Request to '{self._action_type}' was canceled."
            self._return = 'failed'
        elif status == GoalStatus.STATUS_ABORTED:
            Logger.loginfo(f" '{self}' : '{self._topic}' -  goal was aborted! ")
            userdata.msg = f"Request to '{self._action_type}' was aborted."
            self._return = 'failed'

        # If the action has not yet finished, None outcome will be returned and the state stays active.
        return self._return


    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the error state since a previous state execution might have failed
        self._return = None
        self._client.remove_result(self._topic)  # clear any prior result from action server

        # Send the goal.
        try:
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
