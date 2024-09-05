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

"""FlexBE State to command PyRoboSim robot to plan a path to target location."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from geometry_msgs.msg import Pose

from pyrobosim_msgs.action import PlanPath
from pyrobosim_msgs.msg import ExecutionResult, Path

from rclpy.duration import Duration


class PlanPathState(EventState):
    """
    FlexBE state to request plan for PyRoboSim robot

    Elements defined here for UI
    Parameters
    -- action_topic     Action topic name (default= 'robot/plan_path')
    -- timeout          Total time to wait for plan in seconds (default = 5s)

    Outputs
    <= done                Successfully planned path to goal.
    <= failed              Failed to plan path

    User data
    ># goal       string/Pose   Desired object to pick up
    #> msg        string        Result message
    #> path       Path          Planned Path
    """

    def __init__(self, action_topic='robot/plan_path',
                 timeout=2.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=['goal'],
                         output_keys=['msg', 'path'])

        self._goal = None

        self._topic = action_topic
        self._planning_timeout = Duration(seconds=timeout)

        self._client = ProxyActionClient({self._topic: PlanPath},
                                         wait_duration=0.0)  # no need to wait here, we'll check on_enter

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
            result = self._client.get_result(self._topic, clear=True)
            userdata.msg = result.execution_result.message  # Output message
            if status == GoalStatus.STATUS_SUCCEEDED:
                result_status = result.execution_result.status
                userdata.path = None
                if result_status == ExecutionResult.SUCCESS and result.path is not None:
                    Logger.localinfo(f"{self} - planning success with {len(result.path.poses)} waypoints")
                    userdata.path = result.path
                    self._return = 'done'
                else:
                    Logger.localwarn(f"{self} : '{self._topic}' -"
                                   f" planning failure ({result_status}) '{userdata.msg}'")
                    self._return = 'failed'
                return self._return
            else:
                Logger.logwarn(f"{self} : '{self._topic}' - invalid action status '{userdata.msg}'")
                self._return = 'failed'
        elif self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._planning_timeout.nanoseconds:
            # Failed to return call in timely manner
            self._return = 'failed'
            userdata.msg = f"{self._name}: failed to plan path within timeout!"
            Logger.localwarn(userdata.msg)

        # Otherwise check for action status change
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
        # make sure to reset the data from prior executions
        self._return = None
        userdata.path = None
        userdata.msg = ''
        self._client.remove_result(self._topic)  # clear any prior result from action server
        if 'goal' not in userdata:
            self._return = 'failed'
            userdata.msg = f"PlanActionState '{self}' requires userdata.goal key!"
            Logger.localwarn(userdata.msg)
            return

        # Send the goal.
        try:
            goal = userdata.goal
            if isinstance(goal, str):
                self._goal = PlanPath.Goal(target_location = goal)
            elif isinstance(goal, Pose):
                self._goal = PlanPath.Goal(pose=goal)
            else:
                userdata.msg = f"Invalid goal type '{type(goal)}' - must be string or Pose!"
                Logger.localwarn(userdata.msg)
                self._return = 'failed'
                return
            self._client.send_goal(self._topic, self._goal, wait_duration=self._planning_timeout.nanoseconds*1e-9)
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
            # Check for action status change
            status = self._client.get_status(self._topic)  # get status before clearing result
            if status == GoalStatus.STATUS_CANCELED:
                Logger.loginfo(f" '{self}' : '{self._topic}' - request to plan was canceled! ")
                self._return = 'failed'
            else:
                status_string = self._client.get_status_string(self._topic)
                Logger.loginfo(f" '{self}' : '{self._topic}' - Requested to cancel an active plan request ({status_string}).")

        # Local message are shown in terminal but not the UI
        if self._return == 'done':
            Logger.localinfo(f'Successfully planned path.')
        else:
            Logger.localwarn('Failed to plan path.')

        # Choosing to remove in on_enter and retain in proxy for now
        # Either choice can be valid.
        # if self._client.has_result(self._topic):
        #     # remove the old result so we are ready for the next time
        #     # and don't prematurely return
        #     self._client.remove_result(self._topic)
