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

"""FlexBE State to command PyRoboSim robot to place up object at current location."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from pyrobosim_msgs.action import ExecuteTaskAction
from pyrobosim_msgs.msg import ExecutionResult, TaskAction

from rclpy.duration import Duration


class PlaceActionState(EventState):
    """
    FlexBE state to place object action for PyRoboSim robot.

    Elements defined here for UI
    Parameters
    -- robot_name          Robot name (default= 'robot')
    -- action_topic        Action topic name (default= '/execute_action')
    -- timeout             Total time to wait for action to complete in seconds (default = 2s)

    Outputs
    <= done                Successfully grabbed object.
    <= failed              Failed to place object

    User data
    #> msg        string   Result message
    """

    def __init__(self, robot_name='robot',
                 action_topic='/execute_action',
                 timeout=2.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=[],
                         output_keys=['msg'])

        self._goal = ExecuteTaskAction.Goal()
        self._goal.action = TaskAction(robot=robot_name,
                                       type='place')

        self._topic = action_topic
        self._timeout = Duration(seconds=timeout)

        self._client = ProxyActionClient({self._topic: ExecuteTaskAction},
                                         wait_duration=0.0)  # pass required clients as dict (topic: type)

        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._start_time = None

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
                    Logger.localinfo(f"'{self}' - successfully placed object")
                    self._return = 'done'
                elif result.status in (ExecutionResult.PRECONDITION_FAILURE,
                                       ExecutionResult.PLANNING_FAILURE,
                                       ExecutionResult.EXECUTION_FAILURE):
                    Logger.logwarn(f"'{self}' : '{self._topic}' - place failure '{result.message}'")
                    self._return = 'failed'
                elif result.status in (ExecutionResult.CANCELED):
                    Logger.logwarn(f"'{self}' : '{self._topic}' - canceled '{result.message}'")
                    self._return = 'failed'
                else:
                    Logger.logwarn(f"'{self}' : '{self._topic}' -"
                                   f" unknown failure ({result.status}) '{result.message}'")
                    self._return = 'failed'
                return self._return
            else:
                Logger.logwarn(f"{self} : '{self._topic}' - invalid action status '{userdata.msg}'")
                self._return = 'failed'
        elif self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # Failed to return call in timely manner
            self._return = 'failed'
            userdata.msg = f"'{self._name}': failed to place object within timeout!"
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
        Logger.localinfo(f"on_enter '{self}' - '{self.path}' ...")
        self._return = None
        self._client.remove_result(self._topic)  # clear any prior result from action server

        # Send the goal.
        try:
            self._client.send_goal(self._topic, self._goal, wait_duration=self._timeout.nanoseconds * 1e-9)
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
            Logger.localinfo('Successfully completed place action.')
        else:
            Logger.localwarn('Failed to complete place action.')
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
