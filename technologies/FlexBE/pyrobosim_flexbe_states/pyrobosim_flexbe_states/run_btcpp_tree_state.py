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

"""FlexBE State to command PyRoboSim robot to plan a path to target location."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from geometry_msgs.msg import Pose

from btcpp_ros2_interfaces.action import ExecuteTree
from btcpp_ros2_interfaces.msg import NodeStatus

from rclpy.duration import Duration


class RunBtCppState(EventState):
    """
    FlexBE state to request execution of Behavior Tree using BT.cpp system.

    Elements defined here for UI
    Parameters
    -- action_topic     Action topic name (default= '/flexbe_bt_server')
    -- timeout          Total time to wait for bt server in seconds (default = 2s)

    Outputs
    <= success          Successfully executed tree
    <= failure          Tree failure
    <= invalid          Invalid tree specified

    User data
    ># bt_name       string   Tree name
    ># bt_payload    string   Payload to pass to BT executor
    #> msg           string   Result message
    """

    def __init__(self, action_topic='/flexbe_bt_server',
                 timeout=2.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['success', 'failure', 'invalid'],
                         input_keys=['bt_name', 'bt_payload'],
                         output_keys=['msg'])

        self._topic = action_topic
        self._server_timeout = Duration(seconds=timeout)

        self._client = ProxyActionClient({self._topic: ExecuteTree},
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
            userdata.msg = result.return_message  # Output message
            if status == GoalStatus.STATUS_SUCCEEDED:
                node_status = result.node_status.status
                if node_status == NodeStatus.SUCCESS:
                    Logger.localinfo(f"'{self}' - behavior tree returned success!"
                                     f" '{result.return_message}'")
                    self._return = 'success'
                else:
                    Logger.localwarn(f"'{self}' : '{self._topic}' - behavior tree "
                                     f"returned ({node_status}) '{result.return_message}'")
                    self._return = 'failure'
                return self._return
            else:
                Logger.logwarn(f"{self} : '{self._topic}' - invalid action status '{result.return_message}'")
                self._return = 'failure'

        # Otherwise check for action status change
        if status == GoalStatus.STATUS_CANCELED:
            Logger.loginfo(f" '{self}' : '{self._topic}' - behavior tree was canceled! ")
            self._return = 'failure'
        elif status == GoalStatus.STATUS_ABORTED:
            Logger.loginfo(f" '{self}' : '{self._topic}' - behavior tree  was aborted! ")
            self._return = 'failure'

        # If the action has not yet finished, None outcome will be returned and the state stays active.
        return self._return

    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the data from prior executions
        Logger.localinfo(f"on_enter '{self}' - '{self.path}' ...")
        self._return = None
        userdata.msg = ''
        self._client.remove_result(self._topic)  # clear any prior result from action server
        if 'bt_name' not in userdata:
            self._return = 'invalid'
            userdata.msg = f"RunBtCppState '{self}' requires userdata.bt_name key!"
            Logger.localwarn(userdata.msg)
            return

        bt_name  = userdata.bt_name
        bt_payload = ''
        if bt_payload in userdata:
            bt_payload = userdata.bt_payload

        # Send the goal.
        try:
            goal = ExecuteTree.Goal(target_tree=bt_name, payload=bt_payload)
            self._client.send_goal(self._topic, goal, wait_duration=self._server_timeout.nanoseconds * 1e-9)
        except Exception as exc:  # pylint: disable=W0703
            # Since a state failure does not necessarily cause a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn(f"Failed to send the '{self}' command:\n  {type(exc)} - {exc}")
            self._return = 'invalid'

    def on_exit(self, userdata):
        """Call when state is deactivated."""
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if self._client.is_active(self._topic):
            self._client.cancel(self._topic)

            # Check for action status change (blocking call!)
            is_terminal, status = self._client.verify_action_status(self._topic, 0.1)
            if status == GoalStatus.STATUS_CANCELED:
                Logger.loginfo(f" '{self}' : '{self._topic}' - request to run BT was canceled! ")
                self._return = 'failure'
            else:
                status_string = self._client.get_status_string(self._topic)
                Logger.loginfo(f" '{self}' : '{self._topic}' - Requested to cancel an active BT '{status_string}'"
                               f" (terminal={is_terminal})")
            self._return = 'failure'

        # Local message are shown in terminal but not the UI
        if self._return == 'success':
            Logger.localinfo('Successfully executed the BT.')
        else:
            Logger.localwarn('failure to execute the BT.')
        Logger.localinfo(f"on_exit '{self}' - '{self.path}' ...")

    def on_pause(self):
        """Execute each time this state is paused."""
        Logger.localinfo(f"on_pause '{self}' - '{self.path}' ...")
        if self._client.is_active(self._topic):
            self._client.cancel(self._topic)
            # Check for action status change (blocking call!)
            is_terminal, status = self._client.verify_action_status(self._topic, 0.1)
            if status == GoalStatus.STATUS_CANCELED:
                Logger.loginfo(f" '{self}' : '{self._topic}' - request to run BT was canceled! ")
                self._return = 'failure'
            else:
                status_string = self._client.get_status_string(self._topic)
                Logger.loginfo(f" '{self}' : '{self._topic}' - Requested to cancel an active BT '{status_string}'"
                               f" (terminal={is_terminal})")
            self._return = 'failure'

    def on_resume(self, userdata):
        """Execute each time this state is resumed."""
        Logger.localinfo(f"on_resume '{self}' - '{self.path}' ...")
        if self._return is None:
            Logger.localinfo(f"Cannot resume BT '{self}' - require new request ...")
            self._return = 'failure'

    def on_start(self):
        """Call when behavior starts."""
        Logger.localinfo(f" on_start  '{self}' - '{self.path}' ")

    def on_stop(self):
        """Call when behavior stops."""
        Logger.localinfo(f" on_stop  '{self}' - '{self.path}'")
