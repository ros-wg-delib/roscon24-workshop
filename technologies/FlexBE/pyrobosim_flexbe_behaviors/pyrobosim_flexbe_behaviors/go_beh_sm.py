#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 David Conner
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.

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

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Go Beh.

Go to target behavior

Created on Sun Sep 08 2024
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
from pyrobosim_flexbe_states.follow_path_state import FollowPathState
from pyrobosim_flexbe_states.plan_path_state import PlanPathState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class GoBehSM(Behavior):
    """
    Define Go Beh.

    Go to target behavior
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Go Beh'

        # parameters of this behavior
        self.add_parameter('target', 'desk')

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:1061 y:86, x:1083 y:307
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['goal'], output_keys=['msg'])
        _state_machine.userdata.goal = self.target
        _state_machine.userdata.msg = ''

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:117 y:74
            OperatableStateMachine.add('PlanPath',
                                       PlanPathState(action_topic='robot/plan_path',
                                                     timeout=6.0),
                                       transitions={'done': 'UsePlan'  # 310 88 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 274 287 154 127 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal', 'msg': 'msg', 'path': 'path'})

            # x:696 y:190
            OperatableStateMachine.add('AskRetry',
                                       OperatorDecisionState(outcomes=['retry', 'fail'],
                                                             hint='retry',
                                                             suggestion='retry'),
                                       transitions={'retry': 'LogReplan'  # 441 230 -1 -1 362 223
                                                    , 'fail': 'failed'  # 873 278 778 243 -1 -1
                                                    },
                                       autonomy={'retry': Autonomy.High, 'fail': Autonomy.Full})

            # x:584 y:84
            OperatableStateMachine.add('FollowPath',
                                       FollowPathState(action_topic='robot/follow_path',
                                                       server_timeout=2.0),
                                       transitions={'done': 'finished'  # 900 101 -1 -1 -1 -1
                                                    , 'failed': 'AskRetry'  # 789 151 720 114 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'path': 'path', 'msg': 'msg'})

            # x:278 y:194
            OperatableStateMachine.add('LogReplan',
                                       LogState(text="Replan request",
                                                severity=Logger.REPORT_INFO),
                                       transitions={'done': 'PlanPath'  # 220 193 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off})

            # x:345 y:71
            OperatableStateMachine.add('UsePlan',
                                       OperatorDecisionState(outcomes=['use', 'replan', 'fail'],
                                                             hint='use',
                                                             suggestion='use'),
                                       transitions={'use': 'FollowPath'  # 538 99 -1 -1 -1 -1
                                                    , 'replan': 'LogReplan'  # 420 189 419 124 -1 -1
                                                    , 'fail': 'failed'  # 569 256 -1 -1 -1 -1
                                                    },
                                       autonomy={'use': Autonomy.Low,
                                                 'replan': Autonomy.Full,
                                                 'fail': Autonomy.Full})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
