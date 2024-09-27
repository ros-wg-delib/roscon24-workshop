#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 Conner
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
@author: Conner
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
