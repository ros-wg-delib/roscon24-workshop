#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 David Conner
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
Define Test Plan Path.

Simple test of PlanPath action for pyrobosim

Created on Wed Aug 14 2024
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_states.log_key_state import LogKeyState
from pyrobosim_flexbe_states.follow_path_state import FollowPathState
from pyrobosim_flexbe_states.plan_path_state import PlanPathState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class TestPlanPathSM(Behavior):
    """
    Define Test Plan Path.

    Simple test of PlanPath action for pyrobosim
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Test Plan Path'

        # parameters of this behavior

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
        # x:1232 y:54, x:1214 y:117
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.destination = 'dumpster'

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:128 y:59
            OperatableStateMachine.add('PlanDestination',
                                       PlanPathState(action_topic='robot/plan_path',
                                                     timeout=2.0),
                                       transitions={'done': 'FollowPath'  # 428 72 -1 -1 -1 -1
                                                    , 'failed': 'LogFailedMsg'  # 488 225 262 112 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'destination',
                                                  'msg': 'msg',
                                                  'path': 'path'})

            # x:501 y:44
            OperatableStateMachine.add('FollowPath',
                                       FollowPathState(action_topic='robot/follow_path',
                                                       server_timeout=2.0),
                                       transitions={'done': 'finished'  # 953 64 -1 -1 -1 -1
                                                    , 'failed': 'LogFailedMsg'  # 684 211 611 97 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'path': 'path', 'msg': 'msg'})

            # x:670 y:321
            OperatableStateMachine.add('LogFailedMsg',
                                       LogKeyState(text="Failed: {}",
                                                   severity=2),
                                       transitions={'done': 'failed'  # 991 234 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
