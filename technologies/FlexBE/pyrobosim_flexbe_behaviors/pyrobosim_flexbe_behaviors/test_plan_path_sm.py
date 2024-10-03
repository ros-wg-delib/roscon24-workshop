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
