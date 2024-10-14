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
Define delib_ws_p2.

Behavior demonstrating p2 - travel, open door, go to table, pick object,
transport, and place

Created on Sat Aug 31 2024
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
from flexbe_states.operator_decision_state import OperatorDecisionState
from pyrobosim_flexbe_behaviors.detectselect_sm import DetectSelectSM
from pyrobosim_flexbe_states.door_action_state import DoorActionState
from pyrobosim_flexbe_states.follow_path_state import FollowPathState
from pyrobosim_flexbe_states.plan_path_state import PlanPathState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class delib_ws_p2SM(Behavior):
    """
    Define delib_ws_p2.

    Behavior demonstrating p2 - travel, open door, go to table, pick object,
    transport, and place
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'delib_ws_p2'

        # parameters of this behavior
        self.add_parameter('door_location', 'hall_dining_trash')
        self.add_parameter('get_location', 'desk')
        self.add_parameter('put_location', 'dumpster')

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors
        self.add_behavior(DetectSelectSM, 'DetectSelect', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:78 y:340, x:943 y:183
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.goal = self.door_location

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:131 y:80
            OperatableStateMachine.add('OpDecision',
                                       OperatorDecisionState(outcomes=['open', 'go', 'quit'],
                                                             hint="Go to table",
                                                             suggestion='go'),
                                       transitions={'open': 'PlanToDoor'  # 292 105 -1 -1 -1 -1
                                                    , 'go': 'DetectSelect'  # 188 323 -1 -1 448 414
                                                    , 'quit': 'finished'  # 106 234 170 133 -1 -1
                                                    },
                                       autonomy={'open': Autonomy.Full,
                                                 'go': Autonomy.High,
                                                 'quit': Autonomy.Full})

            # x:449 y:388
            OperatableStateMachine.add('DetectSelect',
                                       self.use_behavior(DetectSelectSM, 'DetectSelect',
                                                         parameters={'move_location': self.get_location,
                                                                     'place_location': self.put_location}),
                                       transitions={'finished': 'finished'  # 267 390 448 429 -1 -1
                                                    , 'failed': 'OpDecision'  # 232 304 -1 -1 197 133
                                                    },
                                       autonomy={'finished': Autonomy.Full, 'failed': Autonomy.High})

            # x:559 y:83
            OperatableStateMachine.add('GoToDoor',
                                       FollowPathState(action_topic='robot/follow_path',
                                                       server_timeout=2.0),
                                       transitions={'done': 'OpenDoor'  # 754 117 -1 -1 786 183
                                                    , 'failed': 'LogFailed'  # 471 177 558 130 420 240
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'path': 'path', 'msg': 'msg'})

            # x:369 y:241
            OperatableStateMachine.add('LogFailed',
                                       LogKeyState(text="Failed - {}",
                                                   severity=2),
                                       transitions={'done': 'OpDecision'  # 243 237 -1 -1 212 133
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'msg'})

            # x:742 y:184
            OperatableStateMachine.add('OpenDoor',
                                       DoorActionState(action_type='open',
                                                       robot_name='robot',
                                                       action_topic='/execute_action',
                                                       timeout=2.0),
                                       transitions={'done': 'DetectSelect'  # 581 304 741 223 540 387
                                                    , 'failed': 'LogFailed'  # 582 226 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'msg': 'msg'})

            # x:326 y:80
            OperatableStateMachine.add('PlanToDoor',
                                       PlanPathState(action_topic='robot/plan_path',
                                                     timeout=10.0),
                                       transitions={'done': 'GoToDoor'  # 503 91 -1 -1 -1 -1
                                                    , 'failed': 'LogFailed'  # 390 200 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal', 'msg': 'msg', 'path': 'path'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
