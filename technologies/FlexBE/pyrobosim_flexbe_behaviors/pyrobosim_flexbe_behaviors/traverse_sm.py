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
Define Traverse.

Traverse the world opening doors as required

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
from flexbe_states.log_key_state import LogKeyState
from pyrobosim_flexbe_behaviors.go_beh_sm import GoBehSM
from pyrobosim_flexbe_behaviors.through_door_sm import ThroughDoorSM
from pyrobosim_flexbe_states.next_room_state import NextRoomState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class TraverseSM(Behavior):
    """
    Define Traverse.

    Traverse the world opening doors as required
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Traverse'

        # parameters of this behavior
        self.add_parameter('target', 'charger')

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors
        self.add_behavior(GoBehSM, 'GoLocation', node)
        self.add_behavior(ThroughDoorSM, 'ThruDoor', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:801 y:105, x:838 y:264
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['msg'])
        _state_machine.userdata.goal = self.target
        _state_machine.userdata.msg = "Unknown message"

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:227 y:85
            OperatableStateMachine.add('GoLocation',
                                       self.use_behavior(GoBehSM, 'GoLocation',
                                                         parameters={'target': "desk"}),
                                       transitions={'finished': 'finished'  # 607 109 -1 -1 -1 -1
                                                    , 'failed': 'NextRoom'  # 307 234 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.High,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'goal': 'goal', 'msg': 'msg'})

            # x:467 y:295
            OperatableStateMachine.add('LogNext',
                                       LogKeyState(text="Next room '{}'",
                                                   severity=Logger.REPORT_INFO),
                                       transitions={'done': 'ThruDoor'  # 620 322 -1 -1 658 375
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'room'})

            # x:363 y:218
            OperatableStateMachine.add('NextRoom',
                                       NextRoomState(world='world4',
                                                     package_name='delib_ws_worlds',
                                                     sub_folder='worlds',
                                                     state_topic='robot/robot_state',
                                                     timeout=2.0),
                                       transitions={'done': 'LogNext'  # 412 304 401 271 -1 -1
                                                    , 'failed': 'failed'  # 680 263 499 268 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal',
                                                  'room': 'room',
                                                  'hallway': 'hallway'})

            # x:586 y:376
            OperatableStateMachine.add('ThruDoor',
                                       self.use_behavior(ThroughDoorSM, 'ThruDoor'),
                                       transitions={'finished': 'GoLocation'  # 112 348 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 823 392 -1 -1 846 295
                                                    },
                                       autonomy={'finished': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'room': 'room',
                                                  'hallway': 'hallway',
                                                  'msg': 'msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
