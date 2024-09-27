#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 conner
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
Define Traverse.

Traverse the world opening doors as required

Created on Sun Sep 08 2024
@author: conner
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
