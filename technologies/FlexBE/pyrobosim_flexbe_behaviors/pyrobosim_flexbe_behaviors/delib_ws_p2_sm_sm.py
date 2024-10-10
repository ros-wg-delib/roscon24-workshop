#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 David Conner
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

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define delib_ws_p2_sm.

Behavior demonstrating p2 - travel, open door, go to table, pick object,
transport, and place using embedded state machine and operator selection.

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
from flexbe_states.user_data_state import UserdataState
from pyrobosim_flexbe_behaviors.detectselect_sm import DetectSelectSM
from pyrobosim_flexbe_states.door_action_state import DoorActionState
from pyrobosim_flexbe_states.follow_path_state import FollowPathState
from pyrobosim_flexbe_states.plan_path_state import PlanPathState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class delib_ws_p2_smSM(Behavior):
    """
    Define delib_ws_p2_sm.

    Behavior demonstrating p2 - travel, open door, go to table, pick object,
    transport, and place using embedded state machine and operator selection.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'delib_ws_p2_sm'

        # parameters of this behavior
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
        # x:78 y:340, x:684 y:262
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        # x:1160 y:383, x:775 y:411
        _sm_opendoorsm_0 = OperatableStateMachine(outcomes=['finished', 'failed'],
                                                  output_keys=['msg'])

        with _sm_opendoorsm_0:
            # x:97 y:32
            OperatableStateMachine.add('ChooseGoal',
                                       OperatorDecisionState(outcomes=['dumpster',
                                                                       'hall_dining_trash',
                                                                       'hall_kitchen_office',
                                                                       'hall_kitchen_trash',
                                                                       'quit'],
                                                             hint=None,
                                                             suggestion=None),
                                       transitions={'dumpster': 'Dumpster'  # 321 52 -1 -1 -1 -1
                                                    , 'hall_dining_trash': 'DiningTrashDoor'  # 316 125 -1 -1 -1 -1
                                                    , 'hall_kitchen_office': 'KitchenOfficeDoor'  # 284 216 211 85 -1 -1
                                                    , 'hall_kitchen_trash': 'KitchenTrash'  # 262 287 159 85 -1 -1
                                                    , 'quit': 'QuitMsg'  # 254 392 127 85 -1 -1
                                                    },
                                       autonomy={'dumpster': Autonomy.Off,
                                                 'hall_dining_trash': Autonomy.Off,
                                                 'hall_kitchen_office': Autonomy.Off,
                                                 'hall_kitchen_trash': Autonomy.Off,
                                                 'quit': Autonomy.Off})

            # x:395 y:110
            OperatableStateMachine.add('DiningTrashDoor',
                                       UserdataState(data='hall_dining_trash'),
                                       transitions={'done': 'PlanToDoor'  # 559 131 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'goal'})

            # x:399 y:46
            OperatableStateMachine.add('Dumpster',
                                       UserdataState(data='dumpster'),
                                       transitions={'done': 'PlanToDoor'  # 570 87 -1 -1 638 127
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'goal'})

            # x:817 y:170
            OperatableStateMachine.add('GoToDoor',
                                       FollowPathState(action_topic='robot/follow_path',
                                                       server_timeout=2.0),
                                       transitions={'done': 'OpenDoor',
                                                    'failed': 'failed'  # 789 314 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'path': 'path', 'msg': 'msg'})

            # x:399 y:185
            OperatableStateMachine.add('KitchenOfficeDoor',
                                       UserdataState(data='hall_kitchen_office'),
                                       transitions={'done': 'PlanToDoor'  # 569 173 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'goal'})

            # x:399 y:262
            OperatableStateMachine.add('KitchenTrash',
                                       UserdataState(data='hall_kitchen_trash'),
                                       transitions={'done': 'PlanToDoor'  # 567 230 -1 -1 625 181
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'goal'})

            # x:1026 y:250
            OperatableStateMachine.add('OpenDoor',
                                       DoorActionState(action_type='open',
                                                       robot_name='robot',
                                                       action_topic='/execute_action',
                                                       timeout=2.0),
                                       transitions={'done': 'finished'  # 1108 373 1118 303 -1 -1
                                                    , 'failed': 'failed'  # 989 374 -1 -1 811 418
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'msg': 'msg'})

            # x:596 y:128
            OperatableStateMachine.add('PlanToDoor',
                                       PlanPathState(action_topic='robot/plan_path',
                                                     timeout=10.0),
                                       transitions={'done': 'GoToDoor',
                                                    'failed': 'failed'  # 710 360 688 181 770 426
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal', 'msg': 'msg', 'path': 'path'})

            # x:397 y:368
            OperatableStateMachine.add('QuitMsg',
                                       UserdataState(data='User selected quit!'),
                                       transitions={'done': 'failed'  # 636 412 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'goal'})

        with _state_machine:
            # x:210 y:90
            OperatableStateMachine.add('OpDecision',
                                       OperatorDecisionState(outcomes=['open', 'go', 'quit'],
                                                             hint="Go to table",
                                                             suggestion='go'),
                                       transitions={'open': 'OpenDoorSM'  # 411 88 -1 -1 -1 -1
                                                    , 'go': 'DetectSelect'  # 227 298 258 143 289 373
                                                    , 'quit': 'finished'  # 67 226 240 143 81 339
                                                    },
                                       autonomy={'open': Autonomy.Full,
                                                 'go': Autonomy.High,
                                                 'quit': Autonomy.Full})

            # x:290 y:347
            OperatableStateMachine.add('DetectSelect',
                                       self.use_behavior(DetectSelectSM, 'DetectSelect',
                                                         parameters={'move_location': self.get_location,
                                                                     'place_location': self.put_location}),
                                       transitions={'finished': 'finished'  # 154 370 289 388 -1 -1
                                                    , 'failed': 'OpDecision'  # 282 282 337 346 285 143
                                                    },
                                       autonomy={'finished': Autonomy.Full, 'failed': Autonomy.High})

            # x:396 y:237
            OperatableStateMachine.add('LogFailed',
                                       LogKeyState(text="failed - {}",
                                                   severity=2),
                                       transitions={'done': 'OpDecision'  # 327 229 -1 -1 310 143
                                                    },
                                       autonomy={'done': Autonomy.Low},
                                       remapping={'data': 'msg'})

            # x:471 y:106
            OperatableStateMachine.add('OpenDoorSM',
                                       _sm_opendoorsm_0,
                                       transitions={'finished': 'OpDecision'  # 398 154 470 140 337 123
                                                    , 'failed': 'LogFailed'  # 516 233 519 165 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Off},
                                       remapping={'msg': 'msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
