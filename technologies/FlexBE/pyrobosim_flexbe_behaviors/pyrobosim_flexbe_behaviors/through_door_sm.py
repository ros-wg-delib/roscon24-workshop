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
Define Through Door.

Go through a door, opening if required

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
from flexbe_states.operator_decision_state import OperatorDecisionState
from pyrobosim_flexbe_states.check_door_state import CheckDoorState
from pyrobosim_flexbe_states.door_action_state import DoorActionState
from pyrobosim_flexbe_states.follow_path_state import FollowPathState
from pyrobosim_flexbe_states.plan_path_state import PlanPathState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class ThroughDoorSM(Behavior):
    """
    Define Through Door.

    Go through a door, opening if required
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Through Door'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

        # 0 171 407
        # If not, plan path to door and open

        # 0 419 20
        # If we can plan path, then go through the door.

        # 0 202 269
        # If we don't like round about path, give option to go to relevant door.

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:957 y:80, x:1009 y:269
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['room', 'hallway'], output_keys=['msg'])
        _state_machine.userdata.room = 'closet'
        _state_machine.userdata.hallway = 'hall_dining_closet'
        _state_machine.userdata.msg = ''

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:127 y:57
            OperatableStateMachine.add('PlanPath',
                                       PlanPathState(action_topic='robot/plan_path',
                                                     timeout=10.0),
                                       transitions={'done': 'ConfirmGo'  # 306 86 -1 -1 -1 -1
                                                    , 'failed': 'PlanDoor'  # 190 328 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'goal': 'room',
                                                  'msg': 'msg',
                                                  'path': 'path_to_next'})

            # x:712 y:261
            OperatableStateMachine.add('CheckOpen',
                                       CheckDoorState(call_timeout=3.0,
                                                      wait_timeout=3.0,
                                                      service_name='request_world_state',
                                                      robot=''),
                                       transitions={'open': 'PlanPath'  # 399 219 -1 -1 219 110
                                                    , 'closed': 'OpenDoor'  # 917 311 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 932 282 -1 -1 -1 -1
                                                    },
                                       autonomy={'open': Autonomy.Low,
                                                 'closed': Autonomy.Low,
                                                 'failed': Autonomy.High},
                                       remapping={'name': 'hallway', 'msg': 'msg'})

            # x:346 y:66
            OperatableStateMachine.add('ConfirmGo',
                                       OperatorDecisionState(outcomes=['go', 'door', 'failed'],
                                                             hint='go if path is desired',
                                                             suggestion='go'),
                                       transitions={'go': 'GoRoom'  # 512 93 -1 -1 -1 -1
                                                    , 'door': 'PlanDoor'  # 308 233 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 733 225 -1 -1 -1 -1
                                                    },
                                       autonomy={'go': Autonomy.Low,
                                                 'door': Autonomy.Full,
                                                 'failed': Autonomy.Full})

            # x:506 y:316
            OperatableStateMachine.add('GoDoor',
                                       FollowPathState(action_topic='robot/follow_path',
                                                       server_timeout=2.0),
                                       transitions={'done': 'CheckOpen'  # 673 312 -1 -1 -1 -1
                                                    , 'failed': 'RetryDoor'  # 594 399 -1 -1 610 449
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'path': 'path_to_door', 'msg': 'msg'})

            # x:562 y:69
            OperatableStateMachine.add('GoRoom',
                                       FollowPathState(action_topic='robot/follow_path',
                                                       server_timeout=2.0),
                                       transitions={'done': 'finished'  # 831 91 -1 -1 -1 -1
                                                    , 'failed': 'RetryNav'  # 764 151 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'path': 'path_to_next', 'msg': 'msg'})

            # x:852 y:357
            OperatableStateMachine.add('OpenDoor',
                                       DoorActionState(action_type='open',
                                                       robot_name='robot',
                                                       action_topic='/execute_action',
                                                       timeout=2.0),
                                       transitions={'done': 'CheckOpen'  # 794 364 -1 -1 -1 -1
                                                    , 'failed': 'RetryDoor'  # 784 424 -1 -1 706 463
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'msg': 'msg'})

            # x:252 y:346
            OperatableStateMachine.add('PlanDoor',
                                       PlanPathState(action_topic='robot/plan_path',
                                                     timeout=10.0),
                                       transitions={'done': 'GoDoor'  # 457 346 -1 -1 -1 -1
                                                    , 'failed': 'RetryDoor'  # 442 428 -1 -1 578 463
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'goal': 'hallway',
                                                  'msg': 'msg',
                                                  'path': 'path_to_door'})

            # x:579 y:450
            OperatableStateMachine.add('RetryDoor',
                                       OperatorDecisionState(outcomes=['door',
                                                                       'room',
                                                                       'open',
                                                                       'failed'],
                                                             hint="Retry the door",
                                                             suggestion='door'),
                                       transitions={'door': 'PlanDoor'  # 333 459 578 476 328 399
                                                    , 'room': 'PlanPath'  # 187 505 578 487 152 110
                                                    , 'open': 'OpenDoor'  # 838 460 -1 -1 895 410
                                                    , 'failed': 'failed'  # 979 475 706 487 1021 300
                                                    },
                                       autonomy={'door': Autonomy.Low,
                                                 'room': Autonomy.Full,
                                                 'open': Autonomy.Full,
                                                 'failed': Autonomy.Full})

            # x:790 y:151
            OperatableStateMachine.add('RetryNav',
                                       OperatorDecisionState(outcomes=['retry', 'failed'],
                                                             hint="Retry",
                                                             suggestion='retry'),
                                       transitions={'retry': 'PlanPath'  # 505 200 789 186 245 110
                                                    , 'failed': 'failed'  # 888 226 -1 -1 1005 268
                                                    },
                                       autonomy={'retry': Autonomy.Low, 'failed': Autonomy.Full})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
