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
Define DetectSelect.

Simple test of PlanPath action for pyrobosim with detection and selection
actions.

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
from flexbe_states.log_state import LogState
from flexbe_states.selection_state import SelectionState
from flexbe_states.user_data_state import UserdataState
from pyrobosim_flexbe_states.detect_objects_state import DetectObjectsState
from pyrobosim_flexbe_states.follow_path_state import FollowPathState
from pyrobosim_flexbe_states.pick_action_state import PickActionState
from pyrobosim_flexbe_states.place_action_state import PlaceActionState
from pyrobosim_flexbe_states.plan_path_state import PlanPathState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class DetectSelectSM(Behavior):
    """
    Define DetectSelect.

    Simple test of PlanPath action for pyrobosim with detection and selection
    actions.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'DetectSelect'

        # parameters of this behavior
        self.add_parameter('move_location', 'pantry')
        self.add_parameter('place_location', 'desk')

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
        # x:1283 y:68, x:673 y:457
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.goal = self.move_location

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:128 y:59
            OperatableStateMachine.add('PlanFirstMove',
                                       PlanPathState(action_topic='robot/plan_path',
                                                     timeout=10.0),
                                       transitions={'done': 'FollowFirstPath'  # 354 58 -1 -1 -1 -1
                                                    , 'failed': 'LogFailedMsg'  # 204 408 262 112 325 484
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal', 'msg': 'msg', 'path': 'path'})

            # x:912 y:30
            OperatableStateMachine.add('DetectObjects',
                                       DetectObjectsState(filter=None,
                                                          action_topic='robot/detect_objects',
                                                          timeout=2.0),
                                       transitions={'done': 'SelectObject'  # 971 157 956 83 910 228
                                                    , 'failed': 'LogFailedMsg'  # 638 262 -1 -1 -1 -1
                                                    , 'nothing': 'LogFailedMsg'  # 638 262 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High,
                                                 'failed': Autonomy.Off,
                                                 'nothing': Autonomy.Off},
                                       remapping={'goal': 'goal', 'msg': 'msg', 'items': 'items'})

            # x:423 y:47
            OperatableStateMachine.add('FollowFirstPath',
                                       FollowPathState(action_topic='robot/follow_path',
                                                       server_timeout=2.0),
                                       transitions={'done': 'DetectObjects'  # 763 54 -1 -1 -1 -1
                                                    , 'failed': 'LogFailedMsg'  # 363 361 447 100 358 444
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'path': 'path', 'msg': 'msg'})

            # x:794 y:615
            OperatableStateMachine.add('FollowNextPath',
                                       FollowPathState(action_topic='robot/follow_path',
                                                       server_timeout=2.0),
                                       transitions={'done': 'PlaceObject'  # 1080 654 -1 -1 1131 603
                                                    , 'failed': 'LogFailedMsg'  # 580 580 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'path': 'path', 'msg': 'msg'})

            # x:326 y:445
            OperatableStateMachine.add('LogFailedMsg',
                                       LogKeyState(text="Failed: {}",
                                                   severity=2),
                                       transitions={'done': 'failed'  # 537 469 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Full},
                                       remapping={'data': 'msg'})

            # x:615 y:321
            OperatableStateMachine.add('LogFailure',
                                       LogState(text="Failed to select object",
                                                severity=2),
                                       transitions={'done': 'failed'  # 542 416 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High})

            # x:1217 y:401
            OperatableStateMachine.add('LogFinished',
                                       LogState(text="Placed object!",
                                                severity=2),
                                       transitions={'done': 'finished'  # 1261 238 -1 -1 1288 99
                                                    },
                                       autonomy={'done': Autonomy.Full})

            # x:940 y:376
            OperatableStateMachine.add('NextLocation',
                                       UserdataState(data=self.place_location),
                                       transitions={'done': 'PlanNextMove'  # 897 435 -1 -1 893 486
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'goal'})

            # x:1030 y:310
            OperatableStateMachine.add('PickObject',
                                       PickActionState(robot_name='robot',
                                                       action_topic='/execute_action',
                                                       timeout=2.0),
                                       transitions={'done': 'NextLocation'  # 1075 390 -1 -1 -1 -1
                                                    , 'failed': 'LogFailedMsg'  # 709 409 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'object': 'object', 'msg': 'msg'})

            # x:1079 y:550
            OperatableStateMachine.add('PlaceObject',
                                       PlaceActionState(robot_name='robot',
                                                        action_topic='/execute_action',
                                                        timeout=2.0),
                                       transitions={'done': 'LogFinished'  # 1265 518 -1 -1 -1 -1
                                                    , 'failed': 'LogFailedMsg'  # 745 539 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'msg': 'msg'})

            # x:838 y:487
            OperatableStateMachine.add('PlanNextMove',
                                       PlanPathState(action_topic='robot/plan_path',
                                                     timeout=2.0),
                                       transitions={'done': 'FollowNextPath'  # 844 575 870 540 831 614
                                                    , 'failed': 'LogFailedMsg'  # 602 490 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal', 'msg': 'msg', 'path': 'path'})

            # x:869 y:229
            OperatableStateMachine.add('SelectObject',
                                       SelectionState(message="Select item to pick",
                                                      timeout=1.0,
                                                      action_topic='flexbe/behavior_input'),
                                       transitions={'received': 'PickObject'  # 1041 257 -1 -1 1071 309
                                                    , 'aborted': 'LogFailure'  # 772 301 -1 -1 -1 -1
                                                    , 'no_connection': 'LogFailure'  # 772 301 -1 -1 -1 -1
                                                    , 'data_error': 'LogFailure'  # 772 301 -1 -1 -1 -1
                                                    },
                                       autonomy={'received': Autonomy.High,
                                                 'aborted': Autonomy.Low,
                                                 'no_connection': Autonomy.Low,
                                                 'data_error': Autonomy.Low},
                                       remapping={'items': 'items', 'data': 'object'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
