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
Define BtCppHFSMBTH.

Demo of the mythical HFSMBTH using BehaviorTree.cpp

Created on Sun Sep 29 2024
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
from pyrobosim_flexbe_states.run_btcpp_tree_state import RunBtCppState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class BtCppHFSMBTHSM(Behavior):
    """
    Define BtCppHFSMBTH.

    Demo of the mythical HFSMBTH using BehaviorTree.cpp
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'BtCppHFSMBTH'

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
        # x:722 y:62
        _state_machine = OperatableStateMachine(outcomes=['finished'])
        _state_machine.userdata.nav_tree = 'NavigationDemoTree'
        _state_machine.userdata.p1_tree = 'Problem1Tree'
        _state_machine.userdata.p2_tree = 'Problem2Tree'
        _state_machine.userdata.bt_payload = ''
        _state_machine.userdata.office_tree = 'OfficeNavTree'

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:53 y:37
            OperatableStateMachine.add('ChooseTree',
                                       OperatorDecisionState(outcomes=['nav',
                                                                       'p1',
                                                                       'p2',
                                                                       'quit',
                                                                       'office'],
                                                             hint="Choose a behavior tree",
                                                             suggestion='nav'),
                                       transitions={'nav': 'NavTree'  # 185 213 139 90 -1 -1
                                                    , 'p1': 'P1Tree'  # 186 316 123 90 -1 -1
                                                    , 'p2': 'P2Tree'  # 204 435 105 90 -1 -1
                                                    , 'quit': 'finished'  # 452 61 -1 -1 -1 -1
                                                    , 'office': 'OfficeTree'  # 204 508 75 90 -1 -1
                                                    },
                                       autonomy={'nav': Autonomy.High,
                                                 'p1': Autonomy.Full,
                                                 'p2': Autonomy.Full,
                                                 'quit': Autonomy.Full,
                                                 'office': Autonomy.Full})

            # x:474 y:169
            OperatableStateMachine.add('LogFailure',
                                       LogKeyState(text="Failure {}",
                                                   severity=2),
                                       transitions={'done': 'ChooseTree'  # 272 162 -1 -1 149 90
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'msg'})

            # x:685 y:248
            OperatableStateMachine.add('LogSuccess',
                                       LogKeyState(text="{}",
                                                   severity=2),
                                       transitions={'done': 'ChooseTree'  # 566 136 713 247 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'msg'})

            # x:219 y:227
            OperatableStateMachine.add('NavTree',
                                       RunBtCppState(action_topic='/flexbe_bt_server',
                                                     timeout=2.0),
                                       transitions={'success': 'LogSuccess'  # 589 244 -1 -1 684 260
                                                    , 'failure': 'LogFailure'  # 436 208 -1 -1 473 199
                                                    , 'invalid': 'LogFailure'  # 436 208 -1 -1 473 199
                                                    },
                                       autonomy={'success': Autonomy.Low,
                                                 'failure': Autonomy.Off,
                                                 'invalid': Autonomy.Off},
                                       remapping={'bt_name': 'nav_tree',
                                                  'bt_payload': 'bt_payload',
                                                  'msg': 'msg'})

            # x:603 y:515
            OperatableStateMachine.add('OfficeTree',
                                       RunBtCppState(action_topic='/flexbe_bt_server',
                                                     timeout=2.0),
                                       transitions={'success': 'LogSuccess'  # 733 422 686 514 724 301
                                                    , 'failure': 'LogFailure'  # 590 378 638 514 541 222
                                                    , 'invalid': 'LogFailure'  # 590 378 638 514 541 222
                                                    },
                                       autonomy={'success': Autonomy.Low,
                                                 'failure': Autonomy.Off,
                                                 'invalid': Autonomy.Off},
                                       remapping={'bt_name': 'office_tree',
                                                  'bt_payload': 'bt_payload',
                                                  'msg': 'msg'})

            # x:226 y:319
            OperatableStateMachine.add('P1Tree',
                                       RunBtCppState(action_topic='/flexbe_bt_server',
                                                     timeout=2.0),
                                       transitions={'success': 'LogSuccess'  # 597 301 -1 -1 -1 -1
                                                    , 'failure': 'LogFailure'  # 431 292 -1 -1 481 222
                                                    , 'invalid': 'LogFailure'  # 431 292 -1 -1 481 222
                                                    },
                                       autonomy={'success': Autonomy.Low,
                                                 'failure': Autonomy.Off,
                                                 'invalid': Autonomy.Off},
                                       remapping={'bt_name': 'p1_tree',
                                                  'bt_payload': 'bt_payload',
                                                  'msg': 'msg'})

            # x:402 y:409
            OperatableStateMachine.add('P2Tree',
                                       RunBtCppState(action_topic='/flexbe_bt_server',
                                                     timeout=2.0),
                                       transitions={'success': 'LogSuccess'  # 655 397 -1 -1 699 301
                                                    , 'failure': 'LogFailure'  # 501 344 -1 -1 503 222
                                                    , 'invalid': 'LogFailure'  # 501 344 -1 -1 503 222
                                                    },
                                       autonomy={'success': Autonomy.Low,
                                                 'failure': Autonomy.Off,
                                                 'invalid': Autonomy.Off},
                                       remapping={'bt_name': 'p2_tree',
                                                  'bt_payload': 'bt_payload',
                                                  'msg': 'msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
