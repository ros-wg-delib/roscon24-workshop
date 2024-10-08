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
Define Patrol.

Patrol rooms without concern for battery

Created on Mon Sep 23 2024
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
from pyrobosim_flexbe_behaviors.traverse_sm import TraverseSM

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class PatrolSM(Behavior):
    """
    Define Patrol.

    Patrol rooms without concern for battery
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Patrol'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors
        self.add_behavior(TraverseSM, 'Closet', node)
        self.add_behavior(TraverseSM, 'Dining1', node)
        self.add_behavior(TraverseSM, 'Dining2', node)
        self.add_behavior(TraverseSM, 'Kitchen', node)
        self.add_behavior(TraverseSM, 'Office', node)
        self.add_behavior(TraverseSM, 'Trash', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:339 y:165
        _state_machine = OperatableStateMachine(outcomes=['finished'], output_keys=['msg'])
        _state_machine.userdata.msg = "unknown message"

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:81 y:121
            OperatableStateMachine.add('Patrol',
                                       OperatorDecisionState(outcomes=["patrol", "quit"],
                                                             hint="Continue patrol",
                                                             suggestion="patrol"),
                                       transitions={'patrol': 'Office'  # 293 90 -1 -1 -1 -1
                                                    , 'quit': 'finished'  # 275 196 181 174 -1 -1
                                                    },
                                       autonomy={'patrol': Autonomy.Low, 'quit': Autonomy.Full})

            # x:64 y:368
            OperatableStateMachine.add('Closet',
                                       self.use_behavior(TraverseSM, 'Closet',
                                                         parameters={'target': "closet"}),
                                       transitions={'finished': 'Patrol'  # 104 274 131 367 136 174
                                                    , 'failed': 'LogFailed'  # 422 352 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

            # x:503 y:556
            OperatableStateMachine.add('Dining1',
                                       self.use_behavior(TraverseSM, 'Dining1',
                                                         parameters={'target': "dining"}),
                                       transitions={'finished': 'Closet'  # 313 488 -1 -1 -1 -1
                                                    , 'failed': 'LogFailed'  # 546 449 554 555 543 335
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

            # x:676 y:92
            OperatableStateMachine.add('Dining2',
                                       self.use_behavior(TraverseSM, 'Dining2',
                                                         parameters={'target': "dining"}),
                                       transitions={'finished': 'Kitchen'  # 973 151 -1 -1 1005 212
                                                    , 'failed': 'LogFailed'  # 607 213 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

            # x:917 y:213
            OperatableStateMachine.add('Kitchen',
                                       self.use_behavior(TraverseSM, 'Kitchen',
                                                         parameters={'target': "kitchen"}),
                                       transitions={'finished': 'Trash'  # 1051 360 -1 -1 983 452
                                                    , 'failed': 'LogFailed'  # 727 276 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

            # x:503 y:282
            OperatableStateMachine.add('LogFailed',
                                       LogKeyState(text="Patrol failure {}",
                                                   severity=2),
                                       transitions={'done': 'Patrol'  # 290 283 -1 -1 154 174
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'msg'})

            # x:370 y:24
            OperatableStateMachine.add('Office',
                                       self.use_behavior(TraverseSM, 'Office',
                                                         parameters={'target': "office"}),
                                       transitions={'finished': 'Dining2'  # 638 39 -1 -1 759 91
                                                    , 'failed': 'LogFailed'  # 445 188 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

            # x:891 y:453
            OperatableStateMachine.add('Trash',
                                       self.use_behavior(TraverseSM, 'Trash',
                                                         parameters={'target': "trash"}),
                                       transitions={'finished': 'Dining1'  # 817 568 -1 -1 681 585
                                                    , 'failed': 'LogFailed'  # 720 393 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
