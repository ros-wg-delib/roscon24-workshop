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
Define test navigate.

Test navigation state

Created on Thu Aug 08 2024
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_states.log_state import LogState
from pyrobosim_flexbe_states.navigate_action_state import NavigateActionState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class testnavigateSM(Behavior):
    """
    Define test navigate.

    Test navigation state
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'test navigate'

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
        # Private variables
        action_topic = '/execute_action'
        robot_name = 'robot'

        # Root state machine
        # x:1252 y:96, x:1252 y:256
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:160 y:81
            OperatableStateMachine.add('Start',
                                       LogState(text="Begin navigation ...",
                                                severity=2),
                                       transitions={'done': 'NavigateDesk'  # 305 130 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High})

            # x:965 y:364
            OperatableStateMachine.add('LogCanceled',
                                       LogState(text="Canceled navigation",
                                                severity=2),
                                       transitions={'done': 'failed'  # 1149 321 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Full})

            # x:629 y:102
            OperatableStateMachine.add('LogDesk',
                                       LogState(text="Arrived at desk",
                                                severity=2),
                                       transitions={'done': 'NavigatePantry'  # 753 104 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High})

            # x:1088 y:227
            OperatableStateMachine.add('LogFailed',
                                       LogState(text="Arrived at desk",
                                                severity=2),
                                       transitions={'done': 'failed'  # 1208 246 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Full})

            # x:1086 y:90
            OperatableStateMachine.add('LogPantry',
                                       LogState(text="Arrived at pantry",
                                                severity=2),
                                       transitions={'done': 'finished'  # 1221 89 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High})

            # x:908 y:498
            OperatableStateMachine.add('LogTimeout',
                                       LogState(text="time out error",
                                                severity=2),
                                       transitions={'done': 'failed'  # 1125 394 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Full})

            # x:345 y:144
            OperatableStateMachine.add('NavigateDesk',
                                       NavigateActionState(target_location='desk',
                                                           robot_name=robot_name,
                                                           action_topic=action_topic,
                                                           server_timeout=5.0,
                                                           navigate_timeout=None),
                                       transitions={'done': 'LogDesk'  # 575 134 -1 -1 -1 -1
                                                    , 'planning_failed': 'LogFailed'  # 814 255 -1 -1 -1 -1
                                                    , 'motion_failed': 'LogFailed'  # 814 255 -1 -1 -1 -1
                                                    , 'canceled': 'LogCanceled'  # 761 283 -1 -1 -1 -1
                                                    , 'timeout': 'LogTimeout'  # 718 478 -1 -1 907 540
                                                    },
                                       autonomy={'done': Autonomy.Off,
                                                 'planning_failed': Autonomy.Off,
                                                 'motion_failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'msg': 'msg'})

            # x:806 y:103
            OperatableStateMachine.add('NavigatePantry',
                                       NavigateActionState(target_location='pantry',
                                                           robot_name=robot_name,
                                                           action_topic=action_topic,
                                                           server_timeout=5.0,
                                                           navigate_timeout=None),
                                       transitions={'done': 'LogPantry'  # 1053 96 -1 -1 -1 -1
                                                    , 'planning_failed': 'LogFailed'  # 1055 198 -1 -1 -1 -1
                                                    , 'motion_failed': 'LogFailed'  # 1055 198 -1 -1 -1 -1
                                                    , 'canceled': 'LogCanceled'  # 929 275 -1 -1 -1 -1
                                                    , 'timeout': 'LogTimeout'  # 850 448 853 156 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off,
                                                 'planning_failed': Autonomy.Off,
                                                 'motion_failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'msg': 'msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
