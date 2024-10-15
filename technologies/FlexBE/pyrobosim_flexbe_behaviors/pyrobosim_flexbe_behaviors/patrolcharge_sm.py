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
Define PatrolCharge.

Patrol and recharge as necessary

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
from flexbe_states.log_state import LogState
from pyrobosim_flexbe_behaviors.patrol_sm import PatrolSM
from pyrobosim_flexbe_behaviors.traverse_sm import TraverseSM
from pyrobosim_flexbe_states.monitor_battery_state import MonitorBatteryState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class PatrolChargeSM(Behavior):
    """
    Define PatrolCharge.

    Patrol and recharge as necessary
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'PatrolCharge'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors
        self.add_behavior(TraverseSM, 'PatrolWithBattery/BatteryMaintain/Container/Traverse', node)
        self.add_behavior(PatrolSM, 'PatrolWithBattery/Patrol', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:658 y:127, x:658 y:267
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        # x:562 y:107, x:557 y:221
        _sm_container_0 = PriorityContainer(outcomes=['finished', 'failed'],
                                            output_keys=['msg'])

        with _sm_container_0:
            # x:87 y:45
            OperatableStateMachine.add('Recharge',
                                       LogState(text="Begin recharge ...",
                                                severity=Logger.REPORT_INFO),
                                       transitions={'done': 'Traverse'  # 241 71 -1 -1 250 120
                                                    },
                                       autonomy={'done': Autonomy.Off})

            # x:156 y:121
            OperatableStateMachine.add('Traverse',
                                       self.use_behavior(TraverseSM, 'PatrolWithBattery/BatteryMaintain/Container/Traverse',
                                                         parameters={'target': "charger_dock"}),
                                       transitions={'finished': 'finished'  # 451 132 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 448 189 334 161 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'msg': 'msg'})

        # x:333 y:290
        _sm_batterymaintain_1 = OperatableStateMachine(outcomes=['failed'])

        with _sm_batterymaintain_1:
            # x:126 y:87
            OperatableStateMachine.add('CheckBattery',
                                       MonitorBatteryState(low_battery_level=30.0,
                                                           high_battery_level=105.0,
                                                           state_topic='robot/robot_state',
                                                           timeout=2.0),
                                       transitions={'battery_level': 'Container'  # 326 107 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 207 240 171 140 -1 -1
                                                    },
                                       autonomy={'battery_level': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'battery_level': 'battery_level'})

            # x:398 y:90
            OperatableStateMachine.add('Container',
                                       _sm_container_0,
                                       transitions={'finished': 'CheckBattery'  # 328 222 423 149 228 140
                                                    , 'failed': 'failed'  # 442 232 457 149 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

        # x:621 y:77, x:631 y:283, x:603 y:384, x:616 y:482
        _sm_patrolwithbattery_2 = ConcurrencyContainer(outcomes=['finished', 'failed'],
                                                       output_keys=['msg'],
                                                       conditions=[('failed', [('BatteryMaintain', 'failed')]),
                                                                   ('finished', [('Patrol', 'finished')])
                                                                   ])

        with _sm_patrolwithbattery_2:
            # x:132 y:62
            OperatableStateMachine.add('Patrol',
                                       self.use_behavior(PatrolSM, 'PatrolWithBattery/Patrol'),
                                       transitions={'finished': 'finished'},
                                       autonomy={'finished': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

            # x:163 y:269
            OperatableStateMachine.add('BatteryMaintain',
                                       _sm_batterymaintain_1,
                                       transitions={'failed': 'failed'},
                                       autonomy={'failed': Autonomy.Inherit})

        with _state_machine:
            # x:290 y:93
            OperatableStateMachine.add('PatrolWithBattery',
                                       _sm_patrolwithbattery_2,
                                       transitions={'finished': 'finished'  # 547 125 -1 -1 -1 -1
                                                    , 'failed': 'LogFailed'  # 441 172 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.High,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'msg': 'msg'})

            # x:477 y:188
            OperatableStateMachine.add('LogFailed',
                                       LogKeyState(text="Failed {}",
                                                   severity=Logger.REPORT_ERROR),
                                       transitions={'done': 'failed'  # 610 246 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off},
                                       remapping={'data': 'msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
