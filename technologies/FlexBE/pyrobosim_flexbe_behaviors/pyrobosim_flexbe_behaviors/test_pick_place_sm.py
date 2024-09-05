#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 David Conner
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
Define test pick place.

Test Navigation, Pick and Place states

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
from flexbe_states.log_state import LogState
from pyrobosim_flexbe_states.navigate_action_state import NavigateActionState
from pyrobosim_flexbe_states.pick_action_state import PickActionState
from pyrobosim_flexbe_states.place_action_state import PlaceActionState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class testpickplaceSM(Behavior):
    """
    Define test pick place.

    Test Navigation, Pick and Place states
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'test pick place'

        # parameters of this behavior
        self.add_parameter('location', 'counter0_left')
        self.add_parameter('object', 'banana')
        self.add_parameter('return_location', 'desk0')

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
        # x:1283 y:96, x:1043 y:255
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.object = self.object

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]








        # [/MANUAL_CREATE]

        with _state_machine:
            # x:54 y:52
            OperatableStateMachine.add('Start',
                                       LogState(text="Begin navigation ...",
                                                severity=2),
                                       transitions={'done': 'NavigateLocation'  # 83 181 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High})

            # x:366 y:56
            OperatableStateMachine.add('LogArrived',
                                       LogState(text="Arrived at ",
                                                severity=2),
                                       transitions={'done': 'PickObject'  # 485 48 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.High})

            # x:708 y:366
            OperatableStateMachine.add('LogCanceled',
                                       LogState(text="Canceled navigation",
                                                severity=2),
                                       transitions={'done': 'failed'  # 927 322 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Full})

            # x:734 y:212
            OperatableStateMachine.add('LogFailed',
                                       LogState(text="Failed to complete task!",
                                                severity=2),
                                       transitions={'done': 'failed'  # 960 240 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Full})

            # x:1200 y:206
            OperatableStateMachine.add('LogPlaced',
                                       LogState(text="Placed object",
                                                severity=2),
                                       transitions={'done': 'finished'  # 1257 157 -1 -1 1273 107
                                                    },
                                       autonomy={'done': Autonomy.Off})

            # x:1071 y:82
            OperatableStateMachine.add('LogReturn',
                                       LogState(text="Arrived at ",
                                                severity=2),
                                       transitions={'done': 'PlaceObject'  # 1130 272 -1 -1 1162 388
                                                    },
                                       autonomy={'done': Autonomy.High})

            # x:908 y:498
            OperatableStateMachine.add('LogTimeout',
                                       LogState(text="time out error",
                                                severity=2),
                                       transitions={'done': 'failed'  # 993 394 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Full})

            # x:121 y:182
            OperatableStateMachine.add('NavigateLocation',
                                       NavigateActionState(target_location=self.location,
                                                           robot_name=robot_name,
                                                           action_topic=action_topic,
                                                           server_timeout=5.0,
                                                           navigate_timeout=None),
                                       transitions={'done': 'LogArrived'  # 307 101 258 190 -1 -1
                                                    , 'planning_failed': 'LogFailed'  # 502 242 -1 -1 -1 -1
                                                    , 'motion_failed': 'LogFailed'  # 502 242 -1 -1 -1 -1
                                                    , 'canceled': 'LogCanceled'  # 515 298 -1 -1 -1 -1
                                                    , 'timeout': 'LogTimeout'  # 642 480 -1 -1 907 540
                                                    },
                                       autonomy={'done': Autonomy.Off,
                                                 'planning_failed': Autonomy.Off,
                                                 'motion_failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off})

            # x:827 y:52
            OperatableStateMachine.add('NavigateReturn',
                                       NavigateActionState(target_location=self.return_location,
                                                           robot_name=robot_name,
                                                           action_topic=action_topic,
                                                           server_timeout=5.0,
                                                           navigate_timeout=None),
                                       transitions={'done': 'LogReturn'  # 1042 68 -1 -1 -1 -1
                                                    , 'planning_failed': 'LogFailed'  # 745 176 -1 -1 -1 -1
                                                    , 'motion_failed': 'LogFailed'  # 745 176 -1 -1 -1 -1
                                                    , 'canceled': 'LogCanceled'  # 831 296 848 105 -1 -1
                                                    , 'timeout': 'LogTimeout'  # 862 442 874 105 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'planning_failed': Autonomy.Off,
                                                 'motion_failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off})

            # x:536 y:47
            OperatableStateMachine.add('PickObject',
                                       PickActionState(robot_name='robot',
                                                       action_topic='/execute_action',
                                                       timeout=2.0),
                                       transitions={'done': 'NavigateReturn'  # 772 57 -1 -1 -1 -1
                                                    , 'failed': 'LogFailed'  # 596 185 634 100 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'object': 'object', 'msg': 'msg'})

            # x:1107 y:389
            OperatableStateMachine.add('PlaceObject',
                                       PlaceActionState(robot_name='robot',
                                                        action_topic='/execute_action',
                                                        timeout=2.0),
                                       transitions={'done': 'LogPlaced'  # 1238 310 -1 -1 -1 -1
                                                    , 'failed': 'LogFailed'  # 947 287 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'msg': 'msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]








    # [/MANUAL_FUNC]
