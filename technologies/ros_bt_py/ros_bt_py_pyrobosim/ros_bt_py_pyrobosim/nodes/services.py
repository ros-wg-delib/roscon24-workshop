from pyrobosim_msgs.msg import ObjectState, RobotState

from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.exceptions import BehaviorTreeException

from abc import ABC, abstractmethod
from typing import Optional, Dict, Tuple, List
from ros_bt_py.ros_nodes.service import ServiceForSetType

from pyrobosim_msgs.srv import RequestWorldState


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"robot_name": str, "object_name": str},
        outputs={},
        max_children=0,
    )
)
class CheckIfCarriesObject(ServiceForSetType):

    def set_service_type(self):
        self._service_type = RequestWorldState

    # Set all outputs to none (define output key while overwriting)
    def set_output_none(self):
        pass

    # Sets the service request message, sent to the service.
    def set_request(self):
        self._last_request = RequestWorldState.Request()

    # Sets the output (in relation to the response) (define output key while overwriting)
    # it should return True, if the node state should be SUCCEEDED after receiving the message and
    # False. if it should be in the FAILED state
    def set_outputs(self):
        response: RequestWorldState.Response = self._service_request_future.result()
        robot_with_matching_name: List[RobotState] = [
            robot
            for robot in response.state.robots
            if robot.name == self.inputs["robot_name"]
        ]
        if len(robot_with_matching_name) != 1:
            self.logerr("Could not unambiguously find robot!")
            raise BehaviorTreeException("Could not unambiguously find robot!")
        robot = robot_with_matching_name[0]
        return (
            robot.holding_object
            and robot.manipulated_object == self.inputs["object_name"]
        )


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"object_name": str},
        outputs={"location": str},
        max_children=0,
    )
)
class GetObjectPosition(ServiceForSetType):

    def set_service_type(self):
        self._service_type = RequestWorldState

    # Set all outputs to none (define output key while overwriting)
    def set_output_none(self):
        self.outputs["location"] = ""
        pass

    # Sets the service request message, sent to the service.
    def set_request(self):
        self._last_request = RequestWorldState.Request()

    # Sets the output (in relation to the response) (define output key while overwriting)
    # it should return True, if the node state should be SUCCEEDED after receiving the message and
    # False. if it should be in the FAILED state
    def set_outputs(self):
        response: RequestWorldState.Response = self._service_request_future.result()
        obj_with_matching_name: List[ObjectState] = [
            obj
            for obj in response.state.objects
            if obj.name == self.inputs["object_name"]
        ]
        if len(obj_with_matching_name) != 1:
            self.logerr("Found something other than one matching object!")
            return False
        obj = obj_with_matching_name[0]
        self.outputs["location"] = obj.parent
        return True
