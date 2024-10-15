from pyrobosim_msgs.msg import ExecutionResult

from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"result": ExecutionResult},
        outputs={},
        max_children=0,
    )
)
class PyrobosimActionSuccessful(Leaf):
    """
    Check if a pyrobosim ExecutionResult was a success.

    If true the node will succeed, otherwise it will return a failure.
    """

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.input["result"]:
            if self.input["result"].status == ExecutionResult.SUCCESS:
                return NodeMsg.SUCCEEDED
            else:
                return NodeMsg.FAILED
        else:
            return NodeMsg.RUNNING

    def _do_untic(self):
        return NodeMsg.IDLE

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        return NodeMsg.SHUTDOWN
