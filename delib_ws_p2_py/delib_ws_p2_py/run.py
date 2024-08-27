import rclpy

from problem_interface.perform_action import PerformAction, ACTIONS
from problem_interface.world_state import WorldState
from delib_ws_p1.is_at_goal import get_goal_state

import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("delib_ws_p2_py")


def main():
    ws = WorldState()
    pa = PerformAction()
    assert not ws.get_state(
        get_goal_state()
    ), "Robot must not be at goal at the beginning"
    pa.do_action(ACTIONS.NAVIGATE, "hall_banana_farm_dining_room")
    pa.do_action(ACTIONS.OPEN, "hall_banana_farm_dining_room")
    pa.do_action(ACTIONS.NAVIGATE, "table_source")
    pa.do_action(ACTIONS.PICK, "banana0")
    pa.do_action(ACTIONS.NAVIGATE, "table_sink")
    pa.do_action(ACTIONS.PLACE, "banana0")
    assert ws.get_state(get_goal_state()), "Robot must be at goal at the end"
    logger.info("Goal reached")

    rclpy.shutdown()
