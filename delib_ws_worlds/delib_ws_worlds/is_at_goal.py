"""
Checks if a goal has been reached and validates against the expected goal for that problem.
"""

import sys

from delib_ws_problem_interface.world_state import WorldState


def get_goal_state(problem_number):
    """Returns the goal state based on the problem number specified."""
    if problem_number in (1, 2, 3, 4):
        return (("objects.snacks0.parent", "table_tabletop"),)
    else:
        raise ValueError(f"No goal state for problem number: {problem_number}")


def main():
    ws = WorldState()
    expected_state = get_goal_state(ws.get_problem_number())
    is_at_goal = ws.check_state(expected_state)
    print(f"Is at goal: {is_at_goal}")
    sys.exit(0)
