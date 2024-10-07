## @file
# Copyright (c) 2024, Sebastian Castro, The AI Institute
# SPDX-License-Identifier: BSD-3-Clause
##

"""
Checks if a goal has been reached and validates against the expected goal for that problem.
"""

import sys

from delib_ws_problem_interface.world_state import WorldState


def get_goal_state(problem_number):
    """Returns the goal state based on the problem number specified."""
    if problem_number == 1:
        return (("objects.snacks0.parent", "table_tabletop"),)
    elif problem_number == 2:
        return (
            ("objects.waste0.parent", "dumpster_disposal"),
            ("objects.waste1.parent", "dumpster_disposal"),
            ("locations.dumpster.is_open", False),
        )
    elif problem_number == 3:
        return (
            ("objects.bread0.parent", "table_tabletop"),
            ("objects.butter0.parent", "table_tabletop"),
            ("locations.fridge.is_open", False),
            ("locations.pantry.is_open", False),
        )
    elif problem_number == 4:
        return get_goal_state(2) + get_goal_state(3)
    else:
        raise ValueError(f"No goal state for problem number: {problem_number}")


def main():
    ws = WorldState()
    expected_state = get_goal_state(ws.get_problem_number())
    is_at_goal = ws.check_state(expected_state)
    print(f"Is at goal: {is_at_goal}")
    sys.exit(0)
