## @file
# Copyright (c) 2024, Sebastian Castro, The AI Institute
# SPDX-License-Identifier: BSD-3-Clause
##

import rclpy

from delib_ws_worlds.is_at_goal import get_goal_state
from delib_ws_python import solutions
from delib_ws_problem_interface.perform_action import PerformAction
from delib_ws_problem_interface.world_state import WorldState

import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("delib_ws_python")


def get_solution(problem_number):
    if problem_number == 1:
        return solutions.problem1()
    elif problem_number == 2:
        return solutions.problem2()
    elif problem_number == 3:
        return solutions.problem3()
    elif problem_number == 4:
        return solutions.problem4()
    else:
        raise ValueError(f"No solution implemented for problem {problem_number}")


def execute_solution(problem_number):
    solution = get_solution(problem_number)

    pa = PerformAction()
    for action, argument in solution:
        pa.do_action(action, argument)


def main():
    ws = WorldState()
    expected_state = get_goal_state(ws.get_problem_number())

    assert not ws.check_state(
        expected_state
    ), "Robot must not be at goal at the beginning"

    execute_solution(ws.get_problem_number())

    assert ws.check_state(expected_state), "Robot must be at goal at the end"
    logger.info("Goal reached")

    rclpy.shutdown()
