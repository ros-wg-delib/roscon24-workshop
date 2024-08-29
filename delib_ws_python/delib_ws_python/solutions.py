"""
Solution implementations in Python.
"""

from delib_ws_problem_interface.perform_action import ACTIONS


def problem1():
    """Gets the Problem 1 solution"""
    return (
        (ACTIONS.NAVIGATE, "table_source"),
        (ACTIONS.PICK, "banana0"),
        (ACTIONS.NAVIGATE, "table_sink"),
        (ACTIONS.PLACE, "banana0"),
    )


def problem2():
    """Gets the Problem 2 solution"""
    return (
        (ACTIONS.NAVIGATE, "hall_banana_farm_dining_room"),
        (ACTIONS.OPEN, "hall_banana_farm_dining_room"),
        (ACTIONS.NAVIGATE, "table_source"),
        (ACTIONS.PICK, "banana0"),
        (ACTIONS.NAVIGATE, "table_sink"),
        (ACTIONS.PLACE, "banana0"),
    )


def problem3():
    """Gets the Problem 3 solution"""
    return tuple()  # TODO: Implement solution


def problem4():
    """Gets the Problem 4 solution"""
    return tuple()  # TODO: Implement solution
