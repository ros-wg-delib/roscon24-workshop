## @file
# Copyright (c) 2024, Sebastian Castro, The AI Institute
# SPDX-License-Identifier: BSD-3-Clause
##

"""
Solution implementations in Python.
"""

from delib_ws_problem_interface.perform_action import ACTIONS


def problem1():
    """Gets the Problem 1 solution"""
    return (
        (ACTIONS.NAVIGATE, "pantry_storage"),
        (ACTIONS.PICK, "snacks0"),
        (ACTIONS.NAVIGATE, "table"),
        (ACTIONS.PLACE, "snacks0"),
    )


def problem2():
    """Gets the Problem 2 solution"""
    return (
        (ACTIONS.NAVIGATE, "hall_dining_trash"),
        (ACTIONS.OPEN, "hall_dining_trash"),
        (ACTIONS.NAVIGATE, "dumpster"),
        (ACTIONS.OPEN, "dumpster"),
        (ACTIONS.NAVIGATE, "desk"),
        (ACTIONS.PICK, "waste0"),
        (ACTIONS.NAVIGATE, "dumpster"),
        (ACTIONS.PLACE, "waste0"),
        (ACTIONS.NAVIGATE, "bin"),
        (ACTIONS.PICK, "waste1"),
        (ACTIONS.NAVIGATE, "dumpster"),
        (ACTIONS.PLACE, "waste1"),
        (ACTIONS.CLOSE, "dumpster"),
        (ACTIONS.NAVIGATE, "hall_dining_trash"),
        (ACTIONS.CLOSE, "hall_dining_trash"),
    )


def problem3():
    """Gets the Problem 3 solution"""
    return (
        (ACTIONS.NAVIGATE, "pantry"),
        (ACTIONS.OPEN, "pantry"),
        (ACTIONS.DETECT, "bread"),
        (ACTIONS.PICK, "bread0"),
        (ACTIONS.NAVIGATE, "table"),
        (ACTIONS.PLACE, "bread0"),
        (ACTIONS.NAVIGATE, "fridge"),
        (ACTIONS.OPEN, "fridge"),
        (ACTIONS.DETECT, "butter"),
        (ACTIONS.PICK, "butter0"),
        (ACTIONS.NAVIGATE, "table"),
        (ACTIONS.PLACE, "butter0"),
        (ACTIONS.NAVIGATE, "fridge"),
        (ACTIONS.CLOSE, "fridge"),
        (ACTIONS.NAVIGATE, "pantry"),
        (ACTIONS.CLOSE, "pantry"),
    )


def problem4():
    """Gets the Problem 4 solution"""
    return (
        problem3()
        + (
            (ACTIONS.NAVIGATE, "hall_dining_closet"),
            (ACTIONS.OPEN, "hall_dining_closet"),
            (ACTIONS.NAVIGATE, "charger"),
        )
        + problem2()
    )
