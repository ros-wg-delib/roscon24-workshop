from problem_interface.world_state import WorldState

import sys


def get_goal_state():
    return [("objects.banana0.parent", "table_sink_tabletop")]


def main():
    ws = WorldState()
    is_at_goal = ws.get_state(get_goal_state())
    print(is_at_goal)
    sys.exit(0)
