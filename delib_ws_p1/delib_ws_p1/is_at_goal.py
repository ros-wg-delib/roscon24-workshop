from problem_interface.world_state import WorldState

import sys

def main():
    ws = WorldState()
    expected_state = [
        ('objects.banana0.parent', 'table_sink_tabletop')
    ]
    is_at_goal = ws.get_state(expected_state)
    print(is_at_goal)
    sys.exit(0)