#!/usr/bin/env python

# Copyright 2024 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""FlexBE utility for loading PyRoboSim world structure."""

import os
import re
from collections import deque

from ament_index_python.packages import get_package_share_directory

import numpy as np

import yaml


# Helper function: Distance from point to a line segment
def point_to_line_distance(point, line_start, line_end):
    """Calculate distance of point to line."""
    # Convert to NumPy arrays
    point = np.array(point)
    line_start = np.array(line_start)
    line_end = np.array(line_end)

    # Line segment vector
    line_vec = line_end - line_start
    # Vector from start of line to the point
    point_vec = point - line_start

    # Project the point onto the line (normalized)
    line_len = np.linalg.norm(line_vec)
    line_unitvec = line_vec / line_len
    projection = np.dot(point_vec, line_unitvec)

    # Clamp the projection between 0 and line length to handle the endpoints
    projection = np.clip(projection, 0, line_len)

    # Find the nearest point on the line segment
    nearest = line_start + projection * line_unitvec

    # Return the Euclidean distance between the point and the nearest point on the line segment
    return np.linalg.norm(nearest - point)


# Helper function:
def is_point_inside_polygon(point, polygon):
    """Check if a point is inside the polygon using ray-casting."""
    x, y = point
    inside = False
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]

        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside

    return inside


# Main helper function
def signed_distance_to_polygon(polygon, point):
    """Calculate the signed distance."""
    # Step 1: Calculate the distance to each edge
    min_distance = float('inf')

    for i in range(len(polygon)):
        line_start = polygon[i]
        line_end = polygon[(i + 1) % len(polygon)]  # Wrap around to first vertex

        distance = point_to_line_distance(point, line_start, line_end)
        min_distance = min(min_distance, distance)

    # Step 2: Check if the point is inside the polygon
    inside = is_point_inside_polygon(point, polygon)

    # Step 3: Return the signed distance
    if inside:
        return -min_distance  # Negative if inside
    else:
        return min_distance  # Positive if outside


class WorldConfiguration:
    """
    FlexBE utility to load and allow queries regarding the pyrobosim world definition.

    Parameters
    ----------
    -- world_definition    world definition (default='world4')
    -- package_name        package name (default='delib_ws_worlds')
    -- sub_folder          subfolder for world definition(default='worlds')
    -- state_topic         robot state topic name (default= 'robot/robot_state')

    """

    __world_data = {}  # World data shared by all instances

    def __init__(self,
                 world='world4',
                 package_name='delib_ws_worlds',
                 sub_folder='worlds'):

        if world not in WorldConfiguration.__world_data:
            WorldConfiguration._load_world_data(world, package_name, sub_folder)
        else:
            print(f"'{world}' is already initialized!")

    def get_rooms(self, value, world, pose=None):
        """Return tuple of possible rooms."""
        if world not in WorldConfiguration.__world_data:
            raise KeyError(f"'{world}' has not been initialized")

        world_data = WorldConfiguration.__world_data[world]

        if value.startswith('hall_'):
            # This is a door between two rooms
            possible_rooms = tuple(value[len('hall_'):].split('_'))
            print(f'hallway between rooms {possible_rooms}')
            if pose is not None:
                distances = [signed_distance_to_polygon(world_data['rooms'][room],
                                                        (pose.position.x, pose.position.y)) for room in possible_rooms]
                possible_rooms = (possible_rooms[0], ) if distances[0] < distances[1] else (possible_rooms[1], )
                print(f'  choosing {possible_rooms} based on pose!')
            return possible_rooms

        if value in world_data['hallway_graph']:
            return (value,)  # is a room

        endings = ['_dock', '_tabletop', '_disposal', '_storage']
        locations = [(next((value.replace(ending, '') for ending in endings if value.endswith(ending)), value))]

        category = re.sub(r'\d+$', '', value)  # strip trailing quantifier (e.g. 'bread2')
        if category in world_data['categories']:
            # Assumes category and location are unique
            locations = world_data['categories'][category]
            print(f"  '{value}' can be found in '{locations}'")

        rooms = [world_data['locations'][loc] for loc in locations if loc in world_data['locations']]
        if len(rooms) > 0:
            print(f"  '{value}' is in '{rooms}'")
            return tuple(rooms)

        raise ValueError(f"'{value}' cannot be found in '{world}'")

    def get_hallway(self, current_room, next_room, world):
        """Return hallway name that connects two rooms."""
        return WorldConfiguration.__world_data[world]['hallway_graph'][current_room][next_room]

    def search(self, start, goal, world):
        """Do breadth first search which finds optimal for constant edge costs."""
        if world not in WorldConfiguration.__world_data:
            raise KeyError(f"'{world}' has not been initialized")

        graph = WorldConfiguration.__world_data[world]['hallway_graph']

        if start not in graph or goal not in graph:
            print(f"Invalid rooms for search from '{start}' to '{goal}' in '{world}'")
            return None

        # Queue for BFS (assumes small graph)
        queue = deque([start])
        # Dictionary to track distances and the shortest path
        distances = {start: 0}
        previous_nodes = {start: None}

        while queue:
            current_node = queue.popleft()

            # If we reached the goal, construct the path
            if current_node == goal:
                path = []
                while current_node is not None:
                    path.insert(0, current_node)
                    current_node = previous_nodes[current_node]
                print(f"Found path from '{start}' to '{goal}' : {path}")
                return path

            # Explore neighbors
            for neighbor in graph[current_node]:
                if neighbor not in distances:
                    queue.append(neighbor)
                    distances[neighbor] = distances[current_node] + 1
                    previous_nodes[neighbor] = current_node

        return None  # No path found

    @classmethod
    def _load_world_data(cls, world, package_name, sub_folder):
        """
        Load world data from pyrobosim world definition.

        Use class data so that multiple states only need to load one shared set of data.
        """
        # If this data does not exist, the behavior will throw exception on construction!
        package_path = get_package_share_directory(package_name)
        world_file_path = os.path.join(package_path, sub_folder, world + '.yaml')
        print(f"Attempting to load '{world_file_path}' ...", flush=True)
        with open(world_file_path) as fin:
            world_data = yaml.safe_load(fin)

        hallways = world_data['hallways']
        hallway_graph = {}
        for hallway in hallways:
            room_start = hallway['room_start']
            room_end = hallway['room_end']
            hallway_name = f'hall_{room_start}_{room_end}'
            # Add the edge in both directions since the hallways are bidirectional
            if room_start not in hallway_graph:
                hallway_graph[room_start] = {}
            if room_end not in hallway_graph:
                hallway_graph[room_end] = {}

            hallway_graph[room_start][room_end] = hallway_name
            hallway_graph[room_end][room_start] = hallway_name

        locations = {}
        for location in world_data['locations']:
            locations[location['name']] = location['parent']  # in room

        categories = {}
        for obj in world_data['objects']:
            category = obj['category']
            if category not in categories:
                categories[category] = []
            categories[category].append(obj['parent'])  # possible locations

        rooms = {}
        for room in world_data['rooms']:
            if room['footprint']['type'] != 'polygon':
                print(f"Invalid footprint type '{room['footprint']['type']}' for '{room['name']}'")
                continue
            rooms[room['name']] = room['footprint']['coords']

        print(f'Loaded {len(categories)} categories in {len(locations)} locations'
              f' in {len(hallway_graph)} ({len(rooms)}) rooms')
        WorldConfiguration.__world_data[world] = {'hallway_graph': hallway_graph,
                                                  'locations': locations,
                                                  'categories': categories,
                                                  'rooms': rooms}
