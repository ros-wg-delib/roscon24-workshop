#!/usr/bin/env python

# Copyright 2024 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""FlexBE utility for loading PyRoboSim world structure."""

import os
import re
from collections import deque

from ament_index_python.packages import get_package_share_directory

from shapely.geometry import Point, Polygon

import yaml


def signed_distance_to_polygon(polygon, point):
    """
    Calculate the signed distance to polygon boundary.

    Use negative value for points inside the polygon.
    """
    # Calculate the distance from the point to the polygon boundary
    distance = point.distance(polygon)

    # Check if the point is inside the polygon
    if polygon.contains(point):
        return -distance  # Negative if inside
    else:
        return distance  # Positive if outside


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
                                                        Point(pose.position.x, pose.position.y)) for room in possible_rooms]
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
            rooms[room['name']] = Polygon(room['footprint']['coords'])

        print(f'Loaded {len(categories)} categories in {len(locations)} locations'
              f' in {len(hallway_graph)} ({len(rooms)}) rooms')
        WorldConfiguration.__world_data[world] = {'hallway_graph': hallway_graph,
                                                  'locations': locations,
                                                  'categories': categories,
                                                  'rooms': rooms}
