#!/usr/bin/env python3
# Copyright (c) 2024 - for information on the respective copyright owner
# see the NOTICE file

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Create a new empty solution package for the workshop."""

import logging
import argparse
import os
from typing import List

DELIBWS_PREFIX = 'delib_ws'
PROBLEM_PKG_PREFIX = f'{DELIBWS_PREFIX}_p'
SUPPORTED_TECHNOLOGIES = ['bt_cpp', 'ros_bt_py', 'flexbe', 'python']

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def _get_possible_problem_pkg_nums(repo_path: str) -> List[int]:
    # Get all packages that start with 'delib_ws_p'
    folders = [f for f in os.listdir(repo_path)
               if os.path.isdir(os.path.join(repo_path, f))]
    problem_pkgs = [name for name in folders
                    if name.startswith(PROBLEM_PKG_PREFIX)]
    problem_pkg_nums = []
    for pkg in problem_pkgs:
        potential_num_str = pkg.replace(PROBLEM_PKG_PREFIX, '')
        try:
            potential_num = int(potential_num_str)
            problem_pkg_nums.append(potential_num)
        except ValueError:
            continue
    return problem_pkg_nums


def main():
    parser = argparse.ArgumentParser(
        description='Create a new empty solution package for the workshop.')
    parser.add_argument('ws_path', type=str, help='The path to the workspace.')
    parser.add_argument('problem_num',
                        type=int,
                        help='The number of the problem to solve.')
    parser.add_argument('technology',
                        type=str,
                        help='The technology to use for the solution.',
                        choices=SUPPORTED_TECHNOLOGIES)
    args = parser.parse_args()

    assert os.path.exists(args.ws_path), \
        f'Workspace path {args.ws_path} must exist.'
    repo_path = os.path.join(args.ws_path, 'src', 'roscon24-workshop')
    assert os.path.join(repo_path, '.git'), \
        f'Workspace {args.ws_path} must contain the workshop repository.'
    logging.info('Repo path: %s', repo_path)

    available_problem_numbers = _get_possible_problem_pkg_nums(repo_path)
    assert args.problem_num in available_problem_numbers, \
        f'Problem number {args.problem_num} must be one of' \
        f' {available_problem_numbers}.'

    package_name = f'{PROBLEM_PKG_PREFIX}{args.problem_num}_{args.technology}'
    logger.info('Creating package %s ...', package_name)
    package_path = os.path.join(repo_path, package_name)
    assert not os.path.exists(package_path), \
        f'Package {package_name} must not exist yet.'

    template_pkg_name = f'{DELIBWS_PREFIX}_template_{args.technology}'
    logging.info('Copying files from %s to %s ...',
                 template_pkg_name, package_name)
    template_pkg_path = os.path.join(repo_path, template_pkg_name)
    assert os.path.exists(template_pkg_path), \
        f'Template package {template_pkg_name} must exist.'

    os.system(f'cp -r {template_pkg_path} {package_path}')

    for root, dirnames, files in os.walk(package_path):
        for dir in dirnames:
            # Rename directories
            new_dirname = dir.replace(template_pkg_name, package_name)
            if new_dirname != dir:
                os.rename(os.path.join(root, dir),
                          os.path.join(root, new_dirname))
        for file in files:
            # Rename files
            new_filename = file.replace(template_pkg_name, package_name)
            if new_filename != file:
                os.rename(os.path.join(root, file),
                          os.path.join(root, new_filename))
            # Replace package name in filecontent
            file_path = os.path.join(root, new_filename)
            with open(file_path, 'r') as f:
                filedata = f.read()
            new_filedata = filedata.replace(template_pkg_name, package_name)
            with open(file_path, 'w') as f:
                f.write(new_filedata)


if __name__ == '__main__':
    main()
