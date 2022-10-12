#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from __future__ import annotations

import enum
import sys

import requests
from yaml import YAMLObject
from typing import Optional

from rmf_door_msgs.msg import DoorState
from rclpy.impl.rcutils_logger import RcutilsLogger


class DoorState(enum.IntEnum):
    CLOSED = 0
    MOVING = 1
    OPEN = 2


class MotionState(enum.IntEnum):
    STOPPED = 0
    UP = 1
    DOWN = 2
    UNKNOWN = 3


'''
    The DoorAPI class is a wrapper for API calls to the door. Here users are
    expected to fill up the implementations of functions which will be used by
    the DoorAdapter. For example, if your door has a REST API, you will need to
    make http request calls to the appropriate endpints within these functions.
'''
class DoorAPI:
    # The constructor accepts a safe loaded YAMLObject, which should contain all
    # information that is required to run any of these API calls.
    def __init__(self, config: YAMLObject, logger: RcutilsLogger):
        self.config = config
        self.prefix = 'http://' + config['door_manager']['ip'] + \
                ':' + str(config['door_manager']['port'])
        self.logger = logger
        self.timeout = 1.0

    def door_state(self, door_name) -> Optional[dict]:
        ''' Returns the door state or None if the query failed'''
        try:
            response = requests.get(self.prefix +
                    f'/open-rmf/demo-door/door_state?door_name={door_name}',
                    timeout=self.timeout)
        except Exception as err:
            self.logger.info(f'{err}')
            return None
        if response.status_code != 200 or response.json()['success'] is False:
            return None
        return response.json()['data']

    def command_door(self, door_name, floor: str, door_state: int) -> bool:
        ''' Sends the door cabin to a specific floor and opens all available
            doors for that floor. Returns True if the request was sent out
            successfully, False otherwise'''
        data = {'floor': floor, 'door_state': door_state}
        response = requests.post(self.prefix +
                f'/open-rmf/demo-door/door_request?door_name={door_name}',
                timeout=self.timeout,
                json=data)
        print(response)
        if response.status_code != 200 or response.json()['success'] is False:
            return False
        return True
