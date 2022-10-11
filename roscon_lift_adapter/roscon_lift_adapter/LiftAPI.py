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

from rmf_lift_msgs.msg import LiftState
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
    The LiftAPI class is a wrapper for API calls to the lift. Here users are
    expected to fill up the implementations of functions which will be used by
    the LiftAdapter. For example, if your lift has a REST API, you will need to
    make http request calls to the appropriate endpints within these functions.
'''
class LiftAPI:
    # The constructor accepts a safe loaded YAMLObject, which should contain all
    # information that is required to run any of these API calls.
    def __init__(self, config: YAMLObject, logger: RcutilsLogger):
        self.config = config
        self.prefix = 'http://' + config['lift_manager']['ip'] + \
                ':' + str(config['lift_manager']['port'])
        self.logger = logger
        self.timeout = 1.0

    def lift_state(self, lift_name) -> Optional[dict]:
        ''' Returns the lift state or None if the query failed'''
        try:
            response = requests.get(self.prefix +
                    f'/open-rmf/demo-lift/lift_state?lift_name={lift_name}',
                    timeout=self.timeout)
        except Exception as err:
            self.logger.info(f'{err}')
            return None
        if response.status_code != 200 or response.json()['success'] is False:
            return None
        return response.json()['data']

    def command_lift(self, lift_name, floor: str, door_state: int) -> bool:
        ''' Sends the lift cabin to a specific floor and opens all available
            doors for that floor. Returns True if the request was sent out
            successfully, False otherwise'''
        data = {'floor': floor, 'door_state': door_state}
        response = requests.post(self.prefix +
                f'/open-rmf/demo-lift/lift_request?lift_name={lift_name}',
                timeout=self.timeout,
                json=data)
        print(response)
        if response.status_code != 200 or response.json()['success'] is False:
            return False
        return True
