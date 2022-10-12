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

import sys
import yaml
import argparse
from typing import Optional
from yaml import YAMLObject

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rmf_door_msgs.msg import DoorState, DoorRequest

from .DoorAPI import DoorAPI

'''
    The RosconDoorAdapter is a node which provide updates to Open-RMF, as well
    as handle incoming requests to control the integrated door, by calling the
    implemented functions in DoorAPI.
'''
class RosconDoorAdapter(Node):
    def __init__(self, args, config: YAMLObject):
        super().__init__('roscon_door_adapter')

        self.door_states = {}
        self.door_requests = {}
        self.door_config = config
        self.door_api = DoorAPI(self.door_config, self.get_logger())
        for door in config['doors']:
            self.door_requests[door] = None
            self.door_states[door] = self._door_state(door)

        self.door_state_pub = self.create_publisher(
            DoorState,
            'door_states',
            qos_profile=qos_profile_system_default)
        self.door_request_sub = self.create_subscription(
            DoorRequest,
            'door_requests',
            self.door_request_callback,
            qos_profile=qos_profile_system_default)
        self.update_timer = self.create_timer(0.5, self.update_callback)
        self.pub_state_timer = self.create_timer(1.0, self.publish_states)
        self.get_logger().info('Running RosconDoorAdapter')

    def update_callback(self):
        new_states = {}
        for door_name, door_state in self.door_states.items():
            new_state = self._door_state(door_name)
            new_states[door_name] = new_state
            if new_state is None:
                self.get_logger().error(
                    f'Unable to get new state from door {door_name}')
                continue

            door_request = self.door_requests[door_name]
            # No request to consider
            if door_request is None:
                continue

            # If all is done, set request to None
            if door_request.destination_floor == \
                    new_state.current_floor and \
                    new_state.door_state == DoorState.DOOR_OPEN:
                door_request = None
        self.door_states = new_states

    def _door_state(self, door_name) -> Optional[DoorState]:
        new_state = DoorState()
        new_state.door_time = self.get_clock().now().to_msg()
        new_state.door_name = door_name

        def _retrieve_fail_error(value_name: str):
            self.get_logger().error(f'Unable to retrieve {value_name}')
            return None

        door_state = self.door_api.door_state(door_name)
        if door_state is None:
            return _retrieve_fail_error('door_state')
        new_state.available_floors = door_state['available_floors']
        new_state.current_floor = door_state['current_floor']
        new_state.destination_floor = door_state['destination_floor']
        new_state.door_state= door_state['door_state']
        new_state.motion_state= door_state['motion_state']

        new_state.available_modes = [DoorState.MODE_HUMAN, DoorState.MODE_AGV]
        new_state.current_mode = DoorState.MODE_AGV

        door_request = self.door_requests[door_name]
        if door_request is not None:
            if door_request.request_type == \
                    DoorRequest.REQUEST_END_SESSION:
                new_state.session_id = ''
            else:
                new_state.session_id = door_request.session_id
        return new_state

    def publish_states(self):
        for door_name, door_state in self.door_states.items():
            if door_state is None:
                self.get_logger().info('No door state received for door'
                        f'{door_name}')
                continue
            self.door_state_pub.publish(door_state)

    def door_request_callback(self, msg):
        if msg.door_name not in self.door_states:
            return

        door_state = self.door_states[msg.door_name]
        '''
        if self.door_requests[msg.door_name] is not None:
            self.get_logger().info(
                'Door is currently busy with another request, try again later.')
            return

        '''
        if door_state is not None and \
                msg.destination_floor not in door_state.available_floors:
            self.get_logger().info(
                'Floor {} not available.'.format(msg.destination_floor))
            return

        if not self.door_api.command_door(msg.door_name, msg.destination_floor, msg.door_state):
            self.get_logger().error(
                f'Failed to send door to {msg.destination_floor}.')
            return

        self.get_logger().info(f'Requested door {msg.door_name}'
                f'to {msg.destination_floor}.')
        self.door_requests[msg.door_name] = msg


def main(argv=sys.argv):
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog='roscon_door_adapter',
        description='Roscon door adapter')
    parser.add_argument('-c', '--config', required=True, type=str)
    args = parser.parse_args(args_without_ros[1:])

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    rclpy.init()
    node = RosconDoorAdapter(args, config)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
