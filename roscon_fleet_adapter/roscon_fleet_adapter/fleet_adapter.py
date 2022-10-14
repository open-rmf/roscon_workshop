# Copyright 2021 Open Source Robotics Foundation, Inc.
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
import argparse
import datetime
import yaml
import time
import threading
import datetime

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType
from rmf_fleet_msgs.msg import LaneRequest, ClosedLanes

from functools import partial

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

from .RobotClientAPI import RobotAPI

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


class FleetAdapter:

    def __init__(self, node):
        # global counter for command ids
        self.next_id = 0
        # Keep track of which map the robot is in
        self.last_map = {}
        self.cmd_ids = {}
        self.adapter = self.initialize_fleet(node)

    def initialize_fleet(self, node):
        # Make the easy full control
        easy_full_control = adpt.EasyFullControl.make()

        easy_full_control.node.use_sim_time()

        easy_full_control.start()
        time.sleep(1.0)

        def _check_completed(robot_name):
            if robot_name not in self.cmd_ids:
                return False
            return api.process_completed(robot_name, self.cmd_ids[robot_name])

        def _goal_completed(robot_name, remaining_time, request_replan):
            # Remaining time and request replan should be updated if needed
            completed = _check_completed(robot_name)
            api_res = api.navigation_remaining_duration(robot_name, self.cmd_ids[robot_name])
            if api_res is not None:
                remaining_time = datetime.timedelta(seconds=api_res)
            request_replan = api.requires_replan(robot_name)
            node.get_logger().info(f'Remaining time is {remaining_time}')
            return [completed, remaining_time, True]

        def _robot_state(robot_name):
            data = api.data(robot_name)
            if data is None or data['success'] is False:
                return None
            pos = data['data']['position']
            state = adpt.easy_full_control.RobotState(
                robot,
                robot_config['charger']['waypoint'],
                data['data']['map_name'],
                [pos['x'], pos['y'], pos['yaw']],
                data['data']['battery'])
            self.last_map[robot_name] = data['data']['map_name']
            return state

        def _navigate(robot_name, map_name, goal, update_handle):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            api.navigate(robot_name, cmd_id, goal, map_name)
            node.get_logger().info(f"Navigating robot {robot_name}")
            return adpt.easy_full_control.goal_completed_callback(partial(_goal_completed, robot_name))

        def _stop(robot_name):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            return api.stop(robot_name, cmd_id)

        def _dock(robot_name, dock_name, update_handle):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            api.start_process(robot_name, cmd_id, dock_name, self.last_map[robot_name])
            return adpt.easy_full_control.goal_completed_callback(partial(_goal_completed, robot_name))

        def _action_executor(robot_name: str,
                             category: str,
                             description: dict,
                             execution: adpt.robot_update_handle.ActionExecution):
            pass

        # Add the robots
        for robot in config_yaml['robots']:
            node.get_logger().info(f'Found robot {robot}')
            robot_config = config_yaml['robots'][robot]['rmf_config']
            success = False
            while success is False:
                state = _robot_state(robot)
                if state is None:
                    time.sleep(0.2)
                    continue
                success = True
                # Add robot to fleet
                easy_full_control.add_robot(
                    state,
                    partial(_robot_state, robot),
                    partial(_navigate, robot),
                    partial(_stop, robot),
                    partial(_dock, robot),
                    partial(_action_executor, robot))

        return easy_full_control 


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    # ROS 2 node for the command handle
    node = rclpy.node.Node(f'demo_command_handle')

    # Enable sim time for testing offline
    param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
    node.set_parameters([param])

    adapter = FleetAdapter(node)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
