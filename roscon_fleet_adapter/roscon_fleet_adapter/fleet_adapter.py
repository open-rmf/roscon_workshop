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

from .RobotCommandHandle import RobotCommandHandle
from .RobotClientAPI import RobotAPI

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


class FleetAdapter:

    def __init__(self, config_yaml, nav_graph_path, node, use_sim_time):
        # global counter for command ids
        self.next_id = 0
        # Keep track of which map the robot is in
        self.last_map = {}
        self.cmd_ids = {}
        self.adapter = self.initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time)

    def initialize_fleet(self, config_yaml, nav_graph_path, node, use_sim_time):
        # Profile and traits
        fleet_config = config_yaml['rmf_fleet']
        profile = traits.Profile(geometry.make_final_convex_circle(
            fleet_config['profile']['footprint']),
            geometry.make_final_convex_circle(fleet_config['profile']['vicinity']))
        vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(*fleet_config['limits']['linear']),
            angular=traits.Limits(*fleet_config['limits']['angular']),
            profile=profile)
        vehicle_traits.differential.reversible = fleet_config['reversible']

        # Battery system
        voltage = fleet_config['battery_system']['voltage']
        capacity = fleet_config['battery_system']['capacity']
        charging_current = fleet_config['battery_system']['charging_current']
        battery_sys = battery.BatterySystem.make(
            voltage, capacity, charging_current)

        # Mechanical system
        mass = fleet_config['mechanical_system']['mass']
        moment = fleet_config['mechanical_system']['moment_of_inertia']
        friction = fleet_config['mechanical_system']['friction_coefficient']
        mech_sys = battery.MechanicalSystem.make(mass, moment, friction)

        # Power systems
        ambient_power_sys = battery.PowerSystem.make(
            fleet_config['ambient_system']['power'])
        tool_power_sys = battery.PowerSystem.make(
            fleet_config['tool_system']['power'])

        # Power sinks
        motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
        ambient_sink = battery.SimpleDevicePowerSink(
            battery_sys, ambient_power_sys)
        tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

        nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

        # Adapter
        fleet_name = fleet_config['name']
        adapter = adpt.Adapter.make(f'{fleet_name}_fleet_adapter')
        if use_sim_time:
            adapter.node.use_sim_time()
        assert adapter, ("Unable to initialize fleet adapter. Please ensure "
                         "RMF Schedule Node is running")
        adapter.start()
        time.sleep(1.0)

        node.declare_parameter('server_uri', rclpy.Parameter.Type.STRING)
        server_uri = node.get_parameter(
            'server_uri').get_parameter_value().string_value
        if server_uri == "":
            server_uri = None

        fleet_state_update_frequency = fleet_config['publish_fleet_state']
        fleet_state_update_dt = datetime.timedelta(seconds=1.0/fleet_state_update_frequency)
        # Account for battery drain
        drain_battery = fleet_config['account_for_battery_drain']
        #lane_merge_distance = fleet_config.get('lane_merge_distance', 0.1)
        recharge_threshold = fleet_config['recharge_threshold']
        recharge_soc = fleet_config['recharge_soc']
        finishing_request = fleet_config['task_capabilities']['finishing_request']
        node.get_logger().info(f"Finishing request: [{finishing_request}]")

        # Initialize robot API for this fleet
        prefix = 'http://' + fleet_config['fleet_manager']['ip'] + \
                 ':' + str(fleet_config['fleet_manager']['port'])
        api = RobotAPI(
            prefix,
            fleet_config['fleet_manager']['user'],
            fleet_config['fleet_manager']['password'])

        configuration = adpt.easy_full_control.Configuration(
            fleet_name=fleet_name,
            traits=vehicle_traits,
            graph=nav_graph,
            battery_system=battery_sys,
            motion_sink=motion_sink,
            ambient_sink=ambient_sink,
            tool_sink=tool_sink,
            recharge_threshold=recharge_threshold,
            recharge_soc=recharge_soc,
            account_for_battery_drain=drain_battery,
            action_categories=['loop', 'delivery'], # TODO check action categories
            finishing_request=finishing_request,
            server_uri=server_uri,
            max_delay=datetime.timedelta(10.0), # TODO max delay
            update_interval=fleet_state_update_dt)

        # Make the easy full control
        easy_full_control = adpt.EasyFullControl.make(configuration)

        easy_full_control.start()
        time.sleep(1.0)

        def _check_completed(robot_name):
            if robot_name not in self.cmd_ids:
                return False
            return api.process_completed(robot_name, self.cmd_ids[robot_name])

        def _goal_completed(robot_name, remaining_time, request_replan):
            request_replan = api.requires_replan(robot_name)
            remaining_time = api.navigation_remaining_duration(robot_name, self.cmd_ids[robot_name])
            node.get_logger().info(f"Checking goal completed for robot {robot_name}, time {remaining_time}")
            return api.process_completed(robot_name, self.cmd_ids[robot_name])

        def _robot_state(robot_name):
            data = api.data(robot_name)
            if data is None:
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
            return partial(_goal_completed, robot_name)

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
            return partial(_goal_completed, robot_name)

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

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    adapter = FleetAdapter(
        config_yaml,
        nav_graph_path,
        node,
        args.use_sim_time)

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
