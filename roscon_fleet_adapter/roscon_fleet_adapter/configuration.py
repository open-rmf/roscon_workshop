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

import datetime

import rclpy
import rclpy.node

import rmf_adapter as adpt

import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph

def get_configuration(config_yaml, nav_graph_path, node):
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

    return configuration