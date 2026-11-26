#!/usr/bin/env python3
"""The collision monitor as an rclcpp component in a Nav2-style container.

    smoke_component.py

Starts `component_container_isolated` under the name `nav2_container`, as
nav2_bringup does, and loads into it, with the parameters in the load
request (a component does not see the container's command line):

  - `roslean_collision_monitor::CollisionMonitorNode` as `collision_monitor`;
  - upstream's `nav2_collision_monitor::CollisionMonitor` under another name,
    to show that the Lean runtime and Nav2's own nodes share one process;
  - `nav2_lifecycle_manager::LifecycleManager`, which configures and
    activates `collision_monitor` and forms the bond with it.

Then, on the wall clock: a command passes; a scan with points in the stop
zone and a command give zero and a state message; the manager reports the
node active and bonded. Every wait has a deadline.

What this does not show: a component has no `main` of its own, so the
handling of an exception that leaves a callback (SPEC D15: a `/clock`
message with a negative time) is the container's, not this node's.
"""

import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from composition_interfaces.srv import LoadNode
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import CollisionMonitorState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def pv(value):
    v = ParameterValue()
    if isinstance(value, bool):
        v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
    elif isinstance(value, int):
        v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, value
    elif isinstance(value, float):
        v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, value
    elif isinstance(value, str):
        v.type, v.string_value = ParameterType.PARAMETER_STRING, value
    else:
        v.type, v.string_array_value = ParameterType.PARAMETER_STRING_ARRAY, list(value)
    return v


def params(d):
    return [Parameter(name=k, value=pv(v)) for k, v in d.items()]


CM_PARAMS = {
    'base_frame_id': 'base_footprint',
    'base_shift_correction': False,
    'transform_tolerance': 0.1,
    'source_timeout': 0.0,
    'stop_pub_timeout': 100.0,
    'state_topic': 'collision_monitor_state',
    'polygons': ['Stop'],
    'Stop.type': 'circle',
    'Stop.radius': 0.5,
    'Stop.action_type': 'stop',
    'Stop.min_points': 1,
    'observation_sources': ['scan'],
    'scan.type': 'scan',
    'scan.topic': 'scan',
}


class Driver(Node):

    def __init__(self):
        super().__init__('component_driver')
        self.out = []
        self.states = []
        self.create_subscription(Twist, 'cmd_vel', self.out.append, 10)
        self.create_subscription(
            CollisionMonitorState, 'collision_monitor_state', self.states.append, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, srv_type, name, request, timeout=20.0):
        client = self.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(request)
            end = time.monotonic() + timeout
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            return future.result() if future.done() else None
        finally:
            self.destroy_client(client)

    def load(self, package, plugin, name, parameters):
        req = LoadNode.Request(package_name=package, plugin_name=plugin, node_name=name,
                               parameters=params(parameters))
        res = self.call(LoadNode, '/nav2_container/_container/load_node', req)
        return res is not None and res.success, ('' if res is None else res.error_message)

    def state(self, node):
        res = self.call(GetState, f'/{node}/get_state', GetState.Request(), timeout=5.0)
        return None if res is None else res.current_state.label

    def wait_state(self, node, label, timeout=30.0):
        end = time.monotonic() + timeout
        got = None
        while time.monotonic() < end:
            got = self.state(node)
            if got == label:
                break
            self.spin(0.2)
        return got

    def command(self, x):
        self.out.clear()
        msg = Twist()
        msg.linear.x = x
        end = time.monotonic() + 5.0
        while not self.out and time.monotonic() < end:
            self.cmd_pub.publish(msg)
            self.spin(0.2)
        return self.out[-1].linear.x if self.out else None


def main():
    # Upstream's component is loaded beside the node: it must be upstream's.
    from ament_index_python.packages import get_package_prefix
    from cm_harness import refuse_overlay
    refuse_overlay('nav2_collision_monitor', get_package_prefix('nav2_collision_monitor'))
    container = sys.argv[1] if len(sys.argv) > 1 else 'component_container_isolated'
    log = tempfile.NamedTemporaryFile(prefix='roslean_cm_component_', suffix='.log', delete=False)
    proc = subprocess.Popen(
        ['ros2', 'run', 'rclcpp_components', container, '--ros-args', '-r',
         '__node:=nav2_container'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    d = Driver()
    bad = 0

    def check(ok, what):
        nonlocal bad
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  {what}", flush=True)

    try:
        ok, err = d.load('roslean_collision_monitor',
                         'roslean_collision_monitor::CollisionMonitorNode', 'collision_monitor',
                         CM_PARAMS)
        check(ok, f'the node is loaded into {container} as a component {err}')
        ok, err = d.load('nav2_collision_monitor', 'nav2_collision_monitor::CollisionMonitor',
                         'upstream_cm', {**CM_PARAMS, 'cmd_vel_in_topic': 'up_in',
                                         'cmd_vel_out_topic': 'up_out', 'state_topic': 'up_state'})
        check(ok, f"upstream's collision monitor is loaded into the same process {err}")
        check(d.state('collision_monitor') == 'unconfigured', 'the node is unconfigured')
        ok, err = d.load('nav2_lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager',
                         'lifecycle_manager_cm',
                         {'autostart': True, 'node_names': ['collision_monitor', 'upstream_cm'],
                          'bond_timeout': 4.0})
        check(ok, f"Nav2's lifecycle manager is loaded into the same process {err}")
        check(d.wait_state('collision_monitor', 'active') == 'active',
              'the manager configures and activates the node')
        check(d.wait_state('upstream_cm', 'active') == 'active',
              "and upstream's node after it (the manager goes on only when the bond has formed)")
        d.spin(1.0)
        check(d.command(0.25) == 0.25, 'no scan, source_timeout 0: a command passes')
        scan = LaserScan()
        scan.header.frame_id = 'base_footprint'
        scan.header.stamp = d.get_clock().now().to_msg()
        scan.angle_increment = 0.1
        scan.range_min, scan.range_max = 0.05, 10.0
        scan.ranges = [0.2, 0.2]
        for _ in range(5):
            d.scan_pub.publish(scan)
            d.spin(0.1)
        check(d.command(0.25) == 0.0, 'two points in the stop zone: zero')
        check(any(m.polygon_name == 'Stop' and m.action_type == 1 for m in d.states),
              'and the state message (Stop, STOP)')
        d.spin(5.0)
        with open(log.name) as f:
            text = f.read()
        check('Server collision_monitor connected with bond' in text,
              'the manager logs that the node is connected with bond')
        check('Managed nodes are active' in text, 'the manager logs "Managed nodes are active"')
        check('IS DOWN' not in text and proc.poll() is None,
              'five seconds later the bond holds and the container runs')
    finally:
        d.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=10)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    print(f'{bad} failure(s); container log in {log.name}')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
