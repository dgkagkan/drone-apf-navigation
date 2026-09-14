"""Exercise the real mapper with synthetic scans; never launch or command drones."""

import os
import signal
import struct
import subprocess
import tempfile
import time

from drone_interfaces.msg import SwarmDroneState, SwarmMapCloud, SwarmState
from octomap_msgs.msg import Octomap
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField


MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MapProbe:
    def __init__(self, node):
        self.node = node
        self.latest = {}
        self.counts = {}
        self.subscriptions = []
        self.publishers = {}
        self.states = {}
        self.state_pub = node.create_publisher(SwarmState, '/swarm/state', MAP_QOS)
        self.subscribe('global', '/swarm/octomap_point_cloud_centers')

    def subscribe(self, name, topic, message_type=PointCloud2):
        def receive(message):
            self.latest[name] = message
            self.counts[name] = self.counts.get(name, 0) + 1

        self.subscriptions.append(
            self.node.create_subscription(message_type, topic, receive, MAP_QOS))

    def wait(self, predicate, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f'Map condition timed out; received: {list(self.latest)}')

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish_state(self):
        self.state_pub.publish(SwarmState(drones=list(self.states.values())))

    def register(self, drone):
        self.states[drone] = SwarmDroneState(
            drone_id=drone, boot_id='initial', registered=True, connected=True,
            has_lidar=True)
        self.publishers[drone] = self.node.create_publisher(
            SwarmMapCloud, f'/swarm/{drone}/map_cloud', qos_profile_sensor_data)
        self.publish_state()
        self.wait(lambda: self.publishers[drone].get_subscription_count() == 1)

    def scan(self, drone, points, max_range=30.0):
        message = SwarmMapCloud(drone_id=drone, max_range_m=max_range)
        message.header.frame_id = 'map'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.cloud.header = message.header
        message.sensor_origin.z = 1.1
        message.cloud.height = 1
        message.cloud.width = len(points)
        message.cloud.point_step = 12
        message.cloud.row_step = 12 * len(points)
        message.cloud.fields = [
            PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(('x', 'y', 'z'))]
        message.cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        self.publishers[drone].publish(message)
        return message

    def points(self, name):
        message = self.latest.get(name)
        if message is None:
            return None
        return {
            tuple(round(value, 2) for value in struct.unpack_from('<fff', message.data, offset))
            for offset in range(0, len(message.data), message.point_step)}

    def colors(self):
        message = self.latest['visualization']
        rgb_offset = next(field.offset for field in message.fields if field.name == 'rgb')
        return {
            tuple(round(value, 2) for value in struct.unpack_from('<fff', message.data, offset)):
            struct.unpack_from('<I', message.data, offset + rgb_offset)[0] & 0xffffff
            for offset in range(0, len(message.data), message.point_step)}


@pytest.fixture
def probe(request):
    executable = os.environ.get('MAPPER_EXECUTABLE')
    if not executable:
        pytest.skip('Run through colcon test to select the built mapper and isolated ROS domain')
    assert os.environ.get('ROS_DOMAIN_ID') == '191'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    rclpy.init()
    node = rclpy.create_node('map_integration_probe')
    process = None
    output = tempfile.TemporaryFile(mode='w+')
    try:
        # Do not inject test state into another mapper already using this domain.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not node.count_subscribers('/swarm/state'), 'Test domain is already in use'
        parameters = getattr(request, 'param', {})
        arguments = [
            executable, '--ros-args',
            '-p', 'publish_rate_hz:=5.0',
            '-p', 'dynamic_obstacle_timeout_sec:=30.0',
            '-p', 'static_confirmation_sec:=60.0',
            '-p', 'static_confirmation_hits:=100',
            '-p', 'visualization_min_z_m:=0.0',
            '-p', 'visualization_max_z_m:=5.0',
        ]
        for name, value in parameters.items():
            arguments.extend(['-p', f'{name}:={value}'])
        process = subprocess.Popen(arguments, stdout=output, stderr=output)
        driver = MapProbe(node)
        driver.wait(lambda: driver.state_pub.get_subscription_count() == 1)
        yield driver
        assert process.poll() is None, 'Mapper exited unexpectedly'
    finally:
        shutdown_timed_out = False
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
                shutdown_timed_out = True
        output.seek(0)
        print(output.read())
        output.close()
        node.destroy_node()
        rclpy.shutdown()
        if shutdown_timed_out:
            pytest.fail('Mapper failed to join its workers on shutdown')


def test_global_and_n_locals_with_late_subscribers_and_session_reset(probe):
    shared = (4.1, 0.1, 1.1)
    unique = [(4.1, 2.1, 1.1), (6.1, -2.1, 1.1), (2.1, 4.1, 1.1)]
    for drone in ('one', 'two', 'three'):
        probe.register(drone)
    probe.scan('one', [shared, unique[0]])
    probe.scan('two', [shared, unique[1]])
    repeated = probe.scan('three', [unique[2]])
    expected = {(4.25, 0.25, 1.25), (4.25, 2.25, 1.25),
                (6.25, -2.25, 1.25), (2.25, 4.25, 1.25)}
    probe.wait(lambda: probe.points('global') == expected)

    # Join after geometry stopped changing. Demand-driven publications must
    # still deliver all N local maps and both serialized global representations.
    for drone in ('one', 'two', 'three'):
        probe.subscribe(drone, f'/swarm/mapping/{drone}/occupied_voxels')
    probe.subscribe('binary', '/swarm/octomap_binary', Octomap)
    probe.subscribe('full', '/swarm/octomap_full', Octomap)
    probe.subscribe('visualization', '/swarm/mapping_visualization')
    probe.wait(lambda: all(
        name in probe.latest
        for name in ('one', 'two', 'three', 'binary', 'full', 'visualization')))
    assert probe.points('visualization') == expected
    colors = probe.colors()
    assert len({colors[point] for point in (
        (4.25, 2.25, 1.25), (6.25, -2.25, 1.25), (2.25, 4.25, 1.25))}) == 3
    assert len(probe.points('one')) == 2
    assert len(probe.points('two')) == 2
    assert len(probe.points('three')) == 1
    assert probe.latest['binary'].data
    assert probe.latest['full'].data
    probe.spin_for(0.3)
    count = probe.counts['global']
    for _ in range(10):
        probe.publishers['three'].publish(repeated)
        probe.spin_for(0.03)
    probe.spin_for(0.4)
    assert probe.counts['global'] == count, 'Idle geometry was unnecessarily republished'

    probe.states['one'].boot_id = 'new-session'
    probe.publish_state()
    probe.wait(lambda: probe.points('one') == set())
    assert probe.points('global') == expected
    # Local ownership changes recolor geometry even when global occupancy is idle.
    probe.wait(lambda: probe.colors()[(4.25, 2.25, 1.25)] == 0x00ffff)
    assert probe.points('visualization') == expected
    # A session reset clears only that local diagnostic map. The source-neutral
    # global voxel is removed when a new ray actually observes that space free.
    probe.scan('one', [(8.1, 4.1, 1.1)])
    expected.remove((4.25, 2.25, 1.25))
    expected.add((8.25, 4.25, 1.25))
    probe.wait(lambda: probe.points('global') == expected)
    probe.wait(lambda: probe.points('one') == {(8.25, 4.25, 1.25)})


@pytest.mark.parametrize('probe', [
    {
        'remove_submap_on_disconnect': 'true',
        'dynamic_obstacle_timeout_sec': '1.5',
    },
    {
        'submap_timeout_sec': '1.0',
        'dynamic_obstacle_timeout_sec': '1.5',
    },
], indirect=True)
def test_disconnected_transient_expires_from_global_and_serialized_tree(probe):
    probe.register('one')
    probe.subscribe('full', '/swarm/octomap_full', Octomap)
    probe.scan('one', [(4.1, 0.1, 1.1)])
    probe.wait(lambda: probe.points('global') == {(4.25, 0.25, 1.25)})
    probe.wait(lambda: 'full' in probe.latest and bool(probe.latest['full'].data))
    probe.states['one'].connected = False
    probe.publish_state()
    probe.wait(lambda: probe.points('global') == set())
    probe.wait(lambda: not probe.latest['full'].data)


def test_free_ray_from_another_drone_clears_old_global_obstacle(probe):
    probe.subscribe('visualization', '/swarm/mapping_visualization')
    probe.register('one')
    probe.register('two')
    probe.scan('one', [(4.1, 0.1, 1.1)])
    probe.wait(lambda: probe.points('global') == {(4.25, 0.25, 1.25)})

    probe.scan('two', [(8.1, 0.1, 1.1)])
    probe.wait(lambda: probe.points('global') == {(8.25, 0.25, 1.25)})
    probe.wait(lambda: probe.points('visualization') == {(8.25, 0.25, 1.25)})


def test_retained_local_survives_reconnect(probe):
    probe.register('one')
    probe.scan('one', [(4.1, 2.1, 1.1)])
    expected = {(4.25, 2.25, 1.25)}
    probe.wait(lambda: probe.points('global') == expected)
    probe.states['one'].connected = False
    probe.publish_state()
    probe.wait(lambda: probe.publishers['one'].get_subscription_count() == 0)
    assert probe.points('global') == expected
    probe.states['one'].connected = True
    probe.publish_state()
    probe.wait(lambda: probe.publishers['one'].get_subscription_count() == 1)
    probe.scan('one', [(4.1, -2.1, 1.1)])
    expected.add((4.25, -2.25, 1.25))
    probe.wait(lambda: probe.points('global') == expected)


@pytest.mark.parametrize('probe', [{'dynamic_obstacle_timeout_sec': '0.0'}], indirect=True)
def test_sparse_floor_and_local_color_survive_gap_and_isolated_miss(probe):
    probe.register('one')
    probe.subscribe('one', '/swarm/mapping/one/occupied_voxels')
    probe.subscribe('visualization', '/swarm/mapping_visualization')
    floor = (4.25, 0.25, 0.25)
    probe.scan('one', [(4.1, 0.1, 0.1)])
    probe.wait(lambda: probe.points('one') == {floor})
    probe.wait(lambda: probe.points('visualization') == {floor})
    color = probe.colors()[floor]
    assert color != 0x0033ff  # Height color at z=0.25 with test bounds [0, 5].

    # Beyond the old two-second deadline, with no new measurement of the floor.
    probe.spin_for(2.3)
    assert probe.points('global') == {floor}
    # Process another scan too, exercising the local worker's decay path.
    probe.scan('one', [(4.1, 2.1, 1.1)])
    probe.wait(lambda: probe.points('one') == {floor, (4.25, 2.25, 1.25)})
    probe.wait(lambda: probe.points('visualization') == probe.points('one'))
    assert probe.colors()[floor] == color

    # One grazing/noisy miss through the floor voxel must not replace its local
    # color with the height color. These are synthetic rays, not an invented ground plane.
    probe.scan('one', [(8.1, 0.1, -0.9)])
    farther = (8.25, 0.25, -0.75)
    probe.wait(lambda: farther in (probe.points('one') or set()))
    probe.wait(lambda: farther in (probe.points('visualization') or set()))
    assert floor in probe.points('one')
    assert probe.colors()[floor] == color

    # Another drone's free ray must still clear that now-weak global voxel;
    # its old local cache must not restore the globally cleared geometry.
    probe.register('two')
    probe.scan('two', [(8.1, 0.1, -0.9)])
    probe.wait(lambda: floor not in probe.points('global'))
    probe.wait(lambda: floor not in probe.points('visualization'))


@pytest.mark.parametrize('probe', [{
    'dynamic_obstacle_timeout_sec': '0.0',
    'visualization_min_z_m': '0.25',
    'visualization_max_z_m': '5.25',
}], indirect=True)
def test_global_height_colors_preserve_local_priority_and_geometry(probe):
    probe.register('one')
    probe.subscribe('visualization', '/swarm/mapping_visualization')
    probe.scan('one', [(4.1, 2.1, 0.1), (4.1, -2.1, 2.6),
                       (4.1, 4.1, 5.1), (2.1, 4.1, 2.6)])
    expected = {
        (4.25, 2.25, 0.25): 0x0000ff,
        (4.25, -2.25, 2.75): 0x00ff00,
        (4.25, 4.25, 5.25): 0xff0000,
        (2.25, 4.25, 2.75): 0x00ff00,
    }
    probe.wait(lambda: probe.points('visualization') == set(expected))
    assert len(set(probe.colors().values())) == 1, 'Local color must override all heights'
    probe.states['one'].boot_id = 'reset-local-only'
    probe.publish_state()
    probe.wait(lambda: probe.colors() == expected)
    assert probe.points('global') == set(expected)

    # New height extrema must not rescale/recolor existing global geometry.
    probe.scan('one', [(20.1, 15.1, 20.1)], max_range=50.0)
    probe.wait(lambda: len(probe.colors()) == 5)
    assert all(probe.colors()[point] == color for point, color in expected.items())
