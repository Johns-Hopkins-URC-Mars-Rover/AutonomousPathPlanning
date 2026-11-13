#!/usr/bin/env python3
"""Verify a running real Gazebo stack; ground truth is observation only.

Acceptance thresholds are fixed here before evaluating a run: all six streams,
connected TF, geographic round trip within 2 m, backward movement 0.10--1.0 m for a positive command
for a 0.15 m/s command over 2 simulated seconds, local/truth delta disagreement
below 0.4 m. Navigation: a 1 m forward map goal must succeed within 180 wall
seconds; a second goal must acknowledge cancellation within 10 wall seconds.
Run motion checks with Nav2 disabled; do not command the wheel controller while
Nav2 controls it. The probe has bounded wall deadlines even if /clock stops.
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_localization.srv import FromLL
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from rover_navigation.wait_for_localization import valid_odometry
from rover_navigation.navigation_client import wait_for_navigation_active
from tf2_ros import Buffer, TransformListener


def has_finite_depth(cloud):
    if cloud.width*cloud.height == 0:
        return False
    points = point_cloud2.read_points(cloud, field_names=('x','y','z'), skip_nans=False)
    if isinstance(points, np.ndarray) and points.dtype.names:
        return bool(np.any(np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])))
    return any(all(math.isfinite(v) for v in point) for point in points)


class Probe(Node):
    def __init__(self):
        super().__init__('simulation_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.messages = {}
        self.counts = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        streams = [('clock', Clock, '/clock'), ('truth', Odometry, '/simulation/ground_truth'),
                   ('wheel', Odometry, '/diff_drive_controller/odom'),
                   ('local', Odometry, '/odometry/local'), ('global', Odometry, '/odometry/global'),
                   ('gps', NavSatFix, '/gps/fix'), ('imu', Imu, '/zed/zed_node/imu/data'),
                   ('points', PointCloud2, '/depth/points')]
        self.handles = [self.create_subscription(t, topic, lambda m, k=k: self.record(k, m),
                                                qos_profile_sensor_data) for k, t, topic in streams]
        self.command = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.converter = self.create_client(FromLL, '/fromLL')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def record(self, key, message):
        self.messages[key] = message
        self.counts[key] = self.counts.get(key, 0) + 1

    def wait(self, predicate, seconds=120):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(f'Probe timeout; streams received: {self.counts}')

    def future(self, value, seconds=10):
        self.wait(value.done, seconds)
        result = value.result()
        if result is None:
            raise RuntimeError('Request returned no result')
        return result

    def pose(self, key):
        p = self.messages[key].pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        assert all(math.isfinite(v) for v in (p.position.x, p.position.y, yaw))
        return p.position.x, p.position.y, yaw

    def check_streams(self):
        self.wait(lambda: len(self.messages) == 8 and all(v >= 3 for v in self.counts.values()))
        for parent, child in [('map', 'odom'), ('odom', 'base_link'), ('base_link', 'zed_camera_optical_frame')]:
            self.wait(lambda: self.buffer.can_transform(parent, child, rclpy.time.Time()))
        # Sky pixels are infinite even in a valid depth image; require at
        # least one finite xyz point, not that the first returned pixel is finite.
        self.wait(lambda: has_finite_depth(self.messages['points']), 120)
        imu = self.messages['imu']
        q, w, a = imu.orientation, imu.angular_velocity, imu.linear_acceleration
        assert all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w,w.x,w.y,w.z,a.x,a.y,a.z)), 'Nonfinite IMU'
        assert abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1.) < 0.05, 'Invalid IMU quaternion'
        for covariance in (imu.orientation_covariance, imu.angular_velocity_covariance, imu.linear_acceleration_covariance):
            assert all(math.isfinite(v) for v in covariance)
            assert all(covariance[i] >= 0 for i in (0,4,8))
        for key in ('local', 'global', 'wheel', 'truth'):
            assert valid_odometry(self.messages[key]), f'Corrupt {key} odometry'
        for key in ('local', 'global', 'wheel', 'truth'):
            self.pose(key)
        self.wait(lambda: self.converter.service_is_ready())
        fix = self.messages['gps']
        assert fix.status.status >= 0
        assert all(math.isfinite(v) for v in (fix.latitude,fix.longitude,fix.altitude))
        assert all(math.isfinite(v) for v in fix.position_covariance)
        assert all(fix.position_covariance[i] >= 0 for i in (0,4,8))
        request = FromLL.Request()
        request.ll_point.latitude, request.ll_point.longitude = fix.latitude, fix.longitude
        request.ll_point.altitude = fix.altitude
        point = self.future(self.converter.call_async(request)).map_point
        x, y, _ = self.pose('global')
        error = math.hypot(point.x-x, point.y-y)
        assert error < 2.0, f'GPS conversion disagrees with global pose by {error:.3f}m'
        print(json.dumps({'streams': self.counts, 'gps_conversion_error_m': error}), flush=True)

    def check_motion(self):
        initial_truth, initial_local = self.pose('truth'), self.pose('local')
        initial_time = self.get_clock().now().nanoseconds
        command = Twist()
        command.linear.x = 0.15
        try:
            deadline = time.monotonic() + 180
            while (self.get_clock().now().nanoseconds-initial_time) < 2e9:
                if time.monotonic() > deadline:
                    raise TimeoutError('Simulation clock stalled during motion')
                self.command.publish(command)
                rclpy.spin_once(self, timeout_sec=0.03)
        finally:
            self.command.publish(Twist())
        # Allow the wheel timeout to take effect before assessing final motion.
        until = time.monotonic()+2
        while time.monotonic()<until:
            rclpy.spin_once(self, timeout_sec=0.05)
        truth, local = self.pose('truth'), self.pose('local')
        dt = (truth[0]-initial_truth[0], truth[1]-initial_truth[1])
        dl = (local[0]-initial_local[0], local[1]-initial_local[1])
        distance = math.hypot(*dt)
        forward = dt[0]*math.cos(initial_truth[2])+dt[1]*math.sin(initial_truth[2])
        error = math.hypot(dl[0]-dt[0], dl[1]-dt[1])
        assert 0.10 < distance < 1.0, f'Truth displacement {distance:.3f}m'
        assert -1.0 < forward < -0.10, f'Positive command did not move backward: {forward:.3f}m'
        assert error < 0.4, f'Local/truth motion disagreement {error:.3f}m'
        print(json.dumps({'truth_displacement_m':distance,'forward_m':forward,'local_delta_error_m':error}),flush=True)

    def goal(self, distance, require_accepted=True):
        transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = t.x + distance*math.cos(yaw)
        goal.pose.pose.position.y = t.y + distance*math.sin(yaw)
        goal.pose.pose.orientation = q
        handle = self.future(self.navigator.send_goal_async(goal), 20)
        if require_accepted:
            assert handle.accepted, 'Nav2 rejected goal'
        return handle

    def check_reverse_and_turn(self):
        # Fixed before evaluation: negative command must travel forward by 0.10--1m;
        # +0.4rad/s for 2sim seconds must turn negatively by 0.2--1.6rad.
        def command_for(linear, angular):
            start = self.get_clock().now().nanoseconds
            deadline = time.monotonic() + 180
            message = Twist()
            message.linear.x, message.angular.z = linear, angular
            try:
                while self.get_clock().now().nanoseconds-start < 2e9:
                    if time.monotonic() > deadline:
                        raise TimeoutError('Clock stalled during reverse/turn')
                    self.command.publish(message)
                    rclpy.spin_once(self, timeout_sec=0.03)
            finally:
                self.command.publish(Twist())
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)

        before = self.pose('truth')
        command_for(-0.15, 0.0)
        after = self.pose('truth')
        backward = ((after[0]-before[0])*math.cos(before[2])
                    +(after[1]-before[1])*math.sin(before[2]))
        assert 0.10 < backward < 1.0, f'Negative-command displacement {backward:.3f}m'
        before = self.pose('truth')
        command_for(0.0, 0.4)
        after = self.pose('truth')
        yaw = math.atan2(math.sin(after[2]-before[2]), math.cos(after[2]-before[2]))
        assert -1.6 < yaw < -0.2, f'Positive-command turn yaw {yaw:.3f}rad'
        print(json.dumps({'reverse_forward_axis_m': backward, 'turn_yaw_rad': yaw}), flush=True)

    def check_obstacle(self):
        """A newly spawned physical box must become a lethal local costmap cell.

        This isolates observation/costmap wiring; it does not claim a collision
        free detour. Baseline must be clear at the future box location.
        """
        costmaps = []
        subscription = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', costmaps.append, qos_profile_sensor_data)
        spawner = self.create_client(SpawnEntity, '/spawn_entity')
        remover = self.create_client(DeleteEntity, '/delete_entity')
        spawned = False
        try:
            self.wait(lambda: costmaps and spawner.service_is_ready() and remover.service_is_ready())
            truth_x, truth_y, truth_yaw = self.pose('truth')
            transform = self.buffer.lookup_transform('odom','base_link',rclpy.time.Time()).transform
            t, q = transform.translation, transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            target_x, target_y = t.x+2.5*math.cos(yaw), t.y+2.5*math.sin(yaw)

            def obstacle_cost():
                grid = costmaps[-1]
                assert grid.header.frame_id == 'odom', 'Unexpected local costmap frame'
                resolution = grid.info.resolution
                i = int((target_x-grid.info.origin.position.x)/resolution)
                j = int((target_y-grid.info.origin.position.y)/resolution)
                radius = max(1,int(.3/resolution))
                cells = [grid.data[y*grid.info.width+x]
                         for y in range(max(0,j-radius), min(grid.info.height,j+radius+1))
                         for x in range(max(0,i-radius), min(grid.info.width,i+radius+1))]
                return max(cells,default=-1)

            baseline = obstacle_cost()
            assert 0 <= baseline < 90, f'Obstacle baseline is not clear: {baseline}'
            request = SpawnEntity.Request()
            request.name = 'integration_probe_obstacle'
            request.xml = """<sdf version='1.6'><model name='integration_probe_obstacle'>
              <static>true</static><link name='box'>
              <collision name='collision'><geometry><box><size>0.6 0.8 1.0</size></box></geometry></collision>
              <visual name='visual'><geometry><box><size>0.6 0.8 1.0</size></box></geometry></visual>
              </link></model></sdf>"""
            request.reference_frame = 'world'
            request.initial_pose.position.x = truth_x+2.5*math.cos(truth_yaw)
            request.initial_pose.position.y = truth_y+2.5*math.sin(truth_yaw)
            request.initial_pose.position.z = .5
            request.initial_pose.orientation.w = 1.
            response = self.future(spawner.call_async(request),30)
            assert response.success, response.status_message
            spawned = True
            self.wait(lambda: obstacle_cost() >= 90,120)
            print(json.dumps({'obstacle_baseline_cost':baseline,'obstacle_marked_cost':obstacle_cost()}),flush=True)
        finally:
            if spawned:
                request = DeleteEntity.Request()
                request.name = 'integration_probe_obstacle'
                response = self.future(remover.call_async(request),10)
                assert response.success, response.status_message
            self.destroy_subscription(subscription)
            self.destroy_client(spawner)
            self.destroy_client(remover)

    def check_navigation(self):
        self.wait(self.navigator.server_is_ready)
        wait_for_navigation_active(self)
        handle = self.goal(1.0)
        try:
            result = self.future(handle.get_result_async(), 180)
            assert result.status == GoalStatus.STATUS_SUCCEEDED, f'Nav2 status {result.status}'
        except BaseException:
            self.future(handle.cancel_goal_async(), 10)
            raise
        handle = self.goal(5.0)
        response = self.future(handle.cancel_goal_async(), 10)
        assert response.goals_canceling, 'Nav2 did not acknowledge cancellation'
        result = self.future(handle.get_result_async(), 20)
        assert result.status == GoalStatus.STATUS_CANCELED, f'Cancel terminal status {result.status}'
        # A goal outside the configured 150x150 m map must fail explicitly.
        # This tests planner failure handling, not obstacle avoidance.
        handle = self.goal(200.0, require_accepted=False)
        if handle.accepted:
            try:
                result = self.future(handle.get_result_async(), 60)
                assert result.status == GoalStatus.STATUS_ABORTED, f'Unreachable terminal status {result.status}'
            except BaseException:
                self.future(handle.cancel_goal_async(), 10)
                raise
        self.check_obstacle()
        print('PASS Nav2 reachable goal, cancellation, outside-costmap failure and physical obstacle costmap marking',flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--motion', action='store_true')
    parser.add_argument('--navigation', action='store_true')
    parser.add_argument('--obstacle', action='store_true', help='Only obstacle observation check; Nav2 must be active')
    args = parser.parse_args()
    if args.motion and args.navigation:
        parser.error('Run motion with Nav2 disabled; use separate navigation run')
    rclpy.init()
    node = Probe()
    try:
        node.check_streams()
        if args.motion:
            node.check_motion()
            node.check_reverse_and_turn()
        if args.navigation:
            node.check_navigation()
        elif args.obstacle:
            node.check_obstacle()
        print('PASS real Gazebo integration probe',flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
