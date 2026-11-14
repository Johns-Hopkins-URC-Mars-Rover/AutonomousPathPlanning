"""Nav2 polygon must contain the physical rover URDF collision projection."""
import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2] / 'rover_description'
NAVIGATION_ROOT = Path(__file__).resolve().parents[1]


def rotation(rpy):
    r, p, y = rpy
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def pose(element):
    if element is None:
        return np.eye(3), np.zeros(3)
    return rotation([float(v) for v in element.get('rpy','0 0 0').split()]), np.array([float(v) for v in element.get('xyz','0 0 0').split()])


def mesh_vertices(element):
    prefix = 'package://rover_description/'
    filename = element.get('filename')
    assert filename.startswith(prefix)
    data = (ROOT / filename.removeprefix(prefix)).read_bytes()
    triangle_count = int.from_bytes(data[80:84], 'little')
    if len(data) == 84 + triangle_count * 50:
        records = np.frombuffer(data, dtype=np.dtype([
            ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
            ('attribute', '<u2')]), offset=84, count=triangle_count)
        vertices = records['vertices'].reshape(-1, 3).astype(float)
    else:
        vertices = np.array([
            [float(value) for value in line.split()[1:]]
            for line in data.decode('ascii').splitlines()
            if line.strip().startswith('vertex ')])
    scale = np.array([float(value) for value in element.get('scale', '1 1 1').split()])
    return vertices * scale


def collision_bounds():
    robot = ET.parse(ROOT / 'urdf/rover.urdf').getroot()
    transforms = {'base_link': (np.eye(3), np.zeros(3))}
    pending = list(robot.findall('joint'))
    while pending:
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            rp, tp = transforms[parent]
            r, t = pose(joint.find('origin'))
            transforms[joint.find('child').get('link')] = (rp@r,tp+rp@t)
            pending.remove(joint)
    bounds = []
    for link in robot.findall('link'):
        rl, tl = transforms[link.get('name')]
        for collision in link.findall('collision'):
            rc, tc = pose(collision.find('origin'))
            r, t = rl@rc, tl+rl@tc
            geometry = collision.find('geometry')
            box, cylinder, mesh = (geometry.find('box'), geometry.find('cylinder'),
                                   geometry.find('mesh'))
            if box is not None:
                extent = np.abs(r) @ (np.array([float(v) for v in box.get('size').split()])/2)
                bounds += [t-extent,t+extent]
            elif cylinder is not None:
                axis = r[:,2]
                extent = float(cylinder.get('radius'))*np.sqrt(np.maximum(0.,1-axis*axis))+float(cylinder.get('length'))/2*np.abs(axis)
                bounds += [t-extent,t+extent]
            elif mesh is not None:
                points = (r @ mesh_vertices(mesh).T).T + t
                bounds += [points.min(axis=0), points.max(axis=0)]
            else:
                raise AssertionError('Add explicit bounds for new collision geometry')
    return np.min(bounds,axis=0),np.max(bounds,axis=0)


def test_costmaps_cover_collision_geometry_with_polygon():
    config = yaml.safe_load((ROOT/'config/nav2_params.yaml').read_text())
    lower, upper = collision_bounds()
    for name in ('global_costmap','local_costmap'):
        params = config[name][name]['ros__parameters']
        footprint = np.array(ast.literal_eval(params['footprint']))
        assert footprint.shape == (4,2)
        assert np.all(footprint.min(axis=0) <= lower[:2])
        assert np.all(footprint.max(axis=0) >= upper[:2])
        assert params['footprint_padding'] >= .02
    critics = config['controller_server']['ros__parameters']['FollowPath']['critics']
    assert 'ObstacleFootprint' in critics


def test_normal_navigation_is_forward_only_with_slow_reverse_recovery():
    config = yaml.safe_load((ROOT/'config/nav2_params.yaml').read_text())
    planner = config['planner_server']['ros__parameters']['GridBased']
    controller = config['controller_server']['ros__parameters']['FollowPath']
    progress_checker = config['controller_server']['ros__parameters']['progress_checker']
    goal_checker = config['controller_server']['ros__parameters']['goal_checker']
    velocity_smoother = config['velocity_smoother']['ros__parameters']

    assert planner['plugin'] == 'nav2_smac_planner/SmacPlannerHybrid'
    assert planner['motion_model_for_search'] == 'DUBIN'
    assert planner['minimum_turning_radius'] > 0.0
    assert planner['change_penalty'] >= 0.0
    assert planner['non_straight_penalty'] >= 1.0
    assert planner['analytic_expansion_max_length'] >= (
        4.0 * planner['minimum_turning_radius'])

    assert controller['min_vel_x'] == 0.0
    assert 'min_vel_theta' not in controller
    assert velocity_smoother['min_velocity'][0] == 0.0
    assert velocity_smoother['max_velocity'][0] == controller['max_vel_x']
    assert velocity_smoother['max_velocity'][2] == controller['max_vel_theta']
    assert progress_checker['movement_time_allowance'] == 30.0
    assert goal_checker['xy_goal_tolerance'] == 5.0
    assert goal_checker['yaw_goal_tolerance'] < math.pi / 2.0

    behaviors = config['behavior_server']['ros__parameters']
    assert behaviors['behavior_plugins'] == ['spin', 'backup', 'wait']

    bt_path = NAVIGATION_ROOT / 'behavior_trees/navigate_to_pose_forward_with_slow_backup.xml'
    tree = ET.parse(bt_path)
    backups = tree.findall('.//BackUp')
    assert len(backups) == 1
    assert 0.0 < float(backups[0].get('backup_speed')) <= 0.05
    assert 0.0 < float(backups[0].get('backup_dist')) <= 0.20
    configured_tree = config['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']
    assert configured_tree.endswith('/behavior_trees/' + bt_path.name)
