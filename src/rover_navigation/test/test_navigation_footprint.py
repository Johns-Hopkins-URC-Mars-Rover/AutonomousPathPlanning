"""Nav2 polygon must contain all simplified URDF collision projections."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2] / 'rover_description'


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


def collision_bounds():
    robot = ET.parse(ROOT / 'urdf/simp_rover.urdf').getroot()
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
            box, cylinder = geometry.find('box'), geometry.find('cylinder')
            if box is not None:
                extent = np.abs(r) @ (np.array([float(v) for v in box.get('size').split()])/2)
            elif cylinder is not None:
                axis = r[:,2]
                extent = float(cylinder.get('radius'))*np.sqrt(np.maximum(0.,1-axis*axis))+float(cylinder.get('length'))/2*np.abs(axis)
            else:
                raise AssertionError('Add explicit bounds for new collision geometry')
            bounds += [t-extent,t+extent]
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
