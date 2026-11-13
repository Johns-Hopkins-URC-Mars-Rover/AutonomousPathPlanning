"""Portable model resources must resolve in a clean checkout/install."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]

def test_park_has_no_developer_home():
    assert '/home/' not in (ROOT / 'world/park.world').read_text()

def test_simplified_meshes_exist():
    robot = ET.parse(ROOT / 'urdf/simp_rover.urdf')
    for mesh in robot.findall('.//mesh'):
        uri = mesh.get('filename')
        assert uri.startswith('package://rover_description/')
        assert (ROOT / uri.removeprefix('package://rover_description/')).is_file(), uri


def test_controller_geometry_matches_collision_centers():
    robot = ET.parse(ROOT / 'urdf/simp_rover.urdf')
    parents = {j.find('child').get('link'): j for j in robot.findall('joint')}
    links = {link.get('name'): link for link in robot.findall('link')}
    config = yaml.safe_load((ROOT / 'config/rover_controller.yaml').read_text())
    drive = config['diff_drive_controller']['ros__parameters']
    sides = []
    for side in ('left', 'right'):
        centers = []
        for joint_name in drive[side + '_wheel_names']:
            joint = next(j for j in robot.findall('joint') if j.get('name') == joint_name)
            child = joint.find('child').get('link')
            collision = links[child].find('collision')
            assert abs(float(collision.find('geometry/cylinder').get('radius')) - drive['wheel_radius']) < 1e-6
            y = float(collision.find('origin').get('xyz').split()[1])
            while child in parents:
                joint = parents[child]
                origin = joint.find('origin')
                assert [float(v) for v in origin.get('rpy', '0 0 0').split()] == [0, 0, 0]
                y += float(origin.get('xyz').split()[1])
                child = joint.find('parent').get('link')
            centers.append(y)
        sides.append(sum(centers) / len(centers))
    assert abs(sides[0] - sides[1] - drive['wheel_separation']) < 1e-6
