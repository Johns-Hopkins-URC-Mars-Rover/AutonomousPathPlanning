"""Portable model resources must resolve in a clean checkout/install."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]

def test_park_has_no_developer_home():
    assert '/home/' not in (ROOT / 'world/park.world').read_text()


def test_simulation_defaults_to_park_fixed_heightmap():
    simulation = (ROOT / 'launch/simulation.launch.py').read_text()
    gazebo = (ROOT / 'launch/gazebo_launch.py').read_text()
    park = (ROOT / 'world/park.world').read_text()
    assert "'park.world'" in simulation
    assert "'park.world'" in gazebo
    assert 'park_fixed.tif' in park
    assert (ROOT / 'world/park_fixed.tif').is_file()

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


def _joint_contract(joint):
    origin = joint.find('origin')
    return (
        joint.find('parent').get('link'),
        joint.find('child').get('link'),
        origin.get('xyz', '0 0 0'),
        origin.get('rpy', '0 0 0'),
    )


def test_simplified_model_matches_physical_sensor_tree():
    full = ET.parse(ROOT / 'urdf/rover.urdf').getroot()
    simple = ET.parse(ROOT / 'urdf/simp_rover.urdf').getroot()
    full_links = {link.get('name') for link in full.findall('link')}
    simple_links = {link.get('name') for link in simple.findall('link')}
    required = {
        'base_link', 'chassis', 'gps_link', 'imu_link',
        'zed_camera_link', 'zed_camera_optical_frame', 'zed_imu_link',
    }
    assert required <= full_links
    assert required <= simple_links

    names = {
        'base_link_joint', 'gps_joint', 'imu_joint', 'zed_camera_joint',
        'zed_camera_optical_joint', 'zed_imu_joint',
    }
    full_joints = {joint.get('name'): joint for joint in full.findall('joint')}
    simple_joints = {joint.get('name'): joint for joint in simple.findall('joint')}
    for name in names:
        assert _joint_contract(simple_joints[name]) == _joint_contract(full_joints[name])


def test_simulated_sensor_topics_match_physical_contracts():
    text = (ROOT / 'urdf/simp_rover.urdf').read_text()
    for topic in (
        '/gps/fix',
        '/imu/data_raw',
        '/zed/zed_node/imu/data',
        '/zed/zed_node/odom',
        '/zed/zed_node/point_cloud/cloud_registered',
    ):
        assert topic in text


def test_simulation_uses_ros_forward_wheel_convention_directly():
    robot = ET.parse(ROOT / 'urdf/simp_rover.urdf')
    wheel_joints = [
        joint for joint in robot.findall('joint')
        if joint.get('name').endswith(('_front_joint', '_mid_joint', '_back_joint'))
    ]
    assert len(wheel_joints) == 6
    assert all(joint.find('axis').get('xyz') == '0 1 0'
               for joint in wheel_joints)


def test_controller_does_not_publish_localization_tf():
    config = yaml.safe_load((ROOT / 'config/rover_controller.yaml').read_text())
    drive = config['diff_drive_controller']['ros__parameters']
    assert drive['enable_odom_tf'] is False
