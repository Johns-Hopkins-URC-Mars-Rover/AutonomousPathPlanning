from setuptools import setup

package_name = 'rover_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/madgwick_sim.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Dayal',
    maintainer_email='joshuadayal72@example.com',
    description='Simulation-only sensor adapters for the rover Gazebo stack.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'magnetic_field_simulator = '
            'rover_simulation.magnetic_field_simulator:main',
        ],
    },
)
