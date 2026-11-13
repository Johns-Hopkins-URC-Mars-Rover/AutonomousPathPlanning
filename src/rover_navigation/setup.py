from setuptools import setup

package_name = 'rover_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Dayal',
    maintainer_email='joshuadayal72@example.com',
    description='Navigate the rover to GPS coordinates through Nav2.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'wait_for_localization = rover_navigation.wait_for_localization:main',
                'cmd_vel_relay = rover_navigation.cmd_vel_relay:main',
                'navigation_client = rover_navigation.navigation_client:main'
        ],
    },
)
