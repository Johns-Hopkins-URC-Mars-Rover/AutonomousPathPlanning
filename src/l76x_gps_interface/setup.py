from setuptools import find_packages, setup

package_name = 'l76x_gps_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gps_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/gps_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_publisher = l76x_gps_interface.gps_publisher:main',
            'gps_error_calibrator = l76x_gps_interface.gps_error_calibrator:main',
        ],
    },
)
