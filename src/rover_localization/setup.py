import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    scripts=glob('scripts/*.py'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JoshuaDayal',
    maintainer_email='joshuadayal72@example.com',
    description='Dual-EKF package for JHU Rover',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
