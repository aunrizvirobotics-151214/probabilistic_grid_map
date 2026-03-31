from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'grid_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Occupancy grid mapping with known poses (Ex02)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_mapping_node = grid_mapping.grid_mapping_node:main',
        ],
    },
)
