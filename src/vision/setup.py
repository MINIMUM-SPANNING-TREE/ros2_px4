from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Vision package with DeepSORT tracking algorithm for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepsort_node = vision.deepsort_node:main',
            'deepsort_tracker_node = vision.deepsort_tracker_node:main',
            'yolo_node = vision.yolo_node:main',
        ],
    },
)
