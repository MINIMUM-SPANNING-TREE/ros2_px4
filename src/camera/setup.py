from setuptools import find_packages, setup
from glob import glob
import sys

for arg in ('--uninstall', '--editable'):
    if arg in sys.argv:
        sys.argv.remove(arg)
if '--build-directory' in sys.argv:
    index = sys.argv.index('--build-directory')
    del sys.argv[index:index + 2]

package_name = 'camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Dahua industrial camera driver for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dahua_camera_node = camera.dahua_camera_node:main',
            'camera_node = camera.camera_node:main',
            'camera_test_node = camera.camera_test_node:main',
        ],
    },
)
