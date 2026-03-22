from setuptools import find_packages, setup

package_name = 'fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xun',
    maintainer_email='1919460637@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
              'fsm_node = fsm.uav_fsm_node:main',
              'test_takeoff = fsm.test_takeoff:main',
              'test_takeoff_ctrl = fsm.test_takeoff_ctrl:main',
              'land = fsm.land:main',
              'move = fsm.move:main',
              'test_move_ctrl = fsm.test_move_ctrl:main',
              'test_hover_ctrl = fsm.test_hover_ctrl:main',
              'test_land_ctrl = fsm.test_land_ctrl:main',
              'test_full_mission = fsm.test_full_mission:main',
            
        ],
    },
)
