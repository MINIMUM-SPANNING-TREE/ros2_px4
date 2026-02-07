from setuptools import find_packages, setup

package_name = 'uav_mavros2'

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
            'telemetry_node = uav_mavros2.telemetry:main',
            'uav_ctrl_node = uav_mavros2.uav_ctrl:main',
            'print_node = uav_mavros2.print:main',
        ],
    },
)
