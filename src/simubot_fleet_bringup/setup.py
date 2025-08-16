from setuptools import find_packages, setup

package_name = 'simubot_fleet_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['simubot_fleet_bringup/launch/fleet.launch.py']),
        ('share/' + package_name + '/config', ['simubot_fleet_bringup/config/nav2_common.yaml']),
        ('share/' + package_name + '/worlds', ['simubot_fleet_bringup/worlds/warehouse.world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pheonix',
    maintainer_email='am2836166@gmail.com',
    description='SimuBot Fleet Bringup package for multi-robot simulation and navigation',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
