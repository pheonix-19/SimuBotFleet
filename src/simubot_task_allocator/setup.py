from setuptools import find_packages, setup

package_name = 'simubot_task_allocator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['simubot_task_allocator/launch/task_allocator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pheonix',
    maintainer_email='am2836166@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_allocator = simubot_task_allocator.task_allocator_node:main'
        ],
    },
)
