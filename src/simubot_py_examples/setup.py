from setuptools import setup

package_name = 'simubot_py_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ayush',
    maintainer_email='am2836166@gmail.com',
    description='ROS 2 Code Examples',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = simubot_py_examples.simple_publisher:main',
            'simple_subscriber = simubot_py_examples.simple_subscriber:main',
            'simple_parameter = simubot_py_examples.simple_parameter:main',
            'simple_turtlesim_kinematics = simubot_py_examples.simple_turtlesim_kinematics:main',
            'simple_service_server = simubot_py_examples.simple_service_server:main',
            'simple_service_client = simubot_py_examples.simple_service_client:main',
            'simple_tf_kinematics = simubot_py_examples.simple_tf_kinematics:main',
            'simple_lifecycle_node = simubot_py_examples.simple_lifecycle_node:main',
        ],
    },
)
