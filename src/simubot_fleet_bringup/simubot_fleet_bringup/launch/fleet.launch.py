
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os

def _spawn_robot(context, i, use_sim_time):
    robot_ns = f"robot_{i+1}"
    simubot_desc_share = get_package_share_directory('simubot_description')
    urdf_xacro = os.path.join(simubot_desc_share, 'urdf', 'simubot.urdf.xacro')

    # robot_state_publisher (per robot)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_ns,
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': {'command': f'xacro {urdf_xacro}'}}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Spawn entity in Gazebo (per robot) — uses gazebo_ros
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=robot_ns,
        arguments=[
            '-topic', f'/{robot_ns}/robot_description',
            '-entity', robot_ns,
            '-x', str(i*1.5), '-y', '0.0', '-z', '0.05'
        ],
        output='screen'
    )

    # Nav2 bringup (per robot)
    nav2_params = os.path.join(
        get_package_share_directory('simubot_fleet_bringup'),
        'config', 'nav2_common.yaml'
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'namespace': robot_ns,
            'use_namespace': 'True',
            'use_sim_time': 'True' if use_sim_time else 'False',
            'params_file': nav2_params,
            'slam': 'True',
            'map': ''
        }.items()
    )

    return GroupAction([
        PushRosNamespace(robot_ns),
        rsp, spawn, nav2
    ])

def generate_launch_description():
    robot_count_arg = DeclareLaunchArgument('robot_count', default_value=TextSubstitution(text='2'))
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=TextSubstitution(text='true'))

    def launch_setup(context, *args, **kwargs):
        robot_count = int(LaunchConfiguration('robot_count').perform(context))
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

        # Gazebo server/client
        gazebo_world = os.path.join(
            get_package_share_directory('simubot_fleet_bringup'),
            'worlds', 'warehouse.world.sdf'
        )
        gz_server = IncludeLaunchDescription(PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ]), launch_arguments={'world': gazebo_world}.items())

        gz_client = IncludeLaunchDescription(PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ]))

        group_actions = [gz_server, gz_client]

        for i in range(robot_count):
            group_actions.append(_spawn_robot(context, i, use_sim_time))

        return group_actions

    return LaunchDescription([
        robot_count_arg, use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])
