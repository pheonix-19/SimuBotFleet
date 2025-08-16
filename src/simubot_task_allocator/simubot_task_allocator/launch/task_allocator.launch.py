from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    robot_count_arg = DeclareLaunchArgument('robot_count', default_value=TextSubstitution(text='2'))
    return LaunchDescription([
        robot_count_arg,
        Node(
            package='simubot_task_allocator',
            executable='task_allocator',
            name='task_allocator',
            parameters=[{'robot_count': LaunchConfiguration('robot_count')}],
            output='screen'
        )
    ])
