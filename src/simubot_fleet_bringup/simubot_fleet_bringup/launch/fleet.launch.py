import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _per_robot_group(i, model_path, use_sim_time):
    ns = f"robot_{i+1}"
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            model_path,
            " is_ignition:=", is_ignition,
            " is_sim:=true",
        ]),
        value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=ns,
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", ns,
            "-x", str(1.5 * i), "-y", "0.0", "-z", "0.05",
        ],
    )
    return [rsp, spawn]


def _sanitized_gui_env():
    """Return a clean env for gz GUI: remove SNAP* and any /snap paths."""
    env = dict(os.environ)  # start from current to retain ROS & display
    # Keep resource path (we set it earlier), DISPLAY, XAUTHORITY, etc.
    keep_keys = set(env.keys())

    # Drop snap-related variables entirely
    for k in list(env.keys()):
        if k.startswith("SNAP"):
            env.pop(k, None)

    # Helper to strip /snap segments from path-like vars
    def strip_snap_paths(val):
        if not val:
            return val
        parts = [p for p in val.split(":") if "/snap/" not in p and not p.startswith("/snap/")]
        return ":".join(parts)

    for key in ("PATH", "LD_LIBRARY_PATH", "PYTHONPATH", "LIBGL_DRIVERS_PATH", "QT_PLUGIN_PATH"):
        env[key] = strip_snap_paths(env.get(key, ""))

    # Make sure we point to the system libraries first
    env["PATH"] = "/usr/bin:/usr/local/bin:" + env.get("PATH", "")
    env["LD_LIBRARY_PATH"] = strip_snap_paths(env.get("LD_LIBRARY_PATH", ""))

    # Do not preload snap libs
    for key in ("LD_PRELOAD", "LD_PRELOAD_32", "LD_PRELOAD_64"):
        env.pop(key, None)

    return env


def generate_launch_description():
    simubot_description = get_package_share_directory("simubot_description")
    this_pkg_share = get_package_share_directory("simubot_fleet_bringup")

    # ---- Launch args ----
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(simubot_description, "urdf", "simubot.urdf.xacro"),
        description="Absolute path to robot URDF xacro",
    )
    robot_count_arg = DeclareLaunchArgument(
        name="robot_count", default_value=TextSubstitution(text="2"), description="Number of robots"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", default_value=TextSubstitution(text="true"), description="Use /clock"
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=TextSubstitution(text="empty.sdf"),
        description="World file for gz sim (e.g., 'empty.sdf' or absolute path)",
    )
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value=TextSubstitution(text="false"),
        description="Launch the Gazebo GUI client (APT /usr/bin/gz, sanitized env)",
    )

    # Resources for meshes/URDFs (both server and GUI inherit)
    gz_res = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(simubot_description).parent.resolve()), ":" + this_pkg_share],
    )

    # Bridge /clock once (GZ -> ROS)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    def launch_setup(context, *args, **kwargs):
        model_path = LaunchConfiguration("model").perform(context)
        use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
        robot_count = int(LaunchConfiguration("robot_count").perform(context))
        world = LaunchConfiguration("world")
        want_gui = LaunchConfiguration("gui").perform(context).lower() == "true"

        # ---- Start gz server ONLY (headless & stable) ----
        gz_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ]),
            launch_arguments=[("gz_args", [" -v 4", " -s", " ", world])],
        )

        # ---- Global /robot_description to satisfy any ros2_control listeners ----
        ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
        is_ignition = "True" if ros_distro == "humble" else "False"
        global_robot_description = ParameterValue(
            Command(["xacro ", model_path, " is_ignition:=", is_ignition, " is_sim:=true"]),
            value_type=str,
        )
        global_rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": global_robot_description, "use_sim_time": use_sim_time}],
            output="screen",
        )

        actions = [gz_res, gz_server, clock_bridge, global_rsp]

        # ---- Spawn robots ----
        for i in range(robot_count):
            actions.extend(_per_robot_group(i, model_path, use_sim_time))

        # ---- Optional GUI client: force APT binary + sanitized environment ----
        if want_gui:
            gui_env = _sanitized_gui_env()
            # Ensure GZ resource path is present in the sanitized env
            if "GZ_SIM_RESOURCE_PATH" not in gui_env or not gui_env["GZ_SIM_RESOURCE_PATH"]:
                gui_env["GZ_SIM_RESOURCE_PATH"] = (
                    str(Path(simubot_description).parent.resolve()) + ":" + this_pkg_share
                )

            gz_gui = ExecuteProcess(
                cmd=["/usr/bin/gz", "sim", "-g", "-v", "4"],
                additional_env=gui_env,
                output="screen",
            )
            actions.append(gz_gui)

        return actions

    return LaunchDescription([
        model_arg,
        robot_count_arg,
        use_sim_time_arg,
        world_arg,
        gui_arg,
        OpaqueFunction(function=launch_setup),
    ])
'''# Terminal A
ros2 launch simubot_fleet_bringup fleet.launch.py robot_count:=2

# Terminal B
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix simubot_description)/share:$(ros2 pkg prefix simubot_fleet_bringup)/share
/usr/bin/gz sim -g
'''