import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORKSPACE = os.environ.get("ARDUROVER_NAV_ROOT", "/home/developer/ardurover_navigation")
PKG_SHARE = get_package_share_directory("ardurover_nav")


def generate_launch_description() -> LaunchDescription:
    gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value="true",
        description="Start Gazebo GUI. Set false for headless.",
    )
    path_file = DeclareLaunchArgument(
        "path_file",
        default_value=os.path.join(WORKSPACE, "paths", "recorded.path"),
    )
    output_file = DeclareLaunchArgument(
        "output_file",
        default_value=os.path.join(WORKSPACE, "paths", "score.txt"),
    )
    max_speed = DeclareLaunchArgument("max_speed", default_value="1.0")
    pid_kp = DeclareLaunchArgument("pid_kp", default_value="1.5")
    pid_ki = DeclareLaunchArgument("pid_ki", default_value="0.0")
    pid_kd = DeclareLaunchArgument("pid_kd", default_value="0.2")
    cte_gain = DeclareLaunchArgument("cte_gain", default_value="1.0")
    stanley_k = DeclareLaunchArgument("stanley_k", default_value="2.0")
    stanley_k_soft = DeclareLaunchArgument("stanley_k_soft", default_value="1.0")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(PKG_SHARE, "launch", "sim.launch.py")),
        launch_arguments={"gz_gui": LaunchConfiguration("gz_gui")}.items(),
    )
    controller = Node(
        package="ardurover_nav",
        executable="trajectory_controller_node",
        name="trajectory_controller_node",
        output="screen",
        parameters=[
            {
                "path_file": LaunchConfiguration("path_file"),
                "max_speed": LaunchConfiguration("max_speed"),
                "pid_kp": LaunchConfiguration("pid_kp"),
                "pid_ki": LaunchConfiguration("pid_ki"),
                "pid_kd": LaunchConfiguration("pid_kd"),
                "cte_gain": LaunchConfiguration("cte_gain"),
                "stanley_k": LaunchConfiguration("stanley_k"),
                "stanley_k_soft": LaunchConfiguration("stanley_k_soft"),
            }
        ],
    )
    scorer = Node(
        package="ardurover_nav",
        executable="path_scorer_node",
        name="path_scorer_node",
        output="screen",
        parameters=[
            {
                "path_file": LaunchConfiguration("path_file"),
                "output_file": LaunchConfiguration("output_file"),
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(PKG_SHARE, "rviz", "ugv.rviz")],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_gui,
            path_file,
            output_file,
            max_speed,
            pid_kp,
            pid_ki,
            pid_kd,
            cte_gain,
            stanley_k,
            stanley_k_soft,
            sim,
            controller,
            scorer,
            rviz,
        ]
    )
