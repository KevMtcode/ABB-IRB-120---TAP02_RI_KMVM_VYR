from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "abb_irb120",
            package_name="irb120_moveit_config"
        )
        .to_moveit_configs()
    )

    return LaunchDescription([

        DeclareLaunchArgument("target"),
        DeclareLaunchArgument("planner"),

        Node(
            package="irb120_cartesian",
            executable="named_move",
            output="screen",

            parameters=[
                moveit_config.to_dict(),
                {
                    "target": LaunchConfiguration("target"),
                    "planner": LaunchConfiguration("planner")
                }
            ]
        )
    ])

