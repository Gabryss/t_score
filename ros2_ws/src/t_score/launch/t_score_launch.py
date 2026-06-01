import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def normalize_namespace(namespace):
    if not namespace or namespace == "/":
        return ""
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    return namespace.rstrip("/")


def resolve_topic_name(topic, namespace):
    namespace = normalize_namespace(namespace)
    if not topic or not namespace:
        return topic
    if topic.startswith(namespace + "/"):
        return topic
    if topic.startswith("/"):
        return namespace + topic
    return namespace + "/" + topic


def launch_setup(context, *args, **kwargs):
    default_config = os.path.join(
        get_package_share_directory("t_score"),
        "config",
        "params.json",
    )
    config = LaunchConfiguration("param_path").perform(context) or default_config

    with open(config, "r") as config_file:
        params = json.load(config_file)

    namespace_override = LaunchConfiguration("namespace").perform(context)
    profile_override = LaunchConfiguration("profile").perform(context)
    robot_frame_override = LaunchConfiguration("robot_frame").perform(context)
    map_frame_override = LaunchConfiguration("map_frame").perform(context)

    namespace = normalize_namespace(namespace_override or params.get("ros_namespace", ""))
    tf_topic = resolve_topic_name(params.get("tf_topic", "/tf"), namespace)
    tf_static_topic = resolve_topic_name(params.get("tf_static_topic", "/tf_static"), namespace)

    parameter_overrides = {
        "param_path": config,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "ros_namespace": namespace,
    }

    if profile_override:
        parameter_overrides["traversability_profile"] = profile_override
    if robot_frame_override:
        parameter_overrides["robot_frame_id"] = robot_frame_override
    if map_frame_override:
        parameter_overrides["traversability_frame_id"] = map_frame_override

    return [
        Node(
            package="t_score",
            name="traversability_score_node",
            executable="t_score_node",
            remappings=[
                ("/tf", tf_topic),
                ("/tf_static", tf_static_topic),
            ],
            parameters=[parameter_overrides],
        )
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("t_score"),
        "config",
        "params.json",
    )

    return LaunchDescription([
        DeclareLaunchArgument("param_path", default_value=default_config),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("profile", default_value=""),
        DeclareLaunchArgument("robot_frame", default_value=""),
        DeclareLaunchArgument("map_frame", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
