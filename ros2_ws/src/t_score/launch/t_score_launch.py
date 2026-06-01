import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('t_score'),
        'config',
        'params.json'
        )

    with open(config, 'r') as config_file:
        params = json.load(config_file)

    namespace = normalize_namespace(params.get('ros_namespace', ''))
    tf_topic = resolve_topic_name(params.get('tf_topic', '/tf'), namespace)
    tf_static_topic = resolve_topic_name(params.get('tf_static_topic', '/tf_static'), namespace)
        
    node=Node(
        package = 't_score',
        name = 'traversability_score_node',
        executable = 't_score_node',
        remappings=[
            ('/tf', tf_topic),
            ('/tf_static', tf_static_topic),
        ],
        parameters = [
            {"param_path": config},
            {"use_sim_time": True}]
    )
    ld.add_action(node)
    return ld
