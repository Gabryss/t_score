import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('RoughSense'),
        'config',
        'params.json'
        )
        
    node=Node(
        package = 'RoughSense',
        name = 'roughness_estimation',
        executable = 'roughness_node',
        parameters = [
            {"param_path": config},
            {"use_sim_time": True}]
    )
    ld.add_action(node)
    return ld