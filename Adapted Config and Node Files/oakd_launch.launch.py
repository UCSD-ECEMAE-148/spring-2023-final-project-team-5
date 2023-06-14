# Oakd Launch File from sensor2_pkg

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


#My attempt at creating launch file for oakd_node.py
def generate_launch_description():
    sensor_pkg = 'ucsd_robocar_sensor2_pkg'
    node_name = 'oakd_node'

    ld = LaunchDescription()

    oakd_node = Node(
        package=sensor_pkg,
        executable=node_name,
        output='screen'
    )

    ld.add_action(oakd_node)

    return ld

    