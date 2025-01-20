import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    package_description = "ar_description"
    urdf_file = LaunchConfiguration('urdf_file')
    robot_desc_path = LaunchConfiguration('robot_desc_path')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')
    
    gui_arg = DeclareLaunchArgument(
        'use_joint_state_gui',
        
        default_value='True'
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_joint_state_gui)
    )

    return LaunchDescription(
        [
            gui_arg,
            joint_state_publisher_node,  
            
        ]
    )