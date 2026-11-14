"""Stage 2 launch: simulator (no Nav2) + scan_inspector + RViz with the Stage 2 view.

  ros2 launch warehouse_runner stage2.launch.py
  ros2 launch warehouse_runner stage2.launch.py gui:=false        # no Gazebo window, RViz only
  ros2 launch warehouse_runner stage2.launch.py rviz:=false       # headless (for tests)
Drive with the Warehouse Control app or teleop_twist_keyboard while it runs.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('warehouse_runner')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='True', description='Show the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='True', description='Open RViz'),

        # Same world and robot as Stage 1, nobody driving: you drive, the inspector watches.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share + '/launch/stage1.launch.py'),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'driver': 'False'}.items()),

        Node(package='warehouse_runner', executable='scan_inspector', output='screen',
             parameters=[{'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', arguments=['-d', share + '/rviz/stage2.rviz'],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
