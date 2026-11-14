"""Stage 1 launch: simulator without navigation, plus your square_driver node.

  ros2 launch warehouse_runner stage1.launch.py
  ros2 launch warehouse_runner stage1.launch.py gui:=false side_length:=1.5 loops:=2
  ros2 launch warehouse_runner stage1.launch.py driver:=false   # sim only, drive by teleop
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    warehouse = get_package_share_directory('warehouse_runner') + '/launch/warehouse.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='True', description='Show the Gazebo window'),
        DeclareLaunchArgument('driver', default_value='True', description='Start square_driver'),
        DeclareLaunchArgument('side_length', default_value='1.0'),
        DeclareLaunchArgument('loops', default_value='1'),

        # Reuse the existing world + robot, but leave Nav2 off: you are in control.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(warehouse),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'nav': 'False'}.items()),

        Node(
            package='warehouse_runner', executable='square_driver', output='screen',
            condition=IfCondition(LaunchConfiguration('driver')),
            parameters=[{
                'use_sim_time': True,
                'side_length': ParameterValue(LaunchConfiguration('side_length'), value_type=float),
                'loops': ParameterValue(LaunchConfiguration('loops'), value_type=int),
            }]),
    ])
