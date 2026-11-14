"""Full warehouse: Gazebo world + TurtleBot3 + Nav2 (+ delivery controller).

  ros2 launch warehouse_runner warehouse.launch.py
  ros2 launch warehouse_runner warehouse.launch.py gui:=false
  ros2 launch warehouse_runner warehouse.launch.py map:=$HOME/ros2_ws/maps/slam_map.yaml   # Stage 4: your SLAM map
  ros2 launch warehouse_runner warehouse.launch.py nav:=false                                # manual only
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as share


def generate_launch_description():
    ours = share('warehouse_runner')
    tb = share('turtlebot3_gazebo')
    gz = share('gazebo_ros')
    nav = share('nav2_bringup')

    def include(path, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='True'),
        DeclareLaunchArgument('nav', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='False'),
        DeclareLaunchArgument('map', default_value=ours + '/maps/warehouse.yaml', description='Map for Nav2'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', tb + '/models:' + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        include(gz + '/launch/gazebo.launch.py', {'world': ours + '/worlds/warehouse.world', 'gui': LaunchConfiguration('gui')}),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': open(tb + '/urdf/turtlebot3_burger.urdf').read()}]),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-timeout', '120', '-entity', 'warehouse_bot', '-file', tb + '/models/turtlebot3_burger/model.sdf',
                        '-x', '0', '-y', '-3.8', '-z', '0.01'], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav + '/launch/bringup_launch.py'),
            condition=IfCondition(LaunchConfiguration('nav')),
            launch_arguments={'map': LaunchConfiguration('map'), 'use_sim_time': 'True',
                              'params_file': ours + '/maps/nav2.yaml', 'autostart': 'True',
                              'use_composition': 'False'}.items()),
        # Stage 5-6: the delivery controller (orders on /delivery/order, status on /delivery/status).
        Node(package='warehouse_runner', executable='delivery_controller', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('nav'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', nav + '/rviz/nav2_default_view.rviz'],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
