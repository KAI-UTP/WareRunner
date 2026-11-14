"""Stage 3 launch: simulator (no Nav2) + slam_toolbox mapping (+ optional explorer, RViz).

  ros2 launch warehouse_runner stage3.launch.py                 # drive by hand to build the map
  ros2 launch warehouse_runner stage3.launch.py explore:=true   # automatic route, then saves the map
  ros2 launch warehouse_runner stage3.launch.py gui:=false rviz:=false explore:=true   # headless
Save by hand any time:  ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/slam_map
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
    slam = get_package_share_directory('slam_toolbox')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('explore', default_value='False', description='Drive the exploration route'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share + '/launch/stage1.launch.py'),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'driver': 'False'}.items()),

        # SLAM: reads /scan + odom TF, publishes /map and the map -> odom transform.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam + '/launch/online_async_launch.py'),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': share + '/maps/slam.yaml'}.items()),

        Node(package='warehouse_runner', executable='explorer', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('explore'))),

        # Same Stage 2 view, but drawn in the map frame so you watch the map grow.
        Node(package='rviz2', executable='rviz2', arguments=['-d', share + '/rviz/stage2.rviz', '-f', 'map'],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
