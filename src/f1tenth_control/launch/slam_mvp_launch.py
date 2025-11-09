from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gym_pkg = get_package_share_directory('f1tenth_gym_ros')
    ctrl_pkg = get_package_share_directory('f1tenth_control')

    # gym_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ctrl_pkg, 'launch', 'gym_bridge_launch.py')
    #     )
    # )
    gym_launch = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='gym_bridge',
        output='screen',
        parameters=[os.path.join(gym_pkg, 'config', 'sim.yaml')]
    )
    # and set your SLAM topics to the namespaced ones:
    gt_map_yaml = os.path.join(gym_pkg, 'maps', 'levine.yaml')
    fake_slam = Node(
    package='f1tenth_control',
    executable='mvp_fake_slam',
    name='mvp_fake_slam',
    parameters=[{
        'gt_map_yaml': gt_map_yaml,
        'lap_seconds': 25.0,
        'odom_topic': '/ego_racecar/odom',
        'scan_topic': '/ego_racecar/scan',
    }],
    output='screen'
    )


    return LaunchDescription([gym_launch, fake_slam])
