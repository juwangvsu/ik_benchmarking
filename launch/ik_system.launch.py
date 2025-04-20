from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch configurations for the third component
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    return LaunchDescription([
        # IK Node
        Node(
            package='ros2_kinematics_kdl',
            executable='ik_node',
            name='ik_node',
            output='screen',
            parameters=[{
                'urdf_path': 'src/nv6/urdf/panda.urdf',
                'base_link': 'panda_link0',
                'ee_link': 'panda_link6'
            }]
        ),

        # Motion Agent Node
        Node(
            package='motion_agent',
            executable='agent',
            name='motion_agent',
            output='screen',
            parameters=[{
                'url': 'wss://service.zenimotion.com/nats',
                'sub': 'subject.pose',
                'frame_id': 'panda_link0'
            }],
            remappings=[
                ('/goal_pose', '/pose')
            ]
        ),

        # Include IK Benchmarking RViz launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare('ik_benchmarking').find('ik_benchmarking'),
                    'launch',
                    'launch.rviz.py'
                )
            ),
            launch_arguments={
                'urdf_file': 'src/ik_benchmarking/urdf/pandahandfork_fullpath7_v2.urdf',
                'rviz_config_file': 'src/ik_benchmarking/rviz/panda.rviz'
            }.items()
        )
    ])

